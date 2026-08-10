# 🎙️ AI Audio — Chassis Impact Detection Pipeline

> **Purpose:** Train and export a lightweight CNN that detects physical impacts (hits) on the robot's chassis from raw I2S microphone audio — entirely offline — so the resulting ONNX model can run in real-time on the Raspberry Pi via NCNN.

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture & Data Flow](#architecture--data-flow)
3. [Pipeline Stages](#pipeline-stages)
   - [Stage 1 — Dataset Collection](#stage-1--dataset-collection)
   - [Stage 2 — Feature Preprocessing](#stage-2--feature-preprocessing)
   - [Stage 3 — Model Training](#stage-3--model-training)
   - [Stage 4 — Deployment (Onboard)](#stage-4--deployment-onboard)
4. [Dataset Structure](#dataset-structure)
5. [Model Architecture](#model-architecture)
6. [Training Results](#training-results)
7. [Configuration Reference](#configuration-reference)
8. [Quick Start](#quick-start)

---

## Overview

The Eye-To-Zion robot is equipped with an **ICS-43434 I2S MEMS microphone** (via the Google Voice HAT overlay). During combat, physical impacts on the chassis produce distinctive acoustic signatures. This pipeline captures those signatures, converts them into log-mel spectrograms, and trains a **binary classifier** (`hit` vs. `background`) that runs onboard in real-time.

The pipeline is designed as a **four-stage, offline workflow** that runs across the Raspberry Pi (Stages 1 & 2) and a cloud GPU (Stage 3 — Kaggle), with the trained model deployed back to the Pi (Stage 4).

---

## Architecture & Data Flow

```
┌────────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI (On-Robot)                         │
│                                                                        │
│  ┌──────────────────┐    ┌────────────────────┐    ┌────────────────┐  │
│  │  Stage 1          │    │  Stage 2            │    │  Stage 4       │  │
│  │  Dataset Collector│───▶│  Feature Preprocess │    │  NCNN Inference│  │
│  │  (record_impacts) │    │  (make_features)    │    │  (onboard/)    │  │
│  │                    │    │                      │    │                │  │
│  │  I2S Mic ──▶ WAV   │    │  WAV ──▶ .npy       │    │  .onnx model   │  │
│  └──────────────────┘    └──────┬─────────────┘    └────────────────┘  │
│                                  │                          ▲            │
└──────────────────────────────────┼──────────────────────────┼────────────┘
                                   │                          │
                              Upload zip                 Download
                                   │                          │
                    ┌──────────────▼──────────────────────────┼────────┐
                    │            KAGGLE (Cloud GPU)            │        │
                    │                                          │        │
                    │  ┌────────────────────────────────────┐  │        │
                    │  │  Stage 3 — Training                 │  │        │
                    │  │  (kaggle_train.ipynb)               │  │        │
                    │  │                                      │  │        │
                    │  │  .npy features ──▶ ImpactCNN ──▶────┘  │        │
                    │  │                   (PyTorch)             │        │
                    │  │                                         │        │
                    │  │  Outputs:                               │        │
                    │  │   • impact_cnn.onnx                    │        │
                    │  │   • labels.json                        │        │
                    │  │   • feature_stats.json                 │        │
                    │  │   • confusion_matrix.png               │        │
                    │  │   • training_curves.png                │        │
                    │  │   • fp_breakdown.png                   │        │
                    │  └────────────────────────────────────────┘        │
                    └───────────────────────────────────────────────────┘
```

---

## Pipeline Stages

### Stage 1 — Dataset Collection

| | |
|---|---|
| **Directory** | `pipeline/dataset_collector/` |
| **Script** | `record_impacts.py` |
| **Container** | `python:3.11-slim-bookworm` + `libportaudio2` + `alsa-utils` |
| **Dependencies** | `sounddevice==0.5.5`, `numpy==2.2.4` |
| **Output** | `pipeline/dataset/raw/<label>/*.wav` |

Captures raw **int16 PCM** audio straight from the I2S microphone with **no digital gain, normalization, or AGC** — the saved waveform is an untouched copy of what the ADC produced. This preserves the true shockwave shape for later spectrogram/MFCC work.

#### Two Capture Modes

| Mode | Use Case | Description |
|------|----------|-------------|
| **`trigger`** | `hit` class | Listens continuously; auto-saves a clip when the signal crosses `--threshold`, keeping `--pre-roll` seconds of audio before the hit. Strike the chassis repeatedly, stop with `Ctrl+C`. |
| **`continuous`** | `background` class | Saves fixed-length clips back-to-back. Use for ambient noise, motor whine, voices — no discrete event to trigger on. |

#### Trigger State Machine (`trigger.py`)

```
IDLE ──▶ CAPTURING ──▶ COOLDOWN ──▶ IDLE
```

Each detected event produces exactly `pre_roll_frames + post_roll_frames` of audio, then enforces `cooldown_frames` of silence before re-arming.

> ⚠️ **Important:** `trigger.py` is **intentionally duplicated** at `../../onboard/audio_processor/trigger.py`. The pipeline must build and run as a fully self-contained directory independent of `onboard/`. If you tune the trigger logic, **mirror the change in both copies**, or training-time capture and live-time capture will silently diverge.

#### Example Commands

```bash
# List available audio devices
python3 record_impacts.py --list-devices

# Record "hit" samples (trigger mode)
python3 record_impacts.py --label hit --mode trigger --threshold 0.2

# Record "background" samples (continuous mode)
python3 record_impacts.py --label background --mode continuous --clip-length 5
```

#### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--samplerate` | `48000` | Audio sample rate (Hz) |
| `--channels` | `1` | Number of audio channels (mono) |
| `--threshold` | `0.15` | Trigger level, fraction of full scale (0–1) |
| `--pre-roll` | `0.05` | Seconds kept before a trigger event |
| `--post-roll` | `0.2` | Seconds kept after a trigger event |
| `--cooldown` | `1.0` | Dead-time after a clip is saved |
| `--clip-length` | `5.0` | Seconds per clip (continuous mode) |
| `--max-clips` | `0` | Stop after N clips (0 = unlimited) |

---

### Stage 2 — Feature Preprocessing

| | |
|---|---|
| **Directory** | `pipeline/preprocessing/` |
| **Script** | `make_features.py` |
| **Container** | `python:3.11-slim-bookworm` + `build-essential` + `libsndfile1` |
| **Dependencies** | `librosa==0.10.2.post1`, `numpy==2.2.4` |
| **Input** | `pipeline/dataset/raw/<label>/*.wav` |
| **Output** | `pipeline/dataset/processed/<label>/*.npy` + `manifest.csv` |

Converts raw WAV files into **fixed-size log-mel spectrogram** (or MFCC) feature arrays that the Stage 3 CNN trains on directly.

#### Processing Pipeline

1. **Read WAV** → Raw `int16` samples divided by a fixed `32768` scale (no per-clip peak normalization — relative loudness between a light tap and a damaging hit is preserved).
2. **Windowing** → Each WAV is split into non-overlapping `--window-length` (default 0.25s) windows.
3. **Feature Extraction** → Each window is converted to a `64×26` log-mel spectrogram (or MFCC) via `librosa`.
4. **Output** → One `.npy` file per window, plus a `manifest.csv` mapping every window back to its source WAV with `start_sec` and `duration_sec` columns.

#### Event Detection Mode

For long continuous recordings that contain scattered impact events, pass the label via `--event-detect-labels`. Files longer than `--event-detect-min-duration` get windows centered on each detected peak (same pre/post-roll idea as Stage 1), instead of blind fixed slicing.

#### Example Command

```bash
python3 make_features.py --in-dir dataset/raw --out-dir dataset/processed
python3 make_features.py --event-detect-labels hit background_self_fire
```

---

### Stage 3 — Model Training

| | |
|---|---|
| **Directory** | `pipeline/training/` |
| **Notebook** | `kaggle_train.ipynb` |
| **Platform** | Kaggle (GPU T4 x2) |
| **Framework** | PyTorch |
| **Output** | `pipeline/training/Output/` |

Trains the lightweight **ImpactCNN** on the processed feature windows, then exports to ONNX for onboard NCNN inference.

#### Pre-Training Setup

1. On the Pi: `cd ai_audio/pipeline/dataset && zip -r processed_dataset.zip processed`
2. Upload `processed_dataset.zip` as a new Kaggle Dataset.
3. Attach that dataset to the notebook.
4. Enable GPU accelerator (T4 or P100).
5. Run via **Kernel → Restart & Run All** (cells share global state).

#### Key Training Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| `LABEL_MAP` | `{"background_self_fire": "background"}` | Collapses sub-labels into binary classes |
| `CONTINUOUS_LABELS` | `{"background"}` | Labels needing session-aware splitting |
| `SESSION_GAP_SECONDS` | `30.0` | Gap to define separate recording sessions |
| `GUARD_SECONDS` | `8.0` | Purge band around train/val split boundary |
| `TEST_FRAC` | `0.15` | Held-out test fraction |
| `VAL_FRAC` | `0.2` | Validation fraction |
| `EPOCHS` | `40` | Training epochs |
| `BATCH_SIZE` | `16` | Mini-batch size |
| `LR` | `3e-4` | Adam learning rate |
| `CONV_CHANNELS` | `(16, 32, 64)` | Conv block output channels |

#### Data Splitting Strategy

The notebook implements a sophisticated **time-block split** to prevent data leakage:

- **Discrete events** (e.g., `hit`): Plain whole-file holdout — each file is one physically independent strike.
- **Continuous labels** (e.g., `background`): Files are grouped into *sessions* using recording timestamps. With ≥2 sessions, whole sessions are held out. With exactly one session, the timeline is split with a guard band to prevent near-duplicate windows from crossing the train/val boundary.

#### Training Outputs

| File | Description |
|------|-------------|
| `impact_cnn.onnx` | Exported ONNX model (~96 KB) for NCNN inference |
| `labels.json` | Class index mapping: `{"background": 0, "hit": 1}` |
| `feature_stats.json` | Normalization statistics: `mean`, `std`, `input_shape` |
| `training_curves.png` | Loss and F1-score per epoch |
| `confusion_matrix.png` | Test set confusion matrix |
| `fp_breakdown.png` | False positive analysis by original recording source |

---

### Stage 4 — Deployment (Onboard)

The exported `impact_cnn.onnx` is transferred back to the Raspberry Pi and loaded by the onboard `audio_processor` ROS 2 node, which uses NCNN for real-time inference. This stage lives outside of the `Offboard/` tree (see `onboard/audio_processor/`).

---

## Dataset Structure

```
pipeline/dataset/
├── raw/                          # Stage 1 output
│   ├── background/               # 221 WAV files (continuous mode)
│   ├── background_self_fire/     # 104 WAV files (triggered mode)
│   └── hit/                      # 246 WAV files (triggered mode)
├── processed/                    # Stage 2 output
│   ├── background/               # .npy feature arrays
│   ├── background_self_fire/     # .npy feature arrays
│   ├── hit/                      # .npy feature arrays
│   └── manifest.csv              # 5,439 windows mapped to source WAVs
└── processed_dataset.zip         # Zipped processed/ for Kaggle upload
```

### Dataset Statistics

| Label | Raw Files | Processed Windows | Notes |
|-------|-----------|-------------------|-------|
| `background` | 221 | 4,836 | Ambient noise, motor whine |
| `background_self_fire` | 104 | — | Collapsed into `background` for training |
| `hit` | 246 | 603 | Physical chassis impacts |
| **Total** | **571** | **5,439** | Binary: hit vs. background |

---

## Model Architecture

**ImpactCNN** — A 3-block convolutional neural network with **23,650 trainable parameters**:

```
Input: (1, 64, 26)  — single-channel log-mel spectrogram
  │
  ├─ Conv2d(1→16, 3×3, pad=1) → BatchNorm2d → ReLU → MaxPool2d(2)
  ├─ Conv2d(16→32, 3×3, pad=1) → BatchNorm2d → ReLU → MaxPool2d(2)
  ├─ Conv2d(32→64, 3×3, pad=1) → BatchNorm2d → ReLU → AdaptiveAvgPool2d(1)
  │
  └─ Flatten → Linear(64→2)
  │
Output: 2 logits (background, hit)
```

The final block uses `AdaptiveAvgPool2d(1)` instead of `MaxPool2d(2)`, so the classifier's input size is always `channels[-1]` regardless of spectrogram dimensions.

#### Training Enhancements

- **WeightedRandomSampler** — Compensates for severe class imbalance (603 hit vs. 4836 background).
- **SpecAugment** — Frequency masking (8 bins) and time masking (12 frames) for data augmentation.
- **Gradient Clipping** — `max_norm=1.0` to prevent destructive steps from imbalanced batches.
- **Checkpoint Selection** — Best model selected by validation F1-score (hit class).

---

## Training Results

| Metric | Value |
|--------|-------|
| Best Validation F1 | **0.777** |
| Test Accuracy | **0.966** |
| Test Precision | **0.857** |
| Test Recall | **0.659** |
| Test F1 | **0.745** |
| Model Size (ONNX) | ~96 KB |
| Training Duration | ~2 minutes (GPU T4) |

### Split Summary

| Split | Windows |
|-------|---------|
| Train | 2,671 |
| Validation | 1,557 |
| Test | 1,211 |

---

## Configuration Reference

### Feature Extraction Defaults

| Parameter | Value | Description |
|-----------|-------|-------------|
| `--window-length` | `0.25` s | Duration of each analysis window |
| `--n-mels` | `64` | Number of mel frequency bands |
| `--n-fft` | `1024` | FFT window size |
| `--hop-length` | `480` | STFT hop (~10 ms at 48 kHz) |
| `--feature-type` | `logmel` | Feature representation (`logmel` or `mfcc`) |
| `--event-threshold` | `0.15` | Peak detection threshold |
| `--event-refractory` | `1.0` s | Minimum gap between detected events |

### Normalization Statistics (from training set)

```json
{
  "mean": -59.834,
  "std": 16.181,
  "input_shape": [1, 64, 26]
}
```

---

## Quick Start

### 1. Collect Data (on Raspberry Pi)

```bash
cd ai_audio/pipeline/dataset_collector

# Record hits
python3 record_impacts.py --label hit --mode trigger --threshold 0.2

# Record background
python3 record_impacts.py --label background --mode continuous --clip-length 5
```

### 2. Extract Features (on Raspberry Pi)

```bash
cd ai_audio/pipeline/preprocessing
python3 make_features.py --in-dir ../dataset/raw --out-dir ../dataset/processed
```

### 3. Package for Kaggle

```bash
cd ai_audio/pipeline/dataset
zip -r processed_dataset.zip processed
# Upload to Kaggle as a dataset
```

### 4. Train (on Kaggle)

Open `pipeline/training/kaggle_train.ipynb` → Attach dataset → GPU T4 → **Restart & Run All**.

### 5. Deploy

Download `Output/impact_cnn.onnx`, `Output/labels.json`, and `Output/feature_stats.json` to the Pi for NCNN inference.
