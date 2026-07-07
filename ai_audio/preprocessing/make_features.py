#!/usr/bin/env python3
"""Convert raw impact/background WAVs into fixed-size log-mel (or MFCC)
feature arrays the stage-3 CNN can train on directly.

By default each WAV is split into non-overlapping --window-length windows,
so a short "hit" clip (~1s) yields one window and a longer "background" clip
(e.g. 5s) yields several. Every window is converted straight from its raw
int16 samples (divided by a fixed 32768 scale, not per-clip peak-normalized)
so relative loudness between a light tap and a damaging hit is preserved in
the feature values, not flattened out.

For long continuous recordings that contain multiple scattered impact events
(instead of one pre-isolated event per file), pass the label via
--event-detect-labels: files longer than --event-detect-min-duration then get
windows centered on each detected peak (same pre/post-roll idea as stage 1's
trigger mode, just run offline over an existing file) instead of blind fixed
slicing, which would otherwise miss events or dilute them into mostly-silence
windows. Short files for that label are unaffected -- they still go through
the plain fixed-hop path, so existing pre-isolated clips process identically
to before.

Every window's --in-dir-relative start time and duration are recorded in the
manifest (start_sec, duration_sec) so stage 3 can split long recordings by
contiguous time block instead of by whole file -- see kaggle_train.ipynb's
time_block_split for why that matters when a label has only a few, long
source files.

Output: one .npy per window under --out-dir/<label>/, plus a manifest.csv
mapping every window back to its source WAV.

Example:
  python3 make_features.py --in-dir dataset/raw --out-dir dataset/processed
  python3 make_features.py --event-detect-labels hit background_self_fire
"""

import argparse
import csv
import glob
import os
import wave

import librosa
import numpy as np

FULL_SCALE = 32768.0


def read_wav_mono16(path):
    with wave.open(path, "rb") as wf:
        sr = wf.getframerate()
        n = wf.getnframes()
        raw = wf.readframes(n)
    samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / FULL_SCALE
    return samples, sr


def iter_windows(samples, sr, window_length, hop_seconds):
    """Fixed-hop slicing. Yields (window, start_sample) pairs. Right for
    ambient/background material where there's no discrete event to find --
    any alignment is as good as any other."""
    target = int(window_length * sr)
    hop = int(hop_seconds * sr)
    if len(samples) < target:
        pad = np.zeros(target - len(samples), dtype=np.float32)
        yield np.concatenate([samples, pad]), 0
        return
    start = 0
    while start + target <= len(samples):
        yield samples[start : start + target], start
        start += hop


def detect_events(samples, threshold, refractory_frames):
    """Greedy peak-picking over samples already known to be normalized to
    [-1, 1] (see read_wav_mono16): every index crossing --event-threshold
    starts an event, then the next refractory_frames are skipped so a hit's
    own decay/ringing isn't counted as a second event -- same idea as stage
    1's trigger-mode cooldown, just applied after the fact over a whole file
    instead of live during recording."""
    candidates = np.flatnonzero(np.abs(samples) >= threshold)
    events = []
    last = -refractory_frames
    for idx in candidates:
        if idx - last >= refractory_frames:
            events.append(int(idx))
            last = idx
    return events


def iter_event_windows(samples, sr, window_length, pre_roll, threshold, refractory):
    """Event-centered windows for a long recording with scattered impacts.
    Yields (window, start_sample) pairs, one per detected event."""
    target = int(window_length * sr)
    pre = int(pre_roll * sr)
    refractory_frames = int(refractory * sr)

    for event_idx in detect_events(samples, threshold, refractory_frames):
        start = max(0, min(event_idx - pre, max(0, len(samples) - target)))
        end = start + target
        window = samples[start:end]
        if len(window) < target:
            window = np.concatenate([window, np.zeros(target - len(window), dtype=np.float32)])
        yield window, start


def extract_feature(y, sr, feature_type, n_mels, n_fft, hop_length):
    mel = librosa.feature.melspectrogram(
        y=y, sr=sr, n_fft=n_fft, hop_length=hop_length, n_mels=n_mels, power=2.0
    )
    log_mel = librosa.power_to_db(mel, ref=1.0)
    if feature_type == "logmel":
        return log_mel.astype(np.float32)
    n_mfcc = 20 if n_mels >= 20 else n_mels
    mfcc = librosa.feature.mfcc(S=log_mel, n_mfcc=n_mfcc)
    return mfcc.astype(np.float32)


def process_label(label, args, manifest_rows):
    in_paths = sorted(glob.glob(os.path.join(args.in_dir, label, "*.wav")))
    out_label_dir = os.path.join(args.out_dir, label)
    os.makedirs(out_label_dir, exist_ok=True)

    event_detect = label in args.event_detect_labels

    count = 0
    for wav_path in in_paths:
        samples, sr = read_wav_mono16(wav_path)
        stem = os.path.splitext(os.path.basename(wav_path))[0]
        duration = len(samples) / sr

        if event_detect and duration >= args.event_detect_min_duration:
            windows = iter_event_windows(
                samples, sr, args.window_length, args.event_pre_roll,
                args.event_threshold, args.event_refractory,
            )
        else:
            windows = iter_windows(samples, sr, args.window_length, args.hop)

        for idx, (window, start_sample) in enumerate(windows):
            feat = extract_feature(window, sr, args.feature_type, args.n_mels, args.n_fft, args.hop_length)
            out_path = os.path.join(out_label_dir, f"{stem}_{idx:02d}.npy")
            np.save(out_path, feat)
            manifest_rows.append(
                {
                    "path": os.path.relpath(out_path, args.out_dir),
                    "label": label,
                    "source_wav": os.path.relpath(wav_path, args.in_dir),
                    "window_index": idx,
                    "shape": "x".join(str(d) for d in feat.shape),
                    "start_sec": round(start_sample / sr, 4),
                    "duration_sec": round(len(window) / sr, 4),
                }
            )
            count += 1
    return count, len(in_paths)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--in-dir", default="dataset/raw")
    p.add_argument("--out-dir", default="dataset/processed")
    p.add_argument("--labels", nargs="+", default=None, help="labels to process (default: all subfolders of --in-dir)")
    p.add_argument("--feature-type", choices=["logmel", "mfcc"], default="logmel")
    p.add_argument("--window-length", type=float, default=1.0, help="seconds per window / model input")
    p.add_argument("--hop", type=float, default=None, help="seconds between window starts (default = --window-length, i.e. no overlap)")
    p.add_argument("--n-mels", type=int, default=64)
    p.add_argument("--n-fft", type=int, default=1024)
    p.add_argument("--hop-length", type=int, default=480, help="STFT hop length in samples (~10ms at 48kHz)")
    p.add_argument(
        "--event-detect-labels", nargs="+", default=[],
        help="labels whose long files should be windowed around detected impact events instead of fixed-hop slicing (e.g. hit background_self_fire)",
    )
    p.add_argument(
        "--event-detect-min-duration", type=float, default=3.0,
        help="only run event-detection on files at least this many seconds long; shorter (already-isolated) clips still use fixed-hop slicing unchanged",
    )
    p.add_argument("--event-threshold", type=float, default=0.15, help="peak level to count as an event, fraction of full scale (0-1)")
    p.add_argument("--event-pre-roll", type=float, default=0.2, help="seconds of context kept before a detected event")
    p.add_argument("--event-refractory", type=float, default=1.0, help="minimum seconds between two detected events (avoids double-counting one event's decay)")
    return p.parse_args()


def main():
    args = parse_args()
    if args.hop is None:
        args.hop = args.window_length

    labels = args.labels or sorted(
        d for d in os.listdir(args.in_dir) if os.path.isdir(os.path.join(args.in_dir, d))
    )

    os.makedirs(args.out_dir, exist_ok=True)
    manifest_rows = []
    for label in labels:
        n_windows, n_clips = process_label(label, args, manifest_rows)
        print(f"{label}: {n_clips} clips -> {n_windows} windows")

    manifest_path = os.path.join(args.out_dir, "manifest.csv")
    with open(manifest_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["path", "label", "source_wav", "window_index", "shape", "start_sec", "duration_sec"])
        writer.writeheader()
        writer.writerows(manifest_rows)
    print(f"manifest written to {manifest_path} ({len(manifest_rows)} total windows)")


if __name__ == "__main__":
    main()
