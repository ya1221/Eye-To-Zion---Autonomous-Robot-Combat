#!/usr/bin/env python3
"""Convert raw impact/background WAVs into fixed-size log-mel (or MFCC)
feature arrays the stage-3 CNN can train on directly.

Each WAV is split into non-overlapping (by default) --window-length windows,
so a short "hit" clip (~1s) yields one window and a longer "background" clip
(e.g. 5s) yields several. Every window is converted straight from its raw
int16 samples (divided by a fixed 32768 scale, not per-clip peak-normalized)
so relative loudness between a light tap and a damaging hit is preserved in
the feature values, not flattened out.

Output: one .npy per window under --out-dir/<label>/, plus a manifest.csv
mapping every window back to its source WAV.

Example:
  python3 make_features.py --in-dir dataset/raw --out-dir dataset/processed
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
    target = int(window_length * sr)
    hop = int(hop_seconds * sr)
    if len(samples) < target:
        pad = np.zeros(target - len(samples), dtype=np.float32)
        yield np.concatenate([samples, pad])
        return
    start = 0
    while start + target <= len(samples):
        yield samples[start : start + target]
        start += hop


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

    count = 0
    for wav_path in in_paths:
        samples, sr = read_wav_mono16(wav_path)
        stem = os.path.splitext(os.path.basename(wav_path))[0]
        for idx, window in enumerate(iter_windows(samples, sr, args.window_length, args.hop)):
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
        writer = csv.DictWriter(f, fieldnames=["path", "label", "source_wav", "window_index", "shape"])
        writer.writeheader()
        writer.writerows(manifest_rows)
    print(f"manifest written to {manifest_path} ({len(manifest_rows)} total windows)")


if __name__ == "__main__":
    main()
