#!/usr/bin/env python3
"""Train a small CNN to classify chassis-impact ("hit") vs "background" from
the log-mel spectrogram windows produced by preprocessing/make_features.py.

Splits are made by source WAV (clip), not by window, so multiple windows
sliced from the same background clip never end up split across train/val -
that would leak near-identical audio into validation and inflate accuracy.
Training uses a class-balanced sampler to counter the ~7:1 background:hit
imbalance in a small collected dataset, plus light SpecAugment-style masking.

Output in --out-dir: best_model.pt (state_dict), labels.json (label->index
mapping), feature_stats.json (train-set mean/std used for normalization -
stage 4 inference must apply the exact same normalization).

Example:
  python3 train.py --processed-dir dataset/processed --out-dir runs
"""

import argparse
import csv
import json
import os
import random
from collections import defaultdict

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler


def load_manifest(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def group_split(rows, val_frac, seed):
    """Hold out whole clips (source_wav) for validation, stratified by label,
    so windows from the same clip never appear on both sides of the split."""
    by_label_clip = defaultdict(set)
    for r in rows:
        by_label_clip[r["label"]].add(r["source_wav"])

    rng = random.Random(seed)
    val_clips = set()
    for label, clips in by_label_clip.items():
        clips = sorted(clips)
        rng.shuffle(clips)
        n_val = max(1, round(len(clips) * val_frac))
        val_clips.update(clips[:n_val])

    train_rows = [r for r in rows if r["source_wav"] not in val_clips]
    val_rows = [r for r in rows if r["source_wav"] in val_clips]
    return train_rows, val_rows


def compute_stats(rows, processed_dir):
    vals = [np.load(os.path.join(processed_dir, r["path"])) for r in rows]
    arr = np.stack(vals)
    return float(arr.mean()), float(arr.std())


def spec_augment(feat, freq_mask=8, time_mask=12):
    feat = feat.copy()
    n_mels, n_frames = feat.shape
    f0 = random.randint(0, max(0, n_mels - freq_mask))
    feat[f0 : f0 + freq_mask, :] = 0.0
    t0 = random.randint(0, max(0, n_frames - time_mask))
    feat[:, t0 : t0 + time_mask] = 0.0
    return feat


class SpecDataset(Dataset):
    def __init__(self, rows, processed_dir, label_to_idx, mean, std, augment):
        self.rows = rows
        self.processed_dir = processed_dir
        self.label_to_idx = label_to_idx
        self.mean = mean
        self.std = std
        self.augment = augment

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        row = self.rows[idx]
        feat = np.load(os.path.join(self.processed_dir, row["path"]))
        feat = (feat - self.mean) / self.std
        if self.augment:
            feat = spec_augment(feat)
        x = torch.from_numpy(feat).float().unsqueeze(0)
        y = self.label_to_idx[row["label"]]
        return x, y


class ImpactCNN(nn.Module):
    def __init__(self, n_classes=2):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(1, 16, 3, padding=1), nn.BatchNorm2d(16), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(16, 32, 3, padding=1), nn.BatchNorm2d(32), nn.ReLU(), nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1), nn.BatchNorm2d(64), nn.ReLU(), nn.AdaptiveAvgPool2d(1),
        )
        self.classifier = nn.Linear(64, n_classes)

    def forward(self, x):
        x = self.features(x)
        x = x.flatten(1)
        return self.classifier(x)


def evaluate(model, loader, device):
    model.eval()
    tp = tn = fp = fn = 0
    with torch.no_grad():
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            pred = model(x).argmax(1)
            for p, t in zip(pred.tolist(), y.tolist()):
                tp += p == 1 and t == 1
                tn += p == 0 and t == 0
                fp += p == 1 and t == 0
                fn += p == 0 and t == 1
    total = tp + tn + fp + fn
    accuracy = (tp + tn) / total if total else 0.0
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0
    return {"accuracy": accuracy, "precision": precision, "recall": recall, "f1": f1, "tp": tp, "tn": tn, "fp": fp, "fn": fn}


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--processed-dir", default="dataset/processed")
    p.add_argument("--manifest", default=None, help="default: <processed-dir>/manifest.csv")
    p.add_argument("--out-dir", default="runs")
    p.add_argument("--val-frac", type=float, default=0.2)
    p.add_argument("--epochs", type=int, default=40)
    p.add_argument("--batch-size", type=int, default=16)
    p.add_argument("--lr", type=float, default=1e-3)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--no-augment", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    manifest_path = args.manifest or os.path.join(args.processed_dir, "manifest.csv")

    random.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    rows = load_manifest(manifest_path)
    labels = sorted(set(r["label"] for r in rows))
    label_to_idx = {label: i for i, label in enumerate(labels)}
    print(f"labels: {label_to_idx}")

    train_rows, val_rows = group_split(rows, args.val_frac, args.seed)
    print(f"train windows: {len(train_rows)}  val windows: {len(val_rows)}")

    mean, std = compute_stats(train_rows, args.processed_dir)
    print(f"train feature mean={mean:.3f} std={std:.3f}")

    train_ds = SpecDataset(train_rows, args.processed_dir, label_to_idx, mean, std, augment=not args.no_augment)
    val_ds = SpecDataset(val_rows, args.processed_dir, label_to_idx, mean, std, augment=False)

    class_counts = defaultdict(int)
    for r in train_rows:
        class_counts[r["label"]] += 1
    sample_weights = [1.0 / class_counts[r["label"]] for r in train_rows]
    sampler = WeightedRandomSampler(sample_weights, num_samples=len(train_rows), replacement=True)

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, sampler=sampler)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = ImpactCNN(n_classes=len(labels)).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.CrossEntropyLoss()

    os.makedirs(args.out_dir, exist_ok=True)
    best_f1 = -1.0
    best_path = os.path.join(args.out_dir, "best_model.pt")

    for epoch in range(1, args.epochs + 1):
        model.train()
        total_loss = 0.0
        for x, y in train_loader:
            x, y = x.to(device), y.to(device)
            optimizer.zero_grad()
            loss = criterion(model(x), y)
            loss.backward()
            optimizer.step()
            total_loss += loss.item() * x.size(0)
        train_loss = total_loss / len(train_ds)

        metrics = evaluate(model, val_loader, device)
        print(
            f"epoch {epoch:3d}  train_loss={train_loss:.4f}  "
            f"val_acc={metrics['accuracy']:.3f}  val_f1={metrics['f1']:.3f}  "
            f"(tp={metrics['tp']} fp={metrics['fp']} fn={metrics['fn']} tn={metrics['tn']})"
        )

        if metrics["f1"] >= best_f1:
            best_f1 = metrics["f1"]
            torch.save(model.state_dict(), best_path)

    with open(os.path.join(args.out_dir, "labels.json"), "w") as f:
        json.dump(label_to_idx, f, indent=2)
    with open(os.path.join(args.out_dir, "feature_stats.json"), "w") as f:
        json.dump({"mean": mean, "std": std}, f, indent=2)

    print(f"best val f1={best_f1:.3f}, model saved to {best_path}")


if __name__ == "__main__":
    main()
