#!/usr/bin/env python3
"""Record raw int16 impact-sound clips to WAV. Modes: trigger (auto-save on threshold crossing) or continuous (fixed-length clips)."""

import argparse
import datetime
import os
import queue
import sys
import wave

import numpy as np
import sounddevice as sd

from trigger import TriggerCapture

SAMPLE_WIDTH_BYTES = 2  # int16
FULL_SCALE = 32768.0


class MaxClipsReached(Exception):
    pass


def list_devices():
    print(sd.query_devices())


def resolve_device(name_or_index):
    if name_or_index is None:
        return None
    try:
        return int(name_or_index)
    except ValueError:
        pass
    needle = name_or_index.lower()
    for idx, info in enumerate(sd.query_devices()):
        if info["max_input_channels"] > 0 and needle in info["name"].lower():
            return idx
    raise SystemExit(f"No input device matching '{name_or_index}' found. Run --list-devices.")


def peak_dbfs(block):
    peak = int(np.abs(block).max()) if block.size else 0
    if peak == 0:
        return -float("inf"), peak
    return 20 * np.log10(peak / FULL_SCALE), peak


def write_wav(path, samplerate, channels, frames):
    with wave.open(path, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(SAMPLE_WIDTH_BYTES)
        wf.setframerate(samplerate)
        wf.writeframes(frames.tobytes())


def make_output_path(out_dir, label, index):
    label_dir = os.path.join(out_dir, label)
    os.makedirs(label_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    return os.path.join(label_dir, f"{label}_{stamp}_{index:04d}.wav")


def report_clip(path, samplerate, frames):
    db, peak = peak_dbfs(frames)
    duration = len(frames) / samplerate
    clipped = " CLIPPED" if peak >= FULL_SCALE - 1 else ""
    print(f"saved {path}  ({duration:.2f}s, peak {db:.1f} dBFS{clipped})")


def record_trigger(args, device):
    pre_roll_frames = int(args.pre_roll * args.samplerate)
    post_roll_frames = int(args.post_roll * args.samplerate)
    cooldown_frames = int(args.cooldown * args.samplerate)
    threshold_amp = args.threshold * FULL_SCALE

    audio_q = queue.Queue()

    def callback(indata, frames, time_info, status):
        if status:
            print(f"stream status: {status}", file=sys.stderr)
        audio_q.put(indata.copy())

    clip_index = 0

    def on_capture(frames):
        nonlocal clip_index
        path = make_output_path(args.out_dir, args.label, clip_index)
        write_wav(path, args.samplerate, args.channels, frames)
        report_clip(path, args.samplerate, frames)
        clip_index += 1
        if args.max_clips and clip_index >= args.max_clips:
            raise MaxClipsReached

    trigger = TriggerCapture(pre_roll_frames, post_roll_frames, cooldown_frames, threshold_amp, on_capture)

    print("Listening for impacts (Ctrl+C to stop)...")
    with sd.InputStream(
        samplerate=args.samplerate,
        channels=args.channels,
        dtype="int16",
        device=device,
        blocksize=args.blocksize,
        callback=callback,
    ):
        try:
            while True:
                block = audio_q.get().reshape(-1, args.channels)
                trigger.feed(block)
        except MaxClipsReached:
            print(f"Reached --max-clips {args.max_clips}, stopping.")


def record_continuous(args, device):
    clip_frames_total = int(args.clip_length * args.samplerate)
    audio_q = queue.Queue()

    def callback(indata, frames, time_info, status):
        if status:
            print(f"stream status: {status}", file=sys.stderr)
        audio_q.put(indata.copy())

    clip_index = 0
    buf = []
    buf_frames = 0

    print("Recording continuous clips (Ctrl+C to stop)...")
    with sd.InputStream(
        samplerate=args.samplerate,
        channels=args.channels,
        dtype="int16",
        device=device,
        blocksize=args.blocksize,
        callback=callback,
    ):
        while True:
            block = audio_q.get().reshape(-1, args.channels)
            buf.append(block)
            buf_frames += block.shape[0]
            if buf_frames >= clip_frames_total:
                frames = np.concatenate(buf, axis=0)[:clip_frames_total]
                path = make_output_path(args.out_dir, args.label, clip_index)
                write_wav(path, args.samplerate, args.channels, frames)
                report_clip(path, args.samplerate, frames)
                clip_index += 1
                buf = []
                buf_frames = 0
                if args.max_clips and clip_index >= args.max_clips:
                    print(f"Reached --max-clips {args.max_clips}, stopping.")
                    return


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--list-devices", action="store_true", help="list audio devices and exit")
    p.add_argument("--device", default=None, help="input device index or name substring (e.g. 'voicehat')")
    p.add_argument("--label", help="dataset class name, used as output subfolder (e.g. hit, background)")
    p.add_argument("--mode", choices=["trigger", "continuous"], default="trigger")
    p.add_argument("--out-dir", default="dataset/raw")
    p.add_argument("--samplerate", type=int, default=48000)
    p.add_argument("--channels", type=int, default=1)
    p.add_argument("--blocksize", type=int, default=512, help="frames per audio callback")
    p.add_argument("--threshold", type=float, default=0.15, help="trigger level, fraction of full scale (0-1)")
    p.add_argument("--pre-roll", type=float, default=0.05, help="seconds kept before a trigger (trigger mode)")
    p.add_argument("--post-roll", type=float, default=0.2, help="seconds kept after a trigger (trigger mode)")
    p.add_argument("--cooldown", type=float, default=1.0, help="seconds ignored after a clip is saved (trigger mode)")
    p.add_argument("--clip-length", type=float, default=5.0, help="seconds per clip (continuous mode)")
    p.add_argument("--max-clips", type=int, default=0, help="stop after this many clips (0 = unlimited)")
    return p.parse_args()


def main():
    args = parse_args()

    if args.list_devices:
        list_devices()
        return

    if not args.label:
        raise SystemExit("--label is required (e.g. --label hit)")

    device = resolve_device(args.device)

    try:
        if args.mode == "trigger":
            record_trigger(args, device)
        else:
            record_continuous(args, device)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()