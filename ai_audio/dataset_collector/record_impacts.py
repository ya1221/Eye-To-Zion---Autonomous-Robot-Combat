#!/usr/bin/env python3
"""Impact-sound dataset recorder for the chassis-damage-detection pipeline.

Captures raw int16 PCM straight from the I2S/VoiceHAT capture stream and
writes it to WAV with no digital gain, normalization, or AGC applied at any
point, so the saved waveform is an untouched copy of what the ADC produced.
This preserves the true shockwave shape for later spectrogram/MFCC work and
keeps clipping fully diagnosable (a clipped sample here means the mic/ADC
itself clipped, not the script).

Hardware note: the googlevoicehat-soundcard overlay must be enabled in
/boot/firmware/config.txt (dtoverlay=googlevoicehat-soundcard) and the card
visible via `arecord -l` before running this.

Two capture modes:
  trigger    - listens continuously, auto-saves a clip whenever the signal
               crosses --threshold, keeping --pre-roll seconds of audio from
               before the hit. This is the normal way to collect "hit" data:
               start the script, strike the chassis repeatedly, stop with
               Ctrl+C.
  continuous - saves fixed-length clips back to back for the whole session.
               Use this for the "background" class (ambient noise, motor
               whine, voices) where there is no discrete event to trigger on.

Examples:
  python3 record_impacts.py --list-devices
  python3 record_impacts.py --label hit --mode trigger --threshold 0.2
  python3 record_impacts.py --label background --mode continuous --clip-length 5
"""

import argparse
import collections
import datetime
import os
import queue
import sys
import wave

import numpy as np
import sounddevice as sd

SAMPLE_WIDTH_BYTES = 2  # int16
FULL_SCALE = 32768.0


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

    ring = collections.deque(maxlen=pre_roll_frames)
    capture_buf = []
    frames_captured = 0
    frames_to_skip = 0
    clip_index = 0
    state = "IDLE"  # IDLE -> CAPTURING -> COOLDOWN -> IDLE

    print("Listening for impacts (Ctrl+C to stop)...")
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

            if state == "IDLE":
                peak = int(np.abs(block).max()) if block.size else 0
                if peak >= threshold_amp:
                    capture_buf = list(ring) + [block]
                    frames_captured = len(ring) + block.shape[0]
                    state = "CAPTURING"
                else:
                    for row in block:
                        ring.append(row.reshape(1, -1))

            elif state == "CAPTURING":
                capture_buf.append(block)
                frames_captured += block.shape[0]
                if frames_captured >= pre_roll_frames + post_roll_frames:
                    frames = np.concatenate(capture_buf, axis=0)
                    path = make_output_path(args.out_dir, args.label, clip_index)
                    write_wav(path, args.samplerate, args.channels, frames)
                    report_clip(path, args.samplerate, frames)
                    clip_index += 1
                    capture_buf = []

                    if args.max_clips and clip_index >= args.max_clips:
                        print(f"Reached --max-clips {args.max_clips}, stopping.")
                        return

                    ring.clear()
                    frames_to_skip = cooldown_frames
                    state = "COOLDOWN"

            elif state == "COOLDOWN":
                frames_to_skip -= block.shape[0]
                if frames_to_skip <= 0:
                    tail = block[frames_to_skip:] if frames_to_skip < 0 else block[block.shape[0]:]
                    for row in tail[-pre_roll_frames:]:
                        ring.append(row.reshape(1, -1))
                    state = "IDLE"


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