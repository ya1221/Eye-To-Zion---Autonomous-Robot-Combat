#!/usr/bin/env python3
"""
bench_rpi5_inference.py — Raspberry Pi 5 YOLO inference latency / FPS benchmark
===============================================================================

PURPOSE
-------
Measure the one number this project does not have yet: the *real* per-frame
inference latency and achievable FPS of the deployed detector on the Pi 5.

The repo ships a YOLO26n model exported to NCNN
(`ai_vision/models/best_ncnn_model/`, a DIRECTORY loaded as
`YOLO(path, task='detect')`), trained at imgsz=640 and deployed at imgsz=320,
1 class (`Robot`), confidence_threshold 0.3, camera 640x320 @ 30 fps.
`ai_inference_params.yaml` sets `fps_warn_threshold: 7.0`, but that is a
*config threshold*, not a measurement — nothing in the repo records what the
model actually achieves. This script produces that measurement.

It answers three concrete questions:
  1. What is the per-frame latency / FPS of the NCNN model on this hardware?
  2. How much did the 640 -> 320 deployment resolution actually buy?
     (`--imgsz 320 640`)
  3. How much did the PyTorch -> NCNN export buy?
     (`--model <ncnn_dir> <best.pt>` — both run on byte-identical frames)

It also records CPU temperature and Raspberry Pi throttling state around the
run, because a thermally throttled Pi silently produces numbers that look like
a slow model.

EXACT RUN COMMAND
-----------------
Default (deployed config: NCNN dir, imgsz 320, synthetic frames, no camera):

    python3 measurement_scripts/bench_rpi5_inference.py

Quantify the 640 -> 320 resolution reduction:

    python3 measurement_scripts/bench_rpi5_inference.py --imgsz 320 640

PyTorch vs NCNN on identical input, plus a CSV for the report:

    python3 measurement_scripts/bench_rpi5_inference.py \
        --model ~/models/best_ncnn_model ~/models/best.pt \
        --imgsz 320 640 \
        --source ~/clips/arena.mp4 \
        --frames 300 --warmup 20 \
        --csv results/rpi5_bench.csv

Measure what the ROS node actually pays (adds ByteTrack, as in
`ai_inference_node.py` -> `model.track(..., tracker='bytetrack.yaml')`):

    python3 measurement_scripts/bench_rpi5_inference.py --mode track

PREREQUISITES
-------------
  * Python 3.8+, `numpy`, `ultralytics` (imported lazily — a missing package
    gives a clear message instead of a traceback).
  * `opencv-python` (cv2) ONLY for `--source <video>` / `<image dir>`.
    `--source synthetic` (the default) needs no camera, no video, no cv2.
  * A `.pt` model additionally needs `torch` installed. NCNN needs `ncnn`
    (`pip install ncnn`) — the same dependency the ROS node already requires.
  * Run it ON THE PI. Numbers from a laptop say nothing about the Pi 5.
  * Close the ROS stack first (`ai_inference` competing for the same 4 cores
    will skew every number here).
  * Optional but recommended: `vcgencmd` (present on Raspberry Pi OS) so
    throttling can be read. Degrades gracefully everywhere else.

NOTES ON METHODOLOGY (read before quoting the output)
-----------------------------------------------------
  * Frames are decoded/generated and resized to 640x320 BEFORE timing starts,
    so video decode cost is NOT counted. Only the model call is timed.
  * `--warmup` frames are run and discarded first. The first NCNN inferences
    are dramatically slower (allocator + layer setup); including them would
    understate steady-state FPS badly.
  * `--source synthetic` produces uniform random noise, which yields ~0
    detections. Postprocess/NMS is therefore at its cheapest — treat synthetic
    numbers as a best case and use a real clip for a representative figure.
    The script prints the detection count so you can see this.
  * This measures the model in isolation. The ROS node additionally pays
    shared-memory copy, JSON, tracking, threat scoring and annotation
    publishing, so real node FPS will be LOWER than the FPS reported here.
    This is an upper bound.

EXPECTED OUTPUT FORMAT
----------------------
  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  !! SAMPLE BELOW IS AN ILLUSTRATIVE LAYOUT MOCK-UP.                      !!
  !! EVERY DIGIT IN IT IS INVENTED TO SHOW COLUMN ALIGNMENT.              !!
  !! IT IS NOT A MEASUREMENT AND MUST NEVER BE QUOTED AS ONE.             !!
  !! Run the script on the Pi to obtain real values.                      !!
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    ================================================================
     RPi5 YOLO inference benchmark
    ================================================================
    host          : <hostname>  (Linux <kernel> aarch64)
    python        : 3.x.y
    ultralytics   : <version>
    source        : synthetic  (100 frames, 640x320 BGR, seed=0)
    warmup        : 10 frames (discarded)
    mode          : predict    conf=0.3
    cpu temp      : <t> C  (before)
    throttling    : <hex> (<decoded flags>)  (before)

    ----------------------------------------------------------------
     [1/2]  best_ncnn_model  (ncnn)  imgsz=320
    ----------------------------------------------------------------
      wall-clock per frame (ms)
          min <..>   mean <..>   median <..>   p95 <..>   max <..>
      results[0].speed breakdown (mean ms)
          preprocess <..>   inference <..>   postprocess <..>   sum <..>
      throughput
          mean   FPS = <..>   (1000 / mean latency)
          median FPS = <..>   (1000 / median latency)
      detections  : <n> boxes over 100 frames at conf>=0.3
      thermals    : <t> C -> <t> C  (max <t> C)
      verdict
          vs fps_warn_threshold 7.0 : <PASS|FAIL>  (<+/-x.x> fps margin)
          vs camera 30.0 fps        : <KEEPS UP|BOTTLENECK>  (budget 33.3 ms)

    ================================================================
     SUMMARY  (identical input frames for every row)
    ================================================================
    model                      fmt    imgsz   mean ms  med ms   p95 ms  mean FPS   med FPS
    -------------------------------------------------------------------------------------
    best_ncnn_model            ncnn     320     <..>    <..>     <..>     <..>      <..>
    best_ncnn_model            ncnn     640     <..>    <..>     <..>     <..>      <..>

`--csv PATH` appends one row per model x imgsz combination (header written
only when the file is created), so repeated runs accumulate in one file.
Columns: timestamp_utc, host, model, model_format, imgsz, mode, source,
frames, warmup, conf, lat_min_ms, lat_mean_ms, lat_median_ms, lat_p95_ms,
lat_max_ms, fps_mean, fps_median, pre_ms, inf_ms, post_ms, detections,
temp_start_c, temp_end_c, temp_max_c, throttled_hex, throttled_flags,
ultralytics_version, status.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import platform
import re
import shutil
import socket
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

# ─────────────────────────── project constants ────────────────────────────
# All values below are read from the repo, not guessed.
REPO_ROOT = Path(__file__).resolve().parent.parent

# ai_inference_node.py resolves this from the installed package share dir;
# try the source tree first, then the usual install locations. The source
# layout has moved before, so resolve_default_model() also globs as a
# last resort rather than hard-failing on a path that shifted.
MODEL_SUBPATH = Path("ros2_ws/src/ai_vision/models/best_ncnn_model")
MODEL_CANDIDATES = (
    REPO_ROOT / "AutonomousWarfare" / MODEL_SUBPATH,
    REPO_ROOT / "AutonomousWarfare" / "AutonomousWarfare" / MODEL_SUBPATH,
    Path.home() / "ros2_ws" / "install" / "ai_vision" / "share" / "ai_vision"
    / "models" / "best_ncnn_model",
    Path("/opt/ros_ws/install/ai_vision/share/ai_vision/models/best_ncnn_model"),
)

FPS_WARN_THRESHOLD = 7.0      # ai_inference_params.yaml -> fps_warn_threshold
CAMERA_FPS = 30.0             # camera: 640x320 @ 30 fps
DEFAULT_CONF = 0.3            # ai_inference_params.yaml -> confidence_threshold
DEFAULT_IMGSZ = 320           # ai_inference_params.yaml -> inference_imgsz
DEFAULT_TRACKER = "bytetrack.yaml"   # ai_inference_params.yaml -> tracker_config
CAMERA_W, CAMERA_H = 640, 320

THERMAL_ZONE = "/sys/class/thermal/thermal_zone0/temp"

# `vcgencmd get_throttled` bit meanings (Raspberry Pi documentation).
THROTTLE_BITS = {
    0:  "under-voltage NOW",
    1:  "arm frequency capped NOW",
    2:  "throttled NOW",
    3:  "soft temperature limit active NOW",
    16: "under-voltage has occurred",
    17: "arm frequency capping has occurred",
    18: "throttling has occurred",
    19: "soft temperature limit has occurred",
}

IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"}

# Bare weight names Ultralytics is allowed to download if the path is absent.
DOWNLOADABLE_RE = re.compile(r"^yolo[a-z0-9._-]*\.pt$", re.IGNORECASE)

CSV_COLUMNS = [
    "timestamp_utc", "host", "model", "model_format", "imgsz", "mode",
    "source", "frames", "warmup", "conf",
    "lat_min_ms", "lat_mean_ms", "lat_median_ms", "lat_p95_ms", "lat_max_ms",
    "fps_mean", "fps_median", "pre_ms", "inf_ms", "post_ms", "detections",
    "temp_start_c", "temp_end_c", "temp_max_c",
    "throttled_hex", "throttled_flags", "ultralytics_version", "status",
]


# ───────────────────────────── small helpers ──────────────────────────────
def die(msg: str, code: int = 2) -> "None":
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(code)


def import_or_die(module: str, hint: str):
    try:
        return __import__(module)
    except ImportError as exc:
        die(f"could not import '{module}' ({exc}).\n       {hint}")


def percentile(sorted_vals, q: float) -> float:
    """Linear-interpolation percentile. `sorted_vals` must already be sorted."""
    if not sorted_vals:
        return float("nan")
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    k = (len(sorted_vals) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    if lo == hi:
        return sorted_vals[int(k)]
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


def read_cpu_temp_c():
    """Return CPU temperature in Celsius, or None when unavailable."""
    try:
        raw = Path(THERMAL_ZONE).read_text().strip()
    except OSError:
        return None
    try:
        val = float(raw)
    except ValueError:
        return None
    # sysfs reports millidegrees on the Pi; be tolerant of plain degrees.
    return val / 1000.0 if val > 200.0 else val


def read_throttled():
    """Return (hex_string, [decoded flags]) from vcgencmd, or (None, [])."""
    exe = shutil.which("vcgencmd")
    if not exe:
        return None, []
    try:
        out = subprocess.run([exe, "get_throttled"], capture_output=True,
                             text=True, timeout=5)
    except (OSError, subprocess.SubprocessError):
        return None, []
    match = re.search(r"throttled=(0x[0-9a-fA-F]+)", out.stdout or "")
    if not match:
        return None, []
    hex_str = match.group(1)
    value = int(hex_str, 16)
    flags = [text for bit, text in sorted(THROTTLE_BITS.items())
             if value & (1 << bit)]
    return hex_str, flags


def fmt_temp(value) -> str:
    return "n/a" if value is None else f"{value:.1f} C"


def model_format(path: str) -> str:
    """Best-effort label for how Ultralytics will load this path."""
    p = Path(path)
    if p.is_dir():
        if (p / "model.ncnn.param").exists():
            return "ncnn"
        if list(p.glob("*.mlmodel")) or list(p.glob("*.mlpackage")):
            return "coreml"
        return "dir"
    return (p.suffix.lstrip(".") or "unknown").lower()


def resolve_default_model() -> str:
    for candidate in MODEL_CANDIDATES:
        if candidate.is_dir():
            return str(candidate)
    # Last resort: the ai_vision package has been relocated within this repo
    # before, so search for it rather than fail on a stale hard-coded path.
    try:
        found = next(REPO_ROOT.glob("**/ai_vision/models/best_ncnn_model"), None)
    except OSError:
        found = None
    return str(found) if found else str(MODEL_CANDIDATES[0])


def validate_model_path(path: str) -> None:
    """Refuse to let Ultralytics silently download a same-named model.

    Mirrors the guard in ai_inference_node._resolve_model_path().
    """
    p = Path(path)
    if p.exists():
        return
    if DOWNLOADABLE_RE.match(p.name) and p.parent in (Path("."), Path("")):
        print(f"  note: '{path}' is not on disk — Ultralytics will download it.")
        return
    die(f"model path does not exist: {path}\n"
        f"       The deployed NCNN model is a DIRECTORY. Expected one of:\n"
        + "\n".join(f"         {c}" for c in MODEL_CANDIDATES))


def ncnn_export_imgsz(path: str):
    """Read the exported imgsz out of an NCNN metadata.yaml, if present."""
    meta = Path(path) / "metadata.yaml"
    if not meta.exists():
        return None
    try:
        text = meta.read_text()
    except OSError:
        return None
    # metadata.yaml holds e.g.  imgsz:\n  - 320\n  - 320
    match = re.search(r"^imgsz:\s*\n\s*-\s*(\d+)", text, re.MULTILINE)
    if match:
        return int(match.group(1))
    match = re.search(r"^imgsz:\s*\[?\s*(\d+)", text, re.MULTILINE)
    return int(match.group(1)) if match else None


# ───────────────────────────── frame sources ──────────────────────────────
def make_synthetic_frames(np, count: int, width: int, height: int, seed: int):
    """Random BGR noise at the camera's geometry. No camera required."""
    rng = np.random.default_rng(seed)
    return [rng.integers(0, 256, (height, width, 3), dtype=np.uint8)
            for _ in range(count)]


def fit_frame(cv2, frame, size):
    if size is None:
        return frame
    width, height = size
    if frame.shape[1] == width and frame.shape[0] == height:
        return frame
    return cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)


def load_video_frames(cv2, path: str, count: int, size):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        die(f"could not open video: {path}")
    frames = []
    while len(frames) < count:
        ok, frame = cap.read()
        if not ok:
            if not frames:
                cap.release()
                die(f"decoded 0 frames from {path}")
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)   # loop a short clip
            ok, frame = cap.read()
            if not ok:
                break
        frames.append(fit_frame(cv2, frame, size))
    cap.release()
    return frames


def load_dir_frames(cv2, path: str, count: int, size):
    files = sorted(p for p in Path(path).iterdir()
                   if p.suffix.lower() in IMAGE_SUFFIXES)
    if not files:
        die(f"no images found in {path} (looked for {sorted(IMAGE_SUFFIXES)})")
    frames = []
    idx = 0
    while len(frames) < count:
        img = cv2.imread(str(files[idx % len(files)]))
        idx += 1
        if img is not None:
            frames.append(fit_frame(cv2, img, size))
        elif idx > 2 * len(files):
            break
    if not frames:
        die(f"could not decode any image in {path}")
    return frames


def build_frames(args, np):
    """Materialise every frame BEFORE timing so decode is never measured."""
    total = args.frames + args.warmup
    size = None if args.frame_size == "native" else parse_size(args.frame_size)

    if args.source == "synthetic":
        width, height = size if size else (CAMERA_W, CAMERA_H)
        frames = make_synthetic_frames(np, total, width, height, args.seed)
        desc = (f"synthetic  ({args.frames} frames, {width}x{height} BGR, "
                f"seed={args.seed})")
        return frames, desc

    cv2 = import_or_die(
        "cv2", "Install it with:  pip install opencv-python\n"
               "       (or use --source synthetic, which needs no cv2)")
    src = Path(args.source).expanduser()
    if src.is_dir():
        frames = load_dir_frames(cv2, str(src), total, size)
        kind = "image dir"
    elif src.is_file():
        frames = load_video_frames(cv2, str(src), total, size)
        kind = "video"
    else:
        die(f"--source '{args.source}' is not 'synthetic', a file or a directory")
    height, width = frames[0].shape[:2]
    desc = f"{kind} {src}  ({len(frames)} frames loaded, {width}x{height} BGR)"
    return frames, desc


def parse_size(text: str):
    match = re.fullmatch(r"\s*(\d+)\s*[xX]\s*(\d+)\s*", text)
    if not match:
        die(f"--frame-size must look like 640x320 or be 'native', got '{text}'")
    return int(match.group(1)), int(match.group(2))


# ─────────────────────────── benchmark one combo ──────────────────────────
def benchmark(YOLO, model_path, imgsz, frames, args, temp_sampler):
    """Run one (model, imgsz) combination. Returns a result dict."""
    validate_model_path(model_path)

    fmt = model_format(model_path)
    if fmt == "ncnn":
        exported = ncnn_export_imgsz(model_path)
        if exported and exported != imgsz:
            print(f"  WARNING: this NCNN model was exported at imgsz={exported} "
                  f"but is being run at imgsz={imgsz}.")
            print(f"           Latency is still valid; accuracy at a "
                  f"non-exported size is not guaranteed.")

    # A fresh model object per combo — this also resets tracker state.
    model = YOLO(model_path, task="detect")

    call_kwargs = dict(imgsz=imgsz, conf=args.conf, verbose=False)
    if args.device:
        call_kwargs["device"] = args.device
    if args.mode == "track":
        runner = model.track
        call_kwargs.update(persist=True, tracker=args.tracker)
    else:
        runner = model.predict

    # ── warmup (discarded): first NCNN calls are far slower than steady state
    for i in range(args.warmup):
        runner(frames[i % len(frames)], **call_kwargs)

    latencies, pre, inf, post = [], [], [], []
    detections = 0
    temps = [temp_sampler()]

    timed_frames = frames[args.warmup:args.warmup + args.frames] or frames
    for i, frame in enumerate(timed_frames):
        start = time.perf_counter()
        results = runner(frame, **call_kwargs)
        latencies.append((time.perf_counter() - start) * 1000.0)

        # Everything below is OUTSIDE the timed window on purpose.
        if results:
            speed = getattr(results[0], "speed", None)
            if isinstance(speed, dict):
                for key, bucket in (("preprocess", pre), ("inference", inf),
                                    ("postprocess", post)):
                    value = speed.get(key)
                    if isinstance(value, (int, float)):
                        bucket.append(float(value))
            boxes = getattr(results[0], "boxes", None)
            if boxes is not None:
                try:
                    detections += len(boxes)
                except TypeError:
                    pass
        if args.temp_interval and i % args.temp_interval == 0:
            temps.append(temp_sampler())

    temps.append(temp_sampler())
    temps = [t for t in temps if t is not None]

    ordered = sorted(latencies)
    mean_ms = statistics.fmean(latencies)
    median_ms = statistics.median(latencies)

    def mean_or_none(values):
        return statistics.fmean(values) if values else None

    return {
        "model": model_path,
        "model_name": Path(model_path).name,
        "format": fmt,
        "imgsz": imgsz,
        "n": len(latencies),
        "min": ordered[0],
        "mean": mean_ms,
        "median": median_ms,
        "p95": percentile(ordered, 0.95),
        "max": ordered[-1],
        "fps_mean": 1000.0 / mean_ms if mean_ms > 0 else float("nan"),
        "fps_median": 1000.0 / median_ms if median_ms > 0 else float("nan"),
        "pre": mean_or_none(pre),
        "inf": mean_or_none(inf),
        "post": mean_or_none(post),
        "detections": detections,
        "temp_start": temps[0] if temps else None,
        "temp_end": temps[-1] if temps else None,
        "temp_max": max(temps) if temps else None,
        "status": "ok",
    }


# ───────────────────────────────── report ─────────────────────────────────
def print_combo(index, total, res):
    name = res["model_name"]
    print("\n" + "-" * 78)
    print(f" [{index}/{total}]  {name}  ({res['format']})  imgsz={res['imgsz']}")
    print("-" * 78)

    if res["status"] != "ok":
        print(f"  FAILED: {res['status']}")
        return

    print(f"  wall-clock per frame (ms)   [{res['n']} timed frames]")
    print(f"      min {res['min']:8.2f}   mean {res['mean']:8.2f}   "
          f"median {res['median']:8.2f}   p95 {res['p95']:8.2f}   "
          f"max {res['max']:8.2f}")

    if res["inf"] is not None:
        parts = [res["pre"] or 0.0, res["inf"] or 0.0, res["post"] or 0.0]
        print("  results[0].speed breakdown (mean ms)")
        print(f"      preprocess {parts[0]:6.2f}   inference {parts[1]:7.2f}   "
              f"postprocess {parts[2]:6.2f}   sum {sum(parts):7.2f}")
    else:
        print("  results[0].speed breakdown : not exposed by this ultralytics "
              "version")

    print("  throughput")
    print(f"      mean   FPS = {res['fps_mean']:6.2f}   (1000 / mean latency)")
    print(f"      median FPS = {res['fps_median']:6.2f}   (1000 / median latency)")
    print(f"  detections  : {res['detections']} boxes over {res['n']} frames "
          f"at conf>={DEFAULT_CONF if res.get('conf') is None else res['conf']}")
    print(f"  thermals    : {fmt_temp(res['temp_start'])} -> "
          f"{fmt_temp(res['temp_end'])}  (max {fmt_temp(res['temp_max'])})")

    fps = res["fps_median"]
    print("  verdict")
    if fps >= FPS_WARN_THRESHOLD:
        print(f"      vs fps_warn_threshold {FPS_WARN_THRESHOLD:.1f} : PASS   "
              f"(+{fps - FPS_WARN_THRESHOLD:.2f} fps margin)")
    else:
        print(f"      vs fps_warn_threshold {FPS_WARN_THRESHOLD:.1f} : FAIL   "
              f"({fps - FPS_WARN_THRESHOLD:.2f} fps) — ai_inference would log "
              f"'LOW AVG FPS'")
    budget_ms = 1000.0 / CAMERA_FPS
    if res["median"] <= budget_ms:
        print(f"      vs camera {CAMERA_FPS:.1f} fps        : KEEPS UP   "
              f"(median {res['median']:.2f} ms <= {budget_ms:.1f} ms budget)")
    else:
        drop = 1.0 - (fps / CAMERA_FPS)
        print(f"      vs camera {CAMERA_FPS:.1f} fps        : BOTTLENECK "
              f"(median {res['median']:.2f} ms > {budget_ms:.1f} ms budget; "
              f"~{drop * 100:.0f}% of frames cannot be served)")


def print_summary(results):
    print("\n" + "=" * 86)
    print(" SUMMARY  (identical input frames for every row)")
    print("=" * 86)
    header = (f"{'model':<26}{'fmt':<7}{'imgsz':>6}{'mean ms':>10}"
              f"{'med ms':>9}{'p95 ms':>9}{'mean FPS':>10}{'med FPS':>9}")
    print(header)
    print("-" * 86)
    for res in results:
        if res["status"] != "ok":
            print(f"{res['model_name'][:25]:<26}{res['format']:<7}"
                  f"{res['imgsz']:>6}{'  FAILED — ' + res['status'][:40]}")
            continue
        print(f"{res['model_name'][:25]:<26}{res['format']:<7}{res['imgsz']:>6}"
              f"{res['mean']:>10.2f}{res['median']:>9.2f}{res['p95']:>9.2f}"
              f"{res['fps_mean']:>10.2f}{res['fps_median']:>9.2f}")

    ok = [r for r in results if r["status"] == "ok"]
    if len(ok) > 1:
        base = ok[0]
        print("\n relative to the first row "
              f"({base['model_name']} @ {base['imgsz']}):")
        for res in ok[1:]:
            speedup = base["median"] / res["median"] if res["median"] else float("nan")
            print(f"   {res['model_name'][:25]:<26}imgsz={res['imgsz']:<5} "
                  f"{speedup:5.2f}x  "
                  f"({'faster' if speedup >= 1 else 'slower'})")


def write_csv(path, rows, meta):
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    new_file = not target.exists()
    with target.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        if new_file:
            writer.writeheader()
        for res in rows:
            writer.writerow({
                "timestamp_utc": meta["timestamp"],
                "host": meta["host"],
                "model": res["model"],
                "model_format": res["format"],
                "imgsz": res["imgsz"],
                "mode": meta["mode"],
                "source": meta["source"],
                "frames": res.get("n", 0),
                "warmup": meta["warmup"],
                "conf": meta["conf"],
                "lat_min_ms": f"{res['min']:.4f}" if res["status"] == "ok" else "",
                "lat_mean_ms": f"{res['mean']:.4f}" if res["status"] == "ok" else "",
                "lat_median_ms": f"{res['median']:.4f}" if res["status"] == "ok" else "",
                "lat_p95_ms": f"{res['p95']:.4f}" if res["status"] == "ok" else "",
                "lat_max_ms": f"{res['max']:.4f}" if res["status"] == "ok" else "",
                "fps_mean": f"{res['fps_mean']:.4f}" if res["status"] == "ok" else "",
                "fps_median": f"{res['fps_median']:.4f}" if res["status"] == "ok" else "",
                "pre_ms": "" if res.get("pre") is None else f"{res['pre']:.4f}",
                "inf_ms": "" if res.get("inf") is None else f"{res['inf']:.4f}",
                "post_ms": "" if res.get("post") is None else f"{res['post']:.4f}",
                "detections": res.get("detections", ""),
                "temp_start_c": "" if res.get("temp_start") is None else f"{res['temp_start']:.1f}",
                "temp_end_c": "" if res.get("temp_end") is None else f"{res['temp_end']:.1f}",
                "temp_max_c": "" if res.get("temp_max") is None else f"{res['temp_max']:.1f}",
                "throttled_hex": meta["throttled_hex"] or "",
                "throttled_flags": ";".join(meta["throttled_flags"]),
                "ultralytics_version": meta["ultralytics_version"],
                "status": res["status"],
            })
    print(f"\nCSV {'created' if new_file else 'appended'}: {target}")


# ─────────────────────────────────  main  ─────────────────────────────────
def build_parser():
    parser = argparse.ArgumentParser(
        description="Measure real per-frame latency and FPS of the deployed "
                    "YOLO detector on a Raspberry Pi 5.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="Run it on the Pi, with the ROS stack stopped.")
    parser.add_argument(
        "--model", nargs="+", default=[resolve_default_model()],
        help="One or more models. A directory = an exported model (the "
             "deployed NCNN model is a directory); a .pt = PyTorch. Pass both "
             "to compare PyTorch vs NCNN on identical frames.")
    parser.add_argument(
        "--imgsz", nargs="+", type=int, default=[DEFAULT_IMGSZ],
        help="One or more inference sizes; every model is run at every size. "
             "Use '--imgsz 320 640' to quantify the 640->320 reduction.")
    parser.add_argument(
        "--source", default="synthetic",
        help="'synthetic' (random 640x320 BGR frames, no camera needed), a "
             "video file, or a directory of images.")
    parser.add_argument("--frames", type=int, default=100,
                        help="Timed frames per combination.")
    parser.add_argument("--warmup", type=int, default=10,
                        help="Frames run and DISCARDED before timing starts.")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF,
                        help="Confidence threshold (deployment uses 0.3).")
    parser.add_argument("--mode", choices=("predict", "track"), default="predict",
                        help="'predict' = pure model cost. 'track' = adds the "
                             "tracker, matching ai_inference_node.model.track().")
    parser.add_argument("--tracker", default=DEFAULT_TRACKER,
                        help="Tracker config used when --mode track.")
    parser.add_argument("--frame-size", default=f"{CAMERA_W}x{CAMERA_H}",
                        help="Resize input frames to this WxH before timing "
                             "(the camera is 640x320), or 'native' to keep "
                             "the source resolution.")
    parser.add_argument("--device", default=None,
                        help="Passed to ultralytics (e.g. 'cpu'). Leave unset "
                             "for auto. NCNN is CPU-only; there is no NVIDIA "
                             "GPU on a Pi 5, so TensorRT does not apply.")
    parser.add_argument("--threads", type=int, default=None,
                        help="Pin CPU threads (sets OMP_NUM_THREADS before "
                             "torch is imported). Useful for fair PyTorch vs "
                             "NCNN comparison. Pi 5 has 4 cores.")
    parser.add_argument("--temp-interval", type=int, default=10,
                        help="Sample CPU temperature every N frames (outside "
                             "the timed window). 0 disables.")
    parser.add_argument("--seed", type=int, default=0,
                        help="RNG seed for synthetic frames (reproducibility).")
    parser.add_argument("--csv", default=None, metavar="PATH",
                        help="Append machine-readable results to this CSV.")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    if args.frames < 1:
        die("--frames must be >= 1")
    if args.warmup < 0:
        die("--warmup must be >= 0")

    # Must happen BEFORE torch/ultralytics is imported to take effect.
    if args.threads:
        for var in ("OMP_NUM_THREADS", "MKL_NUM_THREADS",
                    "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
            os.environ[var] = str(args.threads)

    np = import_or_die("numpy", "Install it with:  pip install numpy")

    try:
        import ultralytics
        from ultralytics import YOLO
    except ImportError as exc:
        die("could not import 'ultralytics' "
            f"({exc}).\n"
            "       Install it with:  pip install ultralytics\n"
            "       The NCNN model additionally needs:  pip install ncnn\n"
            "       A .pt model additionally needs:     pip install torch")
        return 2  # unreachable; keeps linters happy

    if args.threads:
        try:
            import torch
            torch.set_num_threads(args.threads)
        except Exception:
            pass

    frames, source_desc = build_frames(args, np)
    if len(frames) <= args.warmup:
        print(f"  note: only {len(frames)} frames available; they will be "
              f"reused for the timed pass.")

    temp_sampler = read_cpu_temp_c
    temp_before = temp_sampler()
    hex_before, flags_before = read_throttled()

    print("=" * 78)
    print(" RPi5 YOLO inference benchmark")
    print("=" * 78)
    print(f"host          : {socket.gethostname()}  "
          f"({platform.system()} {platform.release()} {platform.machine()})")
    print(f"python        : {platform.python_version()}")
    print(f"ultralytics   : {getattr(ultralytics, '__version__', 'unknown')}")
    print(f"source        : {source_desc}")
    print(f"warmup        : {args.warmup} frames (discarded)")
    print(f"mode          : {args.mode}"
          + (f"  tracker={args.tracker}" if args.mode == "track" else "")
          + f"    conf={args.conf}")
    if args.threads:
        print(f"threads       : {args.threads}")
    print(f"cpu temp      : {fmt_temp(temp_before)}  (before)")
    if hex_before is None:
        print("throttling    : vcgencmd not available — not a Pi, or not "
              "installed (thermal throttling cannot be detected)")
    else:
        print(f"throttling    : {hex_before} "
              f"({', '.join(flags_before) if flags_before else 'none'})  (before)")
        if flags_before:
            print("  WARNING: the Pi is ALREADY throttled/under-volted before "
                  "the run started.")
            print("           Numbers below understate this hardware. Fix "
                  "power/cooling and re-run.")

    combos = [(m, s) for m in args.model for s in args.imgsz]
    results = []
    for index, (model_path, imgsz) in enumerate(combos, start=1):
        print(f"\n>>> running [{index}/{len(combos)}] {model_path} @ imgsz={imgsz} ...")
        try:
            res = benchmark(YOLO, model_path, imgsz, frames, args, temp_sampler)
            res["conf"] = args.conf
        except SystemExit:
            raise
        except Exception as exc:                              # noqa: BLE001
            res = {
                "model": model_path, "model_name": Path(model_path).name,
                "format": model_format(model_path), "imgsz": imgsz,
                "status": f"{type(exc).__name__}: {exc}",
            }
        results.append(res)
        print_combo(index, len(combos), res)

    temp_after = temp_sampler()
    hex_after, flags_after = read_throttled()

    print_summary(results)

    print("\n" + "=" * 78)
    print(" THERMAL / THROTTLING STATE AFTER RUN")
    print("=" * 78)
    print(f"cpu temp      : {fmt_temp(temp_before)} -> {fmt_temp(temp_after)}")
    if hex_after is None:
        print("throttling    : vcgencmd not available (cannot verify)")
    else:
        print(f"throttling    : {hex_after} "
              f"({', '.join(flags_after) if flags_after else 'none'})")
        new_flags = set(flags_after) - set(flags_before)
        if new_flags:
            print("  WARNING: throttling/under-voltage occurred DURING this run: "
                  + ", ".join(sorted(new_flags)))
            print("           These FPS numbers are depressed. Improve cooling "
                  "or the power supply and re-run before quoting them.")

    print("\nreminder: this is the model in isolation. ai_inference_node also "
          "pays shared-memory\n          copy, tracking, threat scoring and "
          "annotation publishing, so node FPS\n          will be LOWER than "
          "the figures above.")

    if args.csv:
        write_csv(args.csv, results, {
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "host": socket.gethostname(),
            "mode": args.mode,
            "source": args.source,
            "warmup": args.warmup,
            "conf": args.conf,
            "throttled_hex": hex_after,
            "throttled_flags": flags_after,
            "ultralytics_version": getattr(ultralytics, "__version__", "unknown"),
        })

    return 0 if all(r["status"] == "ok" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
