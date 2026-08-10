#!/usr/bin/env python3
"""
compare_yolo_models.py — apples-to-apples mAP comparison across models / imgsz
==============================================================================

PURPOSE
-------
Run `model.val()` for two or more (model, imgsz) combinations against the SAME
validation set, so the resulting mAP numbers are actually comparable.

The repo currently has exactly one set of accuracy numbers, from one training
run of YOLO26n at imgsz=640. Everything deployed is a different artefact: the
model is exported to NCNN and runs at imgsz=320. Nobody has measured what that
export + resolution reduction costs in accuracy. This script measures it.

Typical uses:
  * deployed NCNN @ 320  vs  the original .pt @ 640   (export + resize cost)
  * the same .pt at 320 vs 640                        (pure resolution cost)
  * a `yolov8n.pt` baseline, if someone wants the YOLOv8 comparison that has
    never actually existed in this repo (there is NO YOLOv8 run anywhere —
    running one here is the only way to get that number honestly).

Note: TensorRT is NOT applicable to this project. There is no NVIDIA GPU on a
Raspberry Pi 5; the deployment format is NCNN.

DATASET — READ THIS FIRST
-------------------------
The dataset is currently sitting in the trash:

    ~/.local/share/Trash/files/Eye To Zion - Detection Robots.v4i.yolo26/data.yaml

RESTORE IT OUT OF THE TRASH BEFORE RUNNING. Anything in Trash can be deleted
by the desktop environment at any time, and its `data.yaml` uses RELATIVE
paths (`train: ../train/images`), so it only resolves correctly while the
directory layout around it is intact. Example:

    mv ~/.local/share/Trash/files/"Eye To Zion - Detection Robots.v4i.yolo26" \
       ~/datasets/eye-to-zion-v4
    # then point --data at ~/datasets/eye-to-zion-v4/data.yaml

Dataset facts (Roboflow "Eye To Zion - Detection Robots" v4):
    798 train / 100 valid / 100 test images, nc=1, names: ['Robot']

EXACT RUN COMMAND
-----------------
Deployed NCNN model vs the original PyTorch weights, at both sizes:

    python3 measurement_scripts/compare_yolo_models.py \
        --models AutonomousWarfare/ros2_ws/src/ai_vision/models/best_ncnn_model \
                 ~/models/best.pt \
        --imgsz 320 640 \
        --data ~/datasets/eye-to-zion-v4/data.yaml \
        --csv results/model_comparison.csv

Add the never-existed YOLOv8 baseline (downloads COCO weights — it will score
near zero on this dataset unless you fine-tune it first; see --help):

    python3 measurement_scripts/compare_yolo_models.py \
        --models ~/models/best.pt yolov8n.pt --imgsz 640 \
        --data ~/datasets/eye-to-zion-v4/data.yaml

PREREQUISITES
-------------
  * `ultralytics` (imported lazily — a missing package gives a clear message).
  * `torch` for .pt models; `ncnn` (`pip install ncnn`) for the NCNN directory.
  * The validation images/labels present on disk at the paths `data.yaml`
    resolves to.
  * Enough time: `val()` runs the whole split once per combination.
    Do NOT run this on the Pi — validate on the training machine. Only the
    *latency* benchmark (bench_rpi5_inference.py) belongs on the Pi.

CAVEATS THAT AFFECT INTERPRETATION
----------------------------------
  * Exported models (NCNN) validate at batch=1; the script forces that
    automatically and says so.
  * The NCNN model in this repo was exported with a FIXED imgsz of 320
    (`best_ncnn_model/metadata.yaml` -> imgsz: [320, 320]). Validating it at
    640 may fail outright or silently letterbox back down — the script warns
    when the requested size differs from the exported size.
  * `--rect` is OFF by default (square letterbox) so every combination sees
    identically shaped tensors, and so fixed-shape exports do not break.
    Ultralytics' own default is rect=True, so numbers here can differ very
    slightly from a plain `yolo val` invocation. Comparability across rows is
    the priority.
  * A COCO-pretrained baseline (`yolov8n.pt`) has different class names and
    will score ~0 on a 1-class `Robot` dataset. That is a real result, not a
    bug — but it is not a meaningful "YOLOv8 vs YOLO26 accuracy" comparison.
    For that you must fine-tune yolov8n on this dataset first.

EXPECTED OUTPUT FORMAT
----------------------
  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  !! SAMPLE BELOW IS AN ILLUSTRATIVE LAYOUT MOCK-UP.                      !!
  !! EVERY DIGIT IN THE TABLE IS INVENTED TO SHOW COLUMN ALIGNMENT.       !!
  !! IT IS NOT A MEASUREMENT AND MUST NEVER BE QUOTED AS ONE.             !!
  !! (The REFERENCE block the script prints is the one place real,        !!
  !!  recorded numbers appear — those come from the actual training run.) !!
  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    ===============================================================
     REFERENCE — original YOLO26n training run (the only run that exists)
    ===============================================================
     final epoch 100 : mAP@50 = 0.9943   mAP@50-95 = 0.9383
                       precision = 0.98977   recall = 0.98980
     best            : mAP@50 = 0.9950 @ epoch 59
     training        : 100 epochs, imgsz 640, batch 16, seed 0 in 1232 s
     -> If you validate the ORIGINAL .pt at imgsz 640 on the val split and
        get roughly the numbers above, you have reproduced the run. A large
        gap means the wrong weights, the wrong split, or the wrong dataset.

    model                     fmt   imgsz    mAP50   dmAP50  mAP50-95  dmAP50-95    prec   recall    secs
    ------------------------------------------------------------------------------------------------------
    best.pt                   pt      640   <....>     base    <....>       base   <...>    <...>   <...>
    best_ncnn_model           ncnn    320   <....>   <+/-..>   <....>     <+/-..>   <...>    <...>   <...>

    baseline row = first combination listed (model order x imgsz order).
    d columns are absolute differences vs that baseline.

`--csv PATH` appends one row per combination (header written only when the
file is created). Columns: timestamp_utc, model, model_format, imgsz, data,
split, batch, conf, iou, rect, map50, map50_95, precision, recall,
delta_map50, delta_map50_95, seconds, ultralytics_version, status.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

# ─────────────────────────── project constants ────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent

MODEL_SUBPATH = Path("ros2_ws/src/ai_vision/models/best_ncnn_model")
_NCNN_CANDIDATES = (
    REPO_ROOT / "AutonomousWarfare" / MODEL_SUBPATH,
    REPO_ROOT / "AutonomousWarfare" / "AutonomousWarfare" / MODEL_SUBPATH,
)
DEFAULT_NCNN_MODEL = next(
    (c for c in _NCNN_CANDIDATES if c.is_dir()), _NCNN_CANDIDATES[0])

# The dataset is in the trash; this default exists so --help can point at it,
# but the script tells you to restore it before relying on it.
TRASH_DATA_YAML = (
    Path.home() / ".local" / "share" / "Trash" / "files"
    / "Eye To Zion - Detection Robots.v4i.yolo26" / "data.yaml"
)

DEFAULT_CONF = 0.3      # ai_inference_params.yaml -> confidence_threshold
DEFAULT_IOU = 0.7       # matches the original training run's args.yaml
DEFAULT_IMGSZ = [320, 640]   # deployed size and trained size

# Recorded results of the ONLY training run that exists in this project.
REFERENCE = {
    "final_epoch": 100,
    "map50": 0.9943,
    "map50_95": 0.9383,
    "precision": 0.98977,
    "recall": 0.98980,
    "best_map50": 0.9950,
    "best_epoch": 59,
    "train_seconds": 1232,
}

DOWNLOADABLE_RE = re.compile(r"^yolo[a-z0-9._-]*\.pt$", re.IGNORECASE)

CSV_COLUMNS = [
    "timestamp_utc", "model", "model_format", "imgsz", "data", "split",
    "batch", "conf", "iou", "rect", "map50", "map50_95", "precision",
    "recall", "delta_map50", "delta_map50_95", "seconds",
    "ultralytics_version", "status",
]


# ───────────────────────────── small helpers ──────────────────────────────
def die(msg: str, code: int = 2):
    print(f"\nERROR: {msg}\n", file=sys.stderr)
    sys.exit(code)


def model_format(path: str) -> str:
    p = Path(path)
    if p.is_dir():
        return "ncnn" if (p / "model.ncnn.param").exists() else "dir"
    return (p.suffix.lstrip(".") or "unknown").lower()


def validate_model_path(path: str) -> None:
    """Do not let Ultralytics silently download a same-named model."""
    p = Path(path)
    if p.exists():
        return
    if DOWNLOADABLE_RE.match(p.name) and p.parent in (Path("."), Path("")):
        print(f"  note: '{path}' is not on disk — Ultralytics will download "
              f"these COCO-pretrained weights.")
        print(f"        A COCO model on this 1-class 'Robot' dataset will "
              f"score near zero. That is expected, not a bug.")
        return
    die(f"model path does not exist: {path}\n"
        f"       (the deployed NCNN model is a DIRECTORY, e.g. "
        f"{DEFAULT_NCNN_MODEL})")


def ncnn_export_imgsz(path: str):
    meta = Path(path) / "metadata.yaml"
    if not meta.exists():
        return None
    try:
        text = meta.read_text()
    except OSError:
        return None
    match = re.search(r"^imgsz:\s*\n\s*-\s*(\d+)", text, re.MULTILINE)
    if match:
        return int(match.group(1))
    match = re.search(r"^imgsz:\s*\[?\s*(\d+)", text, re.MULTILINE)
    return int(match.group(1)) if match else None


def check_data_yaml(path: str) -> Path:
    data = Path(path).expanduser()
    if not data.exists():
        die(f"--data not found: {data}\n"
            f"       The dataset currently lives in the TRASH at:\n"
            f"         {TRASH_DATA_YAML}\n"
            f"       Restore it first, e.g.:\n"
            f"         mv ~/.local/share/Trash/files/'Eye To Zion - Detection "
            f"Robots.v4i.yolo26' ~/datasets/eye-to-zion-v4\n"
            f"       then pass --data ~/datasets/eye-to-zion-v4/data.yaml")
    if ".local/share/Trash" in str(data.resolve()):
        print("\n" + "!" * 78)
        print("!! WARNING: --data points INTO THE TRASH:")
        print(f"!!   {data}")
        print("!! The desktop environment can delete this at any moment, and "
              "these results")
        print("!! would then be unreproducible. Restore the dataset out of "
              "Trash and re-run.")
        print("!" * 78 + "\n")
    return data


def fmt(value, width=8, places=4) -> str:
    if value is None:
        return f"{'n/a':>{width}}"
    return f"{value:>{width}.{places}f}"


def fmt_delta(value, width=9, places=4) -> str:
    if value is None:
        return f"{'base':>{width}}"
    return f"{value:>+{width}.{places}f}"


def print_reference():
    print("=" * 102)
    print(" REFERENCE — original YOLO26n training run (the only run that exists "
          "in this project)")
    print("=" * 102)
    print(f" final epoch {REFERENCE['final_epoch']} : "
          f"mAP@50 = {REFERENCE['map50']:.4f}   "
          f"mAP@50-95 = {REFERENCE['map50_95']:.4f}")
    print(f"                  precision = {REFERENCE['precision']:.5f}   "
          f"recall = {REFERENCE['recall']:.5f}")
    print(f" best           : mAP@50 = {REFERENCE['best_map50']:.4f} @ epoch "
          f"{REFERENCE['best_epoch']}")
    print(f" training       : 100 epochs, imgsz 640, batch 16, seed 0, "
          f"deterministic, in {REFERENCE['train_seconds']} s")
    print(" dataset        : Roboflow 'Eye To Zion - Detection Robots' v4 — "
          "798 train / 100 valid / 100 test, nc=1 ('Robot')")
    print(" -> Validate the ORIGINAL .pt at imgsz 640 on the val split. Landing "
          "near the numbers")
    print("    above means you reproduced the run. A large gap means wrong "
          "weights, wrong split,")
    print("    or wrong dataset — fix that before trusting any other row.")
    print()


# ────────────────────────────── validation ────────────────────────────────
def run_val(YOLO, model_path, imgsz, args):
    """Validate one (model, imgsz) combination. Returns a result dict."""
    validate_model_path(model_path)
    fmt_name = model_format(model_path)

    if fmt_name == "ncnn":
        exported = ncnn_export_imgsz(model_path)
        if exported and exported != imgsz:
            print(f"  WARNING: NCNN model exported at imgsz={exported}, "
                  f"validating at imgsz={imgsz}.")
            print(f"           Fixed-shape exports may fail or silently "
                  f"letterbox. Treat this row with suspicion.")

    batch = args.batch
    if fmt_name != "pt" and batch != 1:
        print(f"  note: exported ({fmt_name}) models validate at batch=1; "
              f"overriding --batch {batch} -> 1.")
        batch = 1

    kwargs = dict(
        data=str(args.data),
        imgsz=imgsz,
        batch=batch,
        split=args.split,
        conf=args.conf,
        iou=args.iou,
        rect=args.rect,
        plots=False,
        verbose=args.verbose,
    )
    if args.device:
        kwargs["device"] = args.device

    model = YOLO(model_path, task="detect")
    start = time.perf_counter()
    metrics = model.val(**kwargs)
    seconds = time.perf_counter() - start

    box = getattr(metrics, "box", None)
    if box is None:
        raise RuntimeError("metrics.box missing — unexpected ultralytics API")

    return {
        "model": str(model_path),
        "model_name": Path(model_path).name,
        "format": fmt_name,
        "imgsz": imgsz,
        "batch": batch,
        "map50": float(box.map50),
        "map50_95": float(box.map),
        "precision": float(box.mp),
        "recall": float(box.mr),
        "seconds": seconds,
        "status": "ok",
    }


def print_table(results):
    print("\n" + "=" * 102)
    print(" RESULTS  (same validation split for every row — directly comparable)")
    print("=" * 102)
    header = (f"{'model':<26}{'fmt':<6}{'imgsz':>6}{'mAP50':>9}{'dmAP50':>10}"
              f"{'mAP50-95':>10}{'dmAP50-95':>11}{'prec':>9}{'recall':>9}"
              f"{'secs':>8}")
    print(header)
    print("-" * 102)

    ok = [r for r in results if r["status"] == "ok"]
    base = ok[0] if ok else None

    for res in results:
        name = res["model_name"][:25]
        if res["status"] != "ok":
            print(f"{name:<26}{res['format']:<6}{res['imgsz']:>6}"
                  f"   FAILED — {res['status'][:55]}")
            continue
        is_base = base is not None and res is base
        d50 = None if is_base else res["map50"] - base["map50"]
        d95 = None if is_base else res["map50_95"] - base["map50_95"]
        print(f"{name:<26}{res['format']:<6}{res['imgsz']:>6}"
              f"{fmt(res['map50'], 9)}{fmt_delta(d50, 10)}"
              f"{fmt(res['map50_95'], 10)}{fmt_delta(d95, 11)}"
              f"{fmt(res['precision'], 9)}{fmt(res['recall'], 9)}"
              f"{res['seconds']:>8.1f}")

    print("-" * 102)
    if base is not None:
        print(f" baseline (d columns are measured against this row): "
              f"{base['model_name']} @ imgsz={base['imgsz']}")
    print(" mAP50 = mAP@0.50   |   mAP50-95 = mAP@0.50:0.95   |   "
          "prec/recall at the given --conf")

    # Sanity check against the recorded run.
    if base is not None:
        gap = abs(base["map50"] - REFERENCE["map50"])
        if base["imgsz"] == 640 and base["format"] == "pt":
            if gap <= 0.01:
                print(f"\n sanity check: baseline mAP@50 is within {gap:.4f} of "
                      f"the recorded {REFERENCE['map50']:.4f} — run reproduced.")
            else:
                print(f"\n sanity check: baseline mAP@50 differs from the "
                      f"recorded {REFERENCE['map50']:.4f} by {gap:.4f}.")
                print("               Check that these are the run's best.pt "
                      "weights and the same val split.")


def write_csv(path, results, meta):
    target = Path(path).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    new_file = not target.exists()
    ok = [r for r in results if r["status"] == "ok"]
    base = ok[0] if ok else None

    with target.open("a", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        if new_file:
            writer.writeheader()
        for res in results:
            good = res["status"] == "ok"
            is_base = good and base is not None and res is base
            writer.writerow({
                "timestamp_utc": meta["timestamp"],
                "model": res["model"],
                "model_format": res["format"],
                "imgsz": res["imgsz"],
                "data": meta["data"],
                "split": meta["split"],
                "batch": res.get("batch", ""),
                "conf": meta["conf"],
                "iou": meta["iou"],
                "rect": meta["rect"],
                "map50": f"{res['map50']:.6f}" if good else "",
                "map50_95": f"{res['map50_95']:.6f}" if good else "",
                "precision": f"{res['precision']:.6f}" if good else "",
                "recall": f"{res['recall']:.6f}" if good else "",
                "delta_map50": ("0.000000" if is_base else
                                (f"{res['map50'] - base['map50']:+.6f}"
                                 if good and base else "")),
                "delta_map50_95": ("0.000000" if is_base else
                                   (f"{res['map50_95'] - base['map50_95']:+.6f}"
                                    if good and base else "")),
                "seconds": f"{res['seconds']:.2f}" if good else "",
                "ultralytics_version": meta["ultralytics_version"],
                "status": res["status"],
            })
    print(f"\nCSV {'created' if new_file else 'appended'}: {target}")


# ─────────────────────────────────  main  ─────────────────────────────────
def build_parser():
    parser = argparse.ArgumentParser(
        description="Validate two or more model/imgsz combinations against the "
                    "SAME dataset so mAP is comparable.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog="Run this on the training machine, not on the Pi.")
    parser.add_argument(
        "--models", nargs="+", required=True,
        help="Model paths. A directory = an exported model (the deployed NCNN "
             "model is a directory); a .pt = PyTorch. 'yolov8n.pt' will be "
             "downloaded if absent — note it is COCO-pretrained and will score "
             "near zero on this 1-class dataset unless fine-tuned first.")
    parser.add_argument(
        "--data", default=str(TRASH_DATA_YAML),
        help="Path to data.yaml. RESTORE THE DATASET OUT OF THE TRASH FIRST — "
             "the default below is its current trash location and is not a "
             "safe place to validate from.")
    parser.add_argument(
        "--imgsz", nargs="+", type=int, default=DEFAULT_IMGSZ,
        help="One or more sizes; the full cross-product of models x imgsz is "
             "run. Defaults are the deployed (320) and trained (640) sizes.")
    parser.add_argument("--split", default="val", choices=("val", "test", "train"),
                        help="Dataset split to validate on (100 valid / 100 "
                             "test images in this dataset).")
    parser.add_argument("--batch", type=int, default=16,
                        help="Batch size (forced to 1 for exported models).")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF,
                        help="Confidence threshold (deployment uses 0.3).")
    parser.add_argument("--iou", type=float, default=DEFAULT_IOU,
                        help="NMS IoU threshold (0.7 matches the training run).")
    parser.add_argument("--rect", action="store_true",
                        help="Enable rectangular validation. OFF by default so "
                             "every row sees identically shaped tensors and "
                             "fixed-shape exports do not break.")
    parser.add_argument("--device", default=None,
                        help="Passed to ultralytics (e.g. 'cpu', '0'). Leave "
                             "unset for auto. TensorRT is not applicable to "
                             "this project (no NVIDIA GPU on the Pi 5).")
    parser.add_argument("--verbose", action="store_true",
                        help="Let ultralytics print its own per-class table.")
    parser.add_argument("--csv", default=None, metavar="PATH",
                        help="Append machine-readable results to this CSV.")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    try:
        import ultralytics
        from ultralytics import YOLO
    except ImportError as exc:
        die("could not import 'ultralytics' "
            f"({exc}).\n"
            "       Install it with:  pip install ultralytics\n"
            "       A .pt model additionally needs:     pip install torch\n"
            "       The NCNN model additionally needs:  pip install ncnn")
        return 2  # unreachable

    args.data = check_data_yaml(args.data)

    print_reference()

    print(f"data   : {args.data}")
    print(f"split  : {args.split}")
    print(f"conf   : {args.conf}    iou: {args.iou}    rect: {args.rect}")
    print(f"ultralytics: {getattr(ultralytics, '__version__', 'unknown')}")

    combos = [(m, s) for m in args.models for s in args.imgsz]
    results = []
    for index, (model_path, imgsz) in enumerate(combos, start=1):
        print(f"\n>>> [{index}/{len(combos)}] validating {model_path} @ "
              f"imgsz={imgsz} ...")
        try:
            results.append(run_val(YOLO, model_path, imgsz, args))
        except SystemExit:
            raise
        except Exception as exc:                              # noqa: BLE001
            print(f"    FAILED: {type(exc).__name__}: {exc}")
            results.append({
                "model": str(model_path),
                "model_name": Path(model_path).name,
                "format": model_format(model_path),
                "imgsz": imgsz,
                "status": f"{type(exc).__name__}: {exc}",
            })

    print_table(results)

    if args.csv:
        write_csv(args.csv, results, {
            "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "data": str(args.data),
            "split": args.split,
            "conf": args.conf,
            "iou": args.iou,
            "rect": args.rect,
            "ultralytics_version": getattr(ultralytics, "__version__", "unknown"),
        })

    return 0 if all(r["status"] == "ok" for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
