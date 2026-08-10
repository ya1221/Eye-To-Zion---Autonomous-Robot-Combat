#!/usr/bin/env bash
#
# train_yolo_ablation.sh — augmentation ablation: mAP BEFORE vs AFTER augmentation
# ==============================================================================
#
# PURPOSE
# -------
# Produce the "mAP before vs after augmentation" comparison that this project
# does not currently have. The repo contains exactly ONE training run — the
# YOLO26n run that reached mAP@50 = 0.9943 / mAP@50-95 = 0.9383 at epoch 100 —
# and that run had augmentation ENABLED. There is no un-augmented counterpart,
# so today there is nothing to compare it against and no honest way to state
# what augmentation contributed.
#
# This script runs both halves of that comparison:
#
#   Run 1  "no_aug"    every augmentation knob set to 0 / none.
#   Run 2  "with_aug"  the EXACT values from the real run's args.yaml:
#                      hsv_h=0.015 hsv_s=0.7 hsv_v=0.4 translate=0.1 scale=0.5
#                      fliplr=0.5 mosaic=1.0 close_mosaic=10 erasing=0.4
#                      auto_augment=randaugment, every other knob 0.
#
# Everything else is identical between the two runs and identical to the
# original: model=yolo26n.pt, epochs=100, imgsz=640, batch=16, seed=0,
# deterministic=True. Augmentation is therefore the ONLY variable.
#
# EXACT RUN COMMAND
# -----------------
#   chmod +x measurement_scripts/train_yolo_ablation.sh
#
#   # See exactly what would run, without training anything:
#   DRY_RUN=1 ./measurement_scripts/train_yolo_ablation.sh
#
#   # Full ablation (matches the original run's hyper-parameters):
#   DATA=~/datasets/eye-to-zion-v4/data.yaml \
#     ./measurement_scripts/train_yolo_ablation.sh
#
#   # Quick smoke test before committing hours of GPU time:
#   EPOCHS=3 DATA=~/datasets/eye-to-zion-v4/data.yaml \
#     ./measurement_scripts/train_yolo_ablation.sh
#
#   # Pin a specific GPU:
#   DEVICE=0 DATA=~/datasets/eye-to-zion-v4/data.yaml \
#     ./measurement_scripts/train_yolo_ablation.sh
#
# ENVIRONMENT VARIABLES (all optional)
# ------------------------------------
#   EPOCHS    default 100     epochs per run (matches the original run)
#   DATA      default <the dataset's current trash path — SEE BELOW>
#   DEVICE    default ""      "" = auto-select, matching the original run's
#                             `device: null`. Set 0 for GPU 0, or cpu.
#   IMGSZ     default 640     training resolution (the original used 640)
#   BATCH     default 16
#   MODEL     default yolo26n.pt
#   PROJECT   default ./runs/ablation
#   DRY_RUN   unset           set to 1 to print the commands and exit
#
# PREREQUISITES
# -------------
#   * The `yolo` CLI on PATH (`pip install ultralytics`). Checked before
#     anything else runs.
#   * A GPU. The original run did 100 epochs in 1232 s; this script does TWO
#     such runs, so budget roughly 2x that on comparable hardware. On a
#     Raspberry Pi 5 CPU this would take days — DO NOT RUN THIS ON THE PI.
#   * The dataset restored out of the trash. It currently lives at:
#       ~/.local/share/Trash/files/Eye To Zion - Detection Robots.v4i.yolo26/data.yaml
#     Restore it first, e.g.:
#       mv ~/.local/share/Trash/files/"Eye To Zion - Detection Robots.v4i.yolo26" \
#          ~/datasets/eye-to-zion-v4
#     Roboflow "Eye To Zion - Detection Robots" v4:
#       798 train / 100 valid / 100 test images, nc=1, names: ['Robot'].
#
# EXPECTED OUTPUT FORMAT
# ----------------------
#   vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#   !! SAMPLE BELOW IS AN ILLUSTRATIVE LAYOUT MOCK-UP.                     !!
#   !! THE no_aug / with_aug / delta CELLS ARE INVENTED PLACEHOLDERS.      !!
#   !! THEY ARE NOT MEASUREMENTS AND MUST NEVER BE QUOTED AS ONE.          !!
#   !! (Only the REFERENCE line holds real, recorded numbers.)             !!
#   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#
#     =====================================================================
#      AUGMENTATION ABLATION — RESULTS
#      metric              no_aug     with_aug        delta
#      ---------------------------------------------------------
#      mAP@50 (last)       <....>       <....>      <+/-....>
#      mAP@50 (best)       <....>       <....>      <+/-....>
#      mAP@50-95 (last)    <....>       <....>      <+/-....>
#      mAP@50-95 (best)    <....>       <....>      <+/-....>
#
#      REFERENCE (recorded, original augmented run):
#        mAP@50 = 0.9943   mAP@50-95 = 0.9383   @ epoch 100
#        best mAP@50 = 0.9950 @ epoch 59
#
#      results.csv (no_aug)   : <PROJECT>/no_aug/results.csv
#      results.csv (with_aug) : <PROJECT>/with_aug/results.csv
#      weights                : <PROJECT>/<run>/weights/best.pt
#
#      Diff the two curves yourself:
#        paste -d, <(cut -d, -f1,8,9 <PROJECT>/no_aug/results.csv) \
#                  <(cut -d, -f8,9   <PROJECT>/with_aug/results.csv) | column -t -s,
#
# NOTE: the "last" row is the FINAL epoch (comparable to the recorded 0.9943
# at epoch 100); the "best" row is the maximum over all epochs (comparable to
# the recorded best of 0.9950 at epoch 59). Quote both — with a 798-image
# 1-class dataset already scoring ~0.99, expect the delta to be small and
# possibly within run-to-run noise. If you need a defensible claim, repeat
# both runs with several seeds and report the spread.
# ==============================================================================

set -euo pipefail

# ─────────────────────────────── configuration ────────────────────────────
EPOCHS="${EPOCHS:-100}"
DATA="${DATA:-$HOME/.local/share/Trash/files/Eye To Zion - Detection Robots.v4i.yolo26/data.yaml}"
DEVICE="${DEVICE:-}"          # "" = auto (the original run had device: null)
IMGSZ="${IMGSZ:-640}"
BATCH="${BATCH:-16}"
MODEL="${MODEL:-yolo26n.pt}"
PROJECT="${PROJECT:-$(pwd)/runs/ablation}"
DRY_RUN="${DRY_RUN:-}"

RUN_NO_AUG="no_aug"
RUN_WITH_AUG="with_aug"

# Recorded results of the ONLY training run that exists in this project.
REF_MAP50="0.9943"
REF_MAP50_95="0.9383"
REF_BEST_MAP50="0.9950"
REF_BEST_EPOCH="59"

# ───────────────────────────────── helpers ────────────────────────────────
say()  { printf '%s\n' "$*"; }
rule() { printf '%s\n' "======================================================================"; }
fail() { printf '\nERROR: %s\n\n' "$*" >&2; exit 1; }

# ───────────────────────────── preflight checks ───────────────────────────
rule
say " AUGMENTATION ABLATION — mAP before vs after augmentation"
rule

command -v yolo >/dev/null 2>&1 || fail \
"the 'yolo' CLI was not found on PATH.
       Install it with:  pip install ultralytics
       Then re-run. (If it is installed in a venv, activate the venv first.)"

say "yolo CLI     : $(command -v yolo)"
say "ultralytics  : $(yolo version 2>/dev/null | tail -n 1 || echo 'unknown')"

if [[ ! -f "$DATA" ]]; then
  fail "DATA not found: $DATA
       The dataset currently lives in the TRASH at:
         ~/.local/share/Trash/files/Eye To Zion - Detection Robots.v4i.yolo26/data.yaml
       RESTORE IT FIRST, e.g.:
         mv ~/.local/share/Trash/files/'Eye To Zion - Detection Robots.v4i.yolo26' \\
            ~/datasets/eye-to-zion-v4
       then re-run with:
         DATA=~/datasets/eye-to-zion-v4/data.yaml $0"
fi

case "$(readlink -f "$DATA")" in
  *.local/share/Trash/*)
    say ""
    say "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    say "!! WARNING: DATA points INTO THE TRASH:"
    say "!!   $DATA"
    say "!! The desktop environment can delete it at any time, which would"
    say "!! make these runs unreproducible. Restore the dataset and re-run."
    say "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    say ""
    ;;
esac

say "model        : $MODEL"
say "data         : $DATA"
say "epochs       : $EPOCHS   imgsz: $IMGSZ   batch: $BATCH   seed: 0   deterministic: True"
say "device       : ${DEVICE:-<auto — same as the original run, which had device: null>}"
say "project      : $PROJECT"
say ""
if [[ "$EPOCHS" != "100" ]]; then
  say "NOTE: EPOCHS=$EPOCHS differs from the original run's 100, so these"
  say "      results are NOT directly comparable to the recorded 0.9943 /"
  say "      0.9383. They remain valid as a no_aug-vs-with_aug comparison."
  say ""
fi
say "TIME BUDGET: the original 100-epoch run took 1232 s on GPU. This script"
say "             performs TWO such runs. Do not run it on the Raspberry Pi."
say ""

# ─────────────────────────── shared training args ─────────────────────────
# Identical for both runs so augmentation is the only variable.
COMMON_ARGS=(
  "model=$MODEL"
  "data=$DATA"
  "epochs=$EPOCHS"
  "imgsz=$IMGSZ"
  "batch=$BATCH"
  "seed=0"
  "deterministic=True"
  "patience=100"
  "optimizer=auto"
  "pretrained=True"
  "val=True"
  "plots=True"
  "project=$PROJECT"
  "exist_ok=True"
)
# ${DEVICE:+...} expands to nothing when DEVICE is empty (safe under set -u),
# letting ultralytics auto-select exactly as the original run did.
if [[ -n "$DEVICE" ]]; then
  COMMON_ARGS+=("device=$DEVICE")
fi

# Run 1 — every augmentation knob off.
NO_AUG_ARGS=(
  "hsv_h=0.0" "hsv_s=0.0" "hsv_v=0.0"
  "degrees=0.0" "translate=0.0" "scale=0.0" "shear=0.0" "perspective=0.0"
  "flipud=0.0" "fliplr=0.0" "bgr=0.0"
  "mosaic=0.0" "close_mosaic=0"
  "mixup=0.0" "cutmix=0.0" "copy_paste=0.0"
  "erasing=0.0" "auto_augment=none"
)

# Run 2 — the EXACT values from the original run's args.yaml.
WITH_AUG_ARGS=(
  "hsv_h=0.015" "hsv_s=0.7" "hsv_v=0.4"
  "degrees=0.0" "translate=0.1" "scale=0.5" "shear=0.0" "perspective=0.0"
  "flipud=0.0" "fliplr=0.5" "bgr=0.0"
  "mosaic=1.0" "close_mosaic=10"
  "mixup=0.0" "cutmix=0.0" "copy_paste=0.0" "copy_paste_mode=flip"
  "erasing=0.4" "auto_augment=randaugment"
)

run_training() {
  local name="$1"; shift
  local -a aug_args=("$@")

  say ""
  rule
  say " RUN: $name"
  rule
  say "# The augmentation arguments (last block) are the ONLY difference"
  say "# between the two runs. Command is shell-quoted and copy-pasteable:"
  say "yolo detect train \\"
  # %q shell-quotes each argument, so paths containing spaces survive a copy-paste.
  printf '    %q \\\n' "${COMMON_ARGS[@]}" "name=$name"
  local i last=$(( ${#aug_args[@]} - 1 ))
  for i in "${!aug_args[@]}"; do
    if (( i == last )); then
      printf '    %q\n' "${aug_args[$i]}"     # no trailing backslash on the last arg
    else
      printf '    %q \\\n' "${aug_args[$i]}"
    fi
  done
  say ""

  if [[ -n "$DRY_RUN" ]]; then
    say "DRY_RUN set — not training."
    return 0
  fi

  yolo detect train "${COMMON_ARGS[@]}" "name=$name" "${aug_args[@]}"
  say ""
  say "RUN '$name' finished. results.csv -> $PROJECT/$name/results.csv"
}

run_training "$RUN_NO_AUG"   "${NO_AUG_ARGS[@]}"
run_training "$RUN_WITH_AUG" "${WITH_AUG_ARGS[@]}"

# ──────────────────────────── results comparison ──────────────────────────
CSV_NO_AUG="$PROJECT/$RUN_NO_AUG/results.csv"
CSV_WITH_AUG="$PROJECT/$RUN_WITH_AUG/results.csv"

if [[ -n "$DRY_RUN" ]]; then
  say ""
  rule
  say " DRY_RUN complete — nothing was trained."
  say " Real runs would write:"
  say "   $CSV_NO_AUG"
  say "   $CSV_WITH_AUG"
  rule
  exit 0
fi

# Extract a metric from a results.csv by HEADER NAME (robust against column
# reordering between ultralytics versions). Prints "<last> <best>".
metric_of() {
  local csv="$1" want="$2"
  [[ -f "$csv" ]] || { printf 'n/a n/a\n'; return 0; }
  awk -F, -v want="$want" '
    NR == 1 {
      for (i = 1; i <= NF; i++) {
        h = $i; gsub(/^[ \t\r]+|[ \t\r]+$/, "", h)
        if (h == want) col = i
      }
      if (!col) { print "n/a n/a"; exit }
      next
    }
    NF >= col && $col != "" {
      last = $col + 0
      if (!seen++ || last > best) best = last
    }
    END { if (seen) printf "%.5f %.5f\n", last, best; else print "n/a n/a" }
  ' "$csv"
}

delta() {   # delta <a> <b> -> b - a, or "n/a"
  if [[ "$1" == "n/a" || "$2" == "n/a" ]]; then printf 'n/a'; return 0; fi
  awk -v a="$1" -v b="$2" 'BEGIN { printf "%+.5f", b - a }'
}

read -r NA_MAP50_LAST  NA_MAP50_BEST  <<<"$(metric_of "$CSV_NO_AUG"   'metrics/mAP50(B)')"
read -r WA_MAP50_LAST  WA_MAP50_BEST  <<<"$(metric_of "$CSV_WITH_AUG" 'metrics/mAP50(B)')"
read -r NA_MAP95_LAST  NA_MAP95_BEST  <<<"$(metric_of "$CSV_NO_AUG"   'metrics/mAP50-95(B)')"
read -r WA_MAP95_LAST  WA_MAP95_BEST  <<<"$(metric_of "$CSV_WITH_AUG" 'metrics/mAP50-95(B)')"

say ""
rule
say " AUGMENTATION ABLATION — RESULTS"
rule
printf ' %-20s %12s %12s %12s\n' "metric" "no_aug" "with_aug" "delta"
printf ' %s\n' "----------------------------------------------------------"
printf ' %-20s %12s %12s %12s\n' "mAP@50 (last)"    "$NA_MAP50_LAST" "$WA_MAP50_LAST" "$(delta "$NA_MAP50_LAST" "$WA_MAP50_LAST")"
printf ' %-20s %12s %12s %12s\n' "mAP@50 (best)"    "$NA_MAP50_BEST" "$WA_MAP50_BEST" "$(delta "$NA_MAP50_BEST" "$WA_MAP50_BEST")"
printf ' %-20s %12s %12s %12s\n' "mAP@50-95 (last)" "$NA_MAP95_LAST" "$WA_MAP95_LAST" "$(delta "$NA_MAP95_LAST" "$WA_MAP95_LAST")"
printf ' %-20s %12s %12s %12s\n' "mAP@50-95 (best)" "$NA_MAP95_BEST" "$WA_MAP95_BEST" "$(delta "$NA_MAP95_BEST" "$WA_MAP95_BEST")"
say ""
say " delta = with_aug - no_aug   (positive means augmentation helped)"
say ""
say " REFERENCE (recorded, original augmented YOLO26n run):"
say "   mAP@50 = $REF_MAP50   mAP@50-95 = $REF_MAP50_95   @ epoch 100"
say "   best mAP@50 = $REF_BEST_MAP50 @ epoch $REF_BEST_EPOCH"
say "   -> the with_aug run above uses that run's exact augmentation values,"
say "      so with EPOCHS=100 it should land close to those numbers."
say ""
say " results.csv (no_aug)   : $CSV_NO_AUG"
say " results.csv (with_aug) : $CSV_WITH_AUG"
say " weights                : $PROJECT/<run>/weights/best.pt"
say " curves / plots         : $PROJECT/<run>/results.png"
say ""
say " Diff the full per-epoch curves yourself:"
say "   paste -d, <(cut -d, -f1,8,9 '$CSV_NO_AUG') \\"
say "             <(cut -d, -f8,9   '$CSV_WITH_AUG') | column -t -s,"
say "   # fields: 1=epoch 8=metrics/mAP50(B) 9=metrics/mAP50-95(B)"
say ""
say " Side-by-side final epochs:"
say "   for f in '$CSV_NO_AUG' '$CSV_WITH_AUG'; do echo \"\$f\"; head -1 \"\$f\"; tail -1 \"\$f\"; done"
say ""
say " CAVEAT: this dataset is small (798 train / 100 valid, 1 class) and"
say "         already scores ~0.99 mAP@50. Expect a small delta that may sit"
say "         inside run-to-run noise. For a defensible claim, repeat both"
say "         runs across several seeds and report the spread."
rule
