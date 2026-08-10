#!/usr/bin/env python3
"""
measure_cumulative_error.py

PURPOSE
=======
Measure how position error accumulates WITH and WITHOUT the overhead-camera
ArUco correction, as a function of DISTANCE TRAVELLED rather than time.

Drift on a wheeled robot is distance-driven, not time-driven: a robot parked
for ten minutes accumulates almost nothing, a robot driven ten metres in the
same ten minutes accumulates a lot.  Reporting "X cm after 5 minutes" hides
that.  Every headline number here is therefore per-metre, and the key output
is the error-vs-distance curve for the corrected and uncorrected signals.

Three pose sources are subscribed simultaneously and timestamp-aligned:

  --pose-topic   /odometry/global   the FUSED, ArUco-corrected estimate
                                    (robot_localization ekf_global, frame map)
  --odom-topic   /odom              the RAW wheel odometry - the UNCORRECTED
                                    signal (frame odom)
  --aruco-topic  /aruco/odom        the overhead ArUco pose - the REFERENCE

*** THIS SCRIPT IS READ-ONLY.  IT NEVER PUBLISHES TO /cmd_vel.  IT NEVER
*** PUBLISHES ANYTHING AT ALL.  It only subscribes.  Drive the robot yourself
*** (teleop, the tactical brain, or measure_position_drift.py in another
*** terminal) while this records.  There is deliberately no motion code in
*** this file, no --i-am-clear flag, and nothing to accidentally set moving.

WHICH TOPIC IS THE RAW WHEEL ODOMETRY?
--------------------------------------
On this stack ekf_global.yaml lists its wheel-odometry input (odom0) as
`/ackermann_steering_controller/odometry`.  Some launch configurations remap
that to `/odom`, and ekf.yaml refers to `/odom` directly.  Check first:

    ros2 topic list | grep -i odom
    ros2 topic info /odom

and pass whichever one actually carries the controller's output via
--odom-topic.  The default is /odom.

Be aware of what that signal is: ackermann_steering_controller.yaml sets
`open_loop: true`, so the published wheel odometry is integrated from the
COMMANDS, not from wheel encoders.  It cannot see wheel slip, a mis-calibrated
pwm_multiplier, or a wrong wheel radius - which is precisely why it drifts and
why the ArUco correction exists.

FRAME ALIGNMENT
---------------
/odom lives in the `odom` frame and /aruco/odom in the `map` frame; the two
origins do not coincide, so comparing raw coordinates is meaningless.  Every
source is therefore re-expressed RELATIVE TO ITS OWN FIRST ALIGNED SAMPLE
(translation removed, and the initial heading rotated out).  What is compared
is accumulated displacement from a common starting point, which is exactly the
quantity that drifts.  A consequence: the reported error is always 0 at
distance 0 by construction - the slope, not the intercept, is the result.

HOW TO RUN THE UNCORRECTED CASE HONESTLY
----------------------------------------
Do NOT try to disable the EKF at runtime, and this script will not attempt it.
Toggling a live filter's inputs mid-session leaves it in a half-converged
state and the numbers mean nothing.  Instead:

  CORRECTED RUN (the normal system):
    Launch everything as usual - overhead tracker up, localization_bridge
    republishing ArUco onto /aruco/odom, ekf_global fusing it, and the
    /ekf_global/set_pose force-reseed armed.  Run this script with no special
    flags.  /odometry/global is corrected; /odom is not; /aruco/odom is truth.

  UNCORRECTED RUN (the honest way):
    Stop the ArUco FEED INTO THE EKF while keeping ArUco as an independent
    reference.  Concretely, either:
      (a) do not launch the node that republishes onto /aruco/odom (the
          tactical brain's localization_bridge), and instead run the overhead
          tracker publishing to a different topic, or
      (b) remap the publisher at its source, e.g.
            ros2 run <pkg> <node> --ros-args -r /aruco/odom:=/aruco/truth
    Then ekf_global receives no absolute pose at all, /odometry/global degrades
    to dead reckoning, and /aruco/truth is still available as ground truth.
    Run this script with:
        --no-aruco --aruco-topic /aruco/truth
    --no-aruco only tags the output as an uncorrected session and adjusts the
    reporting.  It changes nothing on the robot - it cannot, this script does
    not publish.

  Either way, /odom (raw wheel odometry) is the uncorrected signal in BOTH
  runs, so even a single corrected session already yields a corrected-vs-
  uncorrected comparison.  The dedicated uncorrected run additionally shows
  what /odometry/global itself does once the correction is removed.

SNAP DETECTION
--------------
tactical_brain/localization_bridge.py force-reseeds ekf_global through the
/ekf_global/set_pose service whenever ArUco and the tracked pose disagree by
more than DRIFT_SNAP_THRESHOLD_METERS = 0.05.  A reseed shows up in
/odometry/global as a position discontinuity.  This script counts a snap when
two consecutive fused samples are more than --snap-threshold apart AND the
implied speed exceeds --max-speed, i.e. the jump is too large to be real
motion.  Both conditions are required so that fast driving is not miscounted.

EXACT RUN COMMAND
=================
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash

  # corrected run - 5 minutes, plot at the end
  python3 measurement_scripts/measure_cumulative_error.py \
      --duration 300 --csv results/cumulative_corrected.csv --plot

  # uncorrected run - after removing the /aruco/odom feed as described above
  python3 measurement_scripts/measure_cumulative_error.py \
      --duration 300 --no-aruco --aruco-topic /aruco/truth \
      --csv results/cumulative_uncorrected.csv --plot

  # drive the robot in a SEPARATE terminal for the whole duration

PREREQUISITES
=============
  * localization / ekf_global publishing --pose-topic
  * the ackermann controller publishing wheel odometry on --odom-topic
  * overhead_tracker plus whatever republishes ArUco onto --aruco-topic
  * something driving the robot - this script does not
  * verify all three topics are live BEFORE starting the clock:
        ros2 topic hz /odometry/global
        ros2 topic hz /odom
        ros2 topic hz /aruco/odom
  * matplotlib only if you pass --plot; it is skipped gracefully when missing

SAFETY NOTES
============
  * Read-only.  No publishers are created.  Nothing this script does can move
    the robot.
  * The usual arena safety applies to whoever IS driving.
  * The only failure mode here is silent: if a topic is dead the script says
    so and reports what it can, so read the startup topic check.

EXPECTED OUTPUT FORMAT
======================
--- ILLUSTRATIVE SAMPLE ONLY.  THE NUMBERS BELOW ARE PLACEHOLDERS SHOWING THE
--- LAYOUT.  THEY ARE NOT MEASUREMENTS.  DO NOT COPY THEM ANYWHERE.

  ================================================================
   CUMULATIVE ERROR vs DISTANCE     session=corrected
   reference: /aruco/odom     duration: XXX.X s
   distance travelled (reference): XX.XX m      aligned samples: XXXX
  ================================================================
   distance     fused err     raw odom err
      1 m         X.X cm         X.X cm
      2 m         X.X cm         X.X cm
      5 m         X.X cm        XX.X cm
     10 m         not reached    not reached
  ----------------------------------------------------------------
   ERROR GROWTH RATE
     fused  (/odometry/global) ..... X.XX cm per metre
     raw    (/odom) ................ X.XX cm per metre
  ----------------------------------------------------------------
   ARUCO SNAP CORRECTIONS (jump > 5.0 cm in the fused pose)
     snaps detected ................ N
     snaps per metre ............... X.XX
     time between snaps ............ XX.X +/- X.X s (min X.X, max XX.X)
     largest jump .................. X.X cm
  ================================================================

The CSV holds the aligned per-sample time series (one row per aligned sample:
time, distance, each source's relative position, and the two error columns) so
the curve can be re-plotted or re-fitted later without repeating the run.
"""

import argparse
import csv
import datetime
import math
import os
import signal
import sys
import time

import numpy as np

# --------------------------------------------------------------------------
# Constants taken verbatim from the repository.
# --------------------------------------------------------------------------
DRIFT_SNAP_THRESHOLD_M = 0.05       # tactical_brain/localization_bridge.py
DEFAULT_POSE_TOPIC = '/odometry/global'
DEFAULT_ODOM_TOPIC = '/odom'
DEFAULT_ARUCO_TOPIC = '/aruco/odom'
DISTANCE_MARKS_M = (1.0, 2.0, 5.0, 10.0)

_STOP_REQUESTED = False


def _signal_handler(signum, _frame):
    """No motion to stop - just end the recording cleanly and still report."""
    global _STOP_REQUESTED
    _STOP_REQUESTED = True
    sys.stderr.write(f"\n  {signal.Signals(signum).name} received - finishing "
                     "the recording and reporting what was captured.\n")
    sys.stderr.flush()


def install_signal_handlers():
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)


# ==========================================================================
# Pure math
# ==========================================================================
def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def to_relative_frame(track):
    """Re-express a (N, 4) [t, x, y, yaw] track relative to its first sample.

    Translation removed and the initial heading rotated out, so two tracks
    recorded in different frames (odom vs map) become directly comparable as
    accumulated displacement from a shared start.
    """
    arr = np.asarray(track, dtype=float)
    if arr.shape[0] == 0:
        return arr
    x0, y0, yaw0 = arr[0, 1], arr[0, 2], arr[0, 3]
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    rot = np.array([[c, -s], [s, c]])
    out = arr.copy()
    out[:, 1:3] = (arr[:, 1:3] - np.array([x0, y0])) @ rot.T
    out[:, 3] = np.array([wrap_angle(a - yaw0) for a in arr[:, 3]])
    return out


def nearest_index(times, t, max_dt):
    """Index of the sample closest in time to t, or None if none within max_dt."""
    if times.size == 0:
        return None
    i = int(np.searchsorted(times, t))
    best, best_dt = None, max_dt
    for cand in (i - 1, i, i + 1):
        if 0 <= cand < times.size:
            dt = abs(times[cand] - t)
            if dt <= best_dt:
                best, best_dt = cand, dt
    return best


def cumulative_distance(xy, min_step):
    """Path length with a deadband, so sensor jitter is not counted as travel.

    ArUco centres come from integer pixel centroids warped through a
    homography; without the deadband a stationary robot would appear to
    accumulate metres of "travel" from a few millimetres of per-frame noise.
    """
    dist = np.zeros(xy.shape[0], dtype=float)
    total = 0.0
    last = xy[0]
    for i in range(1, xy.shape[0]):
        step = float(np.linalg.norm(xy[i] - last))
        if step >= min_step:
            total += step
            last = xy[i]
        dist[i] = total
    return dist


def growth_rate_cm_per_m(distance, error):
    """Least-squares slope through the origin: err = k * distance.

    Forced through the origin because the relative-frame construction makes
    the error exactly zero at zero distance; a free intercept would only fit
    alignment noise.
    """
    d = np.asarray(distance, dtype=float)
    e = np.asarray(error, dtype=float)
    mask = np.isfinite(d) & np.isfinite(e) & (d > 0)
    if not np.any(mask):
        return float('nan')
    d, e = d[mask], e[mask]
    denom = float(np.sum(d * d))
    if denom < 1e-12:
        return float('nan')
    return float(np.sum(d * e) / denom) * 100.0


def error_at_marks(distance, error, marks):
    """Interpolate the error at each distance mark; None when never reached."""
    out = {}
    d = np.asarray(distance, dtype=float)
    e = np.asarray(error, dtype=float)
    for mark in marks:
        if d.size == 0 or d[-1] < mark:
            out[mark] = None
        else:
            out[mark] = float(np.interp(mark, d, e))
    return out


def detect_snaps(track, snap_threshold, max_speed):
    """Find reseed discontinuities in the fused pose.

    A snap is a position jump larger than snap_threshold whose implied speed
    exceeds max_speed - i.e. too big to be real motion.  Returns a list of
    (time, jump_metres).
    """
    arr = np.asarray(track, dtype=float)
    if arr.shape[0] < 2:
        return []
    dt = np.diff(arr[:, 0])
    jumps = np.linalg.norm(np.diff(arr[:, 1:3], axis=0), axis=1)
    snaps = []
    for i, (gap, jump) in enumerate(zip(dt, jumps)):
        if jump <= snap_threshold:
            continue
        if gap <= 1e-6 or (jump / gap) > max_speed:
            snaps.append((float(arr[i + 1, 0]), float(jump)))
    return snaps


def mean_std(values):
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)],
                   dtype=float)
    if v.size == 0:
        return float('nan'), float('nan'), 0
    if v.size == 1:
        return float(v[0]), 0.0, 1
    return float(np.mean(v)), float(np.std(v, ddof=1)), int(v.size)


# ==========================================================================
# Alignment
# ==========================================================================
def align_tracks(fused, odom, aruco, max_dt, min_step):
    """Timestamp-align the three sources onto the ArUco reference samples.

    For each reference sample, take the nearest fused and nearest raw-odometry
    sample within max_dt.  Reference samples with no partner are dropped.
    Returns a dict of numpy arrays, or None when nothing aligns.
    """
    if len(aruco) < 2:
        return None

    ref = to_relative_frame(aruco)
    fus = to_relative_frame(fused) if len(fused) >= 1 else np.empty((0, 4))
    raw = to_relative_frame(odom) if len(odom) >= 1 else np.empty((0, 4))

    t_fus = fus[:, 0] if fus.size else np.empty(0)
    t_raw = raw[:, 0] if raw.size else np.empty(0)

    rows = []
    for sample in ref:
        t = sample[0]
        i_f = nearest_index(t_fus, t, max_dt)
        i_r = nearest_index(t_raw, t, max_dt)
        rows.append((t, sample[1], sample[2], sample[3],
                     fus[i_f, 1] if i_f is not None else np.nan,
                     fus[i_f, 2] if i_f is not None else np.nan,
                     raw[i_r, 1] if i_r is not None else np.nan,
                     raw[i_r, 2] if i_r is not None else np.nan))
    if not rows:
        return None

    arr = np.asarray(rows, dtype=float)
    ref_xy = arr[:, 1:3]
    dist = cumulative_distance(ref_xy, min_step)
    err_fused = np.linalg.norm(arr[:, 4:6] - ref_xy, axis=1)
    err_raw = np.linalg.norm(arr[:, 6:8] - ref_xy, axis=1)

    return {
        't': arr[:, 0],
        'distance_m': dist,
        'ref_xy': ref_xy,
        'fused_xy': arr[:, 4:6],
        'raw_xy': arr[:, 6:8],
        'err_fused_m': err_fused,
        'err_raw_m': err_raw,
        'n_fused_matched': int(np.sum(np.isfinite(arr[:, 4]))),
        'n_raw_matched': int(np.sum(np.isfinite(arr[:, 6]))),
    }


def divergence_only(fused, odom, max_dt, min_step):
    """Fallback when there is no ArUco reference at all.

    Reports fused-vs-raw divergence, which is NOT an error against truth.  It
    is a lower bound: two dead-reckoning estimates disagreeing by X means at
    least one of them is off by at least X/2.
    """
    if len(fused) < 2 or len(odom) < 2:
        return None
    fus = to_relative_frame(fused)
    raw = to_relative_frame(odom)
    t_raw = raw[:, 0]
    rows = []
    for sample in fus:
        i_r = nearest_index(t_raw, sample[0], max_dt)
        if i_r is None:
            continue
        rows.append((sample[0], sample[1], sample[2],
                     raw[i_r, 1], raw[i_r, 2]))
    if len(rows) < 2:
        return None
    arr = np.asarray(rows, dtype=float)
    fused_xy = arr[:, 1:3]
    raw_xy = arr[:, 3:5]
    return {
        't': arr[:, 0],
        'distance_m': cumulative_distance(fused_xy, min_step),
        'fused_xy': fused_xy,
        'raw_xy': raw_xy,
        'divergence_m': np.linalg.norm(fused_xy - raw_xy, axis=1),
    }


# ==========================================================================
# Reporting
# ==========================================================================
def report(args, aligned, snaps, duration, session_label, fallback):
    bar = "=" * 68
    print()
    print(bar)
    print(f"  CUMULATIVE ERROR vs DISTANCE     session={session_label}")

    if fallback is not None:
        print(f"  reference: NONE (no samples on {args.aruco_topic})")
        print(f"  duration: {duration:.1f} s")
        print(bar)
        print("  !! No ArUco reference was received, so there is NO ground")
        print("  !! truth in this session.  Reporting fused-vs-raw divergence")
        print("  !! instead.  This is a LOWER BOUND on the true error, not the")
        print("  !! error itself.  Re-run with the reference topic live, or")
        print("  !! see 'HOW TO RUN THE UNCORRECTED CASE HONESTLY' in the")
        print("  !! docstring for how to keep truth while removing the")
        print("  !! correction.")
        print("-" * 68)
        d = fallback['distance_m']
        div = fallback['divergence_m']
        print(f"  distance travelled (fused) .... {d[-1]:.2f} m")
        print(f"  final fused-vs-raw divergence . {div[-1] * 100:.1f} cm")
        print(f"  max divergence ................ {div.max() * 100:.1f} cm")
        print(f"  divergence growth rate ........ "
              f"{growth_rate_cm_per_m(d, div):.2f} cm per metre")
        marks = error_at_marks(d, div, DISTANCE_MARKS_M)
        print("-" * 68)
        print(f"  {'distance':<12}{'fused vs raw divergence':>28}")
        for mark in DISTANCE_MARKS_M:
            value = marks[mark]
            txt = f"{value * 100:.1f} cm" if value is not None else "not reached"
            print(f"  {int(mark):>5} m     {txt:>28}")
        print(bar)
        return

    d = aligned['distance_m']
    err_f = aligned['err_fused_m']
    err_r = aligned['err_raw_m']

    print(f"  reference: {args.aruco_topic}     duration: {duration:.1f} s")
    print(f"  distance travelled (reference): {d[-1]:.2f} m      "
          f"aligned samples: {d.size}")
    print(f"  matched: fused {aligned['n_fused_matched']}/{d.size}, "
          f"raw odom {aligned['n_raw_matched']}/{d.size} "
          f"(within {args.max_dt * 1000:.0f} ms)")
    print(bar)
    print(f"  {'distance':<12}{'fused err':>14}{'raw odom err':>18}")
    marks_f = error_at_marks(d, err_f, DISTANCE_MARKS_M)
    marks_r = error_at_marks(d, err_r, DISTANCE_MARKS_M)
    for mark in DISTANCE_MARKS_M:
        vf, vr = marks_f[mark], marks_r[mark]
        tf = f"{vf * 100:.1f} cm" if vf is not None and np.isfinite(vf) else "not reached"
        tr = f"{vr * 100:.1f} cm" if vr is not None and np.isfinite(vr) else "not reached"
        print(f"  {int(mark):>5} m     {tf:>14}{tr:>18}")

    print("-" * 68)
    print("  ERROR GROWTH RATE")
    k_f = growth_rate_cm_per_m(d, err_f)
    k_r = growth_rate_cm_per_m(d, err_r)
    print(f"    fused  ({args.pose_topic}) ... {k_f:.2f} cm per metre")
    print(f"    raw    ({args.odom_topic}) ... {k_r:.2f} cm per metre")
    if np.isfinite(k_f) and np.isfinite(k_r) and k_f > 1e-6:
        print(f"    the correction reduces the growth rate by a factor of "
              f"{k_r / k_f:.1f}x")

    print("-" * 68)
    print(f"  ARUCO SNAP CORRECTIONS (jump > {args.snap_threshold * 100:.1f} cm "
          f"in {args.pose_topic})")
    print(f"    snaps detected ................ {len(snaps)}")
    if snaps:
        if d[-1] > 1e-6:
            print(f"    snaps per metre ............... {len(snaps) / d[-1]:.2f}")
        gaps = [snaps[i + 1][0] - snaps[i][0] for i in range(len(snaps) - 1)]
        if gaps:
            m, s, _ = mean_std(gaps)
            print(f"    time between snaps ............ {m:.1f} +/- {s:.1f} s "
                  f"(min {min(gaps):.1f}, max {max(gaps):.1f})")
        else:
            print("    time between snaps ............ n/a (only one snap)")
        print(f"    largest jump .................. "
              f"{max(j for _t, j in snaps) * 100:.1f} cm")
        print(f"    first / last snap at t ........ {snaps[0][0]:.1f} s / "
              f"{snaps[-1][0]:.1f} s")
    else:
        print("    No snaps fired.  Either drift stayed under "
              f"{DRIFT_SNAP_THRESHOLD_M * 100:.0f} cm for the whole session,")
        print("    or the force-reseed path was not running (localization_bridge "
              "must be")
        print("    live and /ekf_global/set_pose available for it to fire at all).")

    print(bar)
    if args.no_aruco:
        print("  Session was tagged UNCORRECTED (--no-aruco).  The fused error")
        print("  above should grow roughly like the raw odometry error; if it")
        print("  stays flat instead, the /aruco/odom feed into ekf_global was")
        print("  still live and the run is not actually uncorrected.")
    else:
        print(f"  Session was tagged CORRECTED.  Note that "
              f"DRIFT_SNAP_THRESHOLD_METERS = {DRIFT_SNAP_THRESHOLD_M} caps the")
        print("  fused error at roughly 5 cm by construction, so a small fused")
        print("  number here is an architectural guarantee, not evidence that")
        print("  the chassis tracks well.  The raw column is the honest measure")
        print("  of the chassis.")
    print(bar)


# ==========================================================================
# CSV and plot
# ==========================================================================
def write_csv(path, aligned, fallback, args, session_label):
    if not path:
        return
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)

    with open(path, 'w', newline='') as handle:
        writer = csv.writer(handle)
        if aligned is not None:
            writer.writerow(['session', 'pose_topic', 'odom_topic', 'aruco_topic',
                             't_s', 'distance_m',
                             'ref_x_m', 'ref_y_m', 'fused_x_m', 'fused_y_m',
                             'raw_x_m', 'raw_y_m',
                             'err_fused_m', 'err_raw_m'])
            for i in range(aligned['t'].size):
                writer.writerow([
                    session_label, args.pose_topic, args.odom_topic, args.aruco_topic,
                    f"{aligned['t'][i]:.4f}", f"{aligned['distance_m'][i]:.5f}",
                    f"{aligned['ref_xy'][i, 0]:.5f}", f"{aligned['ref_xy'][i, 1]:.5f}",
                    f"{aligned['fused_xy'][i, 0]:.5f}", f"{aligned['fused_xy'][i, 1]:.5f}",
                    f"{aligned['raw_xy'][i, 0]:.5f}", f"{aligned['raw_xy'][i, 1]:.5f}",
                    f"{aligned['err_fused_m'][i]:.5f}", f"{aligned['err_raw_m'][i]:.5f}",
                ])
        elif fallback is not None:
            writer.writerow(['session', 'pose_topic', 'odom_topic', 't_s',
                             'distance_m', 'fused_x_m', 'fused_y_m',
                             'raw_x_m', 'raw_y_m', 'divergence_m'])
            for i in range(fallback['t'].size):
                writer.writerow([
                    session_label, args.pose_topic, args.odom_topic,
                    f"{fallback['t'][i]:.4f}", f"{fallback['distance_m'][i]:.5f}",
                    f"{fallback['fused_xy'][i, 0]:.5f}", f"{fallback['fused_xy'][i, 1]:.5f}",
                    f"{fallback['raw_xy'][i, 0]:.5f}", f"{fallback['raw_xy'][i, 1]:.5f}",
                    f"{fallback['divergence_m'][i]:.5f}",
                ])
        else:
            return
    print(f"\n  CSV: {os.path.abspath(path)}")


def write_summary_csv(path, args, aligned, snaps, duration, session_label):
    if not path or aligned is None:
        return
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    fields = ['timestamp_iso', 'session', 'duration_s', 'distance_m',
              'growth_cm_per_m_fused', 'growth_cm_per_m_raw',
              'err_1m_fused_cm', 'err_1m_raw_cm', 'err_2m_fused_cm',
              'err_2m_raw_cm', 'err_5m_fused_cm', 'err_5m_raw_cm',
              'err_10m_fused_cm', 'err_10m_raw_cm', 'snap_count',
              'snaps_per_m', 'mean_s_between_snaps', 'pose_topic',
              'odom_topic', 'aruco_topic']
    d = aligned['distance_m']
    mf = error_at_marks(d, aligned['err_fused_m'], DISTANCE_MARKS_M)
    mr = error_at_marks(d, aligned['err_raw_m'], DISTANCE_MARKS_M)
    gaps = [snaps[i + 1][0] - snaps[i][0] for i in range(len(snaps) - 1)]
    gap_mean, _, _ = mean_std(gaps)

    def cm(value):
        return '' if value is None or not np.isfinite(value) else f"{value * 100:.3f}"

    row = {
        'timestamp_iso': datetime.datetime.now().isoformat(timespec='seconds'),
        'session': session_label,
        'duration_s': f"{duration:.2f}",
        'distance_m': f"{d[-1]:.4f}",
        'growth_cm_per_m_fused': f"{growth_rate_cm_per_m(d, aligned['err_fused_m']):.4f}",
        'growth_cm_per_m_raw': f"{growth_rate_cm_per_m(d, aligned['err_raw_m']):.4f}",
        'snap_count': len(snaps),
        'snaps_per_m': f"{len(snaps) / d[-1]:.4f}" if d[-1] > 1e-6 else '',
        'mean_s_between_snaps': '' if math.isnan(gap_mean) else f"{gap_mean:.3f}",
        'pose_topic': args.pose_topic,
        'odom_topic': args.odom_topic,
        'aruco_topic': args.aruco_topic,
    }
    for mark in DISTANCE_MARKS_M:
        row[f'err_{int(mark)}m_fused_cm'] = cm(mf[mark])
        row[f'err_{int(mark)}m_raw_cm'] = cm(mr[mark])

    new_file = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        if new_file:
            writer.writeheader()
        writer.writerow(row)
    print(f"  summary CSV: {os.path.abspath(path)}")


def make_plot(args, aligned, fallback, snaps, session_label):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n  --plot requested but matplotlib is not installed - skipping "
              "the plot.")
        print("  The CSV holds everything needed to plot elsewhere.  To install:")
        print("      pip3 install matplotlib      # or: sudo apt install "
              "python3-matplotlib")
        return

    out = args.plot_file or (
        (os.path.splitext(args.csv)[0] + '.png') if args.csv
        else f'cumulative_error_{session_label}.png')

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    if aligned is not None:
        d = aligned['distance_m']
        axes[0].plot(d, aligned['err_raw_m'] * 100, label=f'raw odom ({args.odom_topic})')
        axes[0].plot(d, aligned['err_fused_m'] * 100, label=f'fused ({args.pose_topic})')
        axes[0].axhline(DRIFT_SNAP_THRESHOLD_M * 100, linestyle='--', linewidth=1,
                        label=f'snap threshold {DRIFT_SNAP_THRESHOLD_M * 100:.0f} cm')
        axes[0].set_xlabel('distance travelled [m]')
        axes[0].set_ylabel('position error vs ArUco [cm]')
        axes[0].set_title(f'error vs distance ({session_label})')
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)

        axes[1].plot(aligned['ref_xy'][:, 0], aligned['ref_xy'][:, 1],
                     label='ArUco reference')
        axes[1].plot(aligned['fused_xy'][:, 0], aligned['fused_xy'][:, 1],
                     label='fused')
        axes[1].plot(aligned['raw_xy'][:, 0], aligned['raw_xy'][:, 1],
                     label='raw odom')
        for t_snap, _jump in snaps:
            idx = int(np.argmin(np.abs(aligned['t'] - t_snap)))
            axes[1].plot(aligned['fused_xy'][idx, 0], aligned['fused_xy'][idx, 1],
                         marker='x', markersize=7, linestyle='none', color='red')
        axes[1].set_title('paths, relative to first aligned sample '
                          '(red x = snap)')
    elif fallback is not None:
        d = fallback['distance_m']
        axes[0].plot(d, fallback['divergence_m'] * 100, label='fused vs raw')
        axes[0].set_xlabel('distance travelled [m]')
        axes[0].set_ylabel('divergence [cm]')
        axes[0].set_title(f'divergence vs distance ({session_label}) - '
                          'NO GROUND TRUTH')
        axes[0].legend(fontsize=8)
        axes[0].grid(alpha=0.3)
        axes[1].plot(fallback['fused_xy'][:, 0], fallback['fused_xy'][:, 1],
                     label='fused')
        axes[1].plot(fallback['raw_xy'][:, 0], fallback['raw_xy'][:, 1],
                     label='raw odom')
        axes[1].set_title('paths, relative to first aligned sample')
    else:
        print("\n  nothing to plot")
        plt.close(fig)
        return

    axes[1].set_xlabel('x [m]')
    axes[1].set_ylabel('y [m]')
    axes[1].axis('equal')
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    directory = os.path.dirname(os.path.abspath(out))
    if directory:
        os.makedirs(directory, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"  plot: {os.path.abspath(out)}")


# ==========================================================================
# ROS recording
# ==========================================================================
def record(args):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from nav_msgs.msg import Odometry
    except ImportError as exc:
        print(f"\nERROR: the ROS 2 Python environment is not available ({exc}).",
              file=sys.stderr)
        print("This script needs a sourced ROS 2 Humble workspace:\n"
              "    source /opt/ros/humble/setup.bash\n"
              "    source ~/ros2_ws/install/setup.bash\n"
              "Then re-run.", file=sys.stderr)
        return None, None, 3

    class ErrorRecorder(Node):
        """Subscribe-only.  This node creates no publishers, by design."""

        def __init__(self):
            super().__init__('measure_cumulative_error')
            self.fused, self.odom, self.aruco = [], [], []
            self._t0 = time.monotonic()
            qos = QoSProfile(depth=100,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(Odometry, args.pose_topic,
                                     lambda m: self._store(m, self.fused), qos)
            self.create_subscription(Odometry, args.odom_topic,
                                     lambda m: self._store(m, self.odom), qos)
            self.create_subscription(Odometry, args.aruco_topic,
                                     lambda m: self._store(m, self.aruco), qos)

        def _store(self, msg, sink):
            stamp = msg.header.stamp
            t = stamp.sec + stamp.nanosec * 1e-9
            if t <= 0.0:                       # unstamped publisher - fall back
                t = time.monotonic() - self._t0
            p = msg.pose.pose
            q = p.orientation
            sink.append([t, p.position.x, p.position.y,
                         yaw_from_quaternion(q.x, q.y, q.z, q.w)])

    rclpy.init()
    node = ErrorRecorder()
    install_signal_handlers()

    print(f"\n  recording for up to {args.duration:.0f} s "
          "(Ctrl-C stops early and still reports)")
    print("  DRIVE THE ROBOT NOW - in another terminal.  This script does not "
          "move it.")
    print()

    start = time.monotonic()
    last_report = start
    try:
        while (time.monotonic() - start) < args.duration and not _STOP_REQUESTED:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_report >= args.progress_interval:
                last_report = now
                print(f"    t={now - start:6.1f}s   "
                      f"fused={len(node.fused):5d}  "
                      f"odom={len(node.odom):5d}  "
                      f"aruco={len(node.aruco):5d}")
    finally:
        duration = time.monotonic() - start
        tracks = (list(node.fused), list(node.odom), list(node.aruco))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print()
    for label, topic, track in (('fused', args.pose_topic, tracks[0]),
                                ('raw odom', args.odom_topic, tracks[1]),
                                ('aruco', args.aruco_topic, tracks[2])):
        state = f"{len(track)} samples" if track else "NO MESSAGES RECEIVED"
        print(f"  {label:<9} {topic:<32} {state}")
    return tracks, duration, 0


# ==========================================================================
def build_parser():
    p = argparse.ArgumentParser(
        prog='measure_cumulative_error.py',
        description='READ-ONLY. Cumulative position error vs distance, with '
                    'and without the ArUco correction. Never publishes to '
                    '/cmd_vel or anywhere else.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Drive the robot from another terminal while this records.')

    g = p.add_argument_group('topics (all nav_msgs/Odometry)')
    g.add_argument('--pose-topic', default=DEFAULT_POSE_TOPIC,
                   help=f'fused, ArUco-corrected pose (default {DEFAULT_POSE_TOPIC})')
    g.add_argument('--odom-topic', default=DEFAULT_ODOM_TOPIC,
                   help='raw wheel odometry - the UNCORRECTED signal.  On this '
                        'stack the controller may publish it as '
                        '/ackermann_steering_controller/odometry; check '
                        f'`ros2 topic list` (default {DEFAULT_ODOM_TOPIC})')
    g.add_argument('--aruco-topic', default=DEFAULT_ARUCO_TOPIC,
                   help=f'overhead ArUco reference (default {DEFAULT_ARUCO_TOPIC}). '
                        'For an uncorrected run point this at the untapped '
                        'truth topic, e.g. /aruco/truth')

    g = p.add_argument_group('session')
    g.add_argument('--duration', type=float, default=300.0,
                   help='recording length, seconds (default 300)')
    g.add_argument('--no-aruco', action='store_true',
                   help='tag this session as UNCORRECTED.  Does NOT disable '
                        'anything on the robot - this script cannot publish.  '
                        'See the docstring for how to actually remove the '
                        'correction before running with this flag.')
    g.add_argument('--max-dt', type=float, default=0.1,
                   help='timestamp alignment tolerance, seconds (default 0.1)')
    g.add_argument('--min-step', type=float, default=0.005,
                   help='deadband for the travelled-distance integral, metres. '
                        'Stops ArUco pixel jitter counting as travel '
                        '(default 0.005)')

    g = p.add_argument_group('snap detection')
    g.add_argument('--snap-threshold', type=float, default=DRIFT_SNAP_THRESHOLD_M,
                   help=f'jump size that counts as a reseed, metres (default '
                        f'{DRIFT_SNAP_THRESHOLD_M}, matching '
                        'DRIFT_SNAP_THRESHOLD_METERS)')
    g.add_argument('--max-speed', type=float, default=1.0,
                   help='implied speed above which a jump is a reseed and not '
                        'real motion, m/s (default 1.0)')

    g = p.add_argument_group('output')
    g.add_argument('--csv', default=None,
                   help='write the aligned per-sample time series here')
    g.add_argument('--summary-csv', default=None,
                   help='append one summary row per session here')
    g.add_argument('--plot', action='store_true',
                   help='save a PNG of error-vs-distance and the paths; '
                        'skipped gracefully if matplotlib is missing')
    g.add_argument('--plot-file', default=None,
                   help='PNG path (default: alongside --csv)')
    g.add_argument('--progress-interval', type=float, default=10.0,
                   help='seconds between progress lines (default 10)')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.duration <= 0:
        print("ERROR: --duration must be positive.", file=sys.stderr)
        return 2

    session_label = 'uncorrected' if args.no_aruco else 'corrected'

    print("=" * 68)
    print("  measure_cumulative_error.py - READ-ONLY")
    print("  This script NEVER publishes to /cmd_vel.  It creates no")
    print("  publishers at all and cannot move the robot.  Drive it yourself.")
    print("=" * 68)
    print(f"  session ......... {session_label}")
    print(f"  fused pose ...... {args.pose_topic}")
    print(f"  raw odometry .... {args.odom_topic}")
    print(f"  ArUco reference . {args.aruco_topic}")
    print(f"  duration ........ {args.duration:.0f} s")
    print("=" * 68)

    if args.no_aruco:
        print("""
  --no-aruco: this flag ONLY tags the output.  It does not and cannot switch
  the correction off, because this script does not publish and does not call
  any service.  For the result to mean anything you must have already removed
  the ArUco feed into ekf_global BEFORE starting, by either:

    (a) not launching the node that republishes onto /aruco/odom (the tactical
        brain's localization_bridge), while still running the overhead tracker
        on a separate truth topic, or
    (b) remapping the publisher at its source, e.g.
          ros2 run <pkg> <node> --ros-args -r /aruco/odom:=/aruco/truth

  Then point --aruco-topic at that truth topic.  Do NOT attempt to disable the
  EKF at runtime: a filter whose inputs are toggled mid-session sits in a
  half-converged state and its output is not interpretable.

  Verify before starting:
      ros2 topic info /aruco/odom      # expect 0 publishers for a true
                                       # uncorrected run
""")

    tracks, duration, code = record(args)
    if code != 0:
        return code
    fused, odom, aruco = tracks

    if not fused and not odom:
        print("\n  No pose data at all was received.  Nothing to report.")
        print("  Check the topic names with `ros2 topic list` and that the")
        print("  localization stack is running.")
        return 4

    aligned, fallback = None, None
    if len(aruco) >= 2:
        aligned = align_tracks(fused, odom, aruco, args.max_dt, args.min_step)
    if aligned is None:
        fallback = divergence_only(fused, odom, args.max_dt, args.min_step)
        if fallback is None:
            print("\n  Could not align any samples across sources.")
            print(f"  Try a larger --max-dt (currently {args.max_dt}s), and")
            print("  confirm the publishers are stamping their headers.")
            return 4

    snaps = detect_snaps(to_relative_frame(fused), args.snap_threshold,
                         args.max_speed) if len(fused) >= 2 else []

    report(args, aligned, snaps, duration, session_label, fallback)
    write_csv(args.csv, aligned, fallback, args, session_label)
    write_summary_csv(args.summary_csv, args, aligned, snaps, duration,
                      session_label)
    if args.plot:
        make_plot(args, aligned, fallback, snaps, session_label)

    print()
    print("  To complete the with/without comparison, run this script once")
    print("  more on the other configuration and diff the two summaries:")
    other = '(drop --no-aruco)' if args.no_aruco else '(add --no-aruco, after removing the /aruco/odom feed)'
    print(f"      {other}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
