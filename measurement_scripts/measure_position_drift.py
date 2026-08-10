#!/usr/bin/env python3
"""
measure_position_drift.py

PURPOSE
=======
Quantify the SIM-TO-REAL GAP in position drift: run the exact same commanded
trajectory in Gazebo and on the physical robot, record the achieved path each
time, and diff the two result files.

The script has two jobs and they are usually done on different machines:

  1. RECORD  - publish a repeatable open-loop pattern on /cmd_vel, record the
               achieved pose from --pose-topic, compare it against the ideal
               path the same commands would trace on a perfect robot, and
               append a summary row to --csv.  Tag it with --mode sim or
               --mode real.
  2. COMPARE - `--compare sim.csv real.csv` loads two prior result files and
               prints the sim-vs-real delta table.  No ROS, no motion, no
               hardware.  THIS TABLE IS THE ACTUAL DELIVERABLE.

Every command is open-loop: the pattern is a fixed sequence of
(linear.x, angular.z, duration) segments, so sim and real receive byte-for-byte
identical inputs and any difference in the achieved path is the sim-to-real
gap.  Nothing here closes the loop on pose - closing it would hide exactly the
error being measured.

METRICS
-------
  final position error   distance from the achieved end point to the ideal end
                         point (both expressed relative to the run's start pose)
  max cross-track error  worst perpendicular distance from the ideal path
  RMS cross-track error  root-mean-square of the same, over all samples
  closure error          for square/figure8 the ideal path returns to the start,
                         so the distance from end back to start IS the drift,
                         with no dependence on the ideal-path model at all.
                         This is the cleanest single number in the script.
  heading error          final heading minus ideal final heading, degrees
  total path length      integrated length of the achieved path

WHY CLOSURE ERROR IS THE ONE TO TRUST
-------------------------------------
Cross-track and final-position error are measured against an *ideal* path
integrated from a unicycle model, so they inherit whatever that model gets
wrong.  Closure error compares the robot against itself: the pattern is closed,
so a perfect robot ends where it started, full stop.  Report closure error
first and use the others as supporting detail.

PATTERNS AND ACKERMANN GEOMETRY
-------------------------------
This is an Ackermann chassis: it CANNOT turn on the spot.  angular.z with
linear.x = 0 produces no rotation, so every "turn" here is an arc driven at
--speed.  The minimum radius follows from the geometry in
hardware/config/ackermann_steering_controller.yaml:

    wheelbase = 0.235 m,  planner steering limit +/- 20 deg
    R_min = wheelbase / tan(20 deg) = 0.235 / 0.36397 = 0.65 m

(The URDF steering joints declare +/- 1.1 rad, which is 63 deg and almost
certainly not the physical servo travel - trust the 20 deg planner model.)
--turn-radius therefore defaults to 0.65 m and the script refuses smaller
values unless --allow-tight is passed.  Note the arena is only about 2.0 m
across (ARENA_SIZE_METERS), so a square with 0.65 m arcs barely fits: the
script prints the ideal path's bounding box and warns if it exceeds
--arena-size.

  straight  one leg of --distance metres
  square    4 x (--side straight, then a 90 deg arc at --turn-radius) - closed
  figure8   one full circle left then one full circle right - closed, and it
            exercises both steering directions, which exposes asymmetric
            steering trim that a square can hide

EXACT RUN COMMAND
=================
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash

  # rehearse - publishes nothing
  python3 measurement_scripts/measure_position_drift.py \
      --pattern square --dry-run

  # in Gazebo
  python3 measurement_scripts/measure_position_drift.py \
      --pattern square --mode sim  --runs 3 \
      --csv results/position_drift_sim.csv --i-am-clear

  # on the real robot, identical trajectory arguments
  python3 measurement_scripts/measure_position_drift.py \
      --pattern square --mode real --runs 3 \
      --csv results/position_drift_real.csv --i-am-clear

  # the deliverable
  python3 measurement_scripts/measure_position_drift.py \
      --compare results/position_drift_sim.csv results/position_drift_real.csv

PREREQUISITES
=============
  * ros2_control + ackermann_steering_controller
  * twist_to_ackermann  (/cmd_vel Twist -> controller TwistStamped reference)
  * localization / ekf_global publishing --pose-topic (default /odometry/global)
  * main_brain STOPPED - it publishes /cmd_vel too and will fight this script
  * sim run: Gazebo up, use_sim_time consistent across the stack
  * real run: clear arena, floor identical between runs, charged battery

Keep the trajectory arguments (--pattern, --speed, --side, --turn-radius,
--distance) IDENTICAL between the sim and real runs or the comparison is
meaningless.  --compare warns when it detects a mismatch.

SAFETY NOTES
============
  *** ON --mode real THIS DRIVES A MOTORISED VEHICLE. ***
  * Motion requires --i-am-clear.  Without it the script prints the plan and
    exits.  --dry-run never publishes anything.
  * A countdown precedes every run.
  * Zero Twist is published on every exit path: completion, exception, SIGINT
    and SIGTERM.  The signal handler publishes the stop before unwinding.
  * A runaway guard aborts if the robot leaves a circle of radius
    --abort-radius (default: ideal path extent + 1.0 m) around the start.
  * Speeds above 0.5 m/s require --allow-fast.
  * square and figure8 sweep a wide area - check the printed bounding box
    against the real arena before pressing go.
  * --compare and --dry-run never touch hardware.

EXPECTED OUTPUT FORMAT
======================
--- ILLUSTRATIVE SAMPLE ONLY.  THE NUMBERS BELOW ARE PLACEHOLDERS SHOWING THE
--- LAYOUT.  THEY ARE NOT MEASUREMENTS.  DO NOT COPY THEM ANYWHERE.

  ==================================================================
   RESULTS  pattern=square  mode=real  runs=3
  ==================================================================
   commanded path length ......  X.XXX m
   achieved path length .......  X.XXX +/- X.XXX m
   CLOSURE ERROR ..............  XX.X +/- X.X cm     <-- headline (closed)
   final position error .......  XX.X +/- X.X cm
   max cross-track error ......  XX.X +/- X.X cm
   RMS cross-track error ......  XX.X +/- X.X cm
   final heading error ........  XX.X +/- X.X deg
  ==================================================================

  --compare output:

  =====================================================================
   SIM vs REAL   pattern=square
  =====================================================================
   metric                        sim        real       delta     delta%
   ---------------------------------------------------------------------
   closure_error_cm            X.XX       XX.XX      +XX.XX     +XXX.X%
   final_position_error_cm     X.XX       XX.XX      +XX.XX     +XXX.X%
   max_cross_track_cm          X.XX       XX.XX      +XX.XX     +XXX.X%
   rms_cross_track_cm          X.XX       XX.XX      +XX.XX     +XXX.X%
   heading_error_deg           X.XX       XX.XX      +XX.XX     +XXX.X%
   path_length_m               X.XXX      X.XXX      +X.XXX      +X.X%
  =====================================================================
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
WHEELBASE_M = 0.235                # ackermann_steering_controller.yaml
PLANNER_STEER_LIMIT_DEG = 20.0     # A_planner steering model
MIN_TURN_RADIUS_M = WHEELBASE_M / math.tan(math.radians(PLANNER_STEER_LIMIT_DEG))
DEFAULT_POSE_TOPIC = '/odometry/global'
DEFAULT_CMD_TOPIC = '/cmd_vel'
DEFAULT_ARENA_SIZE_M = 2.0         # ARENA_SIZE_METERS, main_brain.py
DRIFT_SNAP_THRESHOLD_M = 0.05      # tactical_brain/localization_bridge.py

HARD_SPEED_LIMIT_MPS = 0.5
CLOSED_PATTERNS = ('square', 'figure8')

_STOP_REQUESTED = False
_ACTIVE_NODE = None


# ==========================================================================
# Safety plumbing
# ==========================================================================
def _signal_handler(signum, _frame):
    global _STOP_REQUESTED
    _STOP_REQUESTED = True
    sys.stderr.write(f"\n\n*** {signal.Signals(signum).name} RECEIVED - "
                     "EMERGENCY STOP ***\n")
    sys.stderr.flush()
    if _ACTIVE_NODE is not None:
        try:
            _ACTIVE_NODE.emergency_stop()
        except Exception as exc:                                  # noqa: BLE001
            sys.stderr.write(f"*** STOP PUBLISH FAILED: {exc} ***\n")
            sys.stderr.write("*** CUT POWER AT THE BATTERY NOW ***\n")
            sys.stderr.flush()


def install_signal_handlers():
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)


def countdown(seconds, label):
    if seconds <= 0:
        return
    print(f"  {label} in ", end='', flush=True)
    for i in range(int(seconds), 0, -1):
        print(f"{i}... ", end='', flush=True)
        time.sleep(1.0)
    print("GO", flush=True)


# ==========================================================================
# Pattern generation (pure math, no ROS)
# ==========================================================================
def build_segments(args):
    """Return [(linear_x, angular_z, duration_s, label), ...].

    Ackermann cannot spin in place, so every turn is an arc driven at --speed.
    omega = v / R for a circular arc of radius R.
    """
    v = args.speed
    omega = v / args.turn_radius
    quarter_turn_s = (math.pi / 2.0) / omega
    full_circle_s = (2.0 * math.pi) / omega

    if args.pattern == 'straight':
        return [(v, 0.0, args.distance / v, 'straight')]

    if args.pattern == 'square':
        segments = []
        for i in range(4):
            segments.append((v, 0.0, args.side / v, f'side{i + 1}'))
            segments.append((v, omega, quarter_turn_s, f'turn{i + 1}'))
        return segments

    if args.pattern == 'figure8':
        return [(v, +omega, full_circle_s, 'loop_left'),
                (v, -omega, full_circle_s, 'loop_right')]

    raise ValueError(f'unknown pattern {args.pattern!r}')


def integrate_ideal(segments, dt=0.02):
    """Integrate the commanded segments through a unicycle model.

    Returns (N, 3) array of [x, y, yaw] in the START frame (start at origin,
    heading +x).  The ideal path is the trajectory a perfect robot that
    executed the commands exactly would trace.
    """
    x = y = th = 0.0
    pts = [(x, y, th)]
    for v, omega, duration, _label in segments:
        n = max(int(round(duration / dt)), 1)
        step = duration / n
        for _ in range(n):
            x += v * math.cos(th) * step
            y += v * math.sin(th) * step
            th += omega * step
            pts.append((x, y, th))
    return np.asarray(pts, dtype=float)


def polyline_length(pts_xy):
    if pts_xy.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts_xy, axis=0), axis=1)))


def resample_polyline(pts_xy, max_points=2000):
    """Thin an ideal path down so the cross-track distance matrix stays small."""
    if pts_xy.shape[0] <= max_points:
        return pts_xy
    idx = np.linspace(0, pts_xy.shape[0] - 1, max_points).astype(int)
    return pts_xy[idx]


def cross_track_distances(points, polyline, chunk=256):
    """Perpendicular distance from each point to the nearest polyline segment.

    Chunked so a long run on a Pi 5 does not allocate a huge N x M matrix.
    """
    if polyline.shape[0] < 2 or points.shape[0] == 0:
        return np.zeros(points.shape[0])

    a = polyline[:-1]
    ab = polyline[1:] - a
    denom = np.sum(ab * ab, axis=1)
    denom[denom < 1e-18] = 1e-18

    out = np.empty(points.shape[0], dtype=float)
    for start in range(0, points.shape[0], chunk):
        block = points[start:start + chunk]
        ap = block[:, None, :] - a[None, :, :]
        t = np.clip(np.sum(ap * ab[None, :, :], axis=2) / denom[None, :], 0.0, 1.0)
        proj = a[None, :, :] + t[..., None] * ab[None, :, :]
        d = np.linalg.norm(block[:, None, :] - proj, axis=2)
        out[start:start + chunk] = d.min(axis=1)
    return out


def yaw_from_quaternion(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def mean_std(values):
    v = np.asarray([x for x in values
                    if x is not None and not (isinstance(x, float) and math.isnan(x))],
                   dtype=float)
    if v.size == 0:
        return float('nan'), float('nan'), 0
    if v.size == 1:
        return float(v[0]), 0.0, 1
    return float(np.mean(v)), float(np.std(v, ddof=1)), int(v.size)


# ==========================================================================
# Metrics
# ==========================================================================
def run_metrics(samples, ideal, pattern):
    """samples: (N, 4) [t, x, y, yaw] world frame.  ideal: (M, 3) start frame."""
    arr = np.asarray(samples, dtype=float)
    if arr.shape[0] < 2:
        return None

    x0, y0, yaw0 = arr[0, 1], arr[0, 2], arr[0, 3]
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    rot = np.array([[c, -s], [s, c]])

    # Achieved path expressed in the run's own start frame, so it lines up with
    # the ideal path without depending on where in the arena the run happened.
    rel = (arr[:, 1:3] - np.array([x0, y0])) @ rot.T
    yaw_rel = np.array([wrap_angle(a - yaw0) for a in arr[:, 3]])

    ideal_xy = resample_polyline(ideal[:, :2])
    ct = cross_track_distances(rel, ideal_xy)

    final_err = float(np.linalg.norm(rel[-1] - ideal[-1, :2]))
    heading_err = math.degrees(wrap_angle(yaw_rel[-1] - ideal[-1, 2]))
    closure = float(np.linalg.norm(rel[-1])) if pattern in CLOSED_PATTERNS else float('nan')

    return {
        'final_position_error_m': final_err,
        'max_cross_track_m': float(np.max(ct)),
        'rms_cross_track_m': float(np.sqrt(np.mean(ct ** 2))),
        'closure_error_m': closure,
        'heading_error_deg': heading_err,
        'path_length_m': polyline_length(rel),
        'ideal_path_length_m': polyline_length(ideal[:, :2]),
        'n_samples': int(arr.shape[0]),
        'rel_path': rel,
    }


# ==========================================================================
# CSV
# ==========================================================================
CSV_FIELDS = [
    'timestamp_iso', 'mode', 'pattern', 'run_index', 'robot_id', 'pose_topic',
    'speed_mps', 'distance_m', 'side_m', 'turn_radius_m',
    'commanded_duration_s', 'ideal_path_length_m', 'path_length_m',
    'closure_error_cm', 'final_position_error_cm', 'max_cross_track_cm',
    'rms_cross_track_cm', 'heading_error_deg', 'n_samples', 'aborted', 'notes',
]

# Metrics --compare diffs, in report order.  (key, decimals, lower_is_better)
COMPARE_METRICS = [
    ('closure_error_cm', 2, True),
    ('final_position_error_cm', 2, True),
    ('max_cross_track_cm', 2, True),
    ('rms_cross_track_cm', 2, True),
    ('heading_error_deg', 2, True),
    ('path_length_m', 3, None),
]

# Trajectory arguments that must match for a comparison to mean anything.
TRAJECTORY_KEYS = ('pattern', 'speed_mps', 'distance_m', 'side_m', 'turn_radius_m')


def append_csv(path, rows):
    if not path or not rows:
        return
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    new_file = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction='ignore')
        if new_file:
            writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print(f"\n  CSV: appended {len(rows)} row(s) -> {os.path.abspath(path)}")


def save_path_csv(path, samples, run_index, pattern, mode):
    if not path:
        return
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    new_file = not os.path.exists(path) or os.path.getsize(path) == 0
    fields = ['mode', 'pattern', 'run_index', 't_s', 'x_m', 'y_m', 'yaw_rad']
    with open(path, 'a', newline='') as handle:
        writer = csv.writer(handle)
        if new_file:
            writer.writerow(fields)
        for t, x, y, yaw in samples:
            writer.writerow([mode, pattern, run_index,
                             f"{t:.4f}", f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}"])


# ==========================================================================
# --compare : no ROS, no hardware
# ==========================================================================
def load_results(path):
    if not os.path.exists(path):
        print(f"ERROR: no such file: {path}", file=sys.stderr)
        return None
    with open(path, newline='') as handle:
        rows = [r for r in csv.DictReader(handle)
                if (r.get('aborted') or 'false').lower() != 'true']
    if not rows:
        print(f"ERROR: {path} has no completed (non-aborted) runs.", file=sys.stderr)
        return None
    return rows


def _fnum(row, key):
    try:
        value = float(row.get(key, ''))
    except (TypeError, ValueError):
        return float('nan')
    return value


def run_compare(path_a, path_b):
    rows_a = load_results(path_a)
    rows_b = load_results(path_b)
    if rows_a is None or rows_b is None:
        return 2

    label_a = (rows_a[0].get('mode') or os.path.basename(path_a))
    label_b = (rows_b[0].get('mode') or os.path.basename(path_b))
    if label_a == label_b:
        label_a = f"{label_a}(A)"
        label_b = f"{label_b}(B)"

    patterns = sorted({r.get('pattern', '?') for r in rows_a}
                      & {r.get('pattern', '?') for r in rows_b})
    only_a = sorted({r.get('pattern', '?') for r in rows_a} - set(patterns))
    only_b = sorted({r.get('pattern', '?') for r in rows_b} - set(patterns))
    if only_a:
        print(f"  note: patterns only in {path_a}: {', '.join(only_a)}")
    if only_b:
        print(f"  note: patterns only in {path_b}: {', '.join(only_b)}")
    if not patterns:
        print("ERROR: the two files share no pattern - nothing to compare.",
              file=sys.stderr)
        return 2

    for pattern in patterns:
        sub_a = [r for r in rows_a if r.get('pattern') == pattern]
        sub_b = [r for r in rows_b if r.get('pattern') == pattern]

        mismatches = []
        for key in TRAJECTORY_KEYS:
            va = {(r.get(key) or '') for r in sub_a}
            vb = {(r.get(key) or '') for r in sub_b}
            if va != vb:
                mismatches.append(f"{key}: {sorted(va)} vs {sorted(vb)}")

        print()
        print("=" * 70)
        print(f"  {label_a.upper()} vs {label_b.upper()}   pattern={pattern}")
        print(f"  {label_a}: {len(sub_a)} run(s) from {os.path.basename(path_a)}")
        print(f"  {label_b}: {len(sub_b)} run(s) from {os.path.basename(path_b)}")
        print("=" * 70)
        if mismatches:
            print("  !! TRAJECTORY ARGUMENTS DIFFER - the comparison below is "
                  "NOT apples to apples:")
            for line in mismatches:
                print(f"     - {line}")
            print("  !! Re-run both sides with identical trajectory arguments.")
            print("-" * 70)

        print(f"  {'metric':<28}{label_a:>11}{label_b:>11}{'delta':>11}{'delta%':>10}")
        print("  " + "-" * 68)
        for key, decimals, lower_better in COMPARE_METRICS:
            ma, _, na = mean_std([_fnum(r, key) for r in sub_a])
            mb, _, nb = mean_std([_fnum(r, key) for r in sub_b])
            if na == 0 or nb == 0 or math.isnan(ma) or math.isnan(mb):
                print(f"  {key:<28}{'n/a':>11}{'n/a':>11}{'':>11}{'':>10}")
                continue
            delta = mb - ma
            pct = (delta / ma * 100.0) if abs(ma) > 1e-9 else float('nan')
            pct_txt = f"{pct:+.1f}%" if not math.isnan(pct) else "n/a"
            marker = ''
            if lower_better and abs(ma) > 1e-9 and delta > 0:
                marker = '  <-- worse on ' + label_b
            print(f"  {key:<28}{ma:>11.{decimals}f}{mb:>11.{decimals}f}"
                  f"{delta:>+11.{decimals}f}{pct_txt:>10}{marker}")

        print("  " + "-" * 68)
        ma, _, _ = mean_std([_fnum(r, 'closure_error_cm') for r in sub_a])
        mb, _, _ = mean_std([_fnum(r, 'closure_error_cm') for r in sub_b])
        if not math.isnan(ma) and not math.isnan(mb):
            print(f"  SIM-TO-REAL GAP (closure error): "
                  f"{mb - ma:+.2f} cm  ({label_b} minus {label_a})")
            if mb > DRIFT_SNAP_THRESHOLD_M * 100.0:
                print(f"  {label_b} closure error exceeds "
                      f"DRIFT_SNAP_THRESHOLD_METERS "
                      f"({DRIFT_SNAP_THRESHOLD_M * 100:.0f} cm): the ArUco "
                      "force-reseed would have")
                print("  fired during this trajectory, so on the real robot "
                      "this drift is being")
                print("  papered over by the correction rather than avoided.")
        print("=" * 70)
    return 0


# ==========================================================================
# ROS node
# ==========================================================================
def build_node_class(Node, Odometry, Twist, QoSProfile,
                     ReliabilityPolicy, HistoryPolicy):

    class PathRecorder(Node):
        def __init__(self, args):
            super().__init__('measure_position_drift')
            self.args = args
            self.dry_run = args.dry_run
            self.samples = []
            self.recording = False
            self._t0 = time.monotonic()
            self._cmd_count = 0

            qos = QoSProfile(depth=50,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
            self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
            self.create_subscription(Odometry, args.pose_topic, self._pose_cb, qos)

        def now(self):
            return time.monotonic() - self._t0

        def _pose_cb(self, msg):
            if not self.recording:
                return
            p = msg.pose.pose
            q = p.orientation
            self.samples.append([self.now(), p.position.x, p.position.y,
                                 yaw_from_quaternion(q.x, q.y, q.z, q.w)])

        def publish_cmd(self, linear_x, angular_z):
            self._cmd_count += 1
            if self.dry_run:
                return
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)

        def emergency_stop(self, repeats=12, interval=0.02):
            if self.dry_run:
                print("  [dry-run] would publish zero Twist (emergency stop)")
                return
            stop = Twist()
            for _ in range(repeats):
                self.cmd_pub.publish(stop)
                time.sleep(interval)

    return PathRecorder


def drive_pattern(rclpy, node, args, segments, ideal, run_index):
    print(f"\n  --- run {run_index} / {args.runs}  pattern={args.pattern} ---")

    node.samples.clear()
    node.recording = True
    settle_end = time.monotonic() + max(args.settle, 0.2)
    while time.monotonic() < settle_end and not _STOP_REQUESTED:
        rclpy.spin_once(node, timeout_sec=0.02)

    if not node.samples:
        print(f"      !! no pose samples on {args.pose_topic} - is the EKF running?")
        node.recording = False
        return None, True

    start_xy = np.array(node.samples[-1][1:3])
    node.samples = [node.samples[-1]]

    countdown(args.countdown, "MOVING")

    abort_radius = args.abort_radius
    period = 1.0 / max(args.publish_rate, 1.0)
    aborted = False

    try:
        for v, omega, duration, label in segments:
            if _STOP_REQUESTED or aborted:
                break
            print(f"      segment {label:<8} linear.x={v:+.3f} "
                  f"angular.z={omega:+.3f} for {duration:.2f}s"
                  + ("   [dry-run: not published]" if args.dry_run else ""))
            seg_start = time.monotonic()
            next_pub = seg_start
            while time.monotonic() - seg_start < duration:
                if _STOP_REQUESTED:
                    aborted = True
                    print("      !! stop requested - aborting run")
                    break
                now = time.monotonic()
                if now >= next_pub:
                    node.publish_cmd(v, omega)
                    next_pub = now + period
                rclpy.spin_once(node, timeout_sec=0.01)

                if node.samples:
                    dist = float(np.linalg.norm(
                        np.array(node.samples[-1][1:3]) - start_xy))
                    if dist > abort_radius:
                        aborted = True
                        print(f"      !! RUNAWAY GUARD: {dist:.2f} m from start "
                              f"> {abort_radius:.2f} m - stopping")
                        break
    finally:
        node.publish_cmd(0.0, 0.0)
        node.emergency_stop(repeats=6)

    coast_end = time.monotonic() + max(args.settle, 0.5)
    while time.monotonic() < coast_end:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.recording = False

    if args.dry_run:
        print(f"      [dry-run] {node._cmd_count} Twist messages suppressed "
              "this session")

    metrics = run_metrics(node.samples, ideal, args.pattern)
    if metrics is None:
        print("      !! fewer than 2 pose samples - run discarded")
        return None, True

    print(f"      closure {metrics['closure_error_m'] * 100:6.1f} cm | "
          f"final {metrics['final_position_error_m'] * 100:6.1f} cm | "
          f"max XTE {metrics['max_cross_track_m'] * 100:6.1f} cm | "
          f"heading {metrics['heading_error_deg']:+6.1f} deg")

    if args.save_path:
        save_path_csv(args.save_path, node.samples, run_index,
                      args.pattern, args.mode)
    return metrics, aborted


def report(args, runs):
    if not runs:
        print("\n  no usable runs - nothing to report")
        return
    print()
    print("=" * 70)
    print(f"  RESULTS  pattern={args.pattern}  mode={args.mode}  runs={len(runs)}")
    print(f"  pose source: {args.pose_topic}")
    print("=" * 70)

    def col(key):
        return [r[key] for r in runs]

    print(f"  commanded (ideal) path length . {runs[0]['ideal_path_length_m']:.3f} m")
    m, s, _ = mean_std(col('path_length_m'))
    print(f"  achieved path length .......... {m:.3f} +/- {s:.3f} m")

    if args.pattern in CLOSED_PATTERNS:
        m, s, _ = mean_std([v * 100 for v in col('closure_error_m')])
        print(f"  CLOSURE ERROR ................. {m:.2f} +/- {s:.2f} cm"
              "     <-- headline")
    else:
        print("  CLOSURE ERROR ................. n/a (pattern is not closed)")

    m, s, _ = mean_std([v * 100 for v in col('final_position_error_m')])
    print(f"  final position error .......... {m:.2f} +/- {s:.2f} cm")
    m, s, _ = mean_std([v * 100 for v in col('max_cross_track_m')])
    print(f"  max cross-track error ......... {m:.2f} +/- {s:.2f} cm")
    m, s, _ = mean_std([v * 100 for v in col('rms_cross_track_m')])
    print(f"  RMS cross-track error ......... {m:.2f} +/- {s:.2f} cm")
    m, s, _ = mean_std(col('heading_error_deg'))
    print(f"  final heading error ........... {m:+.2f} +/- {s:.2f} deg")
    print("=" * 70)

    if args.pattern in CLOSED_PATTERNS:
        closure_mean, _, _ = mean_std([v * 100 for v in col('closure_error_m')])
        if not math.isnan(closure_mean) and closure_mean > DRIFT_SNAP_THRESHOLD_M * 100:
            print(f"  NOTE: closure error exceeds DRIFT_SNAP_THRESHOLD_METERS "
                  f"({DRIFT_SNAP_THRESHOLD_M * 100:.0f} cm).")
            print("  On the real robot with the overhead camera live, the "
                  "force-reseed in")
            print("  localization_bridge.check_drift would have fired during "
                  "this trajectory,")
            print("  which means the fused pose you just recorded is already "
                  "partly corrected -")
            print("  the physical drift is at least this large, probably larger.")
    print()
    print("  Next: run the same command with --mode "
          f"{'real' if args.mode == 'sim' else 'sim'} on the other platform, "
          "then diff them:")
    print(f"    python3 {os.path.basename(__file__)} --compare "
          "<sim.csv> <real.csv>")


def make_row(args, run_index, metrics, aborted, total_duration):
    def cm(value):
        return '' if (value is None or math.isnan(value)) else f"{value * 100:.3f}"
    return {
        'timestamp_iso': datetime.datetime.now().isoformat(timespec='seconds'),
        'mode': args.mode,
        'pattern': args.pattern,
        'run_index': run_index,
        'robot_id': args.robot_id,
        'pose_topic': args.pose_topic,
        'speed_mps': f"{args.speed:.4f}",
        'distance_m': f"{args.distance:.4f}",
        'side_m': f"{args.side:.4f}",
        'turn_radius_m': f"{args.turn_radius:.4f}",
        'commanded_duration_s': f"{total_duration:.3f}",
        'ideal_path_length_m': f"{metrics['ideal_path_length_m']:.4f}",
        'path_length_m': f"{metrics['path_length_m']:.4f}",
        'closure_error_cm': cm(metrics['closure_error_m']),
        'final_position_error_cm': cm(metrics['final_position_error_m']),
        'max_cross_track_cm': cm(metrics['max_cross_track_m']),
        'rms_cross_track_cm': cm(metrics['rms_cross_track_m']),
        'heading_error_deg': f"{metrics['heading_error_deg']:.3f}",
        'n_samples': metrics['n_samples'],
        'aborted': 'true' if aborted else 'false',
        'notes': 'dry-run' if args.dry_run else '',
    }


def run_record(args, segments, ideal):
    global _ACTIVE_NODE

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
    except ImportError as exc:
        print(f"\nERROR: the ROS 2 Python environment is not available ({exc}).",
              file=sys.stderr)
        print("This script needs a sourced ROS 2 Humble workspace:\n"
              "    source /opt/ros/humble/setup.bash\n"
              "    source ~/ros2_ws/install/setup.bash\n"
              "Then re-run.  (--compare works without ROS.)", file=sys.stderr)
        return 3

    PathRecorder = build_node_class(Node, Odometry, Twist, QoSProfile,
                                    ReliabilityPolicy, HistoryPolicy)
    rclpy.init()
    node = PathRecorder(args)
    _ACTIVE_NODE = node
    install_signal_handlers()

    total_duration = sum(seg[2] for seg in segments)
    rows, runs = [], []
    try:
        print(f"\n  waiting for first message on {args.pose_topic} ...")
        node.recording = True
        deadline = time.monotonic() + args.wait_timeout
        while time.monotonic() < deadline and not node.samples and not _STOP_REQUESTED:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.recording = False
        if not node.samples:
            print(f"  !! nothing on {args.pose_topic} within "
                  f"{args.wait_timeout:.0f}s.  Check `ros2 topic hz "
                  f"{args.pose_topic}`")
            return 4
        print("  pose stream OK")

        for run_index in range(1, args.runs + 1):
            if _STOP_REQUESTED:
                break
            metrics, aborted = drive_pattern(rclpy, node, args, segments,
                                             ideal, run_index)
            if metrics is not None:
                rows.append(make_row(args, run_index, metrics, aborted,
                                     total_duration))
                if not aborted:
                    runs.append(metrics)
            if run_index < args.runs and not _STOP_REQUESTED:
                print(f"      pausing {args.pause:.1f}s - "
                      "return the robot to the start mark")
                time.sleep(args.pause)

        report(args, runs)
    finally:
        try:
            node.emergency_stop()
            print(f"\n  final zero Twist published on {args.cmd_topic}")
        except Exception as exc:                                  # noqa: BLE001
            print(f"\n  !! could not publish final stop: {exc}", file=sys.stderr)
            print("  !! CUT POWER AT THE BATTERY", file=sys.stderr)
        append_csv(args.csv, rows)
        _ACTIVE_NODE = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


# ==========================================================================
def build_parser():
    p = argparse.ArgumentParser(
        prog='measure_position_drift.py',
        description='Sim-to-real position drift over a repeatable open-loop '
                    'trajectory.  DRIVES REAL HARDWARE unless --dry-run or '
                    '--compare.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Record twice (--mode sim, --mode real) with identical '
               'trajectory arguments, then --compare the two CSVs.')

    g = p.add_argument_group('trajectory')
    g.add_argument('--pattern', choices=['straight', 'square', 'figure8'],
                   default='square',
                   help='square and figure8 are closed and give a closure '
                        'error, the cleanest drift signal (default square)')
    g.add_argument('--speed', type=float, default=0.2,
                   help='linear.x for every segment, m/s (default 0.2)')
    g.add_argument('--distance', type=float, default=2.0,
                   help='leg length for --pattern straight, metres (default 2.0)')
    g.add_argument('--side', type=float, default=0.5,
                   help='straight side length for --pattern square, metres '
                        '(default 0.5)')
    g.add_argument('--turn-radius', type=float, default=round(MIN_TURN_RADIUS_M, 2),
                   help=f'arc radius for turns, metres.  Ackermann minimum is '
                        f'wheelbase/tan(20deg) = {MIN_TURN_RADIUS_M:.2f} m '
                        f'(default {MIN_TURN_RADIUS_M:.2f})')
    g.add_argument('--runs', type=int, default=3, help='repeats (default 3)')

    g = p.add_argument_group('topics')
    g.add_argument('--pose-topic', default=DEFAULT_POSE_TOPIC,
                   help=f'nav_msgs/Odometry (default {DEFAULT_POSE_TOPIC})')
    g.add_argument('--cmd-topic', default=DEFAULT_CMD_TOPIC,
                   help=f'geometry_msgs/Twist (default {DEFAULT_CMD_TOPIC})')

    g = p.add_argument_group('output')
    g.add_argument('--mode', choices=['sim', 'real'], default='real',
                   help='written into every row so two runs can be diffed '
                        '(default real)')
    g.add_argument('--robot-id', type=int, default=int(os.environ.get('ROBOT_ID', 3)),
                   help='chassis tag (default $ROBOT_ID or 3)')
    g.add_argument('--csv', default=None, help='append summary rows here')
    g.add_argument('--save-path', default=None,
                   help='also append every raw pose sample to this CSV, for '
                        'plotting the achieved path afterwards')
    g.add_argument('--compare', nargs=2, metavar=('A.csv', 'B.csv'),
                   help='load two prior result files, print the delta table '
                        'and exit.  No ROS, no motion.')

    g = p.add_argument_group('safety')
    g.add_argument('--i-am-clear', action='store_true',
                   help='REQUIRED for motion.  Confirms the arena is clear.')
    g.add_argument('--dry-run', action='store_true',
                   help='publish nothing; print the full segment plan')
    g.add_argument('--allow-fast', action='store_true',
                   help=f'permit --speed above {HARD_SPEED_LIMIT_MPS} m/s')
    g.add_argument('--allow-tight', action='store_true',
                   help=f'permit --turn-radius below the Ackermann minimum '
                        f'({MIN_TURN_RADIUS_M:.2f} m).  The chassis will '
                        'simply fail to follow it.')
    g.add_argument('--abort-radius', type=float, default=None,
                   help='abort if the robot leaves this radius around the '
                        'start, metres (default: ideal path extent + 1.0)')
    g.add_argument('--arena-size', type=float, default=DEFAULT_ARENA_SIZE_M,
                   help=f'arena width for the footprint warning, metres '
                        f'(default {DEFAULT_ARENA_SIZE_M})')
    g.add_argument('--countdown', type=int, default=5,
                   help='seconds of countdown before each run (default 5)')

    g = p.add_argument_group('timing')
    g.add_argument('--publish-rate', type=float, default=20.0,
                   help='/cmd_vel publish rate, Hz (default 20)')
    g.add_argument('--settle', type=float, default=1.0,
                   help='still time before/after each run (default 1.0)')
    g.add_argument('--pause', type=float, default=5.0,
                   help='seconds between runs (default 5.0)')
    g.add_argument('--wait-timeout', type=float, default=15.0,
                   help='seconds to wait for the first pose message (default 15)')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.compare:
        return run_compare(args.compare[0], args.compare[1])

    if args.speed <= 0 or args.runs <= 0 or args.turn_radius <= 0:
        print("ERROR: --speed, --runs and --turn-radius must be positive.",
              file=sys.stderr)
        return 2
    if args.speed > HARD_SPEED_LIMIT_MPS and not args.allow_fast:
        print(f"ERROR: --speed {args.speed} m/s exceeds the "
              f"{HARD_SPEED_LIMIT_MPS} m/s safety limit.  Pass --allow-fast "
              "only with space and a spotter.", file=sys.stderr)
        return 2
    if args.turn_radius < MIN_TURN_RADIUS_M - 1e-9 and not args.allow_tight:
        print(f"ERROR: --turn-radius {args.turn_radius:.2f} m is below the "
              f"Ackermann minimum {MIN_TURN_RADIUS_M:.2f} m "
              f"(wheelbase {WHEELBASE_M} / tan({PLANNER_STEER_LIMIT_DEG:.0f} deg)).",
              file=sys.stderr)
        print("The chassis physically cannot follow it and the recorded "
              "'drift' would just be steering saturation.  Pass --allow-tight "
              "to override.", file=sys.stderr)
        return 2

    segments = build_segments(args)
    ideal = integrate_ideal(segments)
    total_duration = sum(seg[2] for seg in segments)

    extent = float(np.max(np.linalg.norm(ideal[:, :2], axis=1)))
    if args.abort_radius is None:
        args.abort_radius = extent + 1.0

    xs, ys = ideal[:, 0], ideal[:, 1]
    width = float(xs.max() - xs.min())
    height = float(ys.max() - ys.min())

    bar = "=" * 74
    print(bar)
    if args.dry_run:
        print("  DRY RUN - NOTHING WILL BE PUBLISHED TO /cmd_vel")
    elif args.mode == 'real':
        print("  !!  MOTION TEST ON REAL HARDWARE - THE ROBOT WILL DRIVE  !!")
    else:
        print("  SIMULATION RUN - the simulated robot will drive")
    print(bar)
    print(f"  pattern ............... {args.pattern}")
    print(f"  mode .................. {args.mode}")
    print(f"  runs .................. {args.runs}")
    print(f"  speed ................. {args.speed:.3f} m/s")
    print(f"  turn radius ........... {args.turn_radius:.3f} m "
          f"(Ackermann minimum {MIN_TURN_RADIUS_M:.2f} m)")
    print(f"  segments .............. {len(segments)}")
    print(f"  commanded duration .... {total_duration:.1f} s per run")
    print(f"  ideal path length ..... {polyline_length(ideal[:, :2]):.3f} m")
    print(f"  ideal footprint ....... {width:.2f} x {height:.2f} m "
          f"(max {extent:.2f} m from start)")
    print(f"  runaway guard radius .. {args.abort_radius:.2f} m")
    print(f"  command topic ......... {args.cmd_topic}  (geometry_msgs/Twist)")
    print(f"  pose topic ............ {args.pose_topic}")
    if max(width, height) > args.arena_size:
        print()
        print(f"  !! WARNING: the ideal footprint ({width:.2f} x {height:.2f} m) "
              f"is larger than")
        print(f"  !! --arena-size {args.arena_size:.2f} m.  The robot will hit "
              "the arena boundary.")
        print("  !! Reduce --side / --turn-radius, or use --pattern straight.")
    print(bar)
    print("  segment plan:")
    for i, (v, omega, duration, label) in enumerate(segments, start=1):
        radius = (v / omega) if abs(omega) > 1e-9 else float('inf')
        radius_txt = "straight" if math.isinf(radius) else f"R={radius:+.2f} m"
        print(f"    {i:2d}. {label:<9} linear.x={v:+.3f} m/s  "
              f"angular.z={omega:+.4f} rad/s  {duration:6.2f} s   {radius_txt}")
    print(bar)

    if not args.dry_run and not args.i_am_clear:
        print()
        print("*" * 74)
        print("  REFUSING TO MOVE: --i-am-clear was not passed.")
        print()
        print("  Before adding it, confirm ALL of the following:")
        print(f"    [ ] a clear circle of radius {args.abort_radius:.1f} m "
              "around the start point")
        print("    [ ] no people, pets or cables in the arena")
        print("    [ ] the tactical brain (main_brain) is STOPPED - it also "
              "publishes /cmd_vel")
        print("    [ ] the battery disconnect is within arm's reach")
        print("    [ ] you have rehearsed this exact command with --dry-run")
        print()
        print("  Then re-run with --i-am-clear appended.")
        print("*" * 74)
        return 2

    return run_record(args, segments, ideal)


if __name__ == '__main__':
    sys.exit(main())
