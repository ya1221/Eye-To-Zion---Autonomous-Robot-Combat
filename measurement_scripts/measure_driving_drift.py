#!/usr/bin/env python3
"""
measure_driving_drift.py

PURPOSE
=======
Measure how far the robot slides sideways per metre driven ("cm of drift per
metre"), and derive a PER-ROBOT motor calibration constant (`pwm_multiplier`).

The robot is commanded to drive a dead-straight open-loop line
(constant linear.x, angular.z = 0.0 on /cmd_vel) for a fixed *time*
(commanded_distance / commanded_speed) and then stopped.  Because the stop is
time-based and not pose-based, the *commanded* distance and the *measured*
distance are two independent numbers - and their ratio is exactly what the
motor calibration constant has to absorb.

Two different errors are reported and they must not be confused:

  * LATERAL DEVIATION  - how far off the intended straight line the robot
    physically ended up, measured perpendicular to the heading it started
    with.  This is the headline number for the log.
  * DISTANCE RATIO     - commanded metres vs. measured metres along the
    intended direction.  This drives the pwm_multiplier suggestion.

If an independent ground-truth topic is supplied (`--truth-topic`, the
overhead ArUco camera) the script additionally separates *odometry error*
(pose estimate vs. truth) from *true physical drift* (truth vs. the intended
line).  Without it, the two are indistinguishable - `/odometry/global` is
already ArUco-corrected, so on a healthy system it will under-report physical
drift.  See "WHY THE FUSED POSE UNDER-REPORTS DRIFT" below.

WHY THIS MATTERS ON THIS ROBOT
------------------------------
`AutonomousWarfare/ros2_ws/src/hardware/ros2_control.xacro` sets a SINGLE
global `pwm_multiplier` of 1.60 (the code default in motor_driver.cpp is 2.0),
shared by every chassis in the fleet.  Cheap brushed DC motors and gearboxes
vary by 10-20% unit to unit, so one global constant guarantees that most of
the fleet is mis-calibrated.  There is no per-robot calibration file yet -
producing those numbers is the deliverable of this script.

Known contributing drift sources worth noting alongside any result:
  * ackermann_steering_controller.yaml declares rear_wheel_track 0.023 and
    rear_wheels_radius 0.04, while the robot's own URDF
    (robot_description/urdf/wheels.xacro) declares rear_track_width 0.08 and
    rear_wheel_radius 0.05.  These disagree and the discrepancy is
    unexplained; a wrong wheel radius scales odometry linearly and a wrong
    track width biases the yaw rate, so this is a prime suspect for both the
    distance ratio and the lateral drift.
  * twist_to_ackermann.cpp adds a `steering_angle_offset` parameter straight
    onto angular.z.  If it is non-zero the robot drives a constant arc even
    when commanded angular.z = 0.  Verify it is 0.0 before blaming the motors:
      ros2 param get /twist_to_ackermann steering_angle_offset
  * open_loop: true in ackermann_steering_controller.yaml means the published
    wheel odometry is integrated from the *commands*, not from encoders, so
    the wheel odometry cannot see a mis-calibrated motor at all.

EXACT RUN COMMAND
=================
  # 1. Source the workspace on the Pi (nothing here works otherwise)
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash

  # 2. Rehearse with no motion at all - always do this first
  python3 measurement_scripts/measure_driving_drift.py --dry-run

  # 3. Real run: 5 x 2 m forward at 0.2 m/s, fused pose only
  python3 measurement_scripts/measure_driving_drift.py \
      --robot-id 3 --distance 2.0 --speed 0.2 --runs 5 \
      --csv results/driving_drift.csv --i-am-clear

  # 4. Best quality: add the overhead camera as independent ground truth
  python3 measurement_scripts/measure_driving_drift.py \
      --robot-id 3 --direction both --runs 5 \
      --truth-topic /aruco/odom \
      --csv results/driving_drift.csv --i-am-clear

  # 5. No overhead camera and no ROS? Tape measure + arithmetic:
  python3 measurement_scripts/measure_driving_drift.py --manual --robot-id 3

PREREQUISITES
=============
Running nodes (check with `ros2 node list` / `ros2 topic hz <topic>`):
  * ros2_control + ackermann_steering_controller  (accepts the drive command)
  * twist_to_ackermann                            (/cmd_vel Twist -> controller
                                                   TwistStamped reference)
  * localization / ekf_global                     (publishes /odometry/global)
  * IMU + wheel odometry feeding the EKF
  * overhead_tracker + the /aruco/odom bridge     (ONLY if using --truth-topic)

The tactical brain (main_brain) MUST NOT be running - it publishes to /cmd_vel
itself and will fight this script for the topic.  Stop it first.

Physical prerequisites:
  * At least (--distance + 1.0) m of clear, flat, identical floor in front of
    the robot, plus the same behind it if using --direction both/reverse.
  * A chalk/tape line on the floor along the intended path makes the manual
    cross-check trivial and is strongly recommended.
  * Fully charged battery.  A sagging pack changes the effective PWM->speed
    curve and will silently poison the calibration.

SAFETY NOTES
============
  *** THIS SCRIPT DRIVES REAL HARDWARE.  IT MOVES A MOTORISED VEHICLE. ***

  * No motion is published unless you pass --i-am-clear.  Without it the
    script prints the plan and exits.
  * --dry-run publishes NOTHING, ever.  It logs every Twist it would have
    sent.  Use it to rehearse.
  * A countdown is printed before every single run.  Stand clear.
  * Zero Twist is published on every exit path: normal completion, exception,
    Ctrl-C (SIGINT) and SIGTERM.  The signal handler publishes the stop
    immediately, before unwinding.
  * A pose-based runaway guard aborts a run if the robot travels more than
    --abort-factor x the commanded distance.
  * Speeds above 0.5 m/s are refused unless --allow-fast is passed.
  * Keep a hand on the battery disconnect.  A software e-stop is not a
    substitute for cutting power.

WHY THE FUSED POSE UNDER-REPORTS DRIFT
======================================
`/odometry/global` is the output of robot_localization's ekf_global, which
fuses /aruco/odom (covariance 0.01 on x/y/yaw).  On top of that,
tactical_brain/localization_bridge.py force-reseeds the filter through the
/ekf_global/set_pose service whenever ArUco and the tracked pose disagree by
more than DRIFT_SNAP_THRESHOLD_METERS = 0.05.  That is an architectural
ceiling: with the correction live, the fused pose can essentially never report
more than ~5 cm of error, no matter how badly the chassis is actually
drifting.  So:

  * measuring PHYSICAL drift  -> use --truth-topic (overhead ArUco), or
                                 --manual with a tape measure.
  * measuring ODOMETRY error  -> --pose-topic alone is fine, but understand
                                 that you are measuring a corrected signal.

EXPECTED OUTPUT FORMAT
======================
--- ILLUSTRATIVE SAMPLE ONLY.  THE NUMBERS BELOW ARE MADE UP TO SHOW THE
--- LAYOUT.  THEY ARE NOT MEASUREMENTS.  DO NOT COPY THEM ANYWHERE.

  ============================================================
   RESULTS  robot_id=3  direction=forward  runs=5
  ============================================================
   commanded distance ..............  2.000 m
   commanded speed .................  0.200 m/s
   measured along-track distance ...  X.XXX +/- X.XXX m
   LATERAL DEVIATION ...............  XX.X +/- X.X cm      <-- headline
   LATERAL DRIFT PER METRE .........  XX.X +/- X.X cm/m    <-- headline
   straight-line end error .........  X.XXX +/- X.XXX m
   final heading error .............  XX.X +/- X.X deg
  ------------------------------------------------------------
   pwm_multiplier  current .........  1.60
   pwm_multiplier  suggested .......  X.XX   (starting point - iterate!)
  ============================================================

The CSV is append-only: one row per run, tagged with --robot-id, so results
for the whole fleet accumulate into one file and can be diffed per chassis.
"""

import argparse
import csv
import datetime
import json
import math
import os
import signal
import sys
import time

import numpy as np

# --------------------------------------------------------------------------
# Constants taken verbatim from the repository.  Do not "fix" these here -
# fix them at the source and update this block.
# --------------------------------------------------------------------------
CURRENT_PWM_MULTIPLIER = 1.60      # hardware/ros2_control.xacro (code default 2.0)
DRIFT_SNAP_THRESHOLD_M = 0.05      # tactical_brain/localization_bridge.py
DEFAULT_POSE_TOPIC = '/odometry/global'
DEFAULT_CMD_TOPIC = '/cmd_vel'
DEFAULT_ARENA_SIZE_M = 2.0         # ARENA_SIZE_METERS, main_brain.py
DEFAULT_GRID_N = 2000              # grid_n, overhead_tracker params.yaml

HARD_SPEED_LIMIT_MPS = 0.5         # above this, --allow-fast is mandatory

_STOP_REQUESTED = False
_ACTIVE_NODE = None                # set once the node exists, for the signal handler


# ==========================================================================
# Safety plumbing
# ==========================================================================
def _signal_handler(signum, _frame):
    """Publish a stop IMMEDIATELY, then let the main loop unwind."""
    global _STOP_REQUESTED
    _STOP_REQUESTED = True
    name = signal.Signals(signum).name
    sys.stderr.write(f"\n\n*** {name} RECEIVED - EMERGENCY STOP ***\n")
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


def print_safety_banner(args):
    bar = "=" * 74
    print(bar)
    if args.dry_run:
        print("  DRY RUN - NOTHING WILL BE PUBLISHED TO /cmd_vel")
    else:
        print("  !!  MOTION TEST - THE ROBOT WILL DRIVE  !!")
    print(bar)
    total = args.runs * (2 if args.direction == 'both' else 1)
    print(f"  robot id .............. {args.robot_id}")
    print(f"  runs .................. {total} "
          f"({args.runs} per direction, direction={args.direction})")
    print(f"  distance per run ...... {args.distance:.3f} m")
    print(f"  speed ................. {args.speed:.3f} m/s")
    print(f"  open-loop drive time .. {args.distance / args.speed:.2f} s per run")
    print(f"  command topic ......... {args.cmd_topic}  (geometry_msgs/Twist)")
    print(f"  pose topic ............ {args.pose_topic}")
    print(f"  ground-truth topic .... {args.truth_topic or '(none - see docstring)'}")
    clear_ahead = args.distance * args.abort_factor + 1.0
    print(f"  CLEAR SPACE REQUIRED .. {clear_ahead:.1f} m ahead"
          + (f" AND {clear_ahead:.1f} m behind" if args.direction in ('both', 'reverse') else ""))
    print(bar)


def countdown(seconds, label):
    if seconds <= 0:
        return
    print(f"  {label} in ", end='', flush=True)
    for i in range(int(seconds), 0, -1):
        print(f"{i}... ", end='', flush=True)
        time.sleep(1.0)
    print("GO", flush=True)


# ==========================================================================
# Pure math (no ROS) - unit-testable and reused by --manual
# ==========================================================================
def yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def suggest_pwm_multiplier(current, commanded_m, measured_m):
    """new = current * commanded / measured.

    Robot travelled short of the command -> ratio > 1 -> push more PWM.
    Returns None when the measurement is too small to divide by.
    """
    if measured_m is None or abs(measured_m) < 1e-3:
        return None
    return current * (commanded_m / abs(measured_m))


def path_metrics(samples, sign, commanded_distance):
    """Reduce one run's pose samples to the reported metrics.

    samples : (N, 4) array of [t, x, y, yaw], t monotonic seconds.
    sign    : +1 forward, -1 reverse.  The intended direction of travel is
              sign * the heading held at the start of the run.
    """
    arr = np.asarray(samples, dtype=float)
    if arr.shape[0] < 2:
        return None

    xy = arr[:, 1:3]
    yaw0 = float(arr[0, 3])
    yaw_end = float(arr[-1, 3])

    # Unit vector along the intended travel direction, and its left normal.
    u = sign * np.array([math.cos(yaw0), math.sin(yaw0)])
    n = np.array([-u[1], u[0]])

    rel = xy - xy[0]
    displacement = rel[-1]

    longitudinal = float(np.dot(displacement, u))
    lateral = float(np.dot(displacement, n))          # + = left of intended line

    lateral_track = rel @ n
    max_lateral = float(lateral_track[np.argmax(np.abs(lateral_track))])

    steps = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    path_length = float(np.sum(steps))

    ideal_end = u * commanded_distance
    end_error = float(np.linalg.norm(displacement - ideal_end))

    denom = max(abs(longitudinal), 1e-6)
    return {
        'path_length_m': path_length,
        'net_displacement_m': float(np.linalg.norm(displacement)),
        'longitudinal_m': longitudinal,
        'lateral_m': lateral,
        'max_lateral_m': max_lateral,
        'lateral_cm_per_m': (lateral * 100.0) / denom,
        'abs_lateral_cm_per_m': (abs(lateral) * 100.0) / denom,
        'end_error_m': end_error,
        'heading_error_deg': math.degrees(wrap_angle(yaw_end - yaw0)),
        'distance_ratio': longitudinal / commanded_distance if commanded_distance else float('nan'),
        'start_x': float(xy[0, 0]),
        'start_y': float(xy[0, 1]),
        'start_yaw': yaw0,
        'end_x': float(xy[-1, 0]),
        'end_y': float(xy[-1, 1]),
        'end_yaw': yaw_end,
        'n_samples': int(arr.shape[0]),
    }


def mean_std(values):
    """(mean, std, n) ignoring NaNs.  std uses ddof=1 when n > 1."""
    v = np.asarray([x for x in values if x is not None and not math.isnan(x)], dtype=float)
    if v.size == 0:
        return float('nan'), float('nan'), 0
    if v.size == 1:
        return float(v[0]), 0.0, 1
    return float(np.mean(v)), float(np.std(v, ddof=1)), int(v.size)


# ==========================================================================
# CSV
# ==========================================================================
CSV_FIELDS = [
    'timestamp_iso', 'robot_id', 'source', 'mode', 'direction', 'run_index',
    'commanded_distance_m', 'commanded_speed_mps', 'drive_time_s',
    'path_length_m', 'net_displacement_m', 'longitudinal_m', 'lateral_m',
    'max_lateral_m', 'lateral_cm_per_m', 'abs_lateral_cm_per_m',
    'end_error_m', 'heading_error_deg', 'distance_ratio',
    'odom_vs_truth_end_error_m', 'n_samples', 'aborted',
    'current_pwm_multiplier', 'suggested_pwm_multiplier', 'notes',
]


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


# ==========================================================================
# Manual mode - tape measure, no ROS required
# ==========================================================================
def run_manual(args):
    print("=" * 74)
    print("  MANUAL MODE - tape measure, no ROS, no motion published by this script")
    print("=" * 74)
    print("""
  Procedure:
    1. Mark the robot's starting position on the floor: the exact point under
       a chosen reference on the chassis, plus the direction it faces.  A
       chalk line along the intended path makes step 4 much easier.
    2. Drive it straight forward however you normally do (teleop, a scripted
       /cmd_vel burst, the auto mode of this script) for a known commanded
       distance.  Do not steer.  Stop.
    3. Measure ALONG the intended line from start mark to the same chassis
       reference: that is the measured forward distance.
    4. Measure PERPENDICULAR from the intended line to the same reference:
       that is the lateral offset.  Sign convention here: positive = the
       robot ended up LEFT of the line, negative = right.  Consistency across
       runs matters more than which side you call positive.
    5. Enter the three numbers below.  Repeat for as many runs as you did.

  Enter a blank commanded distance to finish and see the summary.
""")

    rows = []
    lat_per_m, ratios = [], []
    run_index = 0
    while True:
        raw = input(f"  run {run_index + 1}  commanded distance [m] (blank = done): ").strip()
        if not raw:
            break
        try:
            commanded = float(raw)
            measured = float(input("            measured forward distance [m]: ").strip())
            lateral_cm = float(input("            lateral offset [cm, +left/-right]: ").strip())
        except ValueError:
            print("  !! not a number - try again")
            continue
        if commanded <= 0 or measured <= 0:
            print("  !! distances must be positive - try again")
            continue

        run_index += 1
        per_m = lateral_cm / measured
        ratio = measured / commanded
        lat_per_m.append(per_m)
        ratios.append(ratio)

        suggestion = suggest_pwm_multiplier(args.current_pwm, commanded, measured)
        print(f"     -> lateral drift        : {per_m:+.2f} cm per metre driven")
        print(f"     -> distance ratio       : {ratio:.4f} (measured/commanded)")
        shown = f"{suggestion:.3f}" if suggestion is not None else "n/a"
        print(f"     -> pwm suggestion (run) : {shown}")

        rows.append({
            'timestamp_iso': datetime.datetime.now().isoformat(timespec='seconds'),
            'robot_id': args.robot_id,
            'source': 'manual_tape_measure',
            'mode': 'manual',
            'direction': 'forward',
            'run_index': run_index,
            'commanded_distance_m': f"{commanded:.4f}",
            'commanded_speed_mps': '',
            'drive_time_s': '',
            'path_length_m': '',
            'net_displacement_m': '',
            'longitudinal_m': f"{measured:.4f}",
            'lateral_m': f"{lateral_cm / 100.0:.4f}",
            'max_lateral_m': '',
            'lateral_cm_per_m': f"{per_m:.4f}",
            'abs_lateral_cm_per_m': f"{abs(per_m):.4f}",
            'end_error_m': '',
            'heading_error_deg': '',
            'distance_ratio': f"{ratio:.6f}",
            'odom_vs_truth_end_error_m': '',
            'n_samples': '',
            'aborted': 'false',
            'current_pwm_multiplier': f"{args.current_pwm:.3f}",
            'suggested_pwm_multiplier': f"{suggestion:.4f}" if suggestion else '',
            'notes': 'manual tape-measure entry',
        })

    if not rows:
        print("\n  No runs entered - nothing to report.")
        return 0

    print()
    print("=" * 74)
    print(f"  MANUAL SUMMARY  robot_id={args.robot_id}  runs={len(rows)}")
    print("=" * 74)
    m, s, n = mean_std(lat_per_m)
    am, asd, _ = mean_std([abs(x) for x in lat_per_m])
    print(f"  lateral drift (signed) .... {m:+.2f} +/- {s:.2f} cm/m   (n={n})")
    print(f"  lateral drift (magnitude) . {am:.2f} +/- {asd:.2f} cm/m  <-- headline")
    rm, rs, _ = mean_std(ratios)
    print(f"  distance ratio ............ {rm:.4f} +/- {rs:.4f}  (measured/commanded)")
    print("-" * 74)
    print_pwm_block(args.current_pwm, rm)
    print_interpretation(am)
    append_csv(args.csv, rows)
    return 0


# ==========================================================================
# Shared reporting
# ==========================================================================
def print_pwm_block(current, ratio_mean):
    print(f"  pwm_multiplier  CURRENT ... {current:.3f}"
          "   (hardware/ros2_control.xacro; code default is 2.0)")
    if ratio_mean is None or math.isnan(ratio_mean) or ratio_mean <= 0:
        print("  pwm_multiplier  SUGGESTED . n/a - measured distance unusable")
        return
    suggested = current / ratio_mean          # == current * commanded / measured
    print(f"  pwm_multiplier  SUGGESTED . {suggested:.3f}"
          f"   ({'increase' if suggested > current else 'decrease'} "
          f"{abs(suggested - current) / current * 100.0:.1f}%)")
    print("""
  *** THE SUGGESTION IS A STARTING POINT, NOT A FINAL VALUE. ***
  PWM duty to wheel speed is not a straight line: there is a deadband at low
  duty and saturation at high duty, so one division will not land it.  Set the
  value, re-run this script, and repeat until the distance ratio sits inside
  about 1.00 +/- 0.02.  Two or three iterations is normal.

  To apply it (per robot - that file is currently shared by the whole fleet):
    AutonomousWarfare/ros2_ws/src/hardware/ros2_control.xacro
      <param name="pwm_multiplier">%.2f</param>
  then rebuild/reload ros2_control.  Record the final value against this
  chassis' robot-id; that per-robot table is the deliverable.""" % suggested)


def print_interpretation(abs_lat_per_m):
    if abs_lat_per_m is None or math.isnan(abs_lat_per_m):
        return
    print()
    print("  Interpretation:")
    print(f"    At {abs_lat_per_m:.2f} cm/m the robot accumulates "
          f"{DRIFT_SNAP_THRESHOLD_M * 100.0:.0f} cm of lateral error - the "
          "ArUco force-reseed")
    if abs_lat_per_m > 1e-6:
        trigger_m = (DRIFT_SNAP_THRESHOLD_M * 100.0) / abs_lat_per_m
        print(f"    threshold (DRIFT_SNAP_THRESHOLD_METERS) - after about "
              f"{trigger_m:.2f} m of driving.")
        print("    Shorter than a typical path leg means the correction is "
              "firing constantly and")
        print("    is masking a mechanical problem rather than trimming a "
              "small residual.")
    print("    This measurement is only valid for the surface it was taken "
          "on - carpet, sand")
    print("    and smooth floor give genuinely different numbers.")


# ==========================================================================
# ROS node (imported lazily so --manual works without a sourced workspace)
# ==========================================================================
def build_node_class(rclpy, Node, Odometry, Twist, String, QoSProfile,
                     ReliabilityPolicy, HistoryPolicy):

    class DriftRecorder(Node):
        def __init__(self, args):
            super().__init__('measure_driving_drift')
            self.args = args
            self.dry_run = args.dry_run
            self.pose_samples = []
            self.truth_samples = []
            self.recording = False
            self._t0 = time.monotonic()
            self._cmd_log = []

            qos = QoSProfile(depth=50,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

            self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
            self.create_subscription(Odometry, args.pose_topic,
                                     self._pose_cb, qos)

            if args.truth_topic:
                if args.truth_type == 'odom':
                    self.create_subscription(Odometry, args.truth_topic,
                                             self._truth_odom_cb, qos)
                else:
                    self.create_subscription(String, args.truth_topic,
                                             self._truth_grid_cb, qos)

        # ---- clock -----------------------------------------------------
        def now(self):
            return time.monotonic() - self._t0

        # ---- subscriptions ---------------------------------------------
        def _pose_cb(self, msg):
            if not self.recording:
                return
            p = msg.pose.pose
            q = p.orientation
            self.pose_samples.append(
                [self.now(), p.position.x, p.position.y,
                 yaw_from_quaternion(q.x, q.y, q.z, q.w)])

        def _truth_odom_cb(self, msg):
            if not self.recording:
                return
            p = msg.pose.pose
            q = p.orientation
            self.truth_samples.append(
                [self.now(), p.position.x, p.position.y,
                 yaw_from_quaternion(q.x, q.y, q.z, q.w)])

        def _truth_grid_cb(self, msg):
            """overhead_tracker publishes JSON in 0..grid_n grid units.

            Conversion is arena_size_meters / grid_n (main_brain.py).  NOTE the
            grid is an image-convention frame: y increases DOWNWARD and the
            marker angle comes from atan2(dy, dx) in that same frame, so it is
            mirrored relative to the ROS map frame.  Distances and drift
            MAGNITUDES are unaffected; only the sign of the lateral offset and
            the sign of the heading error flip.
            """
            if not self.recording:
                return
            try:
                robots = json.loads(msg.data)
            except (ValueError, TypeError):
                return
            if not isinstance(robots, list):
                return
            scale = self.args.arena_size / float(self.args.grid_n)
            for robot in robots:
                if int(robot.get('id', -1)) != int(self.args.robot_id):
                    continue
                self.truth_samples.append(
                    [self.now(),
                     float(robot['x']) * scale,
                     float(robot['y']) * scale,
                     math.radians(float(robot.get('angle', 0.0)))])
                return

        # ---- publishing -------------------------------------------------
        def publish_cmd(self, linear_x, angular_z=0.0):
            if self.dry_run:
                self._cmd_log.append((self.now(), linear_x, angular_z))
                return
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.angular.z = float(angular_z)
            self.cmd_pub.publish(msg)

        def emergency_stop(self, repeats=12, interval=0.02):
            """Spam zero Twist so a dropped packet cannot leave it driving."""
            if self.dry_run:
                print("  [dry-run] would publish zero Twist (emergency stop)")
                return
            stop = Twist()
            for _ in range(repeats):
                self.cmd_pub.publish(stop)
                time.sleep(interval)

        def dry_run_summary(self):
            if not self._cmd_log:
                return
            first, last = self._cmd_log[0], self._cmd_log[-1]
            print(f"  [dry-run] {len(self._cmd_log)} Twist messages suppressed, "
                  f"linear.x from {first[1]:+.3f} to {last[1]:+.3f} m/s, "
                  f"angular.z fixed at {first[2]:+.3f} rad/s, "
                  f"t={first[0]:.2f}s..{last[0]:.2f}s")
            self._cmd_log.clear()

    return DriftRecorder


def wait_for_pose(rclpy, node, timeout_s, label):
    """Block until at least one sample lands, so a dead topic fails loudly."""
    node.recording = True
    node.pose_samples.clear()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not _STOP_REQUESTED:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.pose_samples:
            return True
    node.recording = False
    print(f"  !! no message on {label} within {timeout_s:.0f}s")
    return False


def drive_one_run(rclpy, node, args, sign, run_index, direction_label):
    """Drive one open-loop straight line.  Returns (metrics, truth_metrics, aborted)."""
    drive_time = args.distance / args.speed
    linear_x = sign * args.speed

    print(f"\n  --- run {run_index} / {args.runs}  ({direction_label}) ---")
    print(f"      commanding linear.x = {linear_x:+.3f} m/s, angular.z = 0.000 rad/s "
          f"for {drive_time:.2f} s")

    # Settle and grab the reference pose before moving.
    node.recording = True
    node.pose_samples.clear()
    node.truth_samples.clear()
    settle_end = time.monotonic() + max(args.settle, 0.2)
    while time.monotonic() < settle_end and not _STOP_REQUESTED:
        rclpy.spin_once(node, timeout_sec=0.02)

    if not node.pose_samples:
        print(f"      !! no pose samples on {args.pose_topic} - is the EKF running?")
        node.recording = False
        return None, None, True

    start_xy = np.array(node.pose_samples[-1][1:3])
    # Drop the pre-motion settle samples; the run starts here.
    node.pose_samples = [node.pose_samples[-1]]
    node.truth_samples = node.truth_samples[-1:] if node.truth_samples else []

    countdown(args.countdown, "MOVING")

    aborted = False
    abort_limit = args.distance * args.abort_factor
    period = 1.0 / max(args.publish_rate, 1.0)
    t_start = time.monotonic()
    next_pub = t_start

    try:
        while True:
            elapsed = time.monotonic() - t_start
            if elapsed >= drive_time:
                break
            if _STOP_REQUESTED:
                aborted = True
                print("      !! stop requested - aborting run")
                break

            now = time.monotonic()
            if now >= next_pub:
                node.publish_cmd(linear_x, 0.0)
                next_pub = now + period

            rclpy.spin_once(node, timeout_sec=0.01)

            # Runaway guard - pose-based, independent of the timed stop.
            if node.pose_samples:
                travelled = float(np.linalg.norm(
                    np.array(node.pose_samples[-1][1:3]) - start_xy))
                if travelled > abort_limit:
                    aborted = True
                    print(f"      !! RUNAWAY GUARD: travelled {travelled:.2f} m > "
                          f"{abort_limit:.2f} m limit - stopping")
                    break
    finally:
        node.publish_cmd(0.0, 0.0)
        node.emergency_stop(repeats=6)

    # Let it coast to a genuine stop before reading the final pose.
    coast_end = time.monotonic() + max(args.settle, 0.5)
    while time.monotonic() < coast_end:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.recording = False

    if args.dry_run:
        node.dry_run_summary()

    metrics = path_metrics(node.pose_samples, sign, args.distance)
    truth_metrics = (path_metrics(node.truth_samples, sign, args.distance)
                     if len(node.truth_samples) >= 2 else None)

    if metrics is None:
        print("      !! fewer than 2 pose samples - run discarded")
        return None, None, True

    print(f"      pose  : along {metrics['longitudinal_m']:+.3f} m, "
          f"lateral {metrics['lateral_m'] * 100:+.1f} cm "
          f"({metrics['lateral_cm_per_m']:+.2f} cm/m), "
          f"heading {metrics['heading_error_deg']:+.1f} deg")
    if truth_metrics:
        print(f"      truth : along {truth_metrics['longitudinal_m']:+.3f} m, "
              f"lateral {truth_metrics['lateral_m'] * 100:+.1f} cm "
              f"({truth_metrics['lateral_cm_per_m']:+.2f} cm/m), "
              f"heading {truth_metrics['heading_error_deg']:+.1f} deg")
    return metrics, truth_metrics, aborted


def report_direction(args, direction_label, pose_runs, truth_runs):
    """Print the block for one direction; returns the mean distance ratio."""
    if not pose_runs:
        print(f"\n  no usable runs for direction={direction_label}")
        return None

    def col(runs, key):
        return [r[key] for r in runs if r is not None]

    headline_runs = truth_runs if truth_runs else pose_runs
    source = ("overhead ArUco ground truth" if truth_runs
              else f"{args.pose_topic} (ArUco-CORRECTED - see docstring)")

    print()
    print("=" * 74)
    print(f"  RESULTS  robot_id={args.robot_id}  direction={direction_label}  "
          f"runs={len(headline_runs)}")
    print(f"  headline source: {source}")
    print("=" * 74)
    print(f"  commanded distance ......... {args.distance:.3f} m")
    print(f"  commanded speed ............ {args.speed:.3f} m/s")

    m, s, n = mean_std(col(headline_runs, 'longitudinal_m'))
    print(f"  measured along-track ....... {m:.3f} +/- {s:.3f} m   (n={n})")

    m, s, _ = mean_std([v * 100.0 for v in col(headline_runs, 'lateral_m')])
    print(f"  LATERAL DEVIATION (signed) . {m:+.2f} +/- {s:.2f} cm")
    am, asd, _ = mean_std([abs(v) * 100.0 for v in col(headline_runs, 'lateral_m')])
    print(f"  LATERAL DEVIATION (magnit.). {am:.2f} +/- {asd:.2f} cm      <-- headline")

    pm, ps, _ = mean_std(col(headline_runs, 'lateral_cm_per_m'))
    print(f"  LATERAL DRIFT PER METRE .... {pm:+.2f} +/- {ps:.2f} cm/m (signed)")
    apm, aps, _ = mean_std(col(headline_runs, 'abs_lateral_cm_per_m'))
    print(f"  LATERAL DRIFT PER METRE .... {apm:.2f} +/- {aps:.2f} cm/m    <-- headline")

    m, s, _ = mean_std(col(headline_runs, 'end_error_m'))
    print(f"  straight-line end error .... {m:.3f} +/- {s:.3f} m")
    m, s, _ = mean_std(col(headline_runs, 'path_length_m'))
    print(f"  total path length .......... {m:.3f} +/- {s:.3f} m")
    m, s, _ = mean_std(col(headline_runs, 'heading_error_deg'))
    print(f"  final heading error ........ {m:+.2f} +/- {s:.2f} deg")

    ratio_mean, ratio_std, _ = mean_std(col(headline_runs, 'distance_ratio'))
    print(f"  distance ratio ............. {ratio_mean:.4f} +/- {ratio_std:.4f}"
          "  (measured/commanded)")

    if truth_runs and pose_runs:
        errs = []
        for pose_run, truth_run in zip(pose_runs, truth_runs):
            errs.append(math.hypot(pose_run['longitudinal_m'] - truth_run['longitudinal_m'],
                                   pose_run['lateral_m'] - truth_run['lateral_m']))
        m, s, _ = mean_std(errs)
        print("-" * 74)
        print(f"  ODOMETRY error vs truth .... {m * 100:.2f} +/- {s * 100:.2f} cm")
        print("    (how wrong the fused pose was, separate from how far the "
              "chassis physically drifted)")
        if m > DRIFT_SNAP_THRESHOLD_M:
            print(f"    NOTE: above DRIFT_SNAP_THRESHOLD_METERS "
                  f"({DRIFT_SNAP_THRESHOLD_M * 100:.0f} cm) - the force-reseed "
                  "should have fired.")

    print("-" * 74)
    print_pwm_block(args.current_pwm, ratio_mean)
    print_interpretation(apm)
    print("=" * 74)
    return ratio_mean


def run_auto(args):
    global _ACTIVE_NODE

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
    except ImportError as exc:
        print("\nERROR: the ROS 2 Python environment is not available "
              f"({exc}).", file=sys.stderr)
        print("This script needs a sourced ROS 2 Humble workspace:\n"
              "    source /opt/ros/humble/setup.bash\n"
              "    source ~/ros2_ws/install/setup.bash\n"
              "Then re-run.  (--manual mode works without ROS.)", file=sys.stderr)
        return 3

    DriftRecorder = build_node_class(rclpy, Node, Odometry, Twist, String,
                                     QoSProfile, ReliabilityPolicy, HistoryPolicy)

    rclpy.init()
    node = DriftRecorder(args)
    _ACTIVE_NODE = node
    install_signal_handlers()

    directions = {'forward': [(+1, 'forward')],
                  'reverse': [(-1, 'reverse')],
                  'both': [(+1, 'forward'), (-1, 'reverse')]}[args.direction]

    rows = []
    try:
        print(f"\n  waiting for first message on {args.pose_topic} ...")
        if not wait_for_pose(rclpy, node, args.wait_timeout, args.pose_topic):
            print("  Check: is ekf_global running?  `ros2 topic hz "
                  f"{args.pose_topic}`")
            return 4
        print("  pose stream OK")
        node.recording = False

        for sign, label in directions:
            pose_runs, truth_runs = [], []
            for run_index in range(1, args.runs + 1):
                if _STOP_REQUESTED:
                    break
                metrics, truth, aborted = drive_one_run(
                    rclpy, node, args, sign, run_index, label)
                if metrics is not None:
                    if not aborted:
                        pose_runs.append(metrics)
                        if truth is not None:
                            truth_runs.append(truth)
                    rows.append(make_row(args, label, run_index, metrics,
                                         truth, aborted))
                if run_index < args.runs and not _STOP_REQUESTED:
                    print(f"      pausing {args.pause:.1f}s - "
                          "reposition the robot at the start mark if needed")
                    time.sleep(args.pause)

            # truth_runs must line up 1:1 with pose_runs for the comparison
            if len(truth_runs) != len(pose_runs):
                truth_runs = []
            report_direction(args, label, pose_runs, truth_runs)

    finally:
        try:
            node.emergency_stop()
            print("\n  final zero Twist published on " + args.cmd_topic)
        except Exception as exc:                                  # noqa: BLE001
            print(f"\n  !! could not publish final stop: {exc}", file=sys.stderr)
            print("  !! CUT POWER AT THE BATTERY", file=sys.stderr)
        append_csv(args.csv, rows)
        _ACTIVE_NODE = None
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


def make_row(args, direction, run_index, metrics, truth, aborted):
    src = truth if truth is not None else metrics
    suggestion = suggest_pwm_multiplier(args.current_pwm, args.distance,
                                        src['longitudinal_m'])
    odom_vs_truth = ''
    if truth is not None:
        odom_vs_truth = f"{math.hypot(metrics['longitudinal_m'] - truth['longitudinal_m'], metrics['lateral_m'] - truth['lateral_m']):.4f}"
    return {
        'timestamp_iso': datetime.datetime.now().isoformat(timespec='seconds'),
        'robot_id': args.robot_id,
        'source': ('overhead_truth' if truth is not None else args.pose_topic),
        'mode': 'dry-run' if args.dry_run else 'auto',
        'direction': direction,
        'run_index': run_index,
        'commanded_distance_m': f"{args.distance:.4f}",
        'commanded_speed_mps': f"{args.speed:.4f}",
        'drive_time_s': f"{args.distance / args.speed:.3f}",
        'path_length_m': f"{src['path_length_m']:.4f}",
        'net_displacement_m': f"{src['net_displacement_m']:.4f}",
        'longitudinal_m': f"{src['longitudinal_m']:.4f}",
        'lateral_m': f"{src['lateral_m']:.4f}",
        'max_lateral_m': f"{src['max_lateral_m']:.4f}",
        'lateral_cm_per_m': f"{src['lateral_cm_per_m']:.4f}",
        'abs_lateral_cm_per_m': f"{src['abs_lateral_cm_per_m']:.4f}",
        'end_error_m': f"{src['end_error_m']:.4f}",
        'heading_error_deg': f"{src['heading_error_deg']:.3f}",
        'distance_ratio': f"{src['distance_ratio']:.6f}",
        'odom_vs_truth_end_error_m': odom_vs_truth,
        'n_samples': src['n_samples'],
        'aborted': 'true' if aborted else 'false',
        'current_pwm_multiplier': f"{args.current_pwm:.3f}",
        'suggested_pwm_multiplier': f"{suggestion:.4f}" if suggestion else '',
        'notes': '',
    }


# ==========================================================================
def build_parser():
    p = argparse.ArgumentParser(
        prog='measure_driving_drift.py',
        description='Measure lateral drift per metre driven and derive a '
                    'per-robot pwm_multiplier.  DRIVES REAL HARDWARE.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='Always rehearse with --dry-run first.  Motion requires '
               '--i-am-clear.')

    g = p.add_argument_group('trajectory')
    g.add_argument('--distance', type=float, default=2.0,
                   help='commanded straight-line distance per run, metres (default 2.0)')
    g.add_argument('--speed', type=float, default=0.2,
                   help='commanded linear.x, m/s (default 0.2)')
    g.add_argument('--runs', type=int, default=5,
                   help='runs per direction (default 5)')
    g.add_argument('--direction', choices=['forward', 'reverse', 'both'],
                   default='forward',
                   help='drift is often asymmetric; "both" measures each '
                        'direction separately (default forward)')

    g = p.add_argument_group('topics')
    g.add_argument('--pose-topic', default=DEFAULT_POSE_TOPIC,
                   help=f'fused pose, nav_msgs/Odometry (default {DEFAULT_POSE_TOPIC})')
    g.add_argument('--cmd-topic', default=DEFAULT_CMD_TOPIC,
                   help=f'command topic, geometry_msgs/Twist (default {DEFAULT_CMD_TOPIC})')
    g.add_argument('--truth-topic', default=None,
                   help='independent ground truth, e.g. /aruco/odom or '
                        'teams/team_1/positions.  Strongly recommended - '
                        'without it the fused pose caps reported error at ~5 cm.')
    g.add_argument('--truth-type', choices=['odom', 'grid'], default='odom',
                   help='"odom" = nav_msgs/Odometry in metres; "grid" = the '
                        'overhead_tracker std_msgs/String JSON in 0..grid_n '
                        'units (default odom)')
    g.add_argument('--arena-size', type=float, default=DEFAULT_ARENA_SIZE_M,
                   help=f'arena_size_meters for grid->metre conversion (default {DEFAULT_ARENA_SIZE_M})')
    g.add_argument('--grid-n', type=int, default=DEFAULT_GRID_N,
                   help=f'grid_n for grid->metre conversion (default {DEFAULT_GRID_N})')

    g = p.add_argument_group('output')
    g.add_argument('--robot-id', type=int, default=int(os.environ.get('ROBOT_ID', 3)),
                   help='chassis tag written into every CSV row so per-robot '
                        'results accumulate in one file (default $ROBOT_ID or 3)')
    g.add_argument('--csv', default=None, help='append results to this CSV')
    g.add_argument('--current-pwm', type=float, default=CURRENT_PWM_MULTIPLIER,
                   help=f'pwm_multiplier currently flashed on this robot '
                        f'(default {CURRENT_PWM_MULTIPLIER}, from ros2_control.xacro)')

    g = p.add_argument_group('safety')
    g.add_argument('--i-am-clear', action='store_true',
                   help='REQUIRED for motion.  Confirms the arena is clear of '
                        'people and obstacles.')
    g.add_argument('--dry-run', action='store_true',
                   help='publish nothing; log the Twists that would be sent')
    g.add_argument('--allow-fast', action='store_true',
                   help=f'permit --speed above {HARD_SPEED_LIMIT_MPS} m/s')
    g.add_argument('--abort-factor', type=float, default=1.5,
                   help='abort a run if measured travel exceeds this multiple '
                        'of --distance (default 1.5)')
    g.add_argument('--countdown', type=int, default=5,
                   help='seconds of countdown before each run (default 5)')

    g = p.add_argument_group('timing')
    g.add_argument('--publish-rate', type=float, default=20.0,
                   help='/cmd_vel publish rate, Hz (default 20)')
    g.add_argument('--settle', type=float, default=1.0,
                   help='seconds to sit still before and after each run (default 1.0)')
    g.add_argument('--pause', type=float, default=3.0,
                   help='seconds between runs (default 3.0)')
    g.add_argument('--wait-timeout', type=float, default=15.0,
                   help='seconds to wait for the first pose message (default 15)')

    p.add_argument('--manual', action='store_true',
                   help='tape-measure mode: no ROS, no motion, operator types '
                        'the numbers in')
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)

    if args.manual:
        return run_manual(args)

    if args.distance <= 0 or args.speed <= 0 or args.runs <= 0:
        print("ERROR: --distance, --speed and --runs must all be positive.",
              file=sys.stderr)
        return 2

    if args.speed > HARD_SPEED_LIMIT_MPS and not args.allow_fast:
        print(f"ERROR: --speed {args.speed} m/s exceeds the "
              f"{HARD_SPEED_LIMIT_MPS} m/s safety limit.", file=sys.stderr)
        print("Pass --allow-fast only if you have the space and a spotter.",
              file=sys.stderr)
        return 2

    print_safety_banner(args)

    if not args.dry_run and not args.i_am_clear:
        print()
        print("*" * 74)
        print("  REFUSING TO MOVE: --i-am-clear was not passed.")
        print()
        print("  Before adding it, confirm ALL of the following:")
        print("    [ ] no people, pets or cables anywhere in the run corridor")
        print("    [ ] the tactical brain (main_brain) is STOPPED - it also "
              "publishes /cmd_vel")
        print("    [ ] the battery disconnect is within arm's reach")
        print("    [ ] you have rehearsed this exact command with --dry-run")
        print()
        print("  Then re-run with --i-am-clear appended.")
        print("*" * 74)
        return 2

    return run_auto(args)


if __name__ == '__main__':
    sys.exit(main())
