"""Alignment (aiming) control law for the attack branch.

Deliberately free of rclpy/ROS imports (same separation as A_planner.py
and ballistics_helper.py) - just heading-error-in, twist-out math.

Ackermann steering (this robot's real chassis, see
hardware/config/ackermann_steering_controller.yaml on the rpi5 branch,
open_loop: true) cannot pivot in place: steering angle is derived from
curvature = angular.z / linear.x, so linear.x == 0 makes any angular.z
physically meaningless. Alignment is therefore expressed as a slow
forward creep plus a proportional turn rate, never a stationary spin.
"""

# --- Conservative defaults (m/s, rad/s, deg) ---
# Broken out as named constants specifically so they're easy to retune
# against real hardware once available - values below are untested
# starting points, not measured.

# Forward creep speed while aligning. Constant regardless of heading
# error size - simplicity over an extra speed-scheduling knob;
# ballistics_helper's own envelope decides when alignment is "good
# enough" to fire, not this controller.
ALIGN_CREEP_SPEED_MPS = 0.15

# Turn-rate gain: rad/s of angular.z commanded per degree of heading error.
ALIGN_KP_ANGULAR = 0.03

# Clamp on the commanded turn rate. Kept well under
# heading_pid_controller's own max_correction clamp (1.0 rad/s) so that
# downstream correction loop still has headroom on top of this.
MAX_ALIGN_ANGULAR_Z = 0.6

# Stop closing distance once this near the target - untested placeholder
# like the speed/gain constants above, not a measured value. Without this,
# the robot had no distance-based cutoff at all and would keep creeping
# forward at ALIGN_CREEP_SPEED_MPS indefinitely while firing, closing to
# point-blank/collision range against a stationary or slow target. Chosen
# comfortably below MAX_FIRING_DISTANCE (ballistics_helper.py, 3.0m) so
# most of the firing envelope still gets the full creep-and-align
# behavior; only the final approach is held.
STANDOFF_DISTANCE_METERS = 1.0

# Sign relating YOLO's 'angle' field to ROS +angular.z (CCW / left turn).
# The aim law turns at ALIGN_KP_ANGULAR * angle; whether that reduces or
# GROWS the error depends on YOLO's convention, which is NOT yet verified
# on hardware. +1.0 assumes a positive reported angle should be answered
# with a positive (left) turn. If, on the real robot, the chassis steers
# AWAY from the target instead of toward it, flip this single constant to
# -1.0 - that's the only change needed.
AIM_ANGULAR_SIGN = 1.0


def compute_alignment_twist(heading_error_deg, distance_m):
    """Returns (linear_x_mps, angular_z_rad_s) to reduce heading_error_deg
    while holding at STANDOFF_DISTANCE_METERS rather than closing to the
    target indefinitely.

    Caller is responsible for not calling this with heading_error_deg is
    None (no current target lock) - see AlignToEnemyAction. distance_m may
    be None (distance unknown) - treated the same as "already at/inside
    standoff", i.e. hold rather than blindly creep toward an unknown range.
    """
    angular_z = AIM_ANGULAR_SIGN * ALIGN_KP_ANGULAR * heading_error_deg
    angular_z = max(-MAX_ALIGN_ANGULAR_Z, min(MAX_ALIGN_ANGULAR_Z, angular_z))
    linear_x = ALIGN_CREEP_SPEED_MPS if (distance_m is not None and distance_m > STANDOFF_DISTANCE_METERS) else 0.0
    return linear_x, angular_z