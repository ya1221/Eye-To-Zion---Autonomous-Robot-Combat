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


def compute_alignment_twist(heading_error_deg):
    """Returns (linear_x_mps, angular_z_rad_s) to reduce heading_error_deg.

    Caller is responsible for not calling this with heading_error_deg is
    None (no current target lock) - see AlignToEnemyAction.
    """
    angular_z = ALIGN_KP_ANGULAR * heading_error_deg
    angular_z = max(-MAX_ALIGN_ANGULAR_Z, min(MAX_ALIGN_ANGULAR_Z, angular_z))
    return ALIGN_CREEP_SPEED_MPS, angular_z
