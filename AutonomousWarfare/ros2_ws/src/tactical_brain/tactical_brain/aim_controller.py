"""Alignment (aiming) control law for the attack branch. Provides pure math calculations to creep forward and proportionally turn, since Ackermann steering cannot pivot in place."""

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

# Sign relating YOLO's 'angle' to ROS +angular.z; flip to -1.0 if the real robot steers away from the target.
AIM_ANGULAR_SIGN = 1.0


def compute_alignment_twist(heading_error_deg, distance_m):
    """Returns the required (linear_x_mps, angular_z_rad_s) twist to reduce the heading error while maintaining the standoff distance. Caller must ensure heading_error_deg is not None."""
    angular_z = AIM_ANGULAR_SIGN * ALIGN_KP_ANGULAR * heading_error_deg
    angular_z = max(-MAX_ALIGN_ANGULAR_Z, min(MAX_ALIGN_ANGULAR_Z, angular_z))
    linear_x = ALIGN_CREEP_SPEED_MPS if (distance_m is not None and distance_m > STANDOFF_DISTANCE_METERS) else 0.0
    return linear_x, angular_z
