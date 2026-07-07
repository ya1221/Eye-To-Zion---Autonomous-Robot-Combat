"""Firing-envelope and trigger-debounce logic for the attack branch.

Deliberately free of rclpy/ROS imports (same separation as A_planner.py)
so the threshold math and the hysteresis state machine can be unit tested,
or reasoned about, without a running node.
"""

# --- Firing envelope hyperparameters (degrees, meters) ---

# Angle budget at point-blank range - wide, since even a poorly-aimed
# chassis is usually still roughly on-target at close distance.
MAX_ALLOWABLE_ANGLE = 15.0

# Angle budget at MAX_FIRING_DISTANCE - narrow, but not zero: leaves a
# baseline margin for heading/pose noise rather than demanding a perfect
# lock that noise could never satisfy.
MIN_ALLOWABLE_ANGLE = 2.0

# Beyond this range the target is out of range outright, regardless of
# how well-aimed the chassis is.
MAX_FIRING_DISTANCE = 3.0

# Degrees of allowable-angle shrink per additional meter of distance.
# Sized so the envelope reaches MIN_ALLOWABLE_ANGLE exactly at
# MAX_FIRING_DISTANCE: (15 - 2) / 3.0.
DISTANCE_SLOPE = (MAX_ALLOWABLE_ANGLE - MIN_ALLOWABLE_ANGLE) / MAX_FIRING_DISTANCE

# Once firing, the envelope widens by this many degrees before cutting
# off, so noise/vibration near the boundary doesn't chatter the trigger.
HYSTERESIS_MARGIN_DEG = 2.0

# Consecutive ticks the target must stay inside the (tight, non-widened)
# envelope before the trigger commits to firing.
REQUIRED_CONSECUTIVE_FRAMES = 5


def allowable_angle_deg(distance_m):
    """Linear, bounded firing-angle threshold for a given target distance."""
    raw = MAX_ALLOWABLE_ANGLE - DISTANCE_SLOPE * distance_m
    return max(MIN_ALLOWABLE_ANGLE, min(MAX_ALLOWABLE_ANGLE, raw))


class TriggerController:
    """Debounced, hysteresis-gated trigger decision for one tracked target.

    Call evaluate() once per tick with the latest (distance, heading_error).
    Call reset() whenever the target is no longer being actively engaged
    (out of range, lost, or the attack branch isn't selected this tick) -
    the caller owns detecting that, since a single-tick py_trees leaf that
    always returns SUCCESS can't reliably observe "I was skipped this tick"
    through terminate()/initialise() (see main_brain.py's sense_and_think).
    """

    def __init__(self):
        self._consecutive_frames = 0
        self._firing = False

    def reset(self):
        self._consecutive_frames = 0
        self._firing = False

    def evaluate(self, distance_m, heading_error_deg):
        if distance_m is None or heading_error_deg is None or distance_m > MAX_FIRING_DISTANCE:
            self.reset()
            return False

        threshold = allowable_angle_deg(distance_m)
        band = threshold + HYSTERESIS_MARGIN_DEG if self._firing else threshold

        if abs(heading_error_deg) <= band:
            self._consecutive_frames = min(self._consecutive_frames + 1, REQUIRED_CONSECUTIVE_FRAMES)
        else:
            self._consecutive_frames = 0

        self._firing = self._consecutive_frames >= REQUIRED_CONSECUTIVE_FRAMES
        return self._firing
