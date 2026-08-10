"""Node-level orchestration around the attack branch's shooting outputs: publishes shooting_node's mode/firing topics and calls its fire_once service, gated by ballistics_helper's debounced trigger decision.
Kept separate from ballistics_helper.py's pure threshold/hysteresis math, and from AttackAction's per-tick call site."""
import std_srvs.srv
from std_msgs.msg import String, Bool

from tactical_brain import ballistics_helper


class ShootingControl:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        # One shared TriggerController instance so both AttackAction's
        # per-tick evaluate_and_fire() and the attack-branch safety net's
        # reset() agree on the same debounce/hysteresis state.
        self.trigger_controller = ballistics_helper.TriggerController()
        # Topic/service names matching shooting_node's interface.
        self.firing_pub = ros_node.create_publisher(Bool, '/firing', 10)
        self.shooting_mode_pub = ros_node.create_publisher(String, '/shooting_mode', 10)
        self.fire_once_client = ros_node.create_client(
            std_srvs.srv.Trigger, '/shooting_node/fire_once')

    def evaluate_and_fire(self, distance_m, heading_error_deg, mode):
        """Unified shooting control - checks angle/distance via the
        debounced TriggerController (instead of a simple, noise-sensitive
        if), then manages mode/firing/fire_once."""
        should_fire = self.trigger_controller.evaluate(distance_m, heading_error_deg)

        threshold = ballistics_helper.allowable_angle_deg(distance_m) if distance_m is not None else 0.0
        frames = self.trigger_controller._consecutive_frames
        heading_err_str = f"{heading_error_deg:.1f}" if heading_error_deg is not None else "None"
        dist_str = f"{distance_m:.2f}" if distance_m is not None else "None"

        self.ros_node.get_logger().info(
            f"Shooting Control | dist: {dist_str}m | heading_err: {heading_err_str}° "
            f"| allow_angle: {threshold:.1f}° | frames: {frames}/{ballistics_helper.REQUIRED_CONSECUTIVE_FRAMES} "
            f"| should_fire: {should_fire} | mode: {mode}"
        )

        # Publish the current mode every tick so shooting_node stays in sync
        self.shooting_mode_pub.publish(String(data=mode))

        if not should_fire:
            self.firing_pub.publish(Bool(data=False))
            return should_fire

        # Angle within threshold - fire
        if mode == 'auto':
            self.firing_pub.publish(Bool(data=True))
        else:
            # Single mode: enable firing + call fire_once service
            self.firing_pub.publish(Bool(data=True))
            if self.fire_once_client.service_is_ready():
                self.fire_once_client.call_async(std_srvs.srv.Trigger.Request())
            else:
                self.ros_node.get_logger().warn('fire_once service not ready')

        return should_fire

    def cease_fire(self):
        """Kill all shooting outputs - used by the attack-branch safety net."""
        self.firing_pub.publish(Bool(data=False))

    def reset(self):
        self.trigger_controller.reset()