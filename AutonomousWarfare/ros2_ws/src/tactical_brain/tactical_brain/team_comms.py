"""JSON-over-topic protocol for team-shared enemy sightings and teammate
positions, mirrored across Zenoh exactly like every other teams/team_X/*
topic (see zenoh-bridge-ros2dds in main_brain.py's node setup).

Owns enemies_by_detector, keyed by whichever robot_id reported each
sighting (my own onboard detection, or a teammate's broadcast) so two
robots simultaneously tracking different enemies don't clobber each
other.
"""
import json
import threading
import time

from std_msgs.msg import String

from tactical_brain import world_model


class TeamComms:
    def __init__(self, ros_node, my_team_idx, robot_id, callback_group=None):
        self.ros_node = ros_node
        self.robot_id = robot_id

        # Guards enemies_by_detector against the executor's two threads:
        # record_local_detection/_team_enemy_callback mutate it IN PLACE
        # on the pose callback group's thread, while get_enemies_snapshot
        # (called from the tree timer thread) iterates it in
        # prune_stale_enemies - a concurrent insert during that iteration
        # raises "dictionary changed size during iteration" and kills the
        # node. teammates_dict is only ever REASSIGNED wholesale (never
        # mutated in place), so it doesn't need this.
        self._state_lock = threading.Lock()
        self.enemies_by_detector = {}
        self.teammates_dict = {}

        self.team_enemy_pub = ros_node.create_publisher(
            String, f'teams/team_{my_team_idx}/detected_enemies', 10)
        self.team_enemy_sub = ros_node.create_subscription(
            String, f'teams/team_{my_team_idx}/detected_enemies',
            self._team_enemy_callback, 10, callback_group=callback_group)

    def record_local_detection(self, x, y):
        """Our own sensor_fusion_node sighting - record it, and don't just
        keep it to ourselves: broadcast it to the rest of the team so every
        teammate's TeamComms merges it into their own picture."""
        now = time.time()
        with self._state_lock:
            self.enemies_by_detector[self.robot_id] = {'x': x, 'y': y, 'timestamp': now}
        self.team_enemy_pub.publish(String(data=json.dumps({
            'detector_id': self.robot_id,
            'x': x,
            'y': y,
            'timestamp': now,
        })))

    def _team_enemy_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        detector_id = data.get('detector_id')
        if detector_id is None or detector_id == self.robot_id:
            # Our own broadcast echoing back - already recorded directly in
            # record_local_detection, no need to process again.
            return

        with self._state_lock:
            self.enemies_by_detector[detector_id] = {
                'x': data['x'],
                'y': data['y'],
                'timestamp': data.get('timestamp', time.time()),
            }

    def get_enemies_snapshot(self):
        """Prune stale sightings (from every robot that's reported one) and
        return a plain snapshot list the rest of a tree tick can read from
        without touching the live dict again."""
        with self._state_lock:
            self.enemies_by_detector = world_model.prune_stale_enemies(self.enemies_by_detector)
            return list(self.enemies_by_detector.values())

    def set_teammates(self, teammates_dict):
        self.teammates_dict = teammates_dict

    def get_teammates(self):
        return self.teammates_dict