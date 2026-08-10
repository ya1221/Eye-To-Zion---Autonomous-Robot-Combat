"""JSON-over-topic protocol for team-shared enemy sightings, teammate positions, distress calls, and arena-wide death broadcasts - all mirrored across Zenoh like every other teams/*/* topic (see zenoh-bridge-ros2dds in main_brain.py).
Owns enemies_by_detector and teammate_help_by_id, both keyed by robot_id so two robots tracking different enemies/status don't clobber each other."""
import json
import threading
import time

from std_msgs.msg import String

from tactical_brain import world_model


class TeamComms:
    def __init__(self, ros_node, my_team_idx, robot_id, callback_group=None):
        self.ros_node = ros_node
        self.robot_id = robot_id

        # Guards enemies_by_detector/teammate_help_by_id/dead_robot_locations against the executor's two threads: callbacks mutate them IN PLACE on the pose callback group's thread while tree-timer-thread getters iterate them, and a concurrent insert raises "dictionary/set changed size during iteration" (confirmed for enemies_by_detector, see 667a459).
        # teammates_dict is only ever REASSIGNED wholesale (never mutated in place), so it doesn't need this.
        self._state_lock = threading.Lock()
        self.enemies_by_detector = {}
        self.teammates_dict = {}

        self.team_enemy_pub = ros_node.create_publisher(
            String, f'teams/team_{my_team_idx}/detected_enemies', 10)
        self.team_enemy_sub = ros_node.create_subscription(
            String, f'teams/team_{my_team_idx}/detected_enemies',
            self._team_enemy_callback, 10, callback_group=callback_group)

        # Distress calls: each robot broadcasts its health/outnumbered assessment every sense_and_think tick (not event-driven, so a dropped "I'm fine now" can't leave a teammate stuck believing help is still needed), keyed by robot_id like enemies_by_detector above.
        # Position still comes from teammates_dict/positions - this topic only carries the needs_help/is_fighting booleans nothing else provides.
        self.teammate_help_by_id = {}
        self.team_status_pub = ros_node.create_publisher(
            String, f'teams/team_{my_team_idx}/status', 10)
        self.team_status_sub = ros_node.create_subscription(
            String, f'teams/team_{my_team_idx}/status',
            self._team_status_callback, 10, callback_group=callback_group)

        # Death broadcasts from HP nodes are deliberately not team-scoped so both teams receive them. These location sets are merged into static_obstacles since corpses only matter for their position and never expire.
        self.dead_robot_locations = set()
        self.death_message_sub = ros_node.create_subscription(
            String, 'teams/arena/death_message',
            self._death_message_callback, 10, callback_group=callback_group)

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

    def publish_status(self, needs_help, is_fighting):
        """Broadcast my own distress-call state - see main_brain.py's
        sense_and_think, which calls this every tick with am_i_in_danger/
        am_i_fighting so the two can never drift apart."""
        self.team_status_pub.publish(String(data=json.dumps({
            'id': self.robot_id,
            'needs_help': needs_help,
            'is_fighting': is_fighting,
            'timestamp': time.time(),
        })))

    def _team_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        sender_id = data.get('id')
        if sender_id is None or sender_id == self.robot_id:
            # Our own broadcast echoing back - already known directly via
            # the local am_i_in_danger/am_i_fighting blackboard values, no
            # need to process again.
            return

        with self._state_lock:
            self.teammate_help_by_id[sender_id] = {
                'needs_help': data.get('needs_help', False),
                'is_fighting': data.get('is_fighting', False),
                'timestamp': data.get('timestamp', time.time()),
            }

    def prune_stale_help_status(self):
        with self._state_lock:
            self.teammate_help_by_id = world_model.prune_stale_enemies(self.teammate_help_by_id)

    def get_teammate_help_status(self, teammate_id):
        with self._state_lock:
            return self.teammate_help_by_id.get(teammate_id)

    def _death_message_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        x, y = data.get('x'), data.get('y')
        if x is None or y is None:
            return

        with self._state_lock:
            self.dead_robot_locations.add((x, y))

    def get_dead_robot_locations(self):
        with self._state_lock:
            return set(self.dead_robot_locations)