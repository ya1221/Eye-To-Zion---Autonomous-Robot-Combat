import json
import math
import os
import queue

import zenoh

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from tactical_brain import zenoh_manager


class ZenohNode(Node):
    """Owns the Zenoh session and republishes the decoded fleet world-state
    on plain ROS topics, so main_brain (and anything else) can consume it
    like any other sensor source instead of being handed a queue directly."""

    def __init__(self):
        super().__init__('zenoh_node')
        self.msg_queue = queue.Queue()
        self.enemies_list = []
        self.teammates_dict = {}

        # Anchor IP comes in as a ROS 2 parameter (set from the
        # ZENOH_ANCHOR_ENDPOINT env var by the launch file) rather than
        # being hardcoded, per the anchor/gossip architecture. Empty by
        # default so solo Gazebo testing doesn't require a live anchor.
        self.declare_parameter('zenoh_anchor_endpoint', os.environ.get('ZENOH_ANCHOR_ENDPOINT', ''))
        anchor_endpoint = self.get_parameter('zenoh_anchor_endpoint').get_parameter_value().string_value

        # איזו קבוצה אנחנו - קובע לאיזה teams/team_{idx}/positions להאזין
        self.declare_parameter('my_team_idx', 0)
        my_team_idx = self.get_parameter('my_team_idx').get_parameter_value().integer_value

        self.zenoh_session = None
        if anchor_endpoint:
            try:
                conf = zenoh.Config()
                conf.insert_json5("mode", '"peer"')
                conf.insert_json5("scouting/gossip/enabled", "true")
                conf.insert_json5("connect/endpoints", json.dumps([anchor_endpoint]))
                self.zenoh_session = zenoh.open(conf)
                self.get_logger().info(f"Connected to Zenoh anchor at {anchor_endpoint}")
            except Exception as e:
                self.get_logger().warn(
                    f"Could not reach Zenoh anchor at {anchor_endpoint} ({e}). "
                    "Continuing without fleet comms - enemies/teammates will stay empty."
                )
        else:
            self.get_logger().warn(
                "zenoh_anchor_endpoint not set - running without fleet comms "
                "(expected for solo Gazebo testing)."
            )

        # פונקציית הקולבק של Zenoh שדוחפת הודעות לתור שלנו
        def zenoh_listener(sample):
            self.msg_queue.put({
                "channel": str(sample.key_expr),
                "data": bytes(sample.payload).decode('utf-8')
            })

        if self.zenoh_session is not None:
            TEAM_PREFIX = "team_blue"
            self.zenoh_session.declare_subscriber(f'{TEAM_PREFIX}/detected_enemies', zenoh_listener)
            self.zenoh_session.declare_subscriber(f'{TEAM_PREFIX}/team_positions', zenoh_listener)
            self.zenoh_session.declare_subscriber(f'teams/team_{my_team_idx}/positions', zenoh_listener)

        self.enemies_pub = self.create_publisher(String, '/world/enemies', 10)
        self.teammates_pub = self.create_publisher(String, '/world/teammates', 10)
        # הבקר הסי++ (PID) מאזין כאן ישירות לפוזיציית האמת מצמלת הארוקו
        self.aruco_odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        self.timer = self.create_timer(0.5, self.process_and_publish)

    def process_and_publish(self):
        self.enemies_list, self.teammates_dict, self_pose = zenoh_manager.check_zenoh_updates(
            self.msg_queue, self.enemies_list, self.teammates_dict
        )

        self.enemies_pub.publish(String(data=json.dumps(self.enemies_list)))
        self.teammates_pub.publish(String(data=json.dumps(self.teammates_dict)))

        if self_pose is not None:
            self._publish_self_pose(*self_pose)

    def _publish_self_pose(self, x_m, y_m, angle_rad):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = x_m
        odom_msg.pose.pose.position.y = y_m
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(angle_rad / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(angle_rad / 2.0)
        self.aruco_odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZenohNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
