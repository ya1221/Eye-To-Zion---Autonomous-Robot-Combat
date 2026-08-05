#!/usr/bin/env python3
"""
Registers zenoh-bridged topics in the ROS 2 graph so foxglove_bridge
can discover them. Creates publishers that never publish — the actual
data arrives from zenoh-bridge's DDS DataWriters.

Without this, ros2 topic list (and foxglove_bridge) can't see the topics
even though data is flowing at the DDS level.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


class TopicAdvertiser(Node):
    def __init__(self):
        super().__init__('zenoh_topic_advertiser')
        # /tf and /tf_static are NOT needed here — foxglove_bridge
        # always subscribes to those automatically.
        self.create_publisher(LaserScan, '/scan', 10)
        self.create_publisher(OccupancyGrid, '/map', 10)
        self.create_publisher(String, '/robot_description', 10)
        self.get_logger().info('Zenoh topics registered in ROS 2 graph')


def main():
    rclpy.init()
    rclpy.spin(TopicAdvertiser())


if __name__ == '__main__':
    main()