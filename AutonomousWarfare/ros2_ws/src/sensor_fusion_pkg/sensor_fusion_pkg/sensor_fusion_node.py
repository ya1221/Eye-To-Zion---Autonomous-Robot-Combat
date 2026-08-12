#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import json

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String # your YOLO node sends a String containing JSON
import message_filters
from rclpy.qos import qos_profile_sensor_data

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        self.get_logger().info('Initializing Sensor Fusion Node for Target Tracking...')

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_robot_pose = False

        # 1. Subscriber for the robot's global position
        # Was '~/aruco_global_pose' (PoseStamped) - nothing in this stack
        # ever published there, so fusion_callback never actually ran
        # (has_robot_pose stayed False forever). /odometry/global is
        # ekf_global's real output (map->odom, ArUco-corrected) - same
        # source main_brain.py's pose_sub and hardware's pid_controller.cpp
        # both use, so this node agrees with the rest of the stack.
        self.pose_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.robot_pose_callback,
            10
        )

        # 2. Asynchronous synchronization between the camera (YOLO) and the lidar
        # YOLO publishes JSON inside a String with no header.stamp, so message_filters can't be used.
        self.latest_lidar_msg = None
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )
        
        self.yolo_sub = self.create_subscription(
            String,
            '/ai/detections',
            self.yolo_callback,
            10
        )

        # 3. Publisher for the fused enemy position
        self.enemy_pose_pub = self.create_publisher(
            PoseStamped,
            '~/local_enemy_position',
            10
        )

    def robot_pose_callback(self, msg):
        """Updates the robot's current position and orientation (yaw) in the arena"""
        # Odometry nests one level deeper than the old PoseStamped did:
        # msg.pose.pose (PoseWithCovariance.pose), not msg.pose.
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.has_robot_pose = True

    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg

    def yolo_callback(self, yolo_msg):
        """Fuses the YOLO detection with the most recent lidar scan"""
        lidar_msg = self.latest_lidar_msg
        if lidar_msg is None:
            return
        if not self.has_robot_pose:
            self.get_logger().warn('Missing global robot pose. Skipping fusion.')
            return

        try:
            # Parse the JSON from YOLO
            detections = json.loads(yolo_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse YOLO detections.')
            return

        if not detections:
            return

        # For now, we focus on a single enemy (can be upgraded to multiple later)
        # Take the first enemy in the list
        target = detections[0]

        # Your YOLO node computes angle in degrees (angle_degrees). We need to convert to radians.
        relative_angle_deg = target.get('angle', 0.0)
        relative_angle_rad = math.radians(relative_angle_deg)

        if lidar_msg.angle_increment == 0:
            return

        # Find the lidar index using the angle (in radians!)
        lidar_index = int((relative_angle_rad - lidar_msg.angle_min) / lidar_msg.angle_increment)

        if lidar_index < 0 or lidar_index >= len(lidar_msg.ranges):
            self.get_logger().error('Target angle out of LiDAR bounds.')
            return

        # Search a small window around the index to overcome local 0.0 readings
        window_size = 5 # +/- 5 beams
        start_idx = max(0, lidar_index - window_size)
        end_idx = min(len(lidar_msg.ranges), lidar_index + window_size + 1)
        
        valid_distances = []
        for i in range(start_idx, end_idx):
            d = lidar_msg.ranges[i]
            if not math.isinf(d) and not math.isnan(d) and d >= lidar_msg.range_min and d <= lidar_msg.range_max:
                valid_distances.append(d)
                
        if not valid_distances:
            self.get_logger().warn(f'No valid LiDAR reading around angle {relative_angle_deg:.2f}')
            return
            
        # Take the minimum (closest) distance in this window (likely the hit on the enemy)
        distance = min(valid_distances)

        # Fuse into global cartesian coordinates
        global_enemy_angle = self.robot_yaw + relative_angle_rad
        enemy_x = self.robot_x + (distance * math.cos(global_enemy_angle))
        enemy_y = self.robot_y + (distance * math.sin(global_enemy_angle))

        # Publish
        output_msg = PoseStamped()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.header.frame_id = 'map'
        output_msg.pose.position.x = enemy_x
        output_msg.pose.position.y = enemy_y
        output_msg.pose.position.z = 0.0 
        
        self.enemy_pose_pub.publish(output_msg)
        self.get_logger().info(f'Target [ID:{target.get("id")}] tracked globally: X={enemy_x:.2f}, Y={enemy_y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()