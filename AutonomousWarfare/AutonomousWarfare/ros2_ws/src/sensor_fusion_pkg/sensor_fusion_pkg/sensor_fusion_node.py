#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import json

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String # ה-YOLO שלך שולח String עם JSON
import message_filters

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')
        
        self.get_logger().info('Initializing Sensor Fusion Node for Target Tracking...')

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_robot_pose = False

        # 1. Subscriber למיקום הגלובלי של הרובוט
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

        # 2. סנכרון אסינכרוני בין המצלמה (YOLO) ללידאר
        # ה-YOLO משדר JSON בתוך String לערוץ /ai/detections
        self.yolo_sub = message_filters.Subscriber(self, String, '/ai/detections')
        self.lidar_sub = message_filters.Subscriber(self, LaserScan, '/scan')

        # סנכרון עם גמישות
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.yolo_sub, self.lidar_sub], 
            queue_size=10, 
            slop=0.05 
        )
        self.ts.registerCallback(self.fusion_callback)

        # 3. Publisher למיקום האויב המותך 
        self.enemy_pose_pub = self.create_publisher(
            PoseStamped,
            '~/local_enemy_position',
            10
        )

    def robot_pose_callback(self, msg):
        """מעדכן את המיקום והאוריינטציה (Yaw) הנוכחיים של הרובוט במגרש"""
        # Odometry nests one level deeper than the old PoseStamped did:
        # msg.pose.pose (PoseWithCovariance.pose), not msg.pose.
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.has_robot_pose = True

    def fusion_callback(self, yolo_msg, lidar_msg):
        """מבצע היתוך מידע בין זיהוי ה-YOLO לסריקת הלייזר"""
        if not self.has_robot_pose:
            self.get_logger().warn('Missing global robot pose. Skipping fusion.')
            return

        try:
            # פענוח ה-JSON מה-YOLO
            detections = json.loads(yolo_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('Failed to parse YOLO detections.')
            return

        if not detections:
            return

        # כרגע, אנחנו מתמקדים באויב אחד (אפשר לשדרג לרבים בהמשך)
        # ניקח את האויב הראשון ברשימה
        target = detections[0]
        
        # ה-YOLO שלך מחשב זווית במעלות (angle_degrees). אנחנו חייבים להמיר לרדיאנים.
        relative_angle_deg = target.get('angle', 0.0)
        relative_angle_rad = math.radians(relative_angle_deg)

        if lidar_msg.angle_increment == 0:
            return

        # מציאת האינדקס בלידאר בעזרת הזווית (ברדיאנים!)
        lidar_index = int((relative_angle_rad - lidar_msg.angle_min) / lidar_msg.angle_increment)

        if lidar_index < 0 or lidar_index >= len(lidar_msg.ranges):
            self.get_logger().error('Target angle out of LiDAR bounds.')
            return

        distance = lidar_msg.ranges[lidar_index]

        # סינון קריאות לא תקינות
        if math.isinf(distance) or math.isnan(distance) or distance < lidar_msg.range_min or distance > lidar_msg.range_max:
            return

        # היתוך לקרטזי גלובלי
        global_enemy_angle = self.robot_yaw + relative_angle_rad
        enemy_x = self.robot_x + (distance * math.cos(global_enemy_angle))
        enemy_y = self.robot_y + (distance * math.sin(global_enemy_angle))

        # פרסום
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