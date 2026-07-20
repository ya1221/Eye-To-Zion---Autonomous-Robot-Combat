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
from rclpy.qos import qos_profile_sensor_data

# Same value/reasoning as main_brain.py's own dead-robot-cell merge
# (team_comms.py's dead_robot_locations) - kept independent since these
# are separate packages/processes, not shared state. Covers robot
# footprint + normal fusion noise, not a precise measurement.
DEAD_ROBOT_MATCH_RADIUS_METERS = 0.5

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
        # ה-YOLO משדר JSON בתוך String שאין לו header.stamp, ולכן אי אפשר להשתמש ב-message_filters.
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

        # 3. Publisher למיקום האויב המותך
        self.enemy_pose_pub = self.create_publisher(
            PoseStamped,
            '~/local_enemy_position',
            10
        )

        # 4. Death broadcast - published by a (separate, not-this-package)
        # HP/hit-detection node once a robot's HP reaches 0. Not team-scoped
        # (teams/arena/..., matches the Zenoh bridge's ^/teams/.* allowlist
        # regardless) since a dead enemy needs filtering out here the same
        # as a dead teammate would. Own copy of this data, not shared with
        # tactical_brain_node - separate process, and this one's need
        # (filter before ever publishing) is different from that one's
        # (mark as a planning obstacle).
        self.dead_robot_locations = set()
        self.death_message_sub = self.create_subscription(
            String,
            'teams/arena/death_message',
            self.death_message_callback,
            10
        )

    def death_message_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        x, y = data.get('x'), data.get('y')
        if x is None or y is None:
            return

        self.dead_robot_locations.add((x, y))

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

    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg

    def yolo_callback(self, yolo_msg):
        """מבצע היתוך מידע בין זיהוי ה-YOLO לסריקת הלייזר האחרונה"""
        lidar_msg = self.latest_lidar_msg
        if lidar_msg is None:
            return
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

        # חיפוש חלון קטן סביב האינדקס כדי להתגבר על קריאות 0.0 מקומיות
        window_size = 5 # +/- 5 קרניים
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
            
        # ניקח את המרחק המינימלי (הקרוב ביותר) בחלון זה (כנראה הפגיעה באויב)
        distance = min(valid_distances)

        # היתוך לקרטזי גלובלי
        global_enemy_angle = self.robot_yaw + relative_angle_rad
        enemy_x = self.robot_x + (distance * math.cos(global_enemy_angle))
        enemy_y = self.robot_y + (distance * math.sin(global_enemy_angle))

        # A dead robot's body still sits there and can still trigger a
        # YOLO+lidar detection - filter it out here, at the source, rather
        # than publish a "live enemy" reading for a corpse (see
        # death_message_callback). math.hypot inline rather than a
        # distance() helper - that name's already taken above by the
        # lidar range reading.
        for dead_x, dead_y in self.dead_robot_locations:
            if math.hypot(enemy_x - dead_x, enemy_y - dead_y) <= DEAD_ROBOT_MATCH_RADIUS_METERS:
                return

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