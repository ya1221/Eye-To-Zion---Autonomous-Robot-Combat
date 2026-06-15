import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

class MockArucoCorrector(Node):
    def __init__(self):
        super().__init__('mock_aruco_corrector')
        
        # אנחנו ניצור מפרסם לערוץ התיקון הרשמי של Nav2
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.get_logger().info("Mock ArUco Corrector Node Started.")
        self.get_logger().info("Waiting 5 seconds for Nav2 to stabilize...")
        time.sleep(5)
        
        # שיגור התיקון הראשון
        self.send_correction(x=3.0, y=3.0, yaw=0.0);

    def send_correction(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        
        # הגדרות הכותרת - קריטי ב-ROS2 כדי שהוא ידע שזה ביחס למפת העולם
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # הזרקת הקואורדינטות ("מה שהמצלמה העילית ראתה")
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        
        # חישוב זווית שטוחה ללא סיבוב (רק לצורך הדוגמה)
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0 
        
        # Covariance (מטריצת שונות) - פה אנחנו אומרים ל-Nav2: "אנחנו בטוחים ב-100% שזה המיקום!"
        # (מספרים קטנים מאוד אומרים ודאות גבוהה)
        msg.pose.covariance[0] = 0.01  # ודאות ב-X
        msg.pose.covariance[7] = 0.01  # ודאות ב-Y
        msg.pose.covariance[35] = 0.01 # ודאות בזווית
        
        self.get_logger().info(f"🚨 ALERT: Drift Detected! Snapping robot to Ground Truth: X={x}, Y={y}")
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockArucoCorrector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()