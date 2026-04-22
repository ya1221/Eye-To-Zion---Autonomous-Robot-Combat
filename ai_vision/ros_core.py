import rclpy
import json
import redis
import time # Added for sleep
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import ExternalShutdownException

class ROSCoreNode(Node):
    def __init__(self):
        super().__init__('ros_core')
        
        # Connect to Redis with Retry Loop
        self.redis_client = None
        self.connect_to_redis()

        # Internal subscription to AI results
        self.subscription = self.create_subscription(String, '/ai/detections', self.detection_callback, 10)
            
        self.robot_id = "robot_1"
        self.hfov = 141.0
        self.img_width = 640.0
        
        self.get_logger().info("ROS Core (Ambassador) is ONLINE and listening.")

    def connect_to_redis(self):
        """Attempts to connect to Redis until successful."""
        while rclpy.ok():
            try:
                self.get_logger().info("Attempting to connect to Redis...")
                self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
                if self.redis_client.ping():
                    self.get_logger().info("Connected to Redis successfully!")
                    break
            except Exception as e:
                self.get_logger().warn(f"Redis not ready yet: {e}. Retrying in 2 seconds...")
                time.sleep(2)

    def detection_callback(self, msg):
        # Confirming reception from AI
        self.get_logger().info(f"AI MSG RECEIVED: {msg.data}")
        detections = json.loads(msg.data)
        
        for det in detections:
            bbox = det['bbox']
            box_center_x = (bbox[0] + bbox[2]) / 2.0
            img_center_x = self.img_width / 2.0
            offset_x = box_center_x - img_center_x
            angle_degrees = offset_x * (self.hfov / self.img_width)
            
            payload = {
                "robot_id": self.robot_id,
                "angle": round(angle_degrees, 2),
                "confidence": round(det['conf'], 2)
            }
            
            try:
                self.redis_client.publish('/threats', json.dumps(payload))
                # Log only once in a while to avoid flooding
                self.get_logger().info(f"Target at {payload['angle']}° sent to Redis.")
            except Exception as e:
                self.get_logger().error(f"Failed to publish: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ROSCoreNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
