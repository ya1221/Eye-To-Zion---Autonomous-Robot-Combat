import rclpy
import json
import redis
import time
import threading
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

class RedisBridgeNode(Node):
    def __init__(self):
        super().__init__('redis_bridge')
        
        # Connect to Redis
        self.redis_client = None
        self.connect_to_redis()

        # Listen to external fleet comms in the background
        self.pubsub = self.redis_client.pubsub()
        self.pubsub.subscribe('fleet_tactical_channel')
        
        # Start a background thread to avoid blocking ROS operations
        self.listen_thread = threading.Thread(target=self.redis_listen_loop, daemon=True)
        self.listen_thread.start()
        
        self.get_logger().info("Redis Bridge is ONLINE. Listening to Fleet.")

    def connect_to_redis(self):
        # Retry loop to connect to Redis server
        while rclpy.ok():
            try:
                self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
                if self.redis_client.ping():
                    self.get_logger().info("Connected to Redis Hub.")
                    break
            except Exception as e:
                self.get_logger().warn(f"Redis not ready: {e}. Retrying in 2s...")
                time.sleep(2)

    def redis_listen_loop(self):
        """Background listener for other robots' messages"""
        try:
            for message in self.pubsub.listen():
                if message['type'] == 'message':
                    fleet_data = message['data']
                    self.get_logger().info(f"FLEET MSG: {fleet_data}")
                    # Future integration: Convert fleet_data to ROS message and publish internally
        except Exception as e:
            self.get_logger().error(f"Redis listener crashed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RedisBridgeNode()
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
