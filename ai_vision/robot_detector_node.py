import cv2
import json
import redis
import rclpy
import torch
import time
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

class RobotDetectorNode(Node):
    def __init__(self):
        super().__init__('robot_detector_node')
        
        # ROS2 publisher
        self.publisher_ = self.create_publisher(String, '/robot_detection', 10)
        
        # Timer (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.model = YOLO('results/runs/detect/EyeToZion_AI/yolo26_robot_detect/weights/best.pt')
        
        # Camera setup
        self.HFOV = 141.0
        self.cap = cv2.VideoCapture(0)
        
        # Redis setup
        try:
            self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
            self.redis_client.ping()
            self.get_logger().info("Redis connected")
        except redis.ConnectionError:
            self.get_logger().error("Redis connection failed")

    def timer_callback(self):
        start_time = time.time()
        success, frame = self.cap.read()
        if not success:
            self.get_logger().warn("End of video or failed to grab frame.")
            return
        img_width = frame.shape[1]
        img_center_x = img_width / 2

        results = self.model(frame, verbose=False)

        for result in results:
            for box in result.boxes:
                if box.conf[0] > 0.75:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    box_center_x = (x1 + x2) / 2
                    offset_x = box_center_x - img_center_x
                    angle_degrees = offset_x * (self.HFOV / img_width)
                    confidence_percent = int(box.conf[0] * 100)
                    
                    # Prepare payload
                    payload = {
                        "robot_id": "robot_1",
                        "angle": round(angle_degrees, 2)
                    }
                    json_data = json.dumps(payload)
                    
                    # Publish to Redis
                    self.redis_client.publish('/threats', json_data)
                    
                    # Publish to ROS2
                    msg = String()
                    msg.data = json_data
                    self.publisher_.publish(msg)
                    
                    # Draw UI
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                    cv2.putText(frame, f"Conf: {confidence_percent}% | Angle: {payload['angle']}", (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
	
        end_time = time.time()
        processing_time = end_time - start_time
        
        if processing_time > 0:
            fps = 1.0 / processing_time
        else:
            fps = 0.0
            
        cv2.putText(frame, f"FPS: {round(fps, 1)}", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 3)
        cv2.imshow("ROS2 Camera Node", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RobotDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
