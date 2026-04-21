import rclpy
import json
import numpy as np
import os
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
from multiprocessing import shared_memory
from rclpy.executors import ExternalShutdownException

class AIInferenceNode(Node):
    def __init__(self):
        super().__init__('ai_inference')
        
        # Path must be relative to the /app directory in Docker
        model_path = 'results/runs/detect/EyeToZion_AI/yolo26_robot_detect/weights/best.pt'
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file missing: {model_path}")
            raise FileNotFoundError("Check your Docker volume mapping.")

        self.model = YOLO(model_path)
        
        # Listen for new frames from CV Processor
        self.subscription = self.create_subscription(String, '/camera/metadata', self.metadata_callback, 10)
        # Publish detections for ROS Core
        self.publisher_ = self.create_publisher(String, '/ai/detections', 10)
        
        self.shm = None
        self.get_logger().info("AI Brain is online.")

    def metadata_callback(self, msg):
        data = json.loads(msg.data)
        
        # Initialize connection to shared memory once
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=data['shm_name'])
                self.get_logger().info("Linked to Shared RAM.")
            except Exception as e:
                self.get_logger().error(f"SHM link failed: {e}")
                return
        
        # Access shared buffer as numpy array
        frame = np.ndarray(data['shape'], dtype=data['dtype'], buffer=self.shm.buf)
        
        # Run inference (threshold lowered for testing)
        results = self.model(frame, verbose=False)
        
        detections = []
        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf > 0.5: # 50% confidence threshold
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    detections.append({
                        "bbox": [int(x1), int(y1), int(x2), int(y2)],
                        "conf": conf
                    })
        
        # Notify the system if robots are found
        if detections:
            self.get_logger().info(f"Target found! Sending {len(detections)} detection(s).")
            detection_msg = String()
            detection_msg.data = json.dumps(detections)
            self.publisher_.publish(detection_msg)

    def cleanup(self):
        if self.shm:
            self.shm.close()
        print("AI Inference: Memory unlinked.")

def main(args=None):
    rclpy.init(args=args)
    node = AIInferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
