import rclpy
import json
import numpy as np
import os
import time
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
from multiprocessing import shared_memory
from rclpy.executors import ExternalShutdownException

class AIInferenceNode(Node):
    def __init__(self):
        super().__init__('ai_inference')
        
        # Load exported NCNN model
        model_path = 'best_ncnn_model'
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file missing: {model_path}")
            raise FileNotFoundError("Check your Docker volume mapping.")

        self.model = YOLO(model_path, task='detect')
        
        # ROS 2 Pub/Sub
        self.subscription = self.create_subscription(String, '/camera/metadata', self.metadata_callback, 10)
        self.publisher_ = self.create_publisher(String, '/ai/detections', 10)
        
        self.shm = None
        self.get_logger().info("AI Brain with Tracking is online.")

        # Camera FOV constants for angle calculation
        self.hfov = 141.0
        self.img_width = 640.0

        # FPS calculation variables
        self.last_time = time.time()
        self.frame_count = 0
        
    def metadata_callback(self, msg):
        # Skip alternating frames for performance
        self.frame_count += 1
        if self.frame_count % 2 != 0:
            return
            
        # Calculate instantaneous FPS
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time)
        self.last_time = current_time
        
        # Log critical FPS drops or occasional heartbeat
        if fps < 7.0:
            self.get_logger().warn(f"CRITICAL: FPS dropped to {fps:.2f}!")
        elif (self.frame_count // 2) % 10 == 0:
            self.get_logger().info(f"FPS: {fps:.2f}")
        
        data = json.loads(msg.data)
        
        # Connect to shared memory block if not already linked
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=data['shm_name'])
                self.get_logger().info("Linked to Shared RAM.")
            except Exception as e:
                self.get_logger().error(f"SHM link failed: {e}")
                return
        
        # Construct numpy array from shared memory buffer
        frame = np.ndarray(data['shape'], dtype=data['dtype'], buffer=self.shm.buf)
        
        # Run inference WITH tracking (ByteTrack)
        results = self.model.track(frame, imgsz=320, persist=True, tracker="bytetrack.yaml", verbose=False)
        
        detections = []
        for result in results:
            # Check if tracker assigned an ID to the detections
            if result.boxes.id is not None:
                track_ids = result.boxes.id.int().cpu().tolist()
                boxes = result.boxes.xyxy.cpu().tolist()
                confs = result.boxes.conf.cpu().tolist()

                for box, track_id, conf in zip(boxes, track_ids, confs):
                    #if conf > 0.75:
                    if conf > 0.3:
                        x1, y1, x2, y2 = box
                        
                        # Calculate real-world angle from bounding box center
                        box_center_x = (x1 + x2) / 2.0
                        offset_x = box_center_x - (self.img_width / 2.0)
                        angle_degrees = offset_x * (self.hfov / self.img_width)

                        detections.append({
                            "id": track_id,
                            "angle": round(angle_degrees, 2)
                        })
        
        # Publish final processed data to ROS internal network
        if detections:
            self.get_logger().info(f"Targets found! Sending {len(detections)} detection(s) to ROS.")
            detection_msg = String()
            detection_msg.data = json.dumps(detections)
            self.publisher_.publish(detection_msg)

    def cleanup(self):
        # Safely release shared memory
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
