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
            raise FileNotFoundError("Check Docker volume mapping.")

        self.model = YOLO(model_path, task='detect')
        
        # ROS 2 Pub/Sub
        self.subscription = self.create_subscription(String, '/camera/metadata', self.metadata_callback, 10)
        self.publisher_ = self.create_publisher(String, '/ai/detections', 10)
        
        self.shm = None
        self.get_logger().info("AI Brain with Ensemble Tracking is online.")

        # Camera FOV constants for angle calculation
        self.hfov = 141.0
        self.img_width = 640.0

        # --- ENSEMBLE TRACKING VARIABLES
        self.targets_history = {}  # Stores {id: {'angle': float, 'velocity': float, 'last_seen': float}}
        self.MAX_BLIND_TIME = 2.0  # How many seconds to keep predicting after YOLO loses sight

        # FPS calculation variables
        self.last_time = time.time()
        self.processed_msgs = 0
        
    def metadata_callback(self, msg):
        self.processed_msgs += 1
            
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time)
        self.last_time = current_time
        
        if fps < 7.0:
            self.get_logger().warn(f"CRITICAL: FPS dropped to {fps:.2f}!")
        elif self.processed_msgs == 15:
            self.get_logger().info(f"Target FPS stable: {fps:.2f}")
            self.processed_msgs = 0
        
        data = json.loads(msg.data)
        
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=data['shm_name'])
                self.get_logger().info("Linked to Shared RAM.")
            except Exception as e:
                self.get_logger().error(f"SHM link failed: {e}")
                return
        
        frame = np.ndarray(data['shape'], dtype=data['dtype'], buffer=self.shm.buf)
        
        # Run YOLO with ByteTrack
        results = self.model.track(frame, imgsz=320, persist=True, tracker="bytetrack.yaml", verbose=False)
        
        detections = []
        current_visible_ids = set()

        # 1. PROCESS VISUAL DETECTIONS (YOLO)
        for result in results:
            if result.boxes.id is not None:
                track_ids = result.boxes.id.int().cpu().tolist()
                boxes = result.boxes.xyxy.cpu().tolist()
                confs = result.boxes.conf.cpu().tolist()

                for box, track_id, conf in zip(boxes, track_ids, confs):
                    if conf > 0.3:
                        x1, y1, x2, y2 = box
                        box_center_x = (x1 + x2) / 2.0
                        offset_x = box_center_x - (self.img_width / 2.0)
                        angle_degrees = offset_x * (self.hfov / self.img_width)

                        # DUPLICATE PREVENTION CHECK
                        is_duplicate = False
                        for d in detections:
                            # Ignore the current one
                            if abs(d['angle'] - angle_degrees) < 2.0:
                                is_duplicate = True
                                break
                        
                        if is_duplicate:
                            continue

                        current_visible_ids.add(track_id)

                        # Calculate angular velocity
                        velocity = 0.0
                        if track_id in self.targets_history:
                            prev_data = self.targets_history[track_id]
                            dt = current_time - prev_data['last_seen']
                            if dt > 0:
                                velocity = (angle_degrees - prev_data['angle']) / dt

                        # Update History
                        self.targets_history[track_id] = {
                            'angle': angle_degrees,
                            'velocity': velocity,
                            'last_seen': current_time
                        }

                        detections.append({
                            "id": track_id,
                            "angle": round(angle_degrees, 2),
                            "predicted": False 
                        })
        
        # 2. PROCESS BLIND PREDICTIONS (KALMAN-STYLE ENSEMBLE)
        lost_ids = []
        for t_id, data in self.targets_history.items():
            if t_id not in current_visible_ids:
                dt = current_time - data['last_seen']
                if dt < self.MAX_BLIND_TIME:
                    predicted_angle = data['angle'] + (data['velocity'] * dt)
                    is_covered = False
                    for d in detections:
                        if abs(d['angle'] - predicted_angle) < 2.0:
                            is_covered = True
                            break
                    
                    if not is_covered:
                        detections.append({
                            "id": t_id,
                            "angle": round(predicted_angle, 2),
                            "predicted": True 
                        })
                else:
                    lost_ids.append(t_id)

        # Cleanup memory for targets gone too long
        for t_id in lost_ids:
            del self.targets_history[t_id]

        # Publish final processed data
        if detections:
            det_details = ", ".join([f"[ID:{d['id']} | Ang:{d['angle']}° | Pred:{d['predicted']}]" for d in detections])
            self.get_logger().info(f"Targets: {det_details}")
            
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
