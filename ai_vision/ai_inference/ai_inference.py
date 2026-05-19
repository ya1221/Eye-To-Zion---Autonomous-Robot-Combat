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
        
        self.MODEL_PATH = 'best_ncnn_model'
        self.HFOV = 141.0
        self.IMG_WIDTH = 640.0
        self.IMG_HEIGHT = 320.0
        
        # Tracking & Smoothing Constants
        self.MAX_BLIND_TIME = 2.0
        self.CONF_THRESH = 0.3
        self.DUP_ANGLE_THRESH = 2.0  # Degrees
        self.LPF_ALPHA = 0.2         # Weight for new expansion rate
        
        # Threat Calculation Constants
        self.THREAT_PROXIMITY_MAX = 50
        self.THREAT_VISIBILITY_BONUS = 30
        self.THREAT_STABILITY_HIGH = 20
        self.THREAT_STABILITY_MED = 10
        self.THREAT_RATIO_CLOSE = 0.6
        self.THREAT_VEL_LOW = 5.0
        self.THREAT_VEL_MED = 15.0
        self.THREAT_SCORE_HIGH = 75
        self.THREAT_SCORE_MED = 45
        
        # System Constants
        self.FPS_WARN_THRESH = 7.0
        self.FPS_LOG_INTERVAL = 15

        # Load exported NCNN model
        if not os.path.exists(self.MODEL_PATH):
            self.get_logger().error(f"Model file missing: {self.MODEL_PATH}")
            raise FileNotFoundError("Check Docker volume mapping.")
        self.model = YOLO(self.MODEL_PATH, task='detect')
        
        # ROS 2 Pub/Sub
        self.subscription = self.create_subscription(String, '/camera/metadata', self.metadata_callback, 10)
        self.publisher_ = self.create_publisher(String, '/ai/detections', 10)
        
        self.shm = None
        self.targets_history = {}  
        self.last_time = time.time()
        self.processed_msgs = 0
        
        self.get_logger().info("AI Brain with Ensemble Tracking is online.")

    def _calculate_threat_level(self, box_height, is_predicted, angular_velocity):
        """Calculates tactical threat score based on visual metrics."""
        score = 0
        
        # 1. Proximity Score
        height_ratio = box_height / self.IMG_HEIGHT 
        proximity_score = min((height_ratio / self.THREAT_RATIO_CLOSE) * self.THREAT_PROXIMITY_MAX, self.THREAT_PROXIMITY_MAX) 
        score += proximity_score

        # 2. Visibility Score
        if not is_predicted:
            score += self.THREAT_VISIBILITY_BONUS

        # 3. Angular Stability Score
        abs_vel = abs(angular_velocity)
        if abs_vel < self.THREAT_VEL_LOW:
            score += self.THREAT_STABILITY_HIGH
        elif abs_vel < self.THREAT_VEL_MED:
            score += self.THREAT_STABILITY_MED 

        # Final classification
        if score >= self.THREAT_SCORE_HIGH:
            return "HIGH"
        elif score >= self.THREAT_SCORE_MED:
            return "MEDIUM"
        else:
            return "LOW"
            
    def _update_fps(self):
        """Handles FPS calculation and logging."""
        self.processed_msgs += 1
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time)
        self.last_time = current_time
        
        if fps < self.FPS_WARN_THRESH:
            self.get_logger().warn(f"CRITICAL: FPS dropped to {fps:.2f}!")
        elif self.processed_msgs == self.FPS_LOG_INTERVAL:
            self.get_logger().info(f"Target FPS stable: {fps:.2f}")
            self.processed_msgs = 0
            
        return current_time

    def _get_frame(self, data):
        """Retrieves frame from Shared Memory."""
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=data['shm_name'])
                self.get_logger().info("Linked to Shared RAM.")
            except Exception as e:
                self.get_logger().error(f"SHM link failed: {e}")
                return None
        return np.ndarray(data['shape'], dtype=data['dtype'], buffer=self.shm.buf)

    def _process_visual_detections(self, results, current_time):
        """Processes YOLO output, calculates kinematics, and updates history."""
        visual_detections = []
        current_visible_ids = set()

        for result in results:
            if result.boxes.id is None:
                continue
                
            track_ids = result.boxes.id.int().cpu().tolist()
            boxes = result.boxes.xyxy.cpu().tolist()
            confs = result.boxes.conf.cpu().tolist()

            for box, track_id, conf in zip(boxes, track_ids, confs):
                if conf <= self.CONF_THRESH:
                    continue
                    
                x1, y1, x2, y2 = box
                box_center_x = (x1 + x2) / 2.0
                offset_x = box_center_x - (self.IMG_WIDTH / 2.0)
                angle_degrees = offset_x * (self.HFOV / self.IMG_WIDTH)
                box_height = y2 - y1 

                # Duplicate Prevention Check
                is_duplicate = any(abs(d['angle'] - angle_degrees) < self.DUP_ANGLE_THRESH for d in visual_detections)
                if is_duplicate:
                    continue

                current_visible_ids.add(track_id)

                # Kinematics and Expansion (Sensor Fusion logic)
                velocity = 0.0
                expansion_rate = 0.0

                if track_id in self.targets_history:
                    prev_data = self.targets_history[track_id]
                    dt = current_time - prev_data['last_seen']
                    
                    if dt > 0:
                        velocity = (angle_degrees - prev_data['angle']) / dt
                        raw_expansion = (box_height - prev_data['last_height']) / dt
                        # Exponential Moving Average (LPF)
                        expansion_rate = ((1.0 - self.LPF_ALPHA) * prev_data['exp_rate']) + (self.LPF_ALPHA * raw_expansion)

                # Update History
                self.targets_history[track_id] = {
                    'angle': angle_degrees,
                    'velocity': velocity,
                    'last_height': box_height,
                    'exp_rate': expansion_rate,
                    'last_seen': current_time
                }

                threat_lvl = self._calculate_threat_level(box_height, False, velocity)

                visual_detections.append({
                    "id": track_id,
                    "angle": round(angle_degrees, 2),
                    "threat": threat_lvl,
                    "exp_rate": round(expansion_rate, 2),
                    "predicted": False 
                })
                
        return visual_detections, current_visible_ids

    def _process_blind_predictions(self, current_detections, current_visible_ids, current_time):
        """Generates Kalman-style predictions for occluded targets."""
        all_detections = list(current_detections)
        lost_ids = []
        
        for t_id, data in self.targets_history.items():
            if t_id in current_visible_ids:
                continue
                
            dt = current_time - data['last_seen']
            if dt < self.MAX_BLIND_TIME:
                predicted_angle = data['angle'] + (data['velocity'] * dt)
                
                # Verify predicted space is not occupied by visual detection
                is_covered = any(abs(d['angle'] - predicted_angle) < self.DUP_ANGLE_THRESH for d in all_detections)
                
                if not is_covered:
                    threat_lvl = self._calculate_threat_level(data['last_height'], True, data['velocity'])
                    all_detections.append({
                        "id": t_id,
                        "angle": round(predicted_angle, 2),
                        "threat": threat_lvl,
                        "exp_rate": round(data['exp_rate'], 2),
                        "predicted": True 
                    })
            else:
                lost_ids.append(t_id)
                
        return all_detections, lost_ids

    def metadata_callback(self, msg):
        """Main callback pipeline (Orchestrator)."""
        current_time = self._update_fps()
        data = json.loads(msg.data)
        
        frame = self._get_frame(data)
        if frame is None:
            return
        
        # 1. Run AI Inference
        results = self.model.track(frame, imgsz=int(self.IMG_HEIGHT), persist=True, tracker="bytetrack.yaml", verbose=False)
        
        # 2. Extract Visual Data
        visual_detections, current_visible_ids = self._process_visual_detections(results, current_time)
        
        # 3. Generate Predictions
        final_detections, lost_ids = self._process_blind_predictions(visual_detections, current_visible_ids, current_time)
        
        # 4. Cleanup Memory
        for t_id in lost_ids:
            del self.targets_history[t_id]

        # 5. Publish Results
        if final_detections:
            det_details = ", ".join([f"[ID:{d['id']} | Ang:{d['angle']}° | Threat:{d['threat']} | Pred:{d['predicted']}]" for d in final_detections])
            self.get_logger().info(f"Targets: {det_details}")
            
            detection_msg = String()
            detection_msg.data = json.dumps(final_detections)
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
