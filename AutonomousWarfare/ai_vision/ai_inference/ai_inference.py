import rclpy
import json
import numpy as np
import os
import time
import gc
import cv2
import ncnn
import torch
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from multiprocessing import shared_memory

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

        # Load NCNN model
        if not os.path.exists(os.path.join(self.MODEL_PATH, 'model.ncnn.param')):
            self.get_logger().error(f"Model file missing: {self.MODEL_PATH}/model.ncnn.param")
            raise FileNotFoundError("Check Docker volume mapping.")
        
        self.ncnn_net = ncnn.Net()
        self.ncnn_net.load_param(os.path.join(self.MODEL_PATH, 'model.ncnn.param'))
        self.ncnn_net.load_model(os.path.join(self.MODEL_PATH, 'model.ncnn.bin'))
        self.get_logger().info(f"✓ NCNN Model loaded from {self.MODEL_PATH}")
        
        # ROS 2 Pub/Sub with Zero-Lag QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.subscription = self.create_subscription(String, '/camera/metadata', self.metadata_callback, qos_profile)
        self.publisher_ = self.create_publisher(String, '/ai/detections', 10)
        
        self.shm = None
        self.shared_frame = None  # Store reference to current frame
        self.targets_history = {}  
        self.last_time = time.time()
        self.processed_msgs = 0
        self.frame_count = 0
        self.inference_times = []  # Track inference duration
        self.skip_count = 0  # Adaptive frame skipping
        
        self.get_logger().info("AI Brain with NCNN acceleration is online.")

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
        delta_time = current_time - self.last_time
        fps = 1.0 / delta_time if delta_time > 0 else 0
        self.last_time = current_time
        
        # Track inference times for diagnostics
        if len(self.inference_times) > 30:
            self.inference_times.pop(0)
        self.inference_times.append(delta_time)
        
        if fps < self.FPS_WARN_THRESH:
            avg_inference = np.mean(self.inference_times) * 1000 if self.inference_times else 0
            self.get_logger().warn(f"CRITICAL: FPS={fps:.2f}! Avg inference time: {avg_inference:.2f}ms")
        elif self.processed_msgs == self.FPS_LOG_INTERVAL:
            avg_inference = np.mean(self.inference_times) * 1000 if self.inference_times else 0
            self.get_logger().info(f"FPS={fps:.2f} | Avg inference={avg_inference:.2f}ms | Skip rate={self.skip_count}")
            self.processed_msgs = 0
            self.skip_count = 0
            
        return current_time

    def _get_frame(self, data):
        """Retrieves frame from Shared Memory and manages memory properly."""
        if self.shm is None:
            try:
                self.shm = shared_memory.SharedMemory(name=data['shm_name'])
                self.get_logger().info("Linked to Shared RAM.")
            except Exception as e:
                self.get_logger().error(f"SHM link failed: {e}")
                return None
        
        # Create a copy to avoid holding reference to shared buffer
        frame_data = np.ndarray(data['shape'], dtype=data['dtype'], buffer=self.shm.buf)
        frame_copy = frame_data.copy()
        
        # Clean up old reference
        del frame_data
        
        return frame_copy

    def _ncnn_inference(self, frame):
        """Run NCNN inference on frame. Returns detections with format:
        List of dicts: {'box': [x1,y1,x2,y2], 'conf': float, 'class': int}
        """
        # Preprocess: resize to 320x320
        h, w = frame.shape[:2]
        img = cv2.resize(frame, (int(self.IMG_HEIGHT), int(self.IMG_HEIGHT)))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
        
        # Create NCNN Mat and run inference
        mat_in = ncnn.Mat(img)
        with self.ncnn_net.create_extractor() as ex:
            ex.input("in0", mat_in)
            _, mat_out = ex.extract("out0")
        
        # Convert output back to numpy
        output = np.array(mat_out)  # Shape: [1, num_detections, 6] (x,y,w,h,conf,cls)
        
        detections = []
        if output.size > 0:
            output = output.squeeze()
            if len(output.shape) == 1:
                output = output[np.newaxis, :]
            
            # Parse detections
            for det in output:
                if len(det) >= 6:
                    x, y, bw, bh, conf, cls_id = det[:6]
                    
                    if conf > self.CONF_THRESH:
                        # Convert from center+size to corner coordinates
                        x1 = (x - bw/2) * w / self.IMG_HEIGHT
                        y1 = (y - bh/2) * h / self.IMG_HEIGHT
                        x2 = (x + bw/2) * w / self.IMG_HEIGHT
                        y2 = (y + bh/2) * h / self.IMG_HEIGHT
                        
                        detections.append({
                            'box': [x1, y1, x2, y2],
                            'conf': float(conf),
                            'class': int(cls_id)
                        })
        
        return detections

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

    def _process_visual_detections_ncnn(self, detections, current_time):
        """Processes NCNN detections with centroid tracking (simplified, no YOLO tracking)."""
        visual_detections = []
        current_visible_ids = set()
        
        # Generate simple track IDs based on spatial proximity to history
        for i, det in enumerate(detections):
            x1, y1, x2, y2 = det['box']
            box_center_x = (x1 + x2) / 2.0
            box_height = y2 - y1
            
            # Calculate angle from center
            offset_x = box_center_x - (self.IMG_WIDTH / 2.0)
            angle_degrees = offset_x * (self.HFOV / self.IMG_WIDTH)
            
            # Find closest track in history (within angle threshold)
            track_id = None
            min_angle_diff = self.DUP_ANGLE_THRESH
            
            for t_id, hist_data in self.targets_history.items():
                angle_diff = abs(hist_data['angle'] - angle_degrees)
                if angle_diff < min_angle_diff:
                    min_angle_diff = angle_diff
                    track_id = t_id
            
            # If no match, create new ID
            if track_id is None:
                track_id = max(self.targets_history.keys()) + 1 if self.targets_history else 1
            
            current_visible_ids.add(track_id)
            
            # Calculate kinematics
            velocity = 0.0
            expansion_rate = 0.0
            
            if track_id in self.targets_history:
                prev_data = self.targets_history[track_id]
                dt = current_time - prev_data['last_seen']
                
                if dt > 0:
                    velocity = (angle_degrees - prev_data['angle']) / dt
                    raw_expansion = (box_height - prev_data['last_height']) / dt
                    expansion_rate = ((1.0 - self.LPF_ALPHA) * prev_data['exp_rate']) + (self.LPF_ALPHA * raw_expansion)
            
            # Update history
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
                
                if abs(predicted_angle) > (self.HFOV / 2.0):
                    lost_ids.append(t_id)
                    continue
                
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
        """Main callback pipeline (Orchestrator) with adaptive skipping."""
        inference_start = time.time()
        data = json.loads(msg.data)
        
        frame = self._get_frame(data)
        if frame is None:
            return
        
        # Adaptive frame skipping if inference is too slow
        if len(self.inference_times) > 10:
            avg_inference = np.mean(self.inference_times[-10:])
            # If avg inference > 66ms (target 15 FPS), skip frame
            if avg_inference > 0.066:
                self.skip_count += 1
                del frame
                return
        
        current_time = self._update_fps()
        
        # 1. Run NCNN Inference (much faster than YOLO)
        detections = self._ncnn_inference(frame)
        inference_duration = time.time() - inference_start
        self.inference_times.append(inference_duration)
        
        # 2. Process detections with tracking logic
        visual_detections, current_visible_ids = self._process_visual_detections_ncnn(detections, current_time)
        
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
        
        # 6. Cleanup frame reference and garbage collect periodically
        del frame
        self.frame_count += 1
        if self.frame_count % 30 == 0:  # Every 30 frames (~1 second)
            gc.collect()

    def cleanup(self):
        if self.shm:
            self.shm.close()
        if hasattr(self, 'ncnn_net'):
            self.ncnn_net.clear()
        print("AI Inference: Memory unlinked and NCNN net released.")

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
