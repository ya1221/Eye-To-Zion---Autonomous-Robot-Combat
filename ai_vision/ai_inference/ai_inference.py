import rclpy
import json
import math
import numpy as np
import cv2
import os
import time
import gc
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from foxglove_msgs.msg import ImageAnnotations, PointsAnnotation, TextAnnotation, Point2, Color
from ultralytics import YOLO
from multiprocessing import shared_memory

THREAT_COLORS = {
    "HIGH": (1.0, 0.0, 0.0, 1.0),
    "MEDIUM": (1.0, 1.0, 0.0, 1.0),
    "LOW": (0.0, 1.0, 0.0, 1.0),
}

class AIInferenceNode(Node):
    def __init__(self):
        super().__init__('ai_inference')
        
        # Disable automatic GC to prevent UI freezing/thrashing
        gc.disable()
        self.frame_count = 0
        
        self.MODEL_PATH = 'best_ncnn_model'
        self.HFOV = 141.0
        self.IMG_WIDTH = 640.0
        self.IMG_HEIGHT = 320.0
        
        # Tuning Constants
        self.MAX_BLIND_TIME = 2.0
        self.CONF_THRESH = 0.3
        self.DUP_ANGLE_THRESH = 2.0
        self.LPF_ALPHA = 0.2
        
        self.THREAT_PROXIMITY_MAX = 50
        self.THREAT_VISIBILITY_BONUS = 30
        self.THREAT_STABILITY_HIGH = 20
        self.THREAT_STABILITY_MED = 10
        self.THREAT_RATIO_CLOSE = 0.6
        self.THREAT_VEL_LOW = 5.0
        self.THREAT_VEL_MED = 15.0
        self.THREAT_SCORE_HIGH = 75
        self.THREAT_SCORE_MED = 45
        self.FPS_WARN_THRESH = 7.0
        self.FPS_LOG_INTERVAL = 15

        self.model = YOLO(self.MODEL_PATH, task='detect')

        # Zero-Lag QoS Profile
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.subscription = self.create_subscription(String, '/camera/metadata', self.metadata_callback, qos_profile)
        self.publisher_ = self.create_publisher(String, '/ai/detections', 10)
        self.annotations_publisher_ = self.create_publisher(ImageAnnotations, '/foxglove/annotations', 10)

        # Placeholder intrinsics for a 141 deg HFOV lens at 640x320.
        # NOT calibrated against real hardware - swap in a proper
        # cv2.calibrateCamera() result (checkerboard) before trusting
        # absolute pixel accuracy off-axis.
        hfov_rad = math.radians(self.HFOV)
        fx = self.IMG_WIDTH / (2.0 * math.tan(hfov_rad / 2.0))
        cx, cy = self.IMG_WIDTH / 2.0, self.IMG_HEIGHT / 2.0
        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fx, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)
        # k1, k2, p1, p2 - mild barrel distortion, typical of a wide-angle lens
        self.dist_coeffs = np.array([-0.30, 0.08, 0.0, 0.0], dtype=np.float64)
        self._zero_rvec = np.zeros((3, 1), dtype=np.float64)
        self._zero_tvec = np.zeros((3, 1), dtype=np.float64)

        self.shms = {}
        self.targets_history = {}  
        self.last_time = time.time()
        self.processed_msgs = 0
        self._frame_buffer = None
        self._avg_fps = 30.0
        self.get_logger().info("AI Brain Online.")

    def _calculate_threat_level(self, box_height, is_predicted, angular_velocity):
        score = 0
        height_ratio = box_height / self.IMG_HEIGHT 
        proximity_score = min((height_ratio / self.THREAT_RATIO_CLOSE) * self.THREAT_PROXIMITY_MAX, self.THREAT_PROXIMITY_MAX) 
        score += proximity_score
        
        if not is_predicted: score += self.THREAT_VISIBILITY_BONUS
        
        abs_vel = abs(angular_velocity)
        if abs_vel < self.THREAT_VEL_LOW: score += self.THREAT_STABILITY_HIGH
        elif abs_vel < self.THREAT_VEL_MED: score += self.THREAT_STABILITY_MED 

        if score >= self.THREAT_SCORE_HIGH: return "HIGH"
        elif score >= self.THREAT_SCORE_MED: return "MEDIUM"
        return "LOW"
            
    def _update_fps(self):
        self.processed_msgs += 1
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        instant_fps = 1.0 / dt if dt > 0 else 0.0
        self._avg_fps = 0.9 * self._avg_fps + 0.1 * instant_fps
        
        if self._avg_fps < self.FPS_WARN_THRESH:
            self.get_logger().warn(f"LOW AVG FPS: {self._avg_fps:.1f} (instant: {instant_fps:.1f})")
        elif self.processed_msgs >= self.FPS_LOG_INTERVAL:
            self.get_logger().info(f"FPS: {self._avg_fps:.1f} (instant: {instant_fps:.1f})")
            self.processed_msgs = 0
        return current_time

    def _get_frame(self, data):
        buf_idx = data.get('buf_idx', 0)
        shm_name = data['shm_name']
        if buf_idx not in self.shms:
            self.shms[buf_idx] = shared_memory.SharedMemory(name=shm_name)
        shm_view = np.ndarray(data['shape'], dtype=data['dtype'], buffer=self.shms[buf_idx].buf)
        if self._frame_buffer is None:
            self._frame_buffer = np.empty(data['shape'], dtype=data['dtype'])
        np.copyto(self._frame_buffer, shm_view)
        return self._frame_buffer

    def _process_visual_detections(self, results, current_time):
        visual_detections = []
        current_visible_ids = set()

        for result in results:
            if result.boxes.id is None: continue
                
            track_ids = result.boxes.id.int().cpu().tolist()
            boxes = result.boxes.xyxy.cpu().tolist()
            confs = result.boxes.conf.cpu().tolist()

            for box, track_id, conf in zip(boxes, track_ids, confs):
                if conf <= self.CONF_THRESH: continue
                    
                x1, y1, x2, y2 = box
                box_center_x = (x1 + x2) / 2.0
                offset_x = box_center_x - (self.IMG_WIDTH / 2.0)
                angle_degrees = offset_x * (self.HFOV / self.IMG_WIDTH)
                box_height = y2 - y1 

                if any(abs(d['angle'] - angle_degrees) < self.DUP_ANGLE_THRESH for d in visual_detections):
                    continue

                current_visible_ids.add(track_id)
                velocity, expansion_rate = 0.0, 0.0

                if track_id in self.targets_history:
                    prev_data = self.targets_history[track_id]
                    dt = current_time - prev_data['last_seen']
                    if dt > 0:
                        velocity = (angle_degrees - prev_data['angle']) / dt
                        raw_expansion = (box_height - prev_data['last_height']) / dt
                        expansion_rate = ((1.0 - self.LPF_ALPHA) * prev_data['exp_rate']) + (self.LPF_ALPHA * raw_expansion)

                self.targets_history[track_id] = {
                    'angle': angle_degrees, 'velocity': velocity,
                    'last_height': box_height, 'exp_rate': expansion_rate,
                    'last_seen': current_time
                }

                threat_lvl = self._calculate_threat_level(box_height, False, velocity)
                visual_detections.append({
                    "id": track_id, "angle": round(angle_degrees, 2),
                    "threat": threat_lvl, "exp_rate": round(expansion_rate, 2),
                    "box_height": round(box_height, 1),
                    "predicted": False
                })
        return visual_detections, current_visible_ids

    def _process_blind_predictions(self, current_detections, current_visible_ids, current_time):
        all_detections = list(current_detections)
        lost_ids = []
        
        for t_id, data in self.targets_history.items():
            if t_id in current_visible_ids: continue
                
            dt = current_time - data['last_seen']
            if dt < self.MAX_BLIND_TIME:
                predicted_angle = data['angle'] + (data['velocity'] * dt)
                
                # Check FOV bounds
                if abs(predicted_angle) > (self.HFOV / 2.0):
                    lost_ids.append(t_id)
                    continue
                
                # Verify predicted space is clear
                if not any(abs(d['angle'] - predicted_angle) < self.DUP_ANGLE_THRESH for d in all_detections):
                    threat_lvl = self._calculate_threat_level(data['last_height'], True, data['velocity'])
                    all_detections.append({
                        "id": t_id, "angle": round(predicted_angle, 2),
                        "threat": threat_lvl, "exp_rate": round(data['exp_rate'], 2),
                        "box_height": round(data['last_height'], 1),
                        "predicted": True
                    })
            else:
                lost_ids.append(t_id)
                
        return all_detections, lost_ids

    def _angle_to_pixel(self, angle_degrees):
        # Azimuth-only ray (Y=0, horizon-level) - NOT linear pixel scaling.
        theta = math.radians(angle_degrees)
        ray = np.array([[math.sin(theta), 0.0, math.cos(theta)]], dtype=np.float64)
        image_points, _ = cv2.projectPoints(ray, self._zero_rvec, self._zero_tvec, self.camera_matrix, self.dist_coeffs)
        px, py = image_points[0, 0]
        return float(px), float(py)

    def _build_box_annotation(self, center_x, center_y, box_height, color, stamp):
        half = box_height / 2.0  # fixed square aspect ratio
        corners = [
            (center_x - half, center_y - half),
            (center_x + half, center_y - half),
            (center_x + half, center_y + half),
            (center_x - half, center_y + half),
        ]
        annotation = PointsAnnotation()
        annotation.timestamp = stamp
        annotation.type = PointsAnnotation.LINE_LOOP
        annotation.points = [Point2(x=x, y=y) for x, y in corners]
        annotation.outline_color = Color(r=color[0], g=color[1], b=color[2], a=color[3])
        annotation.thickness = 2.0
        return annotation

    def _build_label_annotation(self, center_x, center_y, box_height, text, color, stamp):
        label = TextAnnotation()
        label.timestamp = stamp
        label.position = Point2(x=center_x - box_height / 2.0, y=center_y - box_height / 2.0 - 4.0)
        label.text = text
        label.font_size = 12.0
        label.text_color = Color(r=color[0], g=color[1], b=color[2], a=color[3])
        label.background_color = Color(r=0.0, g=0.0, b=0.0, a=0.5)
        return label

    def _publish_annotations(self, final_detections, stamp):
        annotations = ImageAnnotations()
        for det in final_detections:
            color = THREAT_COLORS.get(det['threat'], THREAT_COLORS["LOW"])
            center_x, center_y = self._angle_to_pixel(det['angle'])
            box_height = det['box_height']

            annotations.points.append(self._build_box_annotation(center_x, center_y, box_height, color, stamp))

            label_text = f"ID:{det['id']}"
            if det['predicted']:
                label_text += " (PRED)"
            annotations.texts.append(self._build_label_annotation(center_x, center_y, box_height, label_text, color, stamp))

        self.annotations_publisher_.publish(annotations)

    def metadata_callback(self, msg):
        # A subscription callback already queued when SIGINT/SIGTERM arrives
        # can still fire once after shutdown starts tearing down the context -
        # skip it rather than publish into an invalid context.
        if not rclpy.ok(): return
        current_time = self._update_fps()
        data = json.loads(msg.data)
        
        frame = self._get_frame(data)
        if frame is None: return
        
        # Track mode restored. Disabled GC handles the overhead.
        results = self.model.track(frame, imgsz=320, persist=True, tracker="bytetrack.yaml", verbose=False, conf=self.CONF_THRESH)
        
        visual_detections, current_visible_ids = self._process_visual_detections(results, current_time)
        final_detections, lost_ids = self._process_blind_predictions(visual_detections, current_visible_ids, current_time)
        
        for t_id in lost_ids: del self.targets_history[t_id]

        if final_detections:
            det_details = ", ".join([f"[ID:{d['id']} | Ang:{d['angle']}° | Threat:{d['threat']} | Pred:{d['predicted']}]" for d in final_detections])
            self.get_logger().info(f"Targets: {det_details}")

            detection_msg = String()
            detection_msg.data = json.dumps(final_detections)
            self.publisher_.publish(detection_msg)

            stamp_data = data.get('stamp')
            stamp = Time(sec=int(stamp_data['sec']), nanosec=int(stamp_data['nanosec'])) if stamp_data else self.get_clock().now().to_msg()
            self._publish_annotations(final_detections, stamp)
        
        # Incremental GC: gen-0 sweep every 150 frames, full sweep every 1500
        self.frame_count += 1
        if self.frame_count % 150 == 0:
            del results
            gc.collect(0)
        if self.frame_count >= 1500:
            gc.collect()
            self.frame_count = 0

    def cleanup(self):
        for shm in self.shms.values():
            try:
                shm.close()
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = AIInferenceNode()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    except RCLError as e:
        # Residual race: a queued callback can still hit an invalid context
        # between the rclpy.ok() check and the actual publish call. Benign
        # on shutdown, not a bug to fail loud on.
        node.get_logger().warn(f"RCLError during shutdown, ignoring: {e}")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()