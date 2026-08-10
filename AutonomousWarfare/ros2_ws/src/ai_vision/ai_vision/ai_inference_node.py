import rclpy
import json
import math
import numpy as np
import cv2
import os
import time
import gc
from ament_index_python.packages import get_package_share_directory
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
    "HIGH":   (1.0, 0.0, 0.0, 1.0),
    "MEDIUM": (1.0, 1.0, 0.0, 1.0),
    "LOW":    (0.0, 1.0, 0.0, 1.0),
}


class AIInferenceNode(Node):
    def __init__(self):
        super().__init__('ai_inference')

        # Disable automatic GC to prevent UI freezing / thrashing
        gc.disable()
        self.frame_count = 0

        # Declare parameters (overridable via config YAML / CLI) 
        self.declare_parameter('model_path', '')
        self.declare_parameter('inference_imgsz', 320)
        self.declare_parameter('tracker_config', 'bytetrack.yaml')

        # Camera geometry
        self.declare_parameter('hfov_degrees', 141.0)
        self.declare_parameter('image_width', 640.0)
        self.declare_parameter('image_height', 320.0)
        self.declare_parameter('dist_coeffs', [-0.30, 0.08, 0.0, 0.0])

        # Detection tuning
        self.declare_parameter('max_blind_time', 2.0)
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('duplicate_angle_threshold', 2.0)
        self.declare_parameter('lpf_alpha', 0.2)

        # Threat scoring
        self.declare_parameter('threat_proximity_max', 50)
        self.declare_parameter('threat_visibility_bonus', 30)
        self.declare_parameter('threat_stability_high', 20)
        self.declare_parameter('threat_stability_med', 10)
        self.declare_parameter('threat_ratio_close', 0.6)
        self.declare_parameter('threat_vel_low', 5.0)
        self.declare_parameter('threat_vel_med', 15.0)
        self.declare_parameter('threat_score_high', 75)
        self.declare_parameter('threat_score_med', 45)

        # Diagnostics
        self.declare_parameter('fps_warn_threshold', 7.0)
        self.declare_parameter('fps_log_interval', 15)

        # GC tuning
        self.declare_parameter('gc_gen0_interval', 150)
        self.declare_parameter('gc_full_interval', 1500)

        # Topic names
        self.declare_parameter('metadata_topic', '/camera/metadata')
        self.declare_parameter('detections_topic', '/ai/detections')
        self.declare_parameter('annotations_topic', '/foxglove/annotations')

        # Read parameter values 
        self.MODEL_PATH        = self._resolve_model_path()
        self.INFERENCE_IMGSZ   = self.get_parameter('inference_imgsz').value
        self.TRACKER_CONFIG    = self.get_parameter('tracker_config').value

        self.HFOV              = self.get_parameter('hfov_degrees').value
        self.IMG_WIDTH         = self.get_parameter('image_width').value
        self.IMG_HEIGHT        = self.get_parameter('image_height').value

        self.MAX_BLIND_TIME    = self.get_parameter('max_blind_time').value
        self.CONF_THRESH       = self.get_parameter('confidence_threshold').value
        self.DUP_ANGLE_THRESH  = self.get_parameter('duplicate_angle_threshold').value
        self.LPF_ALPHA         = self.get_parameter('lpf_alpha').value

        self.THREAT_PROXIMITY_MAX    = self.get_parameter('threat_proximity_max').value
        self.THREAT_VISIBILITY_BONUS = self.get_parameter('threat_visibility_bonus').value
        self.THREAT_STABILITY_HIGH   = self.get_parameter('threat_stability_high').value
        self.THREAT_STABILITY_MED    = self.get_parameter('threat_stability_med').value
        self.THREAT_RATIO_CLOSE      = self.get_parameter('threat_ratio_close').value
        self.THREAT_VEL_LOW          = self.get_parameter('threat_vel_low').value
        self.THREAT_VEL_MED          = self.get_parameter('threat_vel_med').value
        self.THREAT_SCORE_HIGH       = self.get_parameter('threat_score_high').value
        self.THREAT_SCORE_MED        = self.get_parameter('threat_score_med').value

        self.FPS_WARN_THRESH   = self.get_parameter('fps_warn_threshold').value
        self.FPS_LOG_INTERVAL  = self.get_parameter('fps_log_interval').value

        self.GC_GEN0_INTERVAL  = self.get_parameter('gc_gen0_interval').value
        self.GC_FULL_INTERVAL  = self.get_parameter('gc_full_interval').value

        metadata_topic    = self.get_parameter('metadata_topic').value
        detections_topic  = self.get_parameter('detections_topic').value
        annotations_topic = self.get_parameter('annotations_topic').value

        # Load YOLO model 
        self.model = YOLO(self.MODEL_PATH, task='detect')

        # ROS 2 pub / sub 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            String, metadata_topic, self.metadata_callback, qos_profile)
        self.publisher_ = self.create_publisher(String, detections_topic, 10)
        self.annotations_publisher_ = self.create_publisher(
            ImageAnnotations, annotations_topic, 10)

        # ── Camera intrinsics (placeholder, swap in calibrated values) ──
        hfov_rad = math.radians(self.HFOV)
        fx = self.IMG_WIDTH / (2.0 * math.tan(hfov_rad / 2.0))
        cx, cy = self.IMG_WIDTH / 2.0, self.IMG_HEIGHT / 2.0
        self.camera_matrix = np.array([
            [fx,  0.0, cx],
            [0.0, fx,  cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

        dist_list = self.get_parameter('dist_coeffs').value
        self.dist_coeffs = np.array(dist_list, dtype=np.float64)
        self._zero_rvec = np.zeros((3, 1), dtype=np.float64)
        self._zero_tvec = np.zeros((3, 1), dtype=np.float64)

        # ── Runtime state ──
        self.shms = {}
        self.targets_history = {}
        self.last_time = time.time()
        self.processed_msgs = 0
        self._frame_buffer = None
        self._avg_fps = 30.0
        self.get_logger().info("AI Brain Online.")

    # model resolution 
    def _resolve_model_path(self):
        configured = self.get_parameter('model_path').value
        if configured:
            path = configured
        else:
            path = os.path.join(
                get_package_share_directory('ai_vision'),
                'models', 'best_ncnn_model')

        # Fail here rather than let YOLO() try to download a same-named model
        # off the network when the path turns out not to exist on disk.
        if not os.path.isdir(path):
            raise RuntimeError(
                f"NCNN model directory not found at '{path}'. "
                "Check the model_path parameter, or rebuild the package so the "
                "model is installed into share/ai_vision/models/."
            )
        self.get_logger().info(f"Loading NCNN model from {path}")
        return path

    # threat scoring 
    def _calculate_threat_level(self, box_height, is_predicted, angular_velocity):
        score = 0
        height_ratio = box_height / self.IMG_HEIGHT
        proximity_score = min(
            (height_ratio / self.THREAT_RATIO_CLOSE) * self.THREAT_PROXIMITY_MAX,
            self.THREAT_PROXIMITY_MAX,
        )
        score += proximity_score

        if not is_predicted:
            score += self.THREAT_VISIBILITY_BONUS

        abs_vel = abs(angular_velocity)
        if abs_vel < self.THREAT_VEL_LOW:
            score += self.THREAT_STABILITY_HIGH
        elif abs_vel < self.THREAT_VEL_MED:
            score += self.THREAT_STABILITY_MED

        if score >= self.THREAT_SCORE_HIGH:
            return "HIGH"
        elif score >= self.THREAT_SCORE_MED:
            return "MEDIUM"
        return "LOW"

    # FPS diagnostics 
    def _update_fps(self):
        self.processed_msgs += 1
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        instant_fps = 1.0 / dt if dt > 0 else 0.0
        self._avg_fps = 0.9 * self._avg_fps + 0.1 * instant_fps

        if self._avg_fps < self.FPS_WARN_THRESH:
            self.get_logger().warn(
                f"LOW AVG FPS: {self._avg_fps:.1f} (instant: {instant_fps:.1f})")
        elif self.processed_msgs >= self.FPS_LOG_INTERVAL:
            self.get_logger().info(
                f"FPS: {self._avg_fps:.1f} (instant: {instant_fps:.1f})")
            self.processed_msgs = 0
        return current_time

    # shared-memory read 
    def _get_frame(self, data):
        buf_idx = data.get('buf_idx', 0)
        shm_name = data['shm_name']
        if buf_idx not in self.shms:
            self.shms[buf_idx] = shared_memory.SharedMemory(name=shm_name)
        shm_view = np.ndarray(data['shape'], dtype=data['dtype'],
                              buffer=self.shms[buf_idx].buf)
        if self._frame_buffer is None:
            self._frame_buffer = np.empty(data['shape'], dtype=data['dtype'])
        np.copyto(self._frame_buffer, shm_view)
        return self._frame_buffer

    # detection pipeline
    def _process_visual_detections(self, results, current_time):
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

                if any(abs(d['angle'] - angle_degrees) < self.DUP_ANGLE_THRESH
                       for d in visual_detections):
                    continue

                current_visible_ids.add(track_id)
                velocity, expansion_rate = 0.0, 0.0

                if track_id in self.targets_history:
                    prev_data = self.targets_history[track_id]
                    dt = current_time - prev_data['last_seen']
                    if dt > 0:
                        velocity = (angle_degrees - prev_data['angle']) / dt
                        raw_expansion = (box_height - prev_data['last_height']) / dt
                        expansion_rate = (
                            (1.0 - self.LPF_ALPHA) * prev_data['exp_rate']
                            + self.LPF_ALPHA * raw_expansion
                        )

                self.targets_history[track_id] = {
                    'angle': angle_degrees,
                    'velocity': velocity,
                    'last_height': box_height,
                    'exp_rate': expansion_rate,
                    'last_seen': current_time,
                }

                threat_lvl = self._calculate_threat_level(box_height, False, velocity)
                visual_detections.append({
                    "id": track_id,
                    "angle": round(angle_degrees, 2),
                    "threat": threat_lvl,
                    "exp_rate": round(expansion_rate, 2),
                    "box_height": round(box_height, 1),
                    "predicted": False,
                })
        return visual_detections, current_visible_ids

    def _process_blind_predictions(self, current_detections, current_visible_ids, current_time):
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

                if not any(abs(d['angle'] - predicted_angle) < self.DUP_ANGLE_THRESH
                           for d in all_detections):
                    threat_lvl = self._calculate_threat_level(
                        data['last_height'], True, data['velocity'])
                    all_detections.append({
                        "id": t_id,
                        "angle": round(predicted_angle, 2),
                        "threat": threat_lvl,
                        "exp_rate": round(data['exp_rate'], 2),
                        "box_height": round(data['last_height'], 1),
                        "predicted": True,
                    })
            else:
                lost_ids.append(t_id)

        return all_detections, lost_ids

    # Foxglove annotations
    def _angle_to_pixel(self, angle_degrees):
        theta = math.radians(angle_degrees)
        ray = np.array([[math.sin(theta), 0.0, math.cos(theta)]], dtype=np.float64)
        image_points, _ = cv2.projectPoints(
            ray, self._zero_rvec, self._zero_tvec,
            self.camera_matrix, self.dist_coeffs)
        px, py = image_points[0, 0]
        return float(px), float(py)

    def _build_box_annotation(self, center_x, center_y, box_height, color, stamp):
        half = box_height / 2.0
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
        annotation.outline_color = Color(
            r=color[0], g=color[1], b=color[2], a=color[3])
        annotation.thickness = 2.0
        return annotation

    def _build_label_annotation(self, center_x, center_y, box_height, text, color, stamp):
        label = TextAnnotation()
        label.timestamp = stamp
        label.position = Point2(
            x=center_x - box_height / 2.0,
            y=center_y - box_height / 2.0 - 4.0)
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

            annotations.points.append(
                self._build_box_annotation(center_x, center_y, box_height, color, stamp))

            label_text = f"ID:{det['id']}"
            if det['predicted']:
                label_text += " (PRED)"
            annotations.texts.append(
                self._build_label_annotation(
                    center_x, center_y, box_height, label_text, color, stamp))

        self.annotations_publisher_.publish(annotations)

    # main callback 
    def metadata_callback(self, msg):
        if not rclpy.ok():
            return
        current_time = self._update_fps()
        data = json.loads(msg.data)

        frame = self._get_frame(data)
        if frame is None:
            return

        results = self.model.track(
            frame,
            imgsz=self.INFERENCE_IMGSZ,
            persist=True,
            tracker=self.TRACKER_CONFIG,
            verbose=False,
            conf=self.CONF_THRESH,
        )

        visual_detections, current_visible_ids = self._process_visual_detections(
            results, current_time)
        final_detections, lost_ids = self._process_blind_predictions(
            visual_detections, current_visible_ids, current_time)

        for t_id in lost_ids:
            del self.targets_history[t_id]

        if final_detections:
            det_details = ", ".join([
                f"[ID:{d['id']} | Ang:{d['angle']}° | "
                f"Threat:{d['threat']} | Pred:{d['predicted']}]"
                for d in final_detections
            ])
            self.get_logger().info(f"Targets: {det_details}")

            detection_msg = String()
            detection_msg.data = json.dumps(final_detections)
            self.publisher_.publish(detection_msg)

            stamp_data = data.get('stamp')
            stamp = (
                Time(sec=int(stamp_data['sec']), nanosec=int(stamp_data['nanosec']))
                if stamp_data
                else self.get_clock().now().to_msg()
            )
            self._publish_annotations(final_detections, stamp)

        # Incremental GC: gen-0 sweep periodically, full sweep less often
        self.frame_count += 1
        if self.frame_count % self.GC_GEN0_INTERVAL == 0:
            del results
            gc.collect(0)
        if self.frame_count >= self.GC_FULL_INTERVAL:
            gc.collect()
            self.frame_count = 0

    # cleanup 
    def cleanup(self):
        for shm in self.shms.values():
            try:
                shm.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = AIInferenceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as e:
        node.get_logger().warn(f"RCLError during shutdown, ignoring: {e}")
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()