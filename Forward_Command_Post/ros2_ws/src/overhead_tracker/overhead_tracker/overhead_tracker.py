import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String

class PureModuloTrackerNode(Node):
    def __init__(self):
        super().__init__('overhead_tracker_node')
        
        # 1. Declare ROS 2 Parameters
        self.declare_parameter('cnt_team', 2)
        self.declare_parameter('camera_id', 2)
        self.declare_parameter('grid_n', 2000)
        self.declare_parameter('targets_start', 4)
        self.declare_parameter('proc_width', 1280)
        self.declare_parameter('proc_height', 960)
        self.declare_parameter('display_width', 1280)
        self.declare_parameter('display_height', 960)
        self.declare_parameter('pixels_per_cm', 2.0)
        self.declare_parameter('proximity_threshold_cm', 20.0)

        # 2. Retrieve Parameters
        self.CNT_TEAM = self.get_parameter('cnt_team').value
        self.CAMERA_ID = self.get_parameter('camera_id').value
        self.GRID_N = self.get_parameter('grid_n').value
        self.TARGETS_START = self.get_parameter('targets_start').value
        # Target resolution for the ArUco/KLT pipeline. Kept independent from
        # the camera's native capture resolution (see _open_camera_at_max_resolution)
        # so we never re-trigger the sensor's hardware center-crop.
        self.PROC_WIDTH = self.get_parameter('proc_width').value
        self.PROC_HEIGHT = self.get_parameter('proc_height').value
        # On-screen window size. Independent of PROC_WIDTH/HEIGHT so the
        # display always fits the monitor even if the processing resolution
        # is changed (e.g. to the sensor's full native size).
        self.DISPLAY_WIDTH = self.get_parameter('display_width').value
        self.DISPLAY_HEIGHT = self.get_parameter('display_height').value
        self.WINDOW_NAME = "Tactical Map"

        # Capture-zone proximity radius, expressed in warped-grid pixels (the
        # space produced by get_grid_coordinates(), which spans GRID_N x GRID_N
        # after the perspective warp - NOT raw camera pixels). PIXELS_PER_CM is
        # a placeholder; calibrate it by measuring how many grid pixels span a
        # known real-world distance on your printed/marked arena, then this
        # radius will accurately represent proximity_threshold_cm in real units.
        self.PIXELS_PER_CM = self.get_parameter('pixels_per_cm').value
        self.PROXIMITY_THRESHOLD_CM = self.get_parameter('proximity_threshold_cm').value
        self.capture_radius_pixels = self.PROXIMITY_THRESHOLD_CM * self.PIXELS_PER_CM
        
        if self.CNT_TEAM < 1:
            self.get_logger().fatal(f"cnt_team must be >= 1, got {self.CNT_TEAM}. Shutting down.")
            raise SystemExit(1)
        
        self.ROBOTS_START = self.TARGETS_START + self.CNT_TEAM
        
        # 3. Dynamic publishers setup (Using String for JSON payloads)
        self.team_pubs = {}
        self.target_pubs = {}
        for team_idx in range(self.CNT_TEAM):
            self.team_pubs[team_idx] = self.create_publisher(String, f'teams/team_{team_idx}/positions', 10)
            self.target_pubs[team_idx] = self.create_publisher(String, f'teams/team_{team_idx}/target_position', 10)
        
        # 4. Vision parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.feature_params = dict(maxCorners=20, qualityLevel=0.3, minDistance=7, blockSize=7)
        
        # 5. Grid & State calibration
        self.dst_pts = np.array([[0, 0], [self.GRID_N, 0], [self.GRID_N, self.GRID_N], [0, self.GRID_N]], dtype=np.float32)
        self.perspective_matrix = None
        self.is_matrix_locked = False
        
        self.robots_state = {}
        self.old_gray = None
        self.frame_count = 0
        self.FEATURE_UPDATE_INTERVAL = 10
        self.camera_failure_count = 0
        self.MAX_CAMERA_RETRIES = 10
        
        # 6. Camera init - open at the sensor's native max resolution so the
        # firmware never engages its hardware center-crop, then downsample
        # in software for processing.
        self._resize_buffer = np.empty((self.PROC_HEIGHT, self.PROC_WIDTH, 3), dtype=np.uint8)
        self.cap = self._open_camera_at_max_resolution()
        if self.cap is None:
            self.get_logger().error(f"Failed to open camera with ID {self.CAMERA_ID}")
            exit()

        # WINDOW_NORMAL makes the window user-resizable and lets us pin an
        # initial size that fits the screen, independent of the frame's
        # actual pixel dimensions (cv2.imshow scales to fit automatically).
        cv2.namedWindow(self.WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.WINDOW_NAME, self.DISPLAY_WIDTH, self.DISPLAY_HEIGHT)

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info(f"Modulo Tracker ONLINE ({self.CNT_TEAM} Teams Mode). Architecture: Modular JSON.")

    def _open_camera_at_max_resolution(self):
        """Open the camera and force it to its native maximum resolution.

        UVC/V4L2 drivers clamp an over-large CAP_PROP_FRAME_WIDTH/HEIGHT
        request down to the highest mode the sensor actually supports, so
        this works for any camera without hardcoding its true max dimensions.
        Reading back the values after the request tells us what the hardware
        settled on. This avoids the low-resolution modes where some hardware
        applies a center-crop instead of a full-sensor downscale, which cuts
        the field of view.
        """
        cap = cv2.VideoCapture(self.CAMERA_ID)
        if not cap.isOpened():
            return None

        # Force MJPEG compression before negotiating resolution. Uncompressed
        # YUYV at the sensor's full 8MP resolution far exceeds USB 2.0
        # bandwidth, which is what was silently capping the FPS; MJPEG lets
        # the full-resolution stream fit comfortably within USB 2.0 limits.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 9999)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 9999)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f"Camera {self.CAMERA_ID} opened at native max resolution {actual_w}x{actual_h}; "
            f"downsampling to {self.PROC_WIDTH}x{self.PROC_HEIGHT} for processing."
        )

        self._warn_if_aspect_ratio_mismatch(actual_w, actual_h)
        return cap

    def _warn_if_aspect_ratio_mismatch(self, cam_width, cam_height):
        """Warn if proc_width/proc_height don't share the camera's native aspect ratio.

        A resize across mismatched aspect ratios stretches the image
        non-uniformly, which distorts ArUco marker geometry and throws off
        the perspective-warp calibration. This never blocks startup -
        it just makes a silent quality regression loud.
        """
        if cam_width <= 0 or cam_height <= 0:
            return

        cam_ratio = cam_width / cam_height
        proc_ratio = self.PROC_WIDTH / self.PROC_HEIGHT

        if abs(cam_ratio - proc_ratio) > 1e-2:
            self.get_logger().warn(
                f"Aspect ratio mismatch: camera native {cam_width}x{cam_height} "
                f"({cam_ratio:.3f}) vs proc_width/proc_height {self.PROC_WIDTH}x{self.PROC_HEIGHT} "
                f"({proc_ratio:.3f}). Frames will be stretched non-uniformly, degrading ArUco "
                f"detection and perspective-warp accuracy. Set proc_width/proc_height to match "
                f"the camera's native aspect ratio."
            )

    def get_grid_coordinates(self, cx, cy, angle):
        pt_cam = np.array([[[cx, cy]]], dtype=np.float32)
        pt_grid = cv2.perspectiveTransform(pt_cam, self.perspective_matrix)
        return {
            "x": int(pt_grid[0][0][0]),
            "y": int(pt_grid[0][0][1]),
            "angle": round(angle, 2)
        }

    def process_frame(self):
        if self.cap is None or not self.cap.isOpened():
            self._handle_camera_failure()
            return

        ret, raw_frame = self.cap.read()
        if not ret:
            self._handle_camera_failure()
            return

        self.camera_failure_count = 0
        self.frame_count += 1

        # Downsample from the full-FOV capture to the processing resolution
        # right away, reusing a preallocated buffer so no new frame-sized
        # array is allocated on the hot path.
        cv2.resize(
            raw_frame,
            (self.PROC_WIDTH, self.PROC_HEIGHT),
            dst=self._resize_buffer,
            interpolation=cv2.INTER_AREA,
        )
        frame = self._resize_buffer

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        current_detected_ids, corners_dict = self._detect_markers(frame)

        if not self._lock_perspective(frame, corners_dict):
            return
        
        frame_teams_data = {i: [] for i in range(self.CNT_TEAM)}
        frame_targets_data = {}

        self._process_visible_entities(frame, gray, corners_dict, frame_teams_data, frame_targets_data)
        self._track_hidden_robots(frame, gray, current_detected_ids, frame_teams_data)

        self.old_gray = gray.copy()
        self._publish_telemetry(frame_teams_data, frame_targets_data)

        cv2.imshow(self.WINDOW_NAME, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def _detect_markers(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame)
        current_detected_ids = set()
        corners_dict = {}

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            # detectMarkers returns ids as Nx1 on most backends but flat (N,)
            # on some OpenCV/platform combos - flatten unconditionally so
            # indexing is safe either way, then match by position against
            # `corners` (whose per-marker structure is unaffected by this).
            flat_ids = np.asarray(ids).flatten()
            for i, rid_val in enumerate(flat_ids):
                rid = int(rid_val)
                current_detected_ids.add(rid)
                
                c = corners[i][0]
                center = np.mean(c, axis=0)
                cx, cy = int(center[0]), int(center[1])
                
                dx, dy = c[1][0] - c[0][0], c[1][1] - c[0][1]
                angle = math.degrees(math.atan2(dy, dx))
                corners_dict[rid] = (cx, cy, angle, c)
                
                end_x = int(cx + 50 * math.cos(math.radians(angle)))
                end_y = int(cy + 50 * math.sin(math.radians(angle)))
                cv2.line(frame, (cx, cy), (end_x, end_y), (0, 0, 255), 3)
                
        return current_detected_ids, corners_dict

    def _lock_perspective(self, frame, corners_dict):
        if not self.is_matrix_locked:
            if all(anchor in corners_dict for anchor in range(4)):
                src_pts = np.array([corners_dict[i][:2] for i in [0, 1, 2, 3]], dtype=np.float32)
                self.perspective_matrix = cv2.getPerspectiveTransform(src_pts, self.dst_pts)
                self.is_matrix_locked = True
                self.get_logger().info("Arena Perspective Locked!")
            else:
                cv2.putText(frame, "WAITING FOR 4 ANCHORS...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                cv2.imshow(self.WINDOW_NAME, frame)
                cv2.waitKey(1)
        return self.is_matrix_locked

    def _process_visible_entities(self, frame, gray, corners_dict, frame_teams_data, frame_targets_data):
        for rid, (cx, cy, angle, c) in corners_dict.items():
            if rid < self.TARGETS_START:
                continue
                
            assigned_team = rid % self.CNT_TEAM
            
            if self.TARGETS_START <= rid < self.ROBOTS_START:
                frame_targets_data[assigned_team] = self.get_grid_coordinates(cx, cy, angle)
                cv2.putText(frame, f"TARGET T{assigned_team} (ID:{rid})", (cx-40, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            elif rid >= self.ROBOTS_START:
                robot_coords = self.get_grid_coordinates(cx, cy, angle)
                robot_coords["id"] = rid
                frame_teams_data[assigned_team].append(robot_coords)
                
                # Only compute expensive features on first detection or every N frames
                is_new_robot = rid not in self.robots_state
                is_update_frame = self.frame_count % self.FEATURE_UPDATE_INTERVAL == 0
                
                if is_new_robot or is_update_frame:
                    mask = np.zeros_like(gray)
                    cv2.fillPoly(mask, [np.int32(c)], 255)
                    p0 = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
                    if p0 is not None:
                        self.robots_state[rid] = {'center': np.array([cx, cy], dtype=np.float32), 'angle': angle, 'features': p0, 'missed': 0}
                    elif rid in self.robots_state:
                        self.robots_state[rid]['center'] = np.array([cx, cy], dtype=np.float32)
                        self.robots_state[rid]['angle'] = angle
                        self.robots_state[rid]['missed'] = 0
                else:
                    # Update position/angle but keep existing features
                    self.robots_state[rid]['center'] = np.array([cx, cy], dtype=np.float32)
                    self.robots_state[rid]['angle'] = angle
                    self.robots_state[rid]['missed'] = 0

    def _track_hidden_robots(self, frame, gray, current_detected_ids, frame_teams_data):
        if self.old_gray is None:
            return

        for rid in list(self.robots_state.keys()):
            if rid not in current_detected_ids:
                state = self.robots_state[rid]
                
                if state['missed'] < 30 and state.get('features') is not None:
                    p1, st, _ = cv2.calcOpticalFlowPyrLK(self.old_gray, gray, state['features'], None, **self.lk_params)
                    good_new = p1[st == 1]
                    good_old = state['features'][st == 1]

                    if len(good_new) > 0:
                        movement = np.mean(good_new - good_old, axis=0)
                        state['center'] += movement
                        state['features'] = good_new.reshape(-1, 1, 2)
                        state['missed'] += 1
                        
                        cx, cy = int(state['center'][0]), int(state['center'][1])
                        assigned_team = rid % self.CNT_TEAM
                        
                        robot_coords = self.get_grid_coordinates(cx, cy, state['angle'])
                        robot_coords["id"] = rid
                        frame_teams_data[assigned_team].append(robot_coords)
                        
                        cv2.circle(frame, (cx, cy), 25, (0, 255, 255), 2)
                        cv2.putText(frame, f"KLT ROBOT {rid} (T{assigned_team})", (cx-40, cy-35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    else:
                        del self.robots_state[rid]
                else:
                    del self.robots_state[rid]

    def _handle_camera_failure(self):
        self.camera_failure_count += 1
        self.get_logger().warn(f"Camera read failed (attempt {self.camera_failure_count}/{self.MAX_CAMERA_RETRIES})")
        
        if self.camera_failure_count >= self.MAX_CAMERA_RETRIES:
            self.get_logger().error(f"Camera lost after {self.MAX_CAMERA_RETRIES} consecutive failures. Attempting reconnection...")
            if self.cap is not None:
                self.cap.release()

            # Give the USB bus a moment to settle before re-enumerating the
            # device. Reopening immediately after a drop floods the bus with
            # back-to-back requests, which is what was triggering the
            # errno=19 (ENODEV) disconnects.
            time.sleep(0.5)

            self.cap = self._open_camera_at_max_resolution()
            if self.cap is not None:
                self.camera_failure_count = 0
                self.get_logger().info("Camera reconnected successfully!")
            else:
                self.get_logger().error("Camera reconnection failed. Will retry on next cycle.")
                self.camera_failure_count = 0  # Reset to allow retry cycle

    def _publish_telemetry(self, frame_teams_data, frame_targets_data):
        for team_idx in range(self.CNT_TEAM):
            # .get() rather than indexing - the team's own target may be
            # temporarily missing from this frame's detections (occluded
            # marker, motion blur, etc). Every robot below is only ever
            # compared against this same-team target, so this stays a
            # single O(1) lookup per team rather than a cross-team scan.
            target = frame_targets_data.get(team_idx)

            # Publish Robots as JSON String
            if frame_teams_data[team_idx]:
                robots_payload = []
                for robot in frame_teams_data[team_idx]:
                    zone_occupied = False
                    if target is not None:
                        distance = math.hypot(robot["x"] - target["x"], robot["y"] - target["y"])
                        zone_occupied = distance <= self.capture_radius_pixels

                    robots_payload.append({
                        "id": int(robot["id"]),
                        "x": float(robot["x"]),
                        "y": float(robot["y"]),
                        "angle": float(robot["angle"]),
                        "zone_occupied": zone_occupied
                    })

                msg = String()
                msg.data = json.dumps(robots_payload)
                self.team_pubs[team_idx].publish(msg)

            # Publish Target as JSON String
            if target is not None:
                target_payload = {
                    "x": float(target["x"]),
                    "y": float(target["y"])
                }

                msg = String()
                msg.data = json.dumps(target_payload)
                self.target_pubs[team_idx].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PureModuloTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
