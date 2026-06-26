import cv2
import cv2.aruco as aruco
import numpy as np
import math
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

        # 2. Retrieve Parameters
        self.CNT_TEAM = self.get_parameter('cnt_team').value
        self.CAMERA_ID = self.get_parameter('camera_id').value
        self.GRID_N = self.get_parameter('grid_n').value
        self.TARGETS_START = self.get_parameter('targets_start').value
        
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
        
        # 6. Camera init
        self.cap = cv2.VideoCapture(self.CAMERA_ID)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera with ID {self.CAMERA_ID}")
            exit()
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info(f"Modulo Tracker ONLINE ({self.CNT_TEAM} Teams Mode). Architecture: Modular JSON.")

    def get_grid_coordinates(self, cx, cy, angle):
        pt_cam = np.array([[[cx, cy]]], dtype=np.float32)
        pt_grid = cv2.perspectiveTransform(pt_cam, self.perspective_matrix)
        return {
            "x": int(pt_grid[0][0][0]),
            "y": int(pt_grid[0][0][1]),
            "angle": round(angle, 2)
        }

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self._handle_camera_failure()
            return

        self.camera_failure_count = 0
        self.frame_count += 1
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

        cv2.imshow("Tactical Map", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def _detect_markers(self, frame):
        corners, ids, _ = self.detector.detectMarkers(frame)
        current_detected_ids = set()
        corners_dict = {}

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            for i, marker_id_arr in enumerate(ids):
                rid = int(marker_id_arr[0])
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
                cv2.imshow("Tactical Map", frame)
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
            self.cap.release()
            self.cap = cv2.VideoCapture(self.CAMERA_ID)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.camera_failure_count = 0
                self.get_logger().info("Camera reconnected successfully!")
            else:
                self.get_logger().error("Camera reconnection failed. Will retry on next cycle.")
                self.camera_failure_count = 0  # Reset to allow retry cycle

    def _publish_telemetry(self, frame_teams_data, frame_targets_data):
        for team_idx in range(self.CNT_TEAM):
            
            # Publish Robots as JSON String
            if frame_teams_data[team_idx]:
                robots_payload = []
                for robot in frame_teams_data[team_idx]:
                    robots_payload.append({
                        "id": int(robot["id"]),
                        "x": float(robot["x"]),
                        "y": float(robot["y"]),
                        "angle": float(robot["angle"])
                    })
                
                msg = String()
                msg.data = json.dumps(robots_payload)
                self.team_pubs[team_idx].publish(msg)

            # Publish Target as JSON String
            if team_idx in frame_targets_data:
                target = frame_targets_data[team_idx]
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
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
