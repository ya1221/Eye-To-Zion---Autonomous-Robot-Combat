import cv2
import cv2.aruco as aruco
import numpy as np
import json
import math
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PureModuloTrackerNode(Node):
    def __init__(self):
        super().__init__('overhead_tracker_node')
        
        # Dynamic team count configuration
        self.CNT_TEAM = int(os.environ.get('CNT_TEAM', 2))
        
        # Boundaries based on your formula
        self.TARGETS_START = 4
        self.ROBOTS_START = self.TARGETS_START + self.CNT_TEAM
        
        # Dynamic publishers setup
        self.team_pubs = {}
        self.target_pubs = {}
        for team_idx in range(self.CNT_TEAM):
            self.team_pubs[team_idx] = self.create_publisher(String, f'teams/team_{team_idx}/positions', 10)
            self.target_pubs[team_idx] = self.create_publisher(String, f'teams/team_{team_idx}/target_position', 10)
        
        # Vision parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.feature_params = dict(maxCorners=20, qualityLevel=0.3, minDistance=7, blockSize=7)
        
        # Grid calibration
        self.GRID_N = 2000
        self.dst_pts = np.array([[0, 0], [self.GRID_N, 0], [self.GRID_N, self.GRID_N], [0, self.GRID_N]], dtype=np.float32)
        self.perspective_matrix = None
        self.is_matrix_locked = False
        
        # States
        self.robots_state = {}
        self.old_gray = None
        
        # Camera
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            exit()
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.get_logger().info(f"Modulo Tracker ONLINE ({self.CNT_TEAM} Teams Mode).")

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
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(frame)
        
        current_detected_ids = set()
        corners_dict = {}

        # 1. Parse ArUco
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
                
                # Heading line
                end_x = int(cx + 50 * math.cos(math.radians(angle)))
                end_y = int(cy + 50 * math.sin(math.radians(angle)))
                cv2.line(frame, (cx, cy), (end_x, end_y), (0, 0, 255), 3)

        # 2. Lock Matrix
        if not self.is_matrix_locked:
            if all(anchor in corners_dict for anchor in range(4)):
                src_pts = np.array([corners_dict[i][:2] for i in [1, 2, 3, 0]], dtype=np.float32)
                self.perspective_matrix = cv2.getPerspectiveTransform(src_pts, self.dst_pts)
                self.is_matrix_locked = True
            else:
                cv2.putText(frame, "WAITING FOR 4 ANCHORS...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
                cv2.imshow("Tactical Map", frame)
                cv2.waitKey(1)
                return
        
        # ==================== תוספת זמנית להדפסת פלט בדיקה ====================
        if ids is not None:
            print("\n--- Detected ArUco Status ---")
            for rid in current_detected_ids:
                if rid < self.TARGETS_START:
                    print(f"[ID: {rid}] -> ROLE: Corner Anchor (Arena Boundary)")
                elif self.TARGETS_START <= rid < self.ROBOTS_START:
                    assigned_team = rid % self.CNT_TEAM
                    print(f"[ID: {rid}] -> ROLE: TARGET Zone | Belong to TEAM: {assigned_team}")
                elif rid >= self.ROBOTS_START:
                    assigned_team = rid % self.CNT_TEAM
                    print(f"[ID: {rid}] -> ROLE: ROBOT | Belong to TEAM: {assigned_team}")
            print("------------------------------")
        # =====================================================================
        
        # Prepare frame storage
        frame_teams_data = {i: [] for i in range(self.CNT_TEAM)}
        frame_targets_data = {}

        # 3. Dynamic Sorting via Modulo (%)
        for rid, (cx, cy, angle, c) in corners_dict.items():
            if rid < self.TARGETS_START:
                continue # Skip corners (0-3)
                
            # Compute team using modulo
            assigned_team = rid % self.CNT_TEAM
            
            # Context A: Targets Range
            if self.TARGETS_START <= rid < self.ROBOTS_START:
                frame_targets_data[assigned_team] = self.get_grid_coordinates(cx, cy, angle)
                cv2.putText(frame, f"TARGET T{assigned_team} (ID:{rid})", (cx-40, cy-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            # Context B: Robots Range
            elif rid >= self.ROBOTS_START:
                robot_coords = self.get_grid_coordinates(cx, cy, angle)
                robot_coords["id"] = rid
                frame_teams_data[assigned_team].append(robot_coords)
                
                # Init KLT
                mask = np.zeros_like(gray)
                cv2.fillPoly(mask, [np.int32(c)], 255)
                p0 = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
                if p0 is not None:
                    self.robots_state[rid] = {'center': np.array([cx, cy], dtype=np.float32), 'angle': angle, 'features': p0, 'missed': 0}

        # 4. KLT Flow for missing robots
        if self.old_gray is not None:
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

        self.old_gray = gray.copy()

        # 5. Broadcasters
        for team_idx in range(self.CNT_TEAM):
            if frame_teams_data[team_idx]:
                self.team_pubs[team_idx].publish(String(data=json.dumps(frame_teams_data[team_idx])))
            if team_idx in frame_targets_data:
                self.target_pubs[team_idx].publish(String(data=json.dumps(frame_targets_data[team_idx])))

        cv2.imshow("Tactical Map", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

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
