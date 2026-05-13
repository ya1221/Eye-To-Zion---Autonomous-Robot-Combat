import cv2
import cv2.aruco as aruco
import numpy as np
import redis
import json
import math

redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
feature_params = dict(maxCorners=20, qualityLevel=0.3, minDistance=7, blockSize=7)

GRID_N = 5
VALID_IDS = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10} # Anchors: 0-3, Robots: 4+

dst_pts = np.array([
    [0, 0],
    [GRID_N, 0],
    [GRID_N, GRID_N],
    [0, GRID_N]
], dtype=np.float32)

perspective_matrix = None
robots_state = {}
old_gray = None

print("[INFO] Searching for cameras...")
cap = cv2.VideoCapture(2)
if not cap.isOpened():
    print("[WARN] USB Camera (video2) not found. Falling back to built-in camera (video0).")
    cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("[ERROR] No cameras detected at all! Exiting.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("[INFO] Tactical Tracker (Grid + KLT) is ONLINE. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(frame)
    fleet_positions = []
    current_detected_ids = set()
    corners_dict = {}

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            try:
                marker_id = int(ids[i][0])
                
                if marker_id not in VALID_IDS:
                    continue
                    
                current_detected_ids.add(marker_id)
                c = corners[i][0]

                center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)

                dx = c[1][0] - c[0][0]
                dy = c[1][1] - c[0][1]
                angle_deg = math.degrees(math.atan2(dy, dx))

                corners_dict[marker_id] = (center_x, center_y, angle_deg, c)

                end_x = int(center_x + 50 * math.cos(math.radians(angle_deg)))
                end_y = int(center_y + 50 * math.sin(math.radians(angle_deg)))
                cv2.line(frame, (center_x, center_y), (end_x, end_y), (0, 0, 255), 3)
            except Exception as e:
                pass

    # Strict Anchor Check
    if all(anchor in corners_dict for anchor in [0, 1, 2, 3]):
        src_pts = np.array([
            corners_dict[1][:2],
            corners_dict[2][:2],
            corners_dict[3][:2],
            corners_dict[0][:2]
        ], dtype=np.float32)
        perspective_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
    else:
        # Invalidates the matrix immediately if even one anchor drops
        perspective_matrix = None

    # Hard Block UI
    if perspective_matrix is None:
        cv2.putText(frame, "WAITING FOR 4 ANCHORS...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
        cv2.imshow("Eye To Zion - Tactical Map", frame)
        
        # Reset tracking history to avoid jumping when anchors return
        robots_state.clear() 
        old_gray = None
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue 

    for marker_id, data in corners_dict.items():
        if marker_id in [0, 1, 2, 3]:
            continue

        cx, cy, angle_deg, c = data

        pt_cam = np.array([[[cx, cy]]], dtype=np.float32)
        pt_grid = cv2.perspectiveTransform(pt_cam, perspective_matrix)
        grid_x, grid_y = int(pt_grid[0][0][0]), int(pt_grid[0][0][1])

        fleet_positions.append({
            "id": marker_id,
            "x": grid_x,
            "y": grid_y,
            "angle": round(angle_deg, 2)
        })

        mask = np.zeros_like(gray)
        cv2.fillPoly(mask, [np.int32(c)], 255)
        p0 = cv2.goodFeaturesToTrack(gray, mask=mask, **feature_params)

        if p0 is not None:
            robots_state[marker_id] = {
                'center': np.array([cx, cy], dtype=np.float32),
                'angle': angle_deg,
                'features': p0,
                'missed_frames': 0
            }

    if old_gray is not None:
        for rid in list(robots_state.keys()):
            if rid not in current_detected_ids:
                state = robots_state[rid]
                
                if state['missed_frames'] < 30:
                    p0 = state['features']
                    if p0 is not None and len(p0) > 0:
                        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray, p0, None, **lk_params)
                        
                        good_new = p1[st == 1]
                        good_old = p0[st == 1]

                        if len(good_new) > 0:
                            movement = np.mean(good_new - good_old, axis=0)
                            state['center'] += movement
                            state['features'] = good_new.reshape(-1, 1, 2)
                            state['missed_frames'] += 1

                            cx, cy = int(state['center'][0]), int(state['center'][1])

                            pt_cam = np.array([[[cx, cy]]], dtype=np.float32)
                            pt_grid = cv2.perspectiveTransform(pt_cam, perspective_matrix)
                            grid_x, grid_y = int(pt_grid[0][0][0]), int(pt_grid[0][0][1])

                            fleet_positions.append({
                                "id": rid,
                                "x": grid_x,
                                "y": grid_y,
                                "angle": round(state['angle'], 2)
                            })

                            cv2.circle(frame, (cx, cy), 25, (0, 255, 255), 2)
                            cv2.putText(frame, f"KLT ID: {rid}", (cx-25, cy-35), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        else:
                            del robots_state[rid]
                    else:
                        del robots_state[rid]
                else:
                    del robots_state[rid]

    old_gray = gray.copy()

    try:
        if len(fleet_positions) > 0:
            redis_client.publish('fleet_positions', json.dumps(fleet_positions))
    except Exception as e:
        print(f"[ERROR] Redis Connection Failed: {e}")

    cv2.imshow("Eye To Zion - Tactical Map", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
