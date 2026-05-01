import cv2
import cv2.aruco as aruco
import numpy as np
import redis
import json
import math

# Connect to Redis (Lazy connection)
redis_client = redis.Redis(host='localhost', port=6379, db=0, decode_responses=True)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("[INFO] Tactical Overhead Tracker is ONLINE. Press 'q' to exit.")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    corners, ids, rejected = detector.detectMarkers(frame)
    fleet_positions = []

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            try:
                marker_id = int(ids[i][0])
                c = corners[i][0]

                center_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                center_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)

                dx = c[1][0] - c[0][0]
                dy = c[1][1] - c[0][1]
                angle_rad = math.atan2(dy, dx)
                angle_deg = math.degrees(angle_rad)

                fleet_positions.append({
                    "id": marker_id,
                    "x": center_x,
                    "y": center_y,
                    "angle": round(angle_deg, 2)
                })

                end_x = int(center_x + 50 * math.cos(angle_rad))
                end_y = int(center_y + 50 * math.sin(angle_rad))
                cv2.line(frame, (center_x, center_y), (end_x, end_y), (0, 0, 255), 3)
                
            except Exception as e:
                # Catch math or array parsing errors without crashing
                print(f"[ERROR] Math/Parsing failed for marker: {e}")

        # Attempt to publish to Redis
        try:
            if len(fleet_positions) > 0:
                redis_client.publish('fleet_positions', json.dumps(fleet_positions))
        except Exception as e:
            # Catch Redis connection errors without crashing
            print(f"[ERROR] Redis Connection Failed: {e}")

    cv2.imshow("Eye To Zion - Tactical Map", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
