import cv2
import redis
import json
from ultralytics import YOLO

# 1. Initialize Redis connection
try:
    redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
    redis_client.ping()
    print("Connected to Redis successfully!")
except redis.ConnectionError:
    print("Failed to connect to Redis. Is the Docker container running?")
    exit(1)

# 2. Load custom trained model
model = YOLO('results/runs/detect/EyeToZion_AI/yolo26_robot_detect/weights/best.pt')

# 3. Camera settings & Constants (Based on Student C's specs)
HFOV = 141.0  # Horizontal Field of View of your webcam in degrees

## # Use the libcamera GStreamer pipeline for Raspberry Pi 5
## pipeline = "libcamerasrc ! video/x-raw, width=1280, height=720, framerate=30/1 ! videoconvert ! appsink"
## cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


cap = cv2.VideoCapture("WhatsApp Video 2026-04-13 at 23.17.44.mp4")
# cap = cv2.VideoCapture(0)

print("Camera started. Hunting for enemy robots...")

while True:
    success, frame = cap.read()
    if not success:
        print("Failed to grab frame from camera.")
        break

    # Get image dimensions for math calculations
    img_width = frame.shape[1]
    img_center_x = img_width / 2

    # Run inference (verbose=False keeps the terminal clean)
    results = model(frame, verbose=False)

    for result in results:
        for box in result.boxes:
            # We only care about highly confident detections
            if box.conf[0] > 0.75:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                
                # --- STUDENT C'S MATH LOGIC ---
                # 1. Find object center
                box_center_x = (x1 + x2) / 2
                
                # 2 & 3. Calculate offset from center (Positive = Right, Negative = Left)
                offset_x = box_center_x - img_center_x
                
                # 4. Convert to angle
                angle_degrees = offset_x * (HFOV / img_width)
                
                # --- REDIS COMMUNICATION ---
                # Build the exact JSON payload requested
                payload = {
                    "robot_id": "enemy_1",
                    "type": "enemy",
                    "angle": round(angle_degrees, 2)
                }
                
                # Publish to the '/threats' channel
                json_data = json.dumps(payload)
                redis_client.publish('/threats', json_data)
                
                # --- VISUAL DEBUGGING (Optional but highly recommended) ---
                # Draw bounding box and center dot
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
                cv2.circle(frame, (int(box_center_x), int((y1+y2)/2)), 5, (0, 255, 0), -1)
                
                # Show the calculated angle on screen
                cv2.putText(frame, f"Angle: {round(angle_degrees, 1)} deg", 
                            (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.6, (0, 255, 0), 2)

    # Display the live feed
    cv2.imshow("Robot Combat Vision (YOLO26)", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
