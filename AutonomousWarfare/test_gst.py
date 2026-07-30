import os
# os.environ["OPENCV_LOG_LEVEL"] = "ERROR"
import cv2
cap = cv2.VideoCapture("udpsrc port=5000 ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1", cv2.CAP_GSTREAMER)
print("Opened:", cap.isOpened())
cap.release()
