# ai_vision

## Purpose
Perform object detection to identify targets, obstacles, and navigation markers.

## Logic
Processes incoming video frames using a YOLO-based computer vision model to draw bounding boxes and determine the relative position of enemies or objectives.

## Data Flow
- **Input:** Video stream (UDP/Camera node).
- **Output:** Publishes `DetectionArray` or similar vision data to `/vision/detections`.
