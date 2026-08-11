# sensor_fusion_pkg

## Purpose
Fuse visual detections with LiDAR scans and global robot pose to calculate global enemy coordinates in real time.

## Logic
Pairs YOLO detections with the latest LiDAR scan data. Uses the relative angle to extract the target's range, filters noise within a local beam window, and projects the distance using the robot's heading to compute the global target position in the map frame.

## Data Flow
- **Input:** Global robot pose, raw LiDAR scans, and YOLO relative target angles.
- **Output:** Global coordinates of detected enemy targets.
