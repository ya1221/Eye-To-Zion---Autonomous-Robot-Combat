# localization

## Purpose
Estimate the robot's precise pose and position within the map.

## Logic
Fuses odometry, IMU, and Lidar data using an Extended Kalman Filter (EKF) or AMCL to provide a reliable transform from the `map` frame to the `base_link` frame.

## Data Flow
- **Input:** Odometry (`/odom`), IMU (`/imu/data`), Lidar Scans (`/scan`).
- **Output:** `tf` transforms (`map` -> `odom` -> `base_link`).
