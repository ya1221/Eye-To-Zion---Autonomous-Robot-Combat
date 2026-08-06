# sensor_fusion_pkg

## Purpose
Pre-process and fuse raw sensor inputs before they hit the main EKF.

## Logic
Aligns timestamps and covariance matrices of wheel odometry and IMU data to ensure smooth filtering.

## Data Flow
- **Input:** Raw IMU, raw encoders.
- **Output:** Filtered `/odom` or `/imu/data`.
