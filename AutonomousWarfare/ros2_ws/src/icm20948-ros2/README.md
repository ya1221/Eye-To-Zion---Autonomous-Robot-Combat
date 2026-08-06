# icm20948-ros2

## Purpose
ROS 2 wrapper for the ICM20948 IMU.

## Logic
Polls the Python driver at a high frequency and publishes standard IMU messages.

## Data Flow
- **Input:** Python I2C driver data.
- **Output:** `sensor_msgs/Imu` on `/imu/data`.
