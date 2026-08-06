# ydlidar_ros2_driver

## Purpose
Hardware driver for the YDLidar.

## Logic
Reads raw serial data from the YDLidar and converts it into ROS 2 standard LaserScan messages.

## Data Flow
- **Input:** Serial USB data.
- **Output:** `sensor_msgs/LaserScan` on `/scan`.
