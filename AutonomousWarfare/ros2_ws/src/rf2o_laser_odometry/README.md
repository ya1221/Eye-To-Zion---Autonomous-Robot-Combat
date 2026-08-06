# rf2o_laser_odometry

## Purpose
Compute odometry from 2D laser scans.

## Logic
Matches consecutive Lidar scans using RF2O to estimate the robot's velocity and position changes (useful if wheel slip occurs).

## Data Flow
- **Input:** `/scan`.
- **Output:** `nav_msgs/Odometry` on `/odom_rf2o`.
