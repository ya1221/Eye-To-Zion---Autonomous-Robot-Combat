# navigation

## Purpose
Provide autonomous path planning and obstacle avoidance.

## Logic
Uses the ROS 2 Nav2 stack to generate global paths and local trajectories, avoiding obstacles detected by the Lidar.

## Data Flow
- **Input:** Goals (`/goal_pose`), Lidar (`/scan`), TF tree.
- **Output:** `geometry_msgs/Twist` on `/nav_vel` or directly to `/cmd_vel`.
