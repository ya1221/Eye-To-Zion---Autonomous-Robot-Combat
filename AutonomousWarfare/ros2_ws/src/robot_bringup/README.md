# robot_bringup

## Purpose
Centralized launch configuration for the entire robot stack.

## Logic
Contains master launch files that sequentially start hardware drivers, AI nodes, localization, and navigation with the correct parameters.

## Data Flow
- **Input:** User execution (`ros2 launch robot_bringup ...`).
- **Output:** Process management for all other ROS 2 nodes.
