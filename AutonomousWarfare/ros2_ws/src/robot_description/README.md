# robot_description

## Purpose
Define the physical layout and kinematics of the robot.

## Logic
Provides URDF/Xacro files specifying the locations of the Lidar, Camera, IMU, and wheels relative to the `base_link`.

## Data Flow
- **Input:** Joint states.
- **Output:** Static `tf` transforms published by `robot_state_publisher`.
