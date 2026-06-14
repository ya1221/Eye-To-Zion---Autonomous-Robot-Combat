---
name: ros2-cpp-hardware-standards
description: Enforces C++ coding standards and architectural rules for an autonomous
 urban combat robot built on ROS2 Humble. Covers C++ nodes, launch files,
 URDF/xacro modeling, config YAML, Gazebo Classic simulation, Nav2, SLAM,
 EKF localization, and telemetry.
triggers: 
  - "When the user asks to write or edit C++ code, launch files, yaml files, or xacro files"
  - "When working inside the robot_description directory"
  - "When working on telemetry, InfluxDB, Telegraf, or GStreamer"
  - "When working on the robot model (URDF/xacro)"
---

# ROS2 Guidelines
You are an expert ROS2 developer working on the `/AutonomousWarfare` directory, but the domain directory is 'robot_description'. the project is about an autonomous urban combat robot. Your code must be highly performant, reliable, and integrate seamlessly with the rest of the system.

## 1. Architectural Rules
- **General Rules:** 
  - Always seperate cpp code into header and source files in the /include directory and /src directory respectively.
  - Always seperate yaml files for configuration in the /config directory.
  - Always seperate launch files into python files in the /launch directory.
  - Always use the /world/robot for modeling the robot in the simulation.

- **Node Structure:** 
  - All C++ nodes must inherit from `rclcpp::Node`. Do not use the older ROS1 `NodeHandle` style.
  - Always build C++ nodes as ROS2 Components (rclcpp_components) rather than standalone executables to ensure modularity.
  - Container Grouping (Zero-Copy): Do not group nodes blindly by "logic." Only launch nodes into the same shared Component Container if they pass high-bandwidth data to each other (e.g., LiDAR point clouds, maps occupancy grids, video frames). This enforces intra-process communication (zero-copy memory passing), which bypasses serialization overhead and is critical for hitting the <100ms latency target.

- **Luanch structure:** 
  - Always use the main launch.py file located in the robot_description/launch/ directory as the top-level entry point to include all other launch files in that directory.
  - sub-launch files strictly within their respective domain field (e.g., simulation, hardware, sensors, navigation).
  - Always expose `use_sim_time` as a `LaunchConfiguration` argument in launch files. Pass this argument dynamically to all node parameters so it can be easily toggled between simulation and physical hardware.

- **Robot modeling structure**
  - Always use the main robot.urdf.xacro file located in the robot_description/world/robot/ directory as the top-level entry point to include all other xacro files in that directory
  - Sub xacro files should be organized by the type of component they represent (e.g., base, wheels, sensors, ros2_control, gazebo_references).

## 2. Modern C++ Standards
- **Standard:** Use C++17 or C++20 standard features.
- **Memory Management:** Strictly use smart pointers (`std::make_shared`, `std::unique_ptr`). Never use raw pointers or `new`/`delete`.
- **Logging:** Use `RCLCPP_INFO`, `RCLCPP_WARN`, and `RCLCPP_ERROR` for all console outputs. Never use `std::cout` inside class that inherits from rclcpp::Node.


## 3. Navigation 
- Always use the /config/nav2.yaml for seeing the navigation parameters.
- Always use the /config/slam_toolbox.yaml for seeing the slam parameters.
- The navigation is always need to work with ackermann_steering_controller.


## 4. Simulation
- Simulator: Gazebo Classic (NOT Ignition/Gazebo Sim). Always use `gzserver`/`gzclient`, **not** `ign gazebo`.
- World: Inside `world/maze directory`.
- Plugins: `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`, `libgazebo_ros_ray_sensor.so`, `libgazebo_ros_imu_sensor.so`, `libgazebo_ros_camera.so`

## 5. Robot Physical Parameters
- the robot physical parameters are inside the /config/ackermann_steering_controller.yaml file.

## 6. TF Frame Tree
The canonical frame hierarchy is: map → odom → base_footprint → base_link → (lidar_link | imu_link | camera_link | rear_left_wheel_joint | rear_right_wheel_joint | front_left_steering_joint → front_left_wheel_joint | front_right_steering_joint → front_right_wheel_joint)

## 7. Key ROS2 Topics

| Topic               |     Message Type         |     Publisher   |      Subscriber           |
|---------------------|--------------------------|-----------------|---------------------------|
| `/scan`             | `sensor_msgs/LaserScan`  | Gazebo LiDAR plugin | SLAM, Nav2, rf2o      |
| `/odom`             | `nav_msgs/Odometry`      | ackermann_steering_controller | EKF (odom0) |
| `/rf2o/odom`        | `nav_msgs/Odometry`      | rf2o_laser_odometry | EKF (odom1)           |
| `/imu`              | `sensor_msgs/Imu`        | Gazebo IMU plugin | EKF (imu0)              |
| `/cmd_vel`          | `geometry_msgs/TwistStamped` | Nav2 | TwistToAckermann                 |
| `/ackermann_steering_controller/reference` | `geometry_msgs/TwistStamped` | TwistToAckermann | ackermann_steering_controller |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin | gstreamer_sender |
| `/map`              | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Nav2, map_to_image |

> When creating new nodes, always check this table before choosing topic names. Never duplicate a publisher for an existing topic.


## 8. Sensor Fusion (EKF) Standards for localization
  - Configuration File: Always use `/config/ekf.yaml` for Extended Kalman Filter parameters.
  - odom0 (Wheel Odom): Fuses linear X, Y and angular Z velocities. Set `odom0_differential: false`.
  - odom1 (RF2O Laser Odom): Fuses absolute X and Y position. Critical: Always set `odom1_differential: true` and `odom1_relative: true`. This ensures the EKF uses the changes in laser position rather than absolute coordinates, which prevents the robot from "teleporting" if the laser scan is noisy.
  - imu0 (IMU): Fuses orientation (Roll, Pitch, Yaw) and angular velocities. Set `imu0_remove_gravitational_acceleration: true`.


## 9. Launch Boot Sequence

The main `launch.py` orchestrates 4 sub-launches with **TimerAction delays** to respect dependency ordering:

| Order | Delay | Sub-launch | What it starts | Why this delay |
|---|---|---|---|---|
| 1 | 0s | `simulation.launch.py` | Gazebo + RSP + spawn + controllers | Must start first |
| 2 | 8s | `localization.launch.py` | EKF + rf2o + SLAM | Needs `/scan` and `/odom` from controllers |
| 3 | 12s | `navigation.launch.py` | Nav2 + TwistToAckermann | Needs `odom→map` TF chain |
| 4 | 14s | `visualization.launch.py` | RViz2 + Telegraf + GStreamer | Purely visual, start last |

Within `simulation.launch.py`, controller spawning uses `RegisterEventHandler(OnProcessExit)`:
`spawn_entity → joint_state_broadcaster → ackermann_steering_controller`

> **CRITICAL:** When adding new nodes, place them in the correct sub-launch file based on their dependencies. Never add a node to `launch.py` directly.

## 10. Telemetry & Monitoring Pipeline

- **telegraf_bridge_node** subscribes to ROS topics and writes InfluxDB line protocol to `/tmp/telegraf.sock`
- **Docker infrastructure** is defined in `config/docker-compose.yaml`
- InfluxDB buckets: `sensors` (ROS data), `system` (CPU/mem/disk)
- Grafana uses a custom plugin: `roboticsorg-robot-panel`
- GStreamer sends camera video via UDP (port 5000) using H.264 encoding

> When adding new telemetry data, modify `telegraf_bridge_node` to publish to the existing socket. Never create separate database connections.

## 11. Build System

- **Build tool:** `colcon build` from the workspace root (`/home/itay3711/AutonomousWarfare/`)
- **Build command:** `colcon build --packages-select robot_description`
- **Source overlay:** `source install/setup.bash`
- **Launch:** `ros2 launch robot_description launch.py`
- **CMake standard:** C++17 (`target_compile_features(... PUBLIC cxx_std_17)`)
- **Compiler flags:** `-Wall -Wextra -Wpedantic`

## 12. Adding a New C++ Node
When adding a new node, you must update:
1. `src/<node_name>.cpp` — source file
2. `include/robot_description/<node_name>.hpp` — header file
3. `CMakeLists.txt` — `add_executable()`, `target_include_directories()`, `target_compile_features()`, `ament_target_dependencies()`, and add to `install(TARGETS ...)`
4. `package.xml` — any new `<depend>` entries
5. The appropriate sub-launch file

## 13. External Dependencies (already in workspace)
Do **NOT** add these as rosdep dependencies — they are vendored:
- `gazebo_ros2_control` — ros2_control ↔ Gazebo Classic bridge
- `rf2o_laser_odometry` — laser scan odometry
- `cpr` — C++ HTTP requests library
- `influxdb-cxx` — InfluxDB C++ client library


## 14. Sensors

All sensors are defined in `world/robot/sensors.xacro`. Each has a URDF link, a fixed joint to `base_link`, and a Gazebo plugin.

| Sensor | Link | Gazebo Plugin | Key Params |
|---|---|---|---|
| 2D LiDAR (TG15) | `lidar_link` | `libgazebo_ros_ray_sensor.so` | 1440 samples, 360°, 15m range, 26Hz |
| IMU | `imu_link` | `libgazebo_ros_imu_sensor.so` | 100Hz |
| Camera | `camera_link` | `libgazebo_ros_camera.so` | 1280×720, 30fps, 141° FOV |

> When adding a new sensor, add it to `sensors.xacro` following the existing pattern: link → joint → gazebo plugin.






