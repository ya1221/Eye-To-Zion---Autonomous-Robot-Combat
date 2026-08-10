# 🎮 Gazebo Simulation & Offboard Navigation

> **Purpose:** Provide a high-fidelity Gazebo simulation environment mirroring the physical Eye-To-Zion robot, complete with full ROS 2 navigation (Nav2), SLAM, sensor simulation, and visualization bridges.

---

## Table of Contents

1. [Overview](#overview)
2. [Directory Structure](#directory-structure)
3. [Architecture & Launch System](#architecture--launch-system)
4. [Custom Nodes](#custom-nodes)
5. [Configuration Details](#configuration-details)
6. [Multi-Robot Support](#multi-robot-support)

---

## Overview

The `gazebo` directory encapsulates the complete offboard simulation stack. It allows developers to test mapping, autonomous navigation (Nav2), obstacle avoidance, and control logic in a virtual maze without risking hardware. 

The stack uses **Gazebo Classic** alongside **ROS 2 Humble**, utilizing `gazebo_ros2_control` for physics-based actuation of the Ackermann steering mechanism.

---

## Directory Structure

```
Offboard/gazebo/
├── nav2_params_humble/        # Nav2 parameters (including multi-robot configs)
│   ├── nav2_params.yaml
│   └── nav2_multirobot_params_*.yaml
└── robot_description/         # Primary ROS 2 package
    ├── CMakeLists.txt & package.xml
    ├── launch/                # Layered launch files
    ├── src/                   # C++ custom nodes
    ├── config/                # YAML configs (EKF, SLAM, Nav2, Telegraf)
    ├── world/                 # Gazebo .world files and robot URDF/Xacro models
    └── rviz2/                 # Pre-configured RViz visualization panel
```

---

## Architecture & Launch System

The simulation utilizes a sequenced, event-driven launch hierarchy initiated by `launch.py`.

### Launch Sequence (`robot_description/launch/launch.py`)

1. **`simulation.launch.py` (T=0s):** 
   - Starts `gzserver` and `gzclient` with the maze world.
   - Parses the Xacro robot description (stripping comments to avoid `ros2_control` parser bugs).
   - Runs `robot_state_publisher` and spawns the robot in Gazebo.
   - Starts the `TwistToAckermann` node.
   - Spawns the hardware controllers (`joint_state_broadcaster`, `ackermann_steering_controller`).
2. **`localization.launch.py` (T=8s):**
   - Starts `robot_localization` (EKF) combining odometry.
   - Runs `rf2o_laser_odometry` to generate odometry from `/scan`.
   - Starts `async_slam_toolbox_node` for real-time 2D mapping.
3. **`navigation.launch.py` (T=12s):**
   - Brings up the full **Nav2 stack** (Planner, Controller, BT Navigator, Waypoint Follower, Smoothers).
   - Connects the Nav2 output (`/cmd_vel`) to the local control stack.
4. **`visualization.launch.py` (T=14s):**
   - Opens the custom `rviz2_panel.rviz`.
   - Starts the UDP GStreamer pipeline for camera viewing.

---

## Custom Nodes

The `robot_description` package includes custom C++ nodes located in `src/`:

| Node | Description |
|------|-------------|
| **`TwistToAckermann`** | Subscribes to standard Nav2 `/cmd_vel` (`geometry_msgs/Twist`) and converts it to `/ackermann_steering_controller/reference` (`geometry_msgs/TwistStamped`), bridging differential drive commands to the robot's physical steering mechanism. |
| **`map_to_image_node`** | Subscribes to the SLAM `/map` topic (`OccupancyGrid`), applies an OpenCV lookup table (LUT) to convert probabilities into a grayscale visual map, and saves it as a JPEG file for dashboard ingestion. |
| **`gstreamer_sender`** | Subscribes to `/camera/image_raw`, converts ROS images to OpenCV format, and streams them via a GStreamer pipeline over UDP (`port=5000`) with H.264 encoding for offboard low-latency monitoring. |

*Note: The `telegraf_bridge` and `telemetry_sender` files exist here historically but are actively maintained in the `RemotePC/telemetry_data` package.*

---

## Configuration Details

All core YAML configs reside in `robot_description/config/`:

- **`nav2.yaml`**: Extensive parameter set tuning the DWB Local Planner (adjusting bounds for Ackermann steering), Costmaps (voxel and inflation layers), and Behavior Trees.
- **`ekf.yaml` / `slam.yaml`**: Settings for sensor fusion and SLAM Toolbox.
- **`ackermann_steering_controller.yaml`**: `ros2_control` parameters defining the rear-wheel drive and front-wheel steering kinematics.
- **`telegraf.conf`**: Agent config to pipe simulation metrics into the InfluxDB stack, mirroring real-world telemetry behavior.

---

## Multi-Robot Support

The `nav2_params_humble/` directory includes specialized configuration files (`nav2_multirobot_params_1.yaml`, `nav2_multirobot_params_2.yaml`). 

These configure the TF trees (using namespace prefixes), costmaps, and node names to allow multiple instances of the robot to run simultaneously within the same simulation arena, paving the way for multi-agent combat scenarios.
