# 🛰️ Forward Command Post (FCP)

> **Purpose:** The Forward Command Post acts as the centralized tactical overseer for the battlefield. It tracks all combatants and objectives via an overhead camera, bridges real-time tactical data across the VPN using Zenoh, and orchestrates the multi-team combat scenario.

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture & Data Flow](#architecture--data-flow)
3. [Core Services](#core-services)
   - [Overhead Tracker](#overhead-tracker)
   - [Zenoh Bridge](#zenoh-bridge)
4. [Marker Allocation](#marker-allocation)
5. [Telemetry Publishing](#telemetry-publishing)
6. [Quick Start](#quick-start)

---

## Overview

The `Forward_Command_Post` directory contains a containerized ROS 2 workspace that runs on a central PC (or a dedicated observer node) overlooking the battlefield. 

It uses a high-resolution downward-facing camera to detect ArUco markers placed on the arena corners, the robots, and their objectives (targets). It applies perspective warping to establish a unified `(X, Y)` coordinate grid, computes distances to detect zone capture events, and broadcasts this data across the VPN to all participating robots.

---

## Architecture & Data Flow

```
┌───────────────────────────────────────────────────────────────────────┐
│                        FORWARD COMMAND POST                           │
│                                                                       │
│  ┌──────────────┐     ┌─────────────────────────────────────────┐     │
│  │ USB Camera   │────▶│ overhead_tracker (OpenCV / ArUco / KLT) │     │
│  └──────────────┘     └───────────────────┬─────────────────────┘     │
│                                           │                           │
│                                      ROS 2 DDS (Domain 0)             │
│                                           │                           │
│                       ┌───────────────────▼─────────────────────┐     │
│                       │ zenoh_bridge (eclipse/zenoh-bridge)     │     │
│                       └───────────────────┬─────────────────────┘     │
└───────────────────────────────────────────┼───────────────────────────┘
                                            │
                                     Tailscale VPN
                                            │
               ┌────────────────────────────┼───────────────────────────┐
               ▼                            ▼                           ▼
        ┌────────────┐               ┌────────────┐              ┌────────────┐
        │  Robot A   │               │  Robot B   │              │ Dashboard  │
        └────────────┘               └────────────┘              └────────────┘
```

---

## Core Services

The system is fully containerized via `docker-compose.yml` and runs on the host network to allow seamless DDS and Zenoh communication.

### 1. Overhead Tracker (`overhead_tracker_node`)

A custom ROS 2 Python node (`ros2_ws/src/overhead_tracker/overhead_tracker/overhead_tracker.py`) responsible for machine vision.

**Key Capabilities:**
- **Perspective Warping:** Detects 4 anchor markers (IDs 0–3) at the arena corners to calculate a homography matrix (`cv2.getPerspectiveTransform`). This maps the angled camera view to a perfect top-down 2D grid (`grid_n: 2000`).
- **Modulo Team Assignment:** Robots and Targets are dynamically assigned to teams based on their ArUco ID modulo the total number of teams (`CNT_TEAM`).
- **Hidden Robot Tracking (KLT):** Uses the Lucas-Kanade optical flow algorithm (`cv2.calcOpticalFlowPyrLK`) to continuously track robots even if their ArUco marker is temporarily obscured by weapons or terrain.
- **Capture Zone Detection:** Calculates the Euclidean distance between a robot and its target. If a robot stays within the `proximity_threshold_cm` (default: 20cm), it registers as occupying the zone.

### 2. Zenoh Bridge (`zenoh_bridge`)

Uses `eclipse/zenoh-bridge-ros2dds` to connect the local ROS graph to the Tailscale VPN mesh.

**Security & Bandwidth Control (`zenoh/config.json5`):**
- Operates on a strict **allowlist** policy.
- Only topics, services, and actions matching the regex `^/teams/.*` are permitted to cross the bridge.
- Prevents heavy local robot topics (like `/scan`, `/map`, or `/tf`) from flooding the VPN.
- Caps publish frequencies (e.g., `^/teams/.*=40`) to maintain network stability over lossy wireless links.

---

## Marker Allocation

The tracker uses specific ArUco marker IDs (from `DICT_4X4_50`) to identify different entities. By default, for a 2-team game (`cnt_team: 2`), the ID mapping is:

| Entity Type | ArUco IDs | Description |
|-------------|-----------|-------------|
| **Anchors** | `0, 1, 2, 3` | Placed at the 4 corners of the arena to define the warp plane. |
| **Targets** | `4, 5` | Stationary objectives. Target `4` belongs to Team 0 (4 % 2 = 0). Target `5` belongs to Team 1 (5 % 2 = 1). |
| **Robots** | `6, 7, ...` | Mobile combatants. Robot `6` belongs to Team 0 (6 % 2 = 0). Robot `7` belongs to Team 1 (7 % 2 = 1). |

*(Configurable via `targets_start` and `cnt_team` in `params.yaml`).*

---

## Telemetry Publishing

The tracker translates visual data into JSON strings and publishes them to ROS 2 topics under the allowed `/teams/` namespace.

**1. Robot Positions (`/teams/team_{idx}/positions`)**
```json
[
  {
    "id": 6,
    "x": 1500.5,
    "y": 450.2,
    "angle": 90.0
  }
]
```

**2. Target Positions (`/teams/team_{idx}/target_position`)**
```json
{
  "x": 200.0,
  "y": 1800.0
}
```

**3. Zone Occupation (`/teams/team_{idx}/zone_occupied`)**
- Publishes `true` (throttled to 0.5s intervals) whenever a robot is within the capture radius of its team's target. Each publish act can be used by the backend to award points.

---

## Quick Start

### 1. Hardware Setup
- Mount a USB camera (e.g., Logitech C920) high above the arena.
- Place ArUco markers 0, 1, 2, and 3 on the corners of the play area.

### 2. Configuration
Edit `ros2_ws/src/overhead_tracker/config/params.yaml` to match your setup:
- `camera_id`: The `/dev/videoX` index of your camera.
- `cnt_team`: Number of teams playing.

### 3. Launching

Run the provided script to build the workspace (if needed) and launch the containers:

```bash
cd Forward_Command_Post
./run_project.sh
```

A tactical map window will appear showing the raw camera feed overlaid with detections, bounding boxes, optical flow vectors (yellow circles), and the warped coordinate grid.
