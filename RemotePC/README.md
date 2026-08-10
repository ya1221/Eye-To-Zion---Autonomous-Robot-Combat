# RemotePC — Off-Robot Visualization & Telemetry Station

The **RemotePC** directory contains everything needed to run the operator-side infrastructure on a separate computer (laptop, desktop, or cloud VM). It receives live sensor data from the robot over the network, persists telemetry into a time-series database, and serves real-time dashboards and 3D visualizations — all containerized with Docker Compose.

> **TL;DR** — `docker compose up` on your PC and you get: live 3D robot visualization (Foxglove), Grafana dashboards backed by InfluxDB v3, and full ROS 2 topic access — all streamed from the robot over a Tailscale VPN.

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  ROBOT  (Raspberry Pi 5)                                                    │
│                                                                             │
│   ROS 2 Nodes ──► Zenoh Bridge ──────────── Tailscale VPN ─────────┐       │
│   (sensors, nav, etc.)           tcp/:7447                         │       │
└────────────────────────────────────────────────────────────────────┬┘       │
                                                                     │       │
┌────────────────────────────────────────────────────────────────────▼───────┐
│  REMOTE PC  (This directory)                                               │
│                                                                             │
│   ┌──────────────┐    ROS 2 DDS    ┌───────────────┐                       │
│   │ Zenoh Bridge │ ◄──────────────► │  ros2_core    │                       │
│   │  (listener)  │   (local IPC)   │  (Foxglove    │                       │
│   └──────────────┘                 │   Bridge)     │                       │
│         │                          └──────┬────────┘                       │
│         │ mirrors topics                  │ ws://localhost:8765             │
│         │ to local DDS                    │                                │
│         ▼                                 ▼                                │
│   ┌──────────────┐              ┌──────────────────┐                       │
│   │ Telegraf     │              │ Foxglove Studio  │                       │
│   │ Bridge Node  │              │ (Lichtblick)     │                       │
│   │ (ROS 2 C++)  │              │ :8080            │                       │
│   └──────┬───────┘              └──────────────────┘                       │
│          │ Unix socket                                                     │
│          │ /tmp/sockets/telegraf.sock                                      │
│          ▼                                                                 │
│   ┌──────────────┐    InfluxDB Line    ┌───────────────┐                   │
│   │  Telegraf    │ ──── Protocol ────► │  InfluxDB v3  │                   │
│   │  (agent)     │                     │   Core :8181  │                   │
│   └──────────────┘                     └──────┬────────┘                   │
│                                               │                            │
│                                               ▼                            │
│                                        ┌───────────────┐                   │
│                                        │   Grafana     │                   │
│                                        │    :3000      │                   │
│                                        └───────────────┘                   │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Data Flow

### 1. Robot → RemotePC (Zenoh over Tailscale)

The robot runs a **Zenoh-Bridge-ROS2DDS** that publishes selected ROS 2 topics over a TCP transport. The RemotePC runs a matching Zenoh bridge that connects back to the robot's IP (`tcp/ROBOT_IP:7447`). Together, they transparently mirror a curated subset of topics across the WAN link — no ROS 2 multicast required.

**Bridged topics** (defined in `zenoh_config.json5`):

| Topic | Type | Purpose |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 2D Lidar scan data |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM-generated occupancy map |
| `/robot_description` | `std_msgs/String` | URDF model for 3D rendering |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree (live poses) |
| `/tf_static` | `tf2_msgs/TFMessage` | Static transforms (sensor offsets) |
| `/plan` | `nav_msgs/Path` | Nav2 planned path |

> **Why an allow-list?** Without it, the Zenoh bridge mirrors *every* local ROS 2 topic — including Nav2 internals, `/odometry/filtered`, and other high-bandwidth topics that would saturate a WAN link and risk topic-name collisions between teammates.

### 2. Local ROS 2 Domain (IPC)

Once topics land on the RemotePC's local DDS domain (via the Zenoh bridge), they become available to **any local ROS 2 node** as if the robot were on the same LAN. Two consumers are configured:

- **Foxglove Bridge** (`ros2_core` container) — Exposes all local topics over a WebSocket at `ws://localhost:8765`.
- **Telegraf Bridge** (`telemetry_data` package) — A C++ ROS 2 node that subscribes to game-state topics and forwards formatted metrics to Telegraf.

### 3. Telemetry Pipeline (ROS 2 → Telegraf → InfluxDB → Grafana)

```
ROS 2 Topic                    Telegraf Bridge Node                 Telegraf Agent
/teams_team/team_X/zone_occupied  ──►  formats InfluxDB Line Protocol  ──►  receives via Unix socket
                                       robot_occupied,id=X occupied=true      /tmp/sockets/telegraf.sock
                                                                                      │
                                                                                      ▼
                                                                              InfluxDB v3 Core
                                                                              bucket: "sensors"
                                                                              tagged: source=ros2_bridge
                                                                                      │
                                                                                      ▼
                                                                                   Grafana
                                                                              queries InfluxDB via SQL
```

**Key design decisions:**
- **Unix socket** between Telegraf Bridge and Telegraf Agent — avoids network overhead for co-located containers; the socket is shared via a Docker named volume (`telegraf-sock`).
- **Tag-based routing** — Telegraf only forwards metrics tagged with `source=ros2_bridge` to the `sensors` bucket, keeping game telemetry isolated from system metrics.
- **InfluxDB Line Protocol** — Metrics are formatted directly as line protocol strings (e.g., `robot_occupied,id=0 occupied=true 1723456789000000000`) with nanosecond timestamps for maximum precision.

### 4. Visualization (Foxglove Studio)

**Foxglove Studio** (served by the `lichtblick` container at `http://localhost:8080`) provides a rich web-based 3D visualization interface. Connect to `ws://localhost:8765` from within the UI to see:
- Live Lidar scan overlay on the SLAM map
- Robot URDF model rendered in 3D with live TF updates
- Nav2 planned path visualization

---

## Docker Services

All services are defined in `docker-compose.yaml` under the Compose project name `visualization-system`.

| Service | Container Name | Image / Dockerfile | Port(s) | Purpose |
|---|---|---|---|---|
| `influxdb` | `influxdb3-core` | `influxDB.Dockerfile` (→ `influxdb:3-core`) | `8181` | Time-series database for telemetry storage |
| `grafana` | `grafana` | `grafana/grafana-oss` | `3000` | Dashboard and analytics UI |
| `zenoh-bridge` | `zenoh-bridge` | `eclipse/zenoh-bridge-ros2dds:latest` | `7447` | Bridges robot ROS 2 topics to local DDS domain |
| `ros2_core` | `ros2_humble` | `ros2.Dockerfile` (→ `ros:humble-ros-base`) | `8765` (ws) | Foxglove Bridge — WebSocket relay for ROS 2 topics |
| `telegraf` | `telegraf` | `telegraf:latest` | — | Metrics agent, ingests from Unix socket, writes to InfluxDB |
| `foxglove-studio` | `foxglove-studio` | `ghcr.io/lichtblick-suite/lichtblick:latest` | `8080` | Web-based 3D visualization UI |

> All services use `network_mode: host` — they bind directly to the host's network stack. No port mapping is needed; services communicate via `localhost`.

---

## File Manifest

```
RemotePC/
├── docker-compose.yaml          # Defines all 6 services (the main orchestration file)
├── tailscale-docker.yaml        # Separate Compose file for the Tailscale VPN sidecar
├── influxDB.Dockerfile          # Builds InfluxDB v3 Core with correct directory ownership
├── ros2.Dockerfile              # Builds ROS 2 Humble with Foxglove Bridge + workspace
├── entrypoint.sh                # ROS 2 container entrypoint — auto-builds workspace on first run
├── telegraf.conf                # Telegraf agent config — Unix socket input → InfluxDB v2 output
├── zenoh_config.json5           # Zenoh bridge allow-list (restricts which topics are bridged)
└── ros2_ws/
    └── src/
        └── telemetry_data/      # ROS 2 C++ package — the Telegraf Bridge node
            ├── CMakeLists.txt
            ├── package.xml
            ├── config/
            │   └── params.yaml              # num_teams parameter (default: 2)
            ├── include/telemetry_data/
            │   ├── telegraf_bridge.hpp       # Node class — subscribes to team zone topics
            │   └── telemetry_sender.hpp      # Async Boost.Asio sender over Unix socket
            ├── launch/
            │   └── launch.py                # ROS 2 launch file for telegraf_bridge node
            └── src/
                ├── telegraf_bridge.cpp       # Main node — subscribes & formats metrics
                └── telemetry_sender.cpp      # Socket connection & async write logic
```

---

## Networking

### Tailscale VPN (WAN Connectivity)

A separate Compose file (`tailscale-docker.yaml`) runs a **Tailscale sidecar** container that joins the robot and RemotePC to the same private Tailnet. This provides:

- **Peer-to-peer encrypted tunnels** (`TS_USERSPACE=false` enables kernel-mode WireGuard for best performance).
- **Stable hostnames** — the RemotePC appears as `remote_pc` on the Tailnet.
- **Persistent login state** — stored in `./tailscale-state/` so re-authentication isn't needed across restarts.

> **Deployment note:** Replace `ROBOT_IP` in the `zenoh-bridge` service command with the robot's Tailscale IP (e.g., `100.x.y.z`) or MagicDNS hostname.

### Zenoh Bridge (Topic Filtering)

The Zenoh bridge configuration (`zenoh_config.json5`) defines an **allow-list** of ROS 2 topic patterns that are permitted to cross the WAN link. Only subscriber-side patterns are specified — the bridge will only receive and re-publish these specific topics from the robot:

```json5
{
  plugins: {
    ros2dds: {
      allow: {
        subscribers: ["^/scan$", "^/map$", "^/robot_description$", "^/tf$", "^/tf_static$", "^/plan$"],
      },
    },
  },
}
```

This prevents bandwidth-heavy internal topics (Nav2 costmaps, raw odometry, etc.) from being mirrored unnecessarily.

---

## ROS 2 Package: `telemetry_data`

A C++ ament package that bridges ROS 2 topic data into the Telegraf/InfluxDB pipeline.

### Nodes

#### `telegraf_bridge`

| Property | Value |
|---|---|
| **Subscriptions** | `teams_team/team_{i}/zone_occupied` (`std_msgs/Bool`) for `i` in `[0, num_teams)` |
| **Parameter** | `num_teams` (int, default: `2`) — number of team topics to subscribe to |
| **Output** | Writes InfluxDB line protocol to Telegraf via Unix socket |

**Metric format:**
```
robot_occupied,id=<team_id> occupied=<true|false> <nanosecond_timestamp>
```

### Classes

- **`TelegrafBridge`** — ROS 2 node that dynamically creates `num_teams` subscriptions at startup. Each callback formats the received `Bool` message as an InfluxDB line protocol string and hands it off to the sender.
- **`TelemetrySender`** — A standalone async I/O class using **Boost.Asio** that maintains a persistent Unix socket connection to `/tmp/sockets/telegraf.sock`. Writes are posted to a background `io_context` thread for non-blocking operation.

---

## Quick Start

### Prerequisites
- Docker & Docker Compose v2 installed
- Tailscale account with an auth key

### 1. Start the Tailscale VPN
```bash
cd RemotePC
docker compose -f tailscale-docker.yaml up -d
```

### 2. Configure the robot's IP
Edit `docker-compose.yaml`, line 43 — replace `ROBOT_IP` with the robot's Tailscale IP:
```yaml
command: ["-l", "tcp/0.0.0.0:7447", "-c", "/zenoh_config.json5", "-e", "tcp/100.x.y.z:7447"]
```

### 3. Launch all services
```bash
docker compose up -d
```

### 4. Access the interfaces

| Interface | URL |
|---|---|
| **Foxglove Studio** (3D Viz) | `http://localhost:8080` → connect to `ws://localhost:8765` |
| **Grafana** (Dashboards) | `http://localhost:3000` |
| **InfluxDB v3** (API) | `http://localhost:8181` |

---

## Configuration Reference

| File | What to Change | When |
|---|---|---|
| `docker-compose.yaml` | `ROBOT_IP` in zenoh-bridge command | When the robot's Tailscale IP changes |
| `docker-compose.yaml` | `INFLUXDB3_AUTH_TOKEN` | When rotating database credentials |
| `zenoh_config.json5` | `allow.subscribers` array | When you need to bridge additional/fewer topics |
| `telegraf.conf` | `bucket`, `token`, `organization` | When changing InfluxDB target or credentials |
| `tailscale-docker.yaml` | `TS_AUTHKEY` | When the Tailscale auth key expires |
| `ros2_ws/src/telemetry_data/config/params.yaml` | `num_teams` | When the number of competing teams changes |
