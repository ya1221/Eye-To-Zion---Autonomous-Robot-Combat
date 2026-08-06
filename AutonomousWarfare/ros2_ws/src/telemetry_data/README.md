# telemetry_data

## Purpose
Bridge ROS 2 topics to the Telegraf/InfluxDB dashboard.

## Logic
Subscribes to various internal robot states and formats them into Line Protocol, sending them over a Unix socket to a local Telegraf agent.

## Data Flow
- **Input:** `/robot_status`, `/audio/impact_alert`, Nav2 status.
- **Output:** Unix Domain Socket stream to Telegraf.
