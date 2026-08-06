# robot_stats

## Purpose
Track and manage the robot's internal state (health, ammunition).

## Logic
Monitors impact alerts to decrease health, and monitors shooting commands to decrease ammunition. Uses ROS 2 parameters to store current state.

## Data Flow
- **Input:** `/audio/impact_alert`, `/shooting_cmd`.
- **Output:** Publishes state updates to `/robot_status` or telemetry.
