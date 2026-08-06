# foxglove

## Purpose
Provide a WebSockets bridge for remote visualization.

## Logic
Translates ROS 2 topics into Foxglove Studio's protocol, allowing the user to view the robot's live state on a remote PC.

## Data Flow
- **Input:** All ROS 2 topics.
- **Output:** WebSocket stream on port 8765.
