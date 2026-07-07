#!/bin/bash
# Source the ROS 2 environment for EVERY session
source "/opt/ros/humble/setup.bash"

# Auto-build the workspace if it hasn't been built yet
if [ ! -f /ros2_ws/install/setup.bash ]; then
    echo "=== First run detected: Building ROS 2 workspace... ==="
    cd /ros2_ws && colcon build --symlink-install
    echo "=== Build complete! ==="
fi

exec "$@" 