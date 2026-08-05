FROM osrf/ros:humble-desktop

# System dependencies for OpenCV's GUI backend (X11 display via cv2.imshow)
# and the ArUco contrib module.
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libxcb-xinerama0 \
    fontconfig \
    fonts-dejavu \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir opencv-contrib-python numpy

# --- Build the FCP workspace ---
ENV ROS2_WS=/ros2_ws
WORKDIR ${ROS2_WS}

COPY ros2_ws/src ./src

RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Source both the underlying ROS distro and the workspace overlay before
# running anything, so the overhead_tracker package's console entry point
# (tracker_node) resolves correctly.
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource %s/install/setup.bash\nexec "$@"\n' "${ROS2_WS}" > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

CMD ["ros2", "launch", "overhead_tracker", "tracker_launch.py"]
