# Use the base image (no GUI) to save space and CPU on the Pi
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    # Build tools for C++ and Python
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    # ros2_control framework
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ackermann-steering-controller \
    # Navigation 2 (The "Brain")
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # Extra tools
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# 2. Add Raspberry Pi APT repository (Required for Pi 5 libcamera)
# RUN curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key | gpg --dearmor -o /usr/share/keyrings/raspberrypi-archive-keyring.gpg \
#     && echo "deb [signed-by=/usr/share/keyrings/raspberrypi-archive-keyring.gpg] http://archive.raspberrypi.com/debian/ jammy main" > /etc/apt/sources.list.d/raspberrypi.list

# # 3. Install libcamera and ROS 2 camera drivers
# RUN apt-get update && apt-get install -y \
#     libcamera-dev \
#     libcamera-apps-lite \
#     ros-humble-camera-ros \
#     ros-humble-image-transport-plugins \
#     && rm -rf /var/lib/apt/lists/*
    
# Build and Install YDLidar-SDK (Required for the driver to work)

WORKDIR /ros2_ws/src

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/YDLidar-SDK && \
    mkdir -p /tmp/YDLidar-SDK/build && \
    cd /tmp/YDLidar-SDK/build && \
    cmake .. && \
    make && \
    make install && \
    rm -rf /tmp/YDLidar-SDK

RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*
# Set the internal container workspace(root)
WORKDIR /ros2_ws

# Copy the 'src' folder from your computer into the container
COPY ros2_ws/src ./src

# Update rosdep and install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    rm -rf /var/lib/apt/lists/*

# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]