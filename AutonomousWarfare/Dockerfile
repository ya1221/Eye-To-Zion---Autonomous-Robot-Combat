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


WORKDIR /ros2_ws/src

# For lidar 
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /tmp/YDLidar-SDK && \
    mkdir -p /tmp/YDLidar-SDK/build && \
    cd /tmp/YDLidar-SDK/build && \
    cmake .. && \
    make && \
    make install && \
    rm -rf /tmp/YDLidar-SDK


# For IMU sensor (This can be install by rosdep))
RUN apt-get update && apt-get install -y \
    libi2c-dev \
    ros-humble-imu-filter-madgwick \
    && rm -rf /var/lib/apt/lists/*
    
# For foxglove bridge
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-foxglove-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-pip \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# Install the required Python hardware drivers
# --no-cache-dir prevents pip from saving downloaded packages, keeping the image small
RUN pip3 install --no-cache-dir \
    icm20948 \
    smbus2 \
    sparkfun-qwiic-icm20948

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