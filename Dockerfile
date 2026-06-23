# Base image with ROS2 Humble
FROM osrf/ros:humble-desktop

# Set working directory
WORKDIR /app

# Install system dependencies and GStreamer for video
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    fontconfig \
    ffmpeg \
    nano \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Navigation, Localization, Simulation, and Control dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-nav2-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-ackermann-steering-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*
    
# Install Python libraries (Added py_trees for Student C)
RUN pip3 install --no-cache-dir \
    redis \
    py_trees \
    setuptools==58.2.0

RUN pip3 install eclipse-zenoh

# Set environment variable
ENV PYTHONUNBUFFERED=1

# Default command
CMD ["bash"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc