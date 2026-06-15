# Base image with ROS2 Humble
FROM osrf/ros:humble-desktop

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    fontconfig \
    ffmpeg \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-nav2-msgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup
    
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