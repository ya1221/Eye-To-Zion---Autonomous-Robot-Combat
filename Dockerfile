# Base image with ROS2 Humble
FROM osrf/ros:humble-desktop

# Set working directory
WORKDIR /app

# Install system dependencies for OpenCV and Python
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    fontconfig \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install Python libraries
# Using --no-cache-dir to keep the image small
RUN pip3 install --no-cache-dir \
    ultralytics \
    redis \
    setuptools==58.2.0

# Set environment variable to avoid Python buffering logs
ENV PYTHONUNBUFFERED=1

# Default command
CMD ["bash"]
