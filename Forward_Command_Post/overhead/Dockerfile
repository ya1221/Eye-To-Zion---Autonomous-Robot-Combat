# Start from ROS 2 Humble Base Image
FROM ros:humble-ros-base-jammy

# Install system dependencies for OpenCV GUI and pip
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install Python libraries (Removed redis, kept opencv and numpy)
RUN pip3 install opencv-contrib-python numpy

# Create an entrypoint script to source ROS 2 variables before running Python
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Use the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Start the container
CMD ["python3", "overhead_tracker.py"]
