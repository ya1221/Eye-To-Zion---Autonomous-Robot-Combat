FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1

# Install system dependencies and build tools required for lapx/C++ on ARM
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    build-essential \
    python3-dev \
    ros-humble-foxglove-msgs \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir "numpy<2.0.0"
RUN pip3 install --no-cache-dir ultralytics ncnn lapx

# ── Build the package ──
WORKDIR /ros2_ws
COPY . src/ai_vision/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ai_vision

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "ai_vision", "ai_inference.launch.py"]