# FROM ros:humble

# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#       ros-humble-foxglove-bridge && \
#     rm -rf /var/lib/apt/lists/*

# COPY topic_advertiser.py /topic_advertiser.py
# COPY fastrtps_no_shm.xml /fastrtps_no_shm.xml

# ENV FASTRTPS_DEFAULT_PROFILES_FILE=/fastrtps_no_shm.xml

# CMD ["/bin/bash", "-c", \
#      "source /opt/ros/humble/setup.bash && \
#       python3 /topic_advertiser.py & \
#       sleep 3 && \
#       ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"]


####################
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    # Build tools for C++ and Python
    build-essential \
    cmake \
    python3-colcon-common-extensions \
     # Extra tools
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    && rm -rf /var/lib/apt/lists/*


WORKDIR /ros2_ws/src


# For foxglove bridge
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-foxglove-msgs \
    ros-humble-std-msgs \
    && rm -rf /var/lib/apt/lists/*  

WORKDIR /ros2_ws

# Copy the 'src' folder from your computer into the container
COPY ros2_ws/src ./src

# Update rosdep and install dependencies
# Fix for rosdep running as root and apt timeout/buffer issues
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    rosdep update --rosdistro humble && \
    rosdep install -i --from-path src --rosdistro humble -y \
    --default-yes \
    --as-root "apt:false" && \
    rm -rf /var/lib/apt/lists/*



# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]