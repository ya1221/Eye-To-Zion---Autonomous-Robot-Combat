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


# Set the internal container workspace(root)
WORKDIR /ros2_ws

# Copy the 'src' folder from your computer into the container
COPY ros2_ws/src ./src

# Update rosdep and install dependencies
RUN apt-get update && rosdep update && \
    rosdep install -i --from-path src --rosdistro humble -y

    
# Environment setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]