FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "robot_listener_ARUCO.py"]
