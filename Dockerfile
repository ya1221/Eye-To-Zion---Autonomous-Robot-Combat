# בסיס רזה של ROS 2 Humble
FROM ros:humble-ros-base-jammy

# התקנת רק את מה שהלוגיקה של הרובוט צריכה (בלי OpenCV אם לא חובה)
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# הגדרת ה-Entrypoint כדי ש-ROS 2 יעבוד אוטומטית
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# הרצת הקוד של הלוגיקה
CMD ["python3", "robot_listener_ARUCO.py"]
