FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1

# Install OpenCV and GStreamer plugins required for UDP/H264 decoding.
# libturbojpeg: SIMD JPEG runtime, used via PyTurboJPEG below (Ubuntu names
# the runtime package "libturbojpeg" - no trailing 0 - unlike Debian's
# "libturbojpeg0"; both install libturbojpeg.so.0 at the same multiarch path).
# Recommends are left enabled on purpose: the gstreamer plugin packages pull
# codec dependencies through recommends, and this is the combination that is
# known to decode the rpicam-vid H.264 stream on the Pi.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    libturbojpeg \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# PyTurboJPEG pinned to 1.8.3: 2.0+ hard-requires libjpeg-turbo 3.0+
RUN pip3 install --no-cache-dir numpy "PyTurboJPEG==1.8.3"

# ── Build the package ──
WORKDIR /ros2_ws
COPY . src/ai_vision/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select ai_vision

COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2", "launch", "ai_vision", "cv_processor.launch.py"]