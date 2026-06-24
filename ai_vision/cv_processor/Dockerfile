FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1

# Install OpenCV and GStreamer plugins required for UDP/H264 decoding
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*

# Install required Python libraries
RUN pip3 install --no-cache-dir numpy

WORKDIR /app

# Copy application files (overridden by volume mount in compose for live edits)
COPY . /app/