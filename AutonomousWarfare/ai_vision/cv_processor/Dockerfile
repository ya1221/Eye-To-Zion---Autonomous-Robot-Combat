FROM ros:humble-ros-base
ENV PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y \
    python3-opencv \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    && rm -rf /var/lib/apt/lists/*
COPY cv_processor.py /app/
WORKDIR /app
