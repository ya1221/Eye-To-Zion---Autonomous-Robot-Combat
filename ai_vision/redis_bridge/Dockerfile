FROM osrf/ros:humble-desktop
ENV PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir redis
COPY redis_bridge.py /app/
WORKDIR /app
