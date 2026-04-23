FROM osrf/ros:humble-desktop
ENV PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y python3-opencv && rm -rf /var/lib/apt/lists/*
COPY cv_processor.py /app/
WORKDIR /app
