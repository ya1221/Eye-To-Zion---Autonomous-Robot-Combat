FROM osrf/ros:humble-desktop
ENV PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y python3-pip && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir "numpy<2.0.0" ultralytics ncnn
COPY ai_inference.py /app/
COPY best_ncnn_model /app/best_ncnn_model/
WORKDIR /app
