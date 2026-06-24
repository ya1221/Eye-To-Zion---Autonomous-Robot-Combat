FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1

# Install system dependencies and build tools required for lapx/C++ on ARM
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    build-essential \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install numpy first to satisfy lapx compilation requirements
RUN pip3 install --no-cache-dir "numpy<2.0.0"

# Install YOLO, NCNN runtime, and tracking dependencies
RUN pip3 install --no-cache-dir ultralytics ncnn lapx

WORKDIR /app

# Copy application files (overridden by volume mount in compose for live edits)
COPY . /app/