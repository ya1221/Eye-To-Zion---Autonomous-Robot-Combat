FROM python:3.10-slim

# Install system dependencies for OpenCV GUI
RUN apt-get update && apt-get install -y \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install Python libraries (contrib required for ArUco)
RUN pip install opencv-contrib-python redis numpy

# Start the container
CMD ["python", "overhead_tracker.py"]
