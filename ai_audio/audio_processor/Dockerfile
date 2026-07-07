FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1

# libportaudio2/alsa-utils: I2S mic access (same as dataset_collector)
# build-essential/libsndfile1: librosa's STFT/mel path (same as preprocessing)
RUN apt-get update && apt-get install -y \
    python3-pip \
    libportaudio2 \
    alsa-utils \
    build-essential \
    libsndfile1 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir sounddevice librosa onnxruntime numpy

WORKDIR /app

# Copy application files (overridden by volume mount in compose for live edits)
COPY . /app/
