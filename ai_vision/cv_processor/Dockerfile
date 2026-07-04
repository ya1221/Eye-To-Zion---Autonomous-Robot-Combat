FROM ros:humble-ros-base

ENV PYTHONUNBUFFERED=1

# Install OpenCV and GStreamer plugins required for UDP/H264 decoding
# libturbojpeg: SIMD JPEG runtime, used via PyTurboJPEG below (Ubuntu names
# the runtime package "libturbojpeg" - no trailing 0 - unlike Debian's
# "libturbojpeg0"; both install libturbojpeg.so.0 at the same multiarch path)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    libturbojpeg \
    && rm -rf /var/lib/apt/lists/*

# Install required Python libraries
# PyTurboJPEG pinned to 1.8.3: 2.0+ hard-requires libjpeg-turbo 3.0+, but
# Ubuntu jammy's libturbojpeg package only ships 2.1.2.
RUN pip3 install --no-cache-dir numpy "PyTurboJPEG==1.8.3"

WORKDIR /app

# Copy application files (overridden by volume mount in compose for live edits)
COPY . /app/