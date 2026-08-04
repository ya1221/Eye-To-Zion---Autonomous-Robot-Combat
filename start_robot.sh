#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "Cleaning up Docker garbage (Dangling Images)"
# Deletes only images that are not used and have no name
docker image prune -f

# cv_processor reads H.264 over UDP rather than opening /dev/video0 itself,
# so the camera has to be streaming on the host before the containers come up.
echo "Starting camera stream"
pkill rpicam-vid || true  # free the camera if a previous run left it held
sleep 1
rpicam-vid -t 0 --width 640 --height 320 --framerate 30 --codec h264 \
    --profile baseline --inline -o udp://127.0.0.1:5000 &
CAMERA_PID=$!

# Give the stream a moment to initialize before the decoder tries to attach
sleep 2
echo "Camera stream running (PID: ${CAMERA_PID})"

echo "Starting Robot System with Build"
# Rebuilds updated containers and starts the system
docker compose up -d --build

echo "Showing real-time logs (Ctrl+C to stop viewing)"
docker compose logs -f
