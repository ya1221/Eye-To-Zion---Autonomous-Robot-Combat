#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Kill any existing camera stream to free up the resource
pkill rpicam-vid || true
sleep 1

# 2. Start the camera stream in the background
rpicam-vid -t 0 --width 640 --height 320 --framerate 30 --codec h264 --profile baseline --inline -o udp://127.0.0.1:5000 &
CAMERA_PID=$!

# 2. Give the stream a moment to initialize
sleep 2

# 3. Start the Docker containers
cd "$SCRIPT_DIR"
docker compose up -d

echo "Camera stream running (PID: ${CAMERA_PID}). Docker containers started."
