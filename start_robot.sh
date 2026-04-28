#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

echo "Cleaning up Docker garbage (Dangling Images)"
# Deletes only images that are not used and have no name
docker image prune -f

echo "Starting Robot System with Build"
# Rebuilds updated containers and starts the system
docker compose up -d --build

echo "Showing real-time logs (Ctrl+C to stop viewing)"
docker compose logs -f
