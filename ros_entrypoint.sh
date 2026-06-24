#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /app/ETZ_ws/install/setup.bash ]; then
  source /app/ETZ_ws/install/setup.bash
fi

exec "$@"
