#!/usr/bin/env bash
set -e

# ARCoS-Lab - GELLO teleoperation
# ROS 2 Humble
#
# Usage: start_gello.sh [dynamixel|fake] [serial port]

BACKEND="${1:-dynamixel}"
PORT="${2:-/dev/ttyUSB0}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

if [[ "$BACKEND" == "dynamixel" && ! -e "$PORT" ]]; then
    echo "ERROR: GELLO not found at ${PORT} inside the container."
    echo "Plug in the GELLO, then restart the container:"
    echo "    make stop && make run GELLO_PORT=${PORT}"
    echo "Or test without hardware:  make gello-fake"
    exit 1
fi

echo "Starting GELLO (${BACKEND} backend)..."

ros2 launch gello_teleop gello.launch.py \
    backend:="$BACKEND" \
    port:="$PORT"
