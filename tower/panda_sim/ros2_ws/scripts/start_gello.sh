#!/usr/bin/env bash
set -e

# ARCoS-Lab - GELLO teleoperation
# ROS 2 Humble
#
# Usage: start_gello.sh [dynamixel|fake] [serial port] [sim|real] [on|off]
#   on|off: the gripper (off: the calibrator leaves it alone)

BACKEND="${1:-dynamixel}"
PORT="${2:-auto}"   # auto: gello_find_port looks for the GELLO
ROBOT="${3:-sim}"
GRIPPER="${4:-on}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Refuse to start a second GELLO next to a running one
if pgrep -f "gello_teleop/lib/gello_teleop/gello_" >/dev/null; then
    echo "ERROR: GELLO is already running."
    echo "Stop it with Ctrl+C in its terminal, or restart the container:"
    echo "    make stop && make run"
    exit 1
fi

# Find the GELLO, or check the given port. It may have been plugged in
# after 'make run' (the host's /dev is at /hostdev).
if [[ "$BACKEND" == "dynamixel" ]]; then
    if ! PORT=$(ros2 run gello_teleop gello_find_port "$PORT"); then
        echo "Or test without hardware:  make gello-fake"
        exit 1
    fi
fi

# Let the Makefile stop this session if its terminal is closed
if [[ -n "${SESSION_ID:-}" ]]; then
    ps -o pgid= -p $$ | tr -d ' ' > "/tmp/ros2_session.${SESSION_ID}"
fi

echo "Starting GELLO (${BACKEND} backend, ${ROBOT} Panda, gripper ${GRIPPER})..."

ros2 launch gello_teleop gello.launch.py \
    backend:="$BACKEND" \
    port:="$PORT" \
    robot:="$ROBOT" \
    gripper:="$GRIPPER"
