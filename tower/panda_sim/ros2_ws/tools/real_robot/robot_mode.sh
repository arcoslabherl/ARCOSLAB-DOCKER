#!/usr/bin/env bash
# Enables or disables the lab driver: see robot_mode.py.
#   robot_mode.sh enable|disable [--ns /asimilar/panda_arm] [--yes]
# Run inside the container (real robot, or the fake libfranka).
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

exec python3 "$(dirname "${BASH_SOURCE[0]}")/robot_mode.py" "$@"
