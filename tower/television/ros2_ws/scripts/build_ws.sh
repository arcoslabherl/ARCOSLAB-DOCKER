#!/usr/bin/env bash
set -e

# ARCoS-Lab TeleVision - ROS 2 Humble workspace build

source /opt/ros/humble/setup.bash

cd /ros2_ws

echo "Building TeleVision workspace..."

colcon build --symlink-install

echo "ROS 2 workspace build completed."
