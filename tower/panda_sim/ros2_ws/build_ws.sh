#!/usr/bin/env bash
set -e

# ARCoS-Lab - Franka Panda Simulator
# ROS 2 Humble workspace build

source /opt/ros/humble/setup.bash

cd /ros2_ws

# Ignore real robot drivers during simulation
touch src/panda-driver-topics/ros2_panda_driver_topics/COLCON_IGNORE
touch src/panda-driver-topics/ros2_panda_driver_gripper_sim/COLCON_IGNORE

echo "Building Franka Panda simulation workspace..."

colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "ROS 2 workspace build completed."