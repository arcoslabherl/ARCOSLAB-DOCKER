#!/usr/bin/env bash

# Navigate to ROS 2 workspace
cd /ros2_ws || return 1

# Load ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install \
  --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release || return 1

# Load compiled packages
source /ros2_ws/install/setup.bash

echo "ROS 2 workspace ready."
