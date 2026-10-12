#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

exec ros2 launch ouster_ros driver.launch.py \
    params_file:=/ouster_config/driver_params.yaml \
    viz:=true