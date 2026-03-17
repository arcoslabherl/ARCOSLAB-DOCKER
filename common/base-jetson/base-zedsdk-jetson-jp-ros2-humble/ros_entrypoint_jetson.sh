#!/bin/bash
set -e

export ROS_DISTRO=humble

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Welcome information
echo "ROS2 Humble Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'DDS middleware: ' $RMW_IMPLEMENTATION
echo 'ROS 2 Workspaces:' $COLCON_PREFIX_PATH
echo 'ROS 2 Domain ID:' $ROS_DOMAIN_ID
echo ' * Note: Host and Docker image Domain ID must match to allow communication'
echo 'Local IPs:' $(hostname -I)
echo "---"  
exec "$@"