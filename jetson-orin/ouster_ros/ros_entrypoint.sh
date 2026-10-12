#!/bin/bash

export ROS_DISTRO=humble

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# source env if file exists
ros_ws_setup="/ros2_ws/install/setup.bash"
if [ -f $ros_ws_setup ]; then \
    echo "sourcing   $ros_ws_setup"
    source "$ros_ws_setup"
fi


# Welcome information
echo "Ouster OS1-128 ROS2 Docker Image"
echo "---------------------"
echo 'ROS distro: ' $ROS_DISTRO
echo 'DDS middleware: ' $RMW_IMPLEMENTATION
echo 'ROS 2 Workspaces:' $COLCON_PREFIX_PATH
echo 'ROS 2 Domain ID:' $ROS_DOMAIN_ID
echo ' * Note: Host and Docker image Domain ID must match to allow communication'
echo 'Local IPs:' $(hostname -I)
exec "$@"