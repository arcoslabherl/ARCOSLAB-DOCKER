#!/usr/bin/env bash
set -eo pipefail

# ARCoS-Lab - Franka Panda Gripper Test
# ROS 2 Humble - Simulation only

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export ROS_LOG_DIR=/tmp/panda_ros_logs
mkdir -p "$ROS_LOG_DIR"

SERVICE="/sim_asimilar_01/panda_arm/gripper"
SERVICE_TYPE="panda_topics_msgs_srvs/srv/Pandagripper"

move_gripper() {
    local width="$1"

    echo "Moving gripper to ${width} m..."

    local output
    local status=0

    # A 0.04 m move at 0.02 m/s takes about 2 s
    output=$(timeout 20 ros2 service call \
        "$SERVICE" \
        "$SERVICE_TYPE" \
        "{cmd: 'move', width: ${width}, speed: 0.02, force: 0.0, epsilon_inner: 0.005, epsilon_outer: 0.005}") \
        || status=$?

    echo "$output"

    if (( status == 124 )); then
        echo "ERROR: No gripper response after 20 s."
        return 1
    fi

    if [[ "$output" != *"result='Success'"* ]]; then
        echo "ERROR: Gripper movement failed."
        return 1
    fi
}

echo "=================================="
echo "Franka Panda - Gripper Test"
echo "=================================="

# Check service availability
if ! timeout 15 ros2 service list | grep -Fxq "$SERVICE"; then
    echo "ERROR: Gripper service unavailable."
    exit 1
fi

# Open
move_gripper 0.04

# Close
move_gripper 0.0

echo
echo "Gripper test SUCCEEDED"