#!/usr/bin/env bash
set -e

# ARCoS-Lab - Franka Panda Joint Movement Test
# ROS 2 Humble

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ACTION="/sim_asimilar_01/jr_panda_test/action"
ACTION_TYPE="rbc_actions/action/JointReacher"

send_goal() {
    local target="$1"

    echo "Moving panda_j0 to ${target} rad..."

    local result

    result=$(ros2 action send_goal \
        "$ACTION" \
        "$ACTION_TYPE" \
        "{goal: [{joint: panda_j0, value: ${target}}], precision: 0.01, timeout: {sec: 10, nanosec: 0}}")

    echo "$result"

    if ! grep -q "Goal finished with status: SUCCEEDED" <<< "$result"; then
        echo "ERROR: Joint movement failed."
        return 1
    fi
}

echo "Starting Franka Panda J1 movement test..."

# Move J1
send_goal 0.1

# Return J1 to initial position
send_goal 0.0

echo "Franka Panda J1 test completed successfully."