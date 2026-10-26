#!/usr/bin/env bash
set -eo pipefail

# ARCoS-Lab - Franka Panda J1-J7 Movement Tests
# ROS 2 Humble - Simulation only

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash


# Writable ROS 2 logging directory
export ROS_LOG_DIR=/tmp/panda_ros_logs
mkdir -p "$ROS_LOG_DIR"

NS="/sim_asimilar_01"
ACTION_TYPE="rbc_actions/action/JointReacher"

# Initial positions from the simulated Panda
HOME=(0.0 0.0 0.0 -1.57 0.0 1.57 0.71)

# Small test movements
TARGET=(0.1 0.1 0.1 -1.47 0.1 1.47 0.61)

# Reuse controllers already created for J1 and J2
JR_NAMES=(
    jr_panda_test
    jr_panda_j1
    jr_panda_j2
    jr_panda_j3
    jr_panda_j4
    jr_panda_j5
    jr_panda_j6
)

create_controller() {
    local index="$1"
    local jr_name="${JR_NAMES[$index]}"
    local node="${NS}/${jr_name}"
    local joint="panda_j${index}"

    echo "Configuring J$((index + 1)): ${joint}"

    if ! ros2 node list | grep -Fxq "$node"; then
        ros2 service call \
            "${NS}/rbc_l2_ctrl" \
            rbc_msgs_srvs/srv/RbcL2Ctrl \
            "{cmd: 'create_re', data: 'JR', name: '${jr_name}'}"

        local ready=false

        for attempt in $(seq 1 30); do
            if ros2 param get "$node" weight >/dev/null 2>&1; then
                ready=true
                break
            fi
            sleep 1
        done

        if [[ "$ready" != true ]]; then
            echo "ERROR: Controller ${jr_name} unavailable."
            return 1
        fi

        ros2 param set "$node" joints "['${joint}']"
    fi

    # Confirm that an existing controller has the correct joint
    local configured
    configured=$(ros2 param get "$node" joints)

    if [[ "$configured" != *"['${joint}']"* ]]; then
        echo "ERROR: ${jr_name} is not configured for ${joint}."
        echo "$configured"
        return 1
    fi

    ros2 param set "$node" joint_speed_limits "[0.1]"
    ros2 param set "$node" weight 1.0
}

send_goal() {
    local index="$1"
    local target="$2"

    local joint="panda_j${index}"
    local action="${NS}/${JR_NAMES[$index]}/action"

    echo "Moving ${joint} to ${target} rad..."

    local output

    if ! output=$(ros2 action send_goal \
        "$action" \
        "$ACTION_TYPE" \
        "{goal: [{joint: ${joint}, value: ${target}}], precision: 0.01, timeout: {sec: 15, nanosec: 0}}" 2>&1); then

        echo "$output"
        echo "ERROR: ROS 2 action command failed."
        return 1
    fi

    echo "$output"

    if [[ "$output" != *"Goal finished with status: SUCCEEDED"* ]]; then
        echo "ERROR: Movement did not succeed."
        return 1
    fi
}

echo "=================================="
echo "Franka Panda - Joint Tests"
echo "=================================="

if ! ros2 node list | grep -Fxq "${NS}/panda_arm"; then
    echo "ERROR: Panda simulator is not running."
    exit 1
fi

# Configure all controllers
for index in {0..6}; do
    create_controller "$index"
done

echo "All controllers configured."

# Execute tests sequentially
for index in {0..6}; do
    echo
    echo "=================================="
    echo "Testing J$((index + 1))"
    echo "=================================="

    send_goal "$index" "${TARGET[$index]}"
    send_goal "$index" "${HOME[$index]}"

    echo "J$((index + 1)): PASSED"
done

echo
echo "=================================="
echo "All 7 joint tests SUCCEEDED"
echo "=================================="