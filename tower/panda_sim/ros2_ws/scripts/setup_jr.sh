#!/usr/bin/env bash
set -e

# ARCoS-Lab - Franka Panda Joint Reacher Setup
# ROS 2 Humble

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

NAMESPACE="/sim_asimilar_01"
JR_NAME="jr_panda_test"
JR_NODE="${NAMESPACE}/${JR_NAME}"

echo "Initializing Franka Panda Joint Reacher..."

# Create Joint Reacher. Ask RBC itself whether it already exists:
# 'ros2 node list' can still show nodes from a previous run.
if ! RESULT=$(timeout 15 ros2 service call \
    "${NAMESPACE}/rbc_l2_ctrl" \
    rbc_msgs_srvs/srv/RbcL2Ctrl \
    "{cmd: 'create_re', data: 'JR', name: '${JR_NAME}'}"); then
    echo "ERROR: RBC did not respond."
    exit 1
fi

if grep -q "Already exists" <<< "$RESULT"; then
    echo "Joint Reacher already running."
    exit 0
fi

if ! grep -q "result='Ok'" <<< "$RESULT"; then
    echo "ERROR: Joint Reacher creation failed."
    echo "$RESULT"
    exit 1
fi

# Wait until the new node is available
JR_READY=false

for attempt in $(seq 1 30); do
    if timeout 5 ros2 param get "$JR_NODE" weight >/dev/null 2>&1; then
        JR_READY=true
        break
    fi

    sleep 1
done

if [ "$JR_READY" != "true" ]; then
    echo "ERROR: Joint Reacher did not become available."
    exit 1
fi

# Configure first Panda joint
ros2 param set "$JR_NODE" joints "['panda_j0']"

# Conservative joint velocity limit
ros2 param set "$JR_NODE" joint_speed_limits "[0.1]"

# Enable controller contribution
ros2 param set "$JR_NODE" weight 1.0

echo "Joint Reacher initialized successfully."