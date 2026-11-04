#!/usr/bin/env bash
set -eo pipefail

# ARCoS-Lab - GELLO pipeline test
# ROS 2 Humble
#
# Requires, in other terminals:
#   make sim
#   make gello-fake   (or make gello with the real device)
#
# Checks: GELLO data -> recenter -> enable -> target joint states -> disable.
# Targets are not sent to the robot yet, so the Panda does not move.

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

export ROS_LOG_DIR=/tmp/panda_ros_logs
mkdir -p "$ROS_LOG_DIR"

# ros2 service call waits forever for a missing service: always bound it
call() {
    timeout 10 ros2 service call "$@"
}

fail() {
    echo "ERROR: $1"
    timeout 5 ros2 service call /gello/enable std_srvs/srv/SetBool \
        "{data: false}" >/dev/null 2>&1 || true
    exit 1
}

echo "=================================="
echo "GELLO Pipeline Test"
echo "=================================="

echo "Checking GELLO data..."
timeout 10 ros2 topic echo --once /gello/joint_states >/dev/null \
    || fail "No data on /gello/joint_states. Is 'make gello' running?"

echo "Checking Panda state..."
timeout 10 ros2 topic echo --once /sim_asimilar_01/panda_arm/state >/dev/null \
    || fail "No Panda state. Is 'make sim' running?"

# Recalibration requires control to be disabled
call /gello/enable std_srvs/srv/SetBool "{data: false}" >/dev/null \
    || fail "GELLO calibrator not responding."

echo "Recentering..."
RECENTERED=false
for attempt in $(seq 1 10); do
    output=$(call /gello/recenter std_srvs/srv/Trigger) || true
    if grep -q "success=True" <<< "$output"; then
        RECENTERED=true
        break
    fi
    # Panda may still be settling
    sleep 1
done
[[ "$RECENTERED" == true ]] || fail "Recenter failed: ${output##*response:}"

echo "Enabling..."
output=$(call /gello/enable std_srvs/srv/SetBool "{data: true}") || true
grep -q "success=True" <<< "$output" || fail "Enable failed: ${output##*response:}"

echo "Checking target joint states..."
target=$(timeout 10 ros2 topic echo --once /gello/target_joint_states) \
    || fail "No data on /gello/target_joint_states."

for joint in panda_j0 panda_j1 panda_j2 panda_j3 panda_j4 panda_j5 panda_j6; do
    grep -q "$joint" <<< "$target" || fail "Target is missing ${joint}."
done

enabled=$(timeout 5 ros2 topic echo --once /gello/enabled) || true
grep -q "data: true" <<< "$enabled" || fail "/gello/enabled is not true."

echo "Disabling..."
call /gello/enable std_srvs/srv/SetBool "{data: false}" >/dev/null \
    || fail "Disable failed."

echo
echo "GELLO pipeline test SUCCEEDED"
