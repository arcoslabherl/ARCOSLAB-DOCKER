#!/usr/bin/env bash
set -eo pipefail

# ARCoS-Lab - GELLO pipeline test
# ROS 2 Humble
#
# Requires, in other terminals:
#   make sim-teleop
#   make gello-fake   (with the real device, first match the Panda's pose)
#
# Checks: GELLO data -> enable -> the Panda follows the targets -> disable.
# The simulated Panda moves during this test.

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
    || fail "No Panda state. Is 'make sim-teleop' running?"

call /gello/enable std_srvs/srv/SetBool "{data: false}" >/dev/null \
    || fail "GELLO calibrator not responding."

echo "Enabling..."
ENABLED=false
for attempt in $(seq 1 10); do
    output=$(call /gello/enable std_srvs/srv/SetBool "{data: true}") || true
    if grep -q "success=True" <<< "$output"; then
        ENABLED=true
        break
    fi
    # The Panda may still be going home
    sleep 1
done
[[ "$ENABLED" == true ]] || fail "Enable failed: ${output##*response:}"

echo "Checking target joint states..."
target=$(timeout 10 ros2 topic echo --once /gello/target_joint_states) \
    || fail "No data on /gello/target_joint_states."

for joint in panda_j0 panda_j1 panda_j2 panda_j3 panda_j4 panda_j5 panda_j6; do
    grep -q "$joint" <<< "$target" || fail "Target is missing ${joint}."
done

enabled=$(timeout 5 ros2 topic echo --once /gello/enabled) || true
grep -q "data: true" <<< "$enabled" || fail "/gello/enabled is not true."

echo "Checking that the Panda follows the targets..."
python3 - <<'EOF' || fail "The Panda is not following the targets."
import sys
import time

import rclpy
from impedance_control_msgs.msg import State
from sensor_msgs.msg import JointState

MAX_ERROR = 0.1    # rad
DURATION = 6.0     # s

rclpy.init()
node = rclpy.create_node("test_gello_tracking")
data = {"target": None, "state": None, "first_target": None, "spread": 0.0}


def on_target(msg):
    target = list(msg.position)
    if data["first_target"] is None:
        data["first_target"] = target
    data["spread"] = max(
        data["spread"],
        max(abs(a - b) for a, b in zip(target, data["first_target"])))
    data["target"] = target


def on_state(msg):
    data["state"] = list(msg.position)


node.create_subscription(JointState, "/gello/target_joint_states", on_target, 10)
node.create_subscription(State, "/sim_asimilar_01/panda_arm/state", on_state, 10)

end = time.monotonic() + DURATION
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)

if data["target"] is None or data["state"] is None:
    print("  No target or Panda state received.")
    sys.exit(1)

error = max(abs(a - b) for a, b in zip(data["target"], data["state"]))
print(f"  Target moved {data['spread']:.3f} rad, tracking error {error:.3f} rad")

node.destroy_node()
rclpy.try_shutdown()
sys.exit(0 if error < MAX_ERROR else 1)
EOF

echo "Disabling..."
call /gello/enable std_srvs/srv/SetBool "{data: false}" >/dev/null \
    || fail "Disable failed."

echo
echo "GELLO pipeline test SUCCEEDED"
