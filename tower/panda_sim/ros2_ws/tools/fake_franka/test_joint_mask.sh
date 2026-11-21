#!/usr/bin/env bash
# GELLO joint mask test: starts the driver on the fake libfranka and the fake
# GELLO pointed at it with joints:=7 range:=10 (every joint swinging 0.3 rad),
# runs test_joint_mask.py, and stops both.
#   test_joint_mask.sh [--patched]     (--patched: the driver with our patch)
# Run inside the container, with nothing else running on /asimilar or /gello.
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/panda_ros_logs
mkdir -p "$ROS_LOG_DIR"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT=/tmp/fake_franka_test
mkdir -p "$OUT"
rm -f /tmp/fake_franka_reflex /tmp/fake_franka_summary

# Each in its own process group, so cleanup stops the whole tree
setsid "$HERE/run_driver.sh" "$@" > "$OUT/driver.out" 2>&1 &
DRIVER=$!
# The fake GELLO swings 0.3 rad (17 deg), past the 10 degree range
FAKE_GELLO_CONFIG=/tmp/gello_fake_mask.yaml
sed 's/^\(\s*fake_amplitude:\).*/\1 0.3/' \
    /ros2_ws/gello_teleop/config/gello_panda.yaml > "$FAKE_GELLO_CONFIG"
# As 'make gello ROBOT=real GRIPPER=off JOINTS=7 RANGE=10'
setsid ros2 launch gello_teleop gello.launch.py backend:=fake robot:=real \
    config:="$FAKE_GELLO_CONFIG" gripper:=off joints:=7 range:=10 \
    > "$OUT/gello.out" 2>&1 &
GELLO=$!

cleanup() {
    kill -INT -- -"$GELLO" -"$DRIVER" 2> /dev/null || true
    sleep 2
    kill -KILL -- -"$GELLO" -"$DRIVER" 2> /dev/null || true
}
trap cleanup EXIT

sleep 5
set +e
python3 "$HERE/test_joint_mask.py"
RESULT=$?
grep -a -m1 "Joint mask" "$OUT/gello.out"
echo "Logs: $OUT/driver.out, $OUT/gello.out"
grep -a -E "VIOLATION|STALL" "$OUT/driver.out" | head -5
exit $RESULT
