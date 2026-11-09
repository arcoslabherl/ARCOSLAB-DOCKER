#!/usr/bin/env bash
# Reflex-during-teleop test: starts the driver on the fake libfranka and the
# fake GELLO pointed at it, runs test_reflex.py, and stops both.
#   test_reflex.sh [--patched]     (--patched: the driver with our patch)
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
sed "s#/sim_asimilar_01/#/asimilar/#g; s#gripper_grasp: false#gripper_grasp: true#" \
    /ros2_ws/gello_teleop/config/gello_panda.yaml > "$OUT/gello_asimilar.yaml"
setsid ros2 launch gello_teleop gello.launch.py backend:=fake \
    config:="$OUT/gello_asimilar.yaml" > "$OUT/gello.out" 2>&1 &
GELLO=$!

cleanup() {
    kill -INT -- -"$GELLO" -"$DRIVER" 2> /dev/null || true
    sleep 2
    kill -KILL -- -"$GELLO" -"$DRIVER" 2> /dev/null || true
}
trap cleanup EXIT

sleep 5
set +e
python3 "$HERE/test_reflex.py"
RESULT=$?
echo "Logs: $OUT/driver.out, $OUT/gello.out"
grep -a -E "VIOLATION|STALL" "$OUT/driver.out" | head -5
exit $RESULT
