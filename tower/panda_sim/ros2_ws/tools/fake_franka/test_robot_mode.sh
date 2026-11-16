#!/usr/bin/env bash
# Tests 'make robot-enable' / 'make robot-disable' (tools/real_robot/
# robot_mode.py) against the real driver on the fake libfranka, with the fake
# GELLO configured for the real robot:
#   1. enable cancelled when not confirmed     4. disable refused while the
#   2. enable works                               arm moves
#   3. disable refused while GELLO controls    5. disable works with the arm still
# and no limit violations in the driver.
#   test_robot_mode.sh [--patched]   (--patched: the driver with our patch)
# Run inside the container, with nothing else running on /asimilar or /gello.

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/panda_ros_logs
mkdir -p "$ROS_LOG_DIR"

TOOLS=/ros2_ws/tools
MODE="bash $TOOLS/real_robot/robot_mode.sh"
OUT=/tmp/fake_franka_test
mkdir -p "$OUT"
POSE="0.0 1.57 0.71"       # J5..J7 of the poses below (the sim home)

# Each in its own process group, so cleanup stops the whole tree
setsid "$TOOLS/fake_franka/run_driver.sh" "$@" > "$OUT/driver.out" 2>&1 &
DRIVER=$!
setsid ros2 launch gello_teleop gello.launch.py backend:=fake robot:=real \
    > "$OUT/gello.out" 2>&1 &
GELLO=$!
MOVER=""

cleanup() {
    [ -n "$MOVER" ] && kill -- -"$MOVER" 2> /dev/null
    kill -INT -- -"$GELLO" -"$DRIVER" 2> /dev/null
    sleep 2
    kill -KILL -- -"$GELLO" -"$DRIVER" 2> /dev/null
}
trap cleanup EXIT

FAILED=0
check() {   # check <name> <output> <expected text>
    if grep -q -- "$3" <<< "$2"; then
        echo "  PASS  $1"
    else
        echo "  FAIL  $1: expected '$3', got:"
        sed 's/^/        /' <<< "$2"
        FAILED=1
    fi
}

gello() {
    timeout 10 ros2 service call /gello/enable std_srvs/srv/SetBool "{data: $1}"
}

sleep 6

echo "1. Enable, not confirmed"
check "cancelled" "$(echo no | $MODE enable)" "NOT DONE: cancelled"

echo "2. Enable"
check "driver enabled" "$($MODE enable --yes)" "Driver: Success, enabled=\[True"

echo "3. Disable while the GELLO controls the arm"
for _ in $(seq 15); do
    gello true | grep -q "success=True" && break
    sleep 1
done
check "refused" "$($MODE disable)" "NOT DONE: the GELLO controls the arm"

echo "4. Disable while the arm moves"
gello false > /dev/null
# J1 back and forth over 5 rad: more than 6 s per move at 0.8 rad/s, so the
# arm keeps moving
setsid bash -c "for k in 1 2 3; do
    python3 $TOOLS/fake_franka/posref.py '-2.5 0.0 0.0 -1.57 $POSE' 4
    python3 $TOOLS/fake_franka/posref.py '2.5 0.0 0.0 -1.57 $POSE' 4
done" > /dev/null 2>&1 &
MOVER=$!
sleep 5
check "refused" "$($MODE disable)" "NOT DONE: the arm moves"
kill -- -"$MOVER" 2> /dev/null
MOVER=""

echo "5. Disable with the arm still"
sleep 8
check "driver disabled" "$($MODE disable)" "Driver: Success, enabled=\[False"

VIOLATIONS=$(grep -acE "VIOLATION|STALL" "$OUT/driver.out")
check "no limit violations in the driver" "$VIOLATIONS" "^0$"

echo "Logs: $OUT/driver.out, $OUT/gello.out"
if [ "$FAILED" -eq 0 ]; then
    echo "RESULT: PASS"
else
    echo "RESULT: FAIL"
fi
exit $FAILED
