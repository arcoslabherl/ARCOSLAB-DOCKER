#!/usr/bin/env bash
# Tests 'make robot-nudge' on the fake libfranka: real_robot/nudge (libfranka
# alone) and real_robot/driver_nudge.py (through the real driver, with the fake
# GELLO configured for the real robot). Checks the refusals, that the joint
# goes and comes back, and that no command breaks the Panda's limits.
#   test_nudge.sh [--patched]   (--patched: the driver with our patch)
# Run inside the container, with nothing else running on /asimilar or /gello.

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export ROS_LOG_DIR=/tmp/panda_ros_logs
mkdir -p "$ROS_LOG_DIR"

TOOLS=/ros2_ws/tools
NUDGE=/ros2_ws/build/_fake_franka/nudge
DRIVER_NUDGE="bash $TOOLS/real_robot/nudge.sh driver"
OUT=/tmp/fake_franka_test
mkdir -p "$OUT"
export NUDGE_LOG_DIR="$OUT/nudge_logs"   # keep tools/data for the real robot
[ -x "$NUDGE" ] || { echo "Build it first: make fake-driver-build"; exit 1; }

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

echo "libfranka alone"
check "more than 10 degrees refused" "$($NUDGE fake 7 12 --yes 2> /dev/null)" \
    "NOT DONE: the angle must be"
check "past the joint limit refused" \
    "$(FAKE_FRANKA_Q0="0 0 0 -0.1 0 1.57 0.71" $NUDGE fake 4 5 --yes 2> /dev/null)" \
    "NOT DONE: J4 would end"
check "cancelled when not confirmed" "$(echo no | $NUDGE fake 7 5 2> /dev/null)" \
    "NOT DONE: cancelled"
output=$($NUDGE fake 7 3 --stiff --yes 2>&1)
check "J7 3 deg and back, --stiff" "$output" "Back at the start within 0.00 deg"
check "  libfranka examples' impedance" "$output" "setJointImpedance 3000 3000 3000 2500 2500 2000 2000"
for move in "7 5" "1 -10" "4 10"; do
    output=$($NUDGE fake $move --yes 2>&1)
    check "J${move% *} ${move#* } deg and back" "$output" "Back at the start within 0.00 deg"
    check "  no limit violations" "$(grep -c VIOLATION <<< "$output")" "^0$"
done

# The real Panda goes on by itself while packets are lost; a nudge that falls
# behind makes libfranka's rate limiter oscillate (2026-11-17). The fake runs
# the real limiter: the 1 ms-per-call nudge kept it busy ~6600 of 7500 cycles.
for ex in acceleration velocity; do
    output=$(FAKE_FRANKA_EXTRAPOLATE=$ex FAKE_FRANKA_LOSE_EVERY=97 $NUDGE fake 7 3 --yes 2>&1)
    check "J7 3 deg with lost packets ($ex)" "$output" "Back at the start within 0.00 deg"
    check "  no limit violations" "$(grep -c VIOLATION <<< "$output")" "^0$"
    check "  rate limiter never needed" "$output" "rate limiter changed 0 commands"
done
# --no-limit (NOLIMIT=1): no rate limiter, and the fake stops at the first
# violation as the robot would. The fixed nudge must not jump, even with
# lost packets; the 1 ms-per-call one stopped here at 0.3 s.
for ex in acceleration velocity; do
    output=$(FAKE_FRANKA_STRICT=1 FAKE_FRANKA_EXTRAPOLATE=$ex FAKE_FRANKA_LOSE_EVERY=23 \
        $NUDGE fake 7 5 --no-limit --yes 2>&1)
    check "J7 5 deg, no rate limiter, lost packets ($ex)" "$output" "Back at the start within 0.00 deg"
    check "  limiter off" "$output" "limit_rate 0"
    check "  no limit violations" "$(grep -c VIOLATION <<< "$output")" "^0$"
done

echo "Through the driver (with lost packets)"
export FAKE_FRANKA_LOSE_EVERY=97
setsid "$TOOLS/fake_franka/run_driver.sh" "$@" > "$OUT/driver.out" 2>&1 &
DRIVER=$!
setsid ros2 launch gello_teleop gello.launch.py backend:=fake robot:=real \
    > "$OUT/gello.out" 2>&1 &
GELLO=$!
cleanup() {
    kill -INT -- -"$GELLO" -"$DRIVER" 2> /dev/null
    sleep 2
    kill -KILL -- -"$GELLO" -"$DRIVER" 2> /dev/null
}
trap cleanup EXIT
sleep 6

check "libfranka alone refused while the driver runs" \
    "$(bash $TOOLS/real_robot/nudge.sh libfranka 172.16.0.2 7 5)" "NOT DONE: the driver is running"
check "refused before robot-enable" "$($DRIVER_NUDGE 7 5 --yes)" "run 'make robot-enable' first"
bash $TOOLS/real_robot/robot_mode.sh enable --yes > /dev/null
sleep 1
for move in "7 5" "2 -10"; do
    output=$($DRIVER_NUDGE $move --yes)
    check "J${move% *} ${move#* } deg and back" "$output" "Done. Back at the start"
    check "  no oscillation" "$output" "Verdict: clean."
    check "  CSV written" "$output" "Log ([0-9]* driver states of J${move% *}): $NUDGE_LOG_DIR/nudge_driver_J"
done
for _ in $(seq 15); do
    timeout 10 ros2 service call /gello/enable std_srvs/srv/SetBool "{data: true}" \
        | grep -q "success=True" && break
    sleep 1
done
check "refused while the GELLO controls the arm" "$($DRIVER_NUDGE 7 5 --yes)" \
    "NOT DONE: the GELLO controls the arm"
timeout 10 ros2 service call /gello/enable std_srvs/srv/SetBool "{data: false}" > /dev/null

check "no limit violations in the driver" "$(grep -acE "VIOLATION|STALL" "$OUT/driver.out")" "^0$"

echo "Logs: $OUT/driver.out, $OUT/gello.out"
if [ "$FAILED" -eq 0 ]; then
    echo "RESULT: PASS"
else
    echo "RESULT: FAIL"
fi
exit $FAILED
