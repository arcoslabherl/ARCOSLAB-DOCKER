#!/usr/bin/env bash
# Driver test on the fake libfranka: starts the driver, runs test_driver.py,
# and checks the command stream the fake saw (violations, stalls).
#   test_driver.sh [--patched]     (--patched: the driver with our patch)
# Run inside the container, with nothing else running on /asimilar.
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT=/tmp/fake_franka_test
mkdir -p "$OUT"
rm -f /tmp/fake_franka_reflex /tmp/fake_franka_summary

setsid "$HERE/run_driver.sh" "$@" > "$OUT/driver.out" 2>&1 &
DRIVER=$!
cleanup() {
    kill -INT -- -"$DRIVER" 2> /dev/null || true
    sleep 1
    kill -KILL -- -"$DRIVER" 2> /dev/null || true
}
trap cleanup EXIT

sleep 3
set +e
python3 "$HERE/test_driver.py"
RESULT=$?

touch /tmp/fake_franka_summary
sleep 0.5
echo "Command stream (fake libfranka):"
grep -a "^\[fake_franka\]" "$OUT/driver.out" | grep -a -A9 "summary (segment" | tail -10 | sed "s/^\[fake_franka\]/ /"
violations=$(grep -a -c "VIOLATION" "$OUT/driver.out")
stalls=$(grep -a -c "STALL" "$OUT/driver.out")
echo "  printed violations: $violations, stalls: $stalls (log: $OUT/driver.out)"
if [ "$violations" -gt 0 ] || [ "$stalls" -gt 0 ]; then
    echo "  FAIL  the command stream would trigger reflexes on the real robot"
    RESULT=1
else
    echo "  PASS  command stream within the Panda's limits"
fi
echo "RESULT: $([ $RESULT -eq 0 ] && echo PASS || echo FAIL)"
exit $RESULT
