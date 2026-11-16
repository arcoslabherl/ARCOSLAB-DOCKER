#!/usr/bin/env bash
# Runs one of libfranka's examples on the real Panda. Run inside the real
# image's container.
#   examples.sh echo [<robot ip>]   echo_robot_state: reads the state 100
#                                   times, no motion
#   examples.sh comm [<robot ip>]   communication_test: how many control
#                                   packets the robot missed. MOVES the arm.
set -e

EXAMPLES=/opt/libfranka/build/examples
EXAMPLE="$1"
IP="${2:-172.16.0.2}"
[ -d "$EXAMPLES" ] || { echo "No libfranka examples here: use 'make run REAL=1'."; exit 1; }

# libfranka waits a long time, silently, if the robot isn't there. Desk's
# HTTPS port answers as soon as the controller is reachable.
if ! timeout 3 bash -c "</dev/tcp/$IP/443" 2>/dev/null; then
    echo "The Panda doesn't answer at $IP. Check the cable (Control's LAN"
    echo "port), this computer's address (172.16.0.1/24) and that Desk opens."
    echo "If Desk works but this check doesn't, run it directly ('make debug'):"
    echo "    $EXAMPLES/<example> $IP"
    exit 1
fi

case "$EXAMPLE" in
    echo)
        exec "$EXAMPLES/echo_robot_state" "$IP"
        ;;
    comm)
        # What libfranka 0.8.0's example does (examples/communication_test.cpp)
        echo "communication_test MOVES THE ARM:"
        echo "  1. to the joint pose (0, -45, 0, -135, 0, 90, 45) deg at half speed,"
        echo "  2. then 10 s with zero torque (the arm floats, gravity compensated"
        echo "     only: it sags if the flange load in Desk is wrong)."
        echo "Check: path to that pose clear (table, camera, extension), nothing"
        echo "in the gripper, flange load checked, e-stop in hand."
        read -r -p "Type 'yes' to start: " answer
        [ "$answer" == "yes" ] || { echo "Not started."; exit 1; }
        exec "$EXAMPLES/communication_test" "$IP"
        ;;
    *)
        echo "Usage: examples.sh echo|comm [<robot ip>]"
        exit 1
        ;;
esac
