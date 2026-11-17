#!/usr/bin/env bash
# Moves one joint a few degrees and back: see nudge.cpp (libfranka alone) and
# driver_nudge.py (through the driver).
#   nudge.sh libfranka <robot ip> <joint> <degrees> [--stiff]   the driver must not run
#   nudge.sh driver <joint> <degrees> [--ns /asimilar/panda_arm]
# Run inside the real image's container.
set -e

case "$1" in
    libfranka)
        IP="$2"
        if pgrep -f "[p]anda_driver_topics" > /dev/null; then
            echo "NOT DONE: the driver is running, and only one program can control"
            echo "the Panda. Stop it (Ctrl+C in its terminal), or use the driver:"
            echo "    make robot-nudge VIA=driver JOINT=.. DEG=.."
            exit 1
        fi
        BIN=/ros2_ws/build/_real_robot/nudge
        [ -x "$BIN" ] || { echo "Build it first: make real-driver-build"; exit 1; }
        # libfranka waits a long time, silently, if the robot isn't there
        if ! timeout 3 bash -c "</dev/tcp/$IP/443" 2>/dev/null; then
            echo "The Panda doesn't answer at $IP. Check the cable, this computer's"
            echo "address (172.16.0.1/24) and that Desk opens."
            exit 1
        fi
        shift 2
        exec "$BIN" "$IP" "$@"   # <joint> <degrees> [--stiff]
        ;;
    driver)
        shift
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash
        exec python3 "$(dirname "${BASH_SOURCE[0]}")/driver_nudge.py" "$@"
        ;;
    *)
        echo "Usage: nudge.sh libfranka <robot ip> <joint> <degrees>"
        echo "       nudge.sh driver <joint> <degrees> [--ns <driver namespace>]"
        exit 1
        ;;
esac
