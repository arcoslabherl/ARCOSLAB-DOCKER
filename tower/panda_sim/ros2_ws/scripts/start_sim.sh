#!/usr/bin/env bash
set -e

# ARCoS-Lab - Franka Panda Simulator
# ROS 2 Humble
#
# Usage: start_sim.sh [rbc|teleop] [max velocity] [max acceleration] [max jerk]
#   rbc     RBC and a Joint Reacher (default)
#   teleop  no RBC: GELLO is the only source of arm commands. The limits
#           default to the ones in the real robot's driver.

MODE="${1:-rbc}"
ARM_VEL="${2:-0.8}"
ARM_ACC="${3:-1.0}"
ARM_JERK="${4:-50}"
NAMESPACE="/sim_asimilar_01"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Refuse to start a second simulator next to a running one
if pgrep -f "arcos-lab-asimilar-robot-startup-sim.launch.py" >/dev/null \
    || pgrep -f "/lib/rbc/rbc" >/dev/null; then
    echo "ERROR: The simulator is already running."
    echo "Stop it with Ctrl+C in its terminal, or restart the container:"
    echo "    make stop && make run"
    exit 1
fi

# Let the Makefile stop this session if its terminal is closed
if [[ -n "${SESSION_ID:-}" ]]; then
    ps -o pgid= -p $$ | tr -d ' ' > "/tmp/ros2_session.${SESSION_ID}"
fi

echo "Starting Franka Panda simulator..."

# Wait for a service of the simulator or RBC to become available
wait_for_service() {
    for attempt in $(seq 1 30); do
        if timeout 10 ros2 service list | grep -Fxq "$1"; then
            return 0
        fi

        sleep 1
    done

    echo "ERROR: Service $1 did not become available."
    return 1
}

if [[ "$MODE" == "teleop" ]]; then
    LAUNCH_ARGS=(
        use_rbc:=false
        arm_max_velocity:="$ARM_VEL"
        arm_max_acceleration:="$ARM_ACC"
        arm_max_jerk:="$ARM_JERK"
    )

    echo "Arm limits: ${ARM_VEL} rad/s, ${ARM_ACC} rad/s^2, ${ARM_JERK} rad/s^3"

    # RBC is what normally sends the arm home; the simulated arm starts at
    # all zeros. Simulator only: the real robot must never get this jump.
    (
        wait_for_service "${NAMESPACE}/panda_arm/control" || exit 1

        ros2 topic pub --times 50 --rate 10 \
            "${NAMESPACE}/panda_arm/posref" \
            impedance_control_msgs/msg/PosRef \
            "{position: [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.71]}" >/dev/null

        echo "Panda at home pose. Ready for GELLO."
    ) &
else
    LAUNCH_ARGS=(use_rbc:=true)

    # Initialize Joint Reacher when RBC becomes available
    (
        wait_for_service "${NAMESPACE}/rbc_l2_ctrl" || exit 1

        bash /ros2_ws/scripts/setup_jr.sh
    ) &
fi

SETUP_PID=$!

# Stop the initialization helper when the simulator exits
trap 'kill "$SETUP_PID" 2>/dev/null || true' EXIT

# Keep the simulator in the foreground
ros2 launch \
    arcos-lab-asimilar-robot-startup \
    arcos-lab-asimilar-robot-startup-sim.launch.py \
    "${LAUNCH_ARGS[@]}"