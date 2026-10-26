#!/usr/bin/env bash
set -e

# ARCoS-Lab - Franka Panda Simulator
# ROS 2 Humble

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "Starting Franka Panda simulator..."

# Initialize Joint Reacher when RBC becomes available
(
    RBC_READY=false

    for attempt in $(seq 1 30); do
        if ros2 service list | grep -Fxq "/sim_asimilar_01/rbc_l2_ctrl"; then
            RBC_READY=true
            break
        fi

        sleep 1
    done

    if [ "$RBC_READY" != "true" ]; then
        echo "ERROR: RBC service did not become available."
        exit 1
    fi

    bash /ros2_ws/setup_jr.sh
) &

SETUP_PID=$!

# Stop the initialization helper when the simulator exits
trap 'kill "$SETUP_PID" 2>/dev/null || true' EXIT

# Keep the simulator in the foreground
ros2 launch \
    arcos-lab-asimilar-robot-startup \
    arcos-lab-asimilar-robot-startup-sim.launch.py