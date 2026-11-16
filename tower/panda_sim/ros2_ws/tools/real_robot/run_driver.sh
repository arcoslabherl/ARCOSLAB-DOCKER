#!/usr/bin/env bash
# Runs the real driver on the real Panda (172.16.0.2, hardcoded in the
# driver), named as in the lab's real launch: namespace /asimilar, node
# panda_arm, joint_states remapped to /asimilar/joint_states, cycle_freq 300.
# Only the driver: no platform, RViz or robot_state_publisher.
#   run_driver.sh [--patched]   (--patched: the driver with our patch applied)
# Run inside the real image's container.
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

BIN=/ros2_ws/build/_real_robot/driver/panda_driver_topics
if [ "$1" == "--patched" ]; then
    BIN=/ros2_ws/build/_real_robot/driver_patched/panda_driver_topics
fi
[ -x "$BIN" ] || { echo "Build it first: make real-driver-build"; exit 1; }

# The driver connects as soon as it starts and homes the gripper: the
# fingers open and close. The arm only moves once it's enabled.
echo "The driver connects to the Panda at 172.16.0.2 and HOMES THE GRIPPER"
echo "(the fingers open and close). The arm stays still until enabled."
echo "Check: nothing in the gripper, area clear, e-stop in hand."
read -r -p "Type 'yes' to start: " answer
[ "$answer" == "yes" ] || { echo "Not started."; exit 1; }

exec "$BIN" --ros-args \
    -r __ns:=/asimilar -r __node:=panda_arm \
    -r /asimilar/panda_arm/joint_states:=/asimilar/joint_states \
    -p robot_name:=panda -p frame_id_basename:=_base -p cycle_freq:=300.0
