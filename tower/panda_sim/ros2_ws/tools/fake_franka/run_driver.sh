#!/usr/bin/env bash
# Runs the real driver, built against the fake libfranka, the way the real
# launch (arcos-lab-asimilar-robot-startup.launch.py) starts it: namespace
# /asimilar, node panda_arm, joint_states remapped to /asimilar/joint_states,
# cycle_freq 300. Only the driver: no platform, RViz or robot_state_publisher.
# Fake settings come from FAKE_FRANKA_* / FAKE_GRIPPER_* (see README.md).
#   run_driver.sh [--patched]   (--patched: the driver with our patch applied)
# Run inside the container.
set -e

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

BIN=/ros2_ws/build/_fake_franka/driver/panda_driver_topics
if [ "$1" == "--patched" ]; then
    BIN=/ros2_ws/build/_fake_franka/driver_patched/panda_driver_topics
fi
[ -x "$BIN" ] || { echo "Build it first: tools/fake_franka/build.sh"; exit 1; }

exec "$BIN" --ros-args \
    -r __ns:=/asimilar -r __node:=panda_arm \
    -r /asimilar/panda_arm/joint_states:=/asimilar/joint_states \
    -p robot_name:=panda -p frame_id_basename:=_base -p cycle_freq:=300.0
