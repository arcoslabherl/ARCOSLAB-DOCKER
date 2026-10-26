#!/usr/bin/env bash
set -eo pipefail

# ARCoS-Lab - Full Franka Panda Simulation Test

echo "Testing all Panda joints..."

bash /ros2_ws/test_all_joints.sh

echo
echo "Testing Panda gripper..."

bash /ros2_ws/test_gripper.sh

echo
echo "=================================="
echo "Full Panda test SUCCEEDED"
echo "=================================="