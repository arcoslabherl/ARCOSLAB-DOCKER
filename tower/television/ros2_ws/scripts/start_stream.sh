#!/usr/bin/env bash
set -e

# ARCoS-Lab TeleVision - stereo video to the headset
#
# Usage: start_stream.sh [zed|fake]

SOURCE="${1:-zed}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

if pgrep -f "lib/television_teleop/[t]elevision_stream" >/dev/null; then
    echo "ERROR: the stream is already running."
    echo "Stop it with Ctrl+C in its terminal, or restart the container:"
    echo "    make stop && make run"
    exit 1
fi

# Vuer's port: another server (an old TeleVision container?) may hold it
if ! python3 -c "import socket; s = socket.socket(); s.bind(('', 8012))" 2>/dev/null; then
    echo "ERROR: port 8012 is in use. Find what holds it on the host:"
    echo "    sudo ss -ltnp 'sport = :8012'      docker ps"
    exit 1
fi

# Let the Makefile stop this session if its terminal is closed
if [[ -n "${SESSION_ID:-}" ]]; then
    ps -o pgid= -p $$ | tr -d ' ' > "/tmp/ros2_session.${SESSION_ID}"
fi

echo "Starting the stream (${SOURCE})..."

ros2 run television_teleop television_stream --ros-args \
    --params-file /ros2_ws/install/television_teleop/share/television_teleop/config/stream.yaml \
    -p source:="$SOURCE"
