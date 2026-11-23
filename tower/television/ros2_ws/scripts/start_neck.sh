#!/usr/bin/env bash
set -e

# ARCoS-Lab TeleVision - the camera mount follows the head
#
# Usage: start_neck.sh [live] [range_deg] [port] [p_gain] [i_gain]
#   live: 0 (dry run, nothing written) or 1

LIVE="${1:-0}"
RANGE="${2:-}"
PORT="${3:-}"
P_GAIN="${4:-}"
I_GAIN="${5:-}"

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

if pgrep -f "lib/television_teleop/[t]elevision_neck" >/dev/null; then
    echo "ERROR: the neck is already running. Ctrl+C in its terminal first."
    exit 1
fi

# Let the Makefile stop this session if its terminal is closed
if [[ -n "${SESSION_ID:-}" ]]; then
    ps -o pgid= -p $$ | tr -d ' ' > "/tmp/ros2_session.${SESSION_ID}"
fi

ARGS=(-p live:=$([[ "$LIVE" == 1 ]] && echo true || echo false))
# A double for ROS: 10 -> 10.0
[[ -n "$RANGE" && "$RANGE" != *.* ]] && RANGE="$RANGE.0"
[[ -n "$RANGE" ]] && ARGS+=(-p range_deg:="$RANGE")
[[ -n "$P_GAIN" ]] && ARGS+=(-p position_p_gain:="$P_GAIN")
[[ -n "$I_GAIN" ]] && ARGS+=(-p position_i_gain:="$I_GAIN")
if [[ -n "$PORT" ]]; then
    [[ "$PORT" == /dev/* ]] && PORT="/hostdev/${PORT#/dev/}"
    ARGS+=(-p port:="$PORT")
fi

ros2 run television_teleop television_neck --ros-args \
    --params-file /ros2_ws/install/television_teleop/share/television_teleop/config/neck.yaml \
    "${ARGS[@]}"
