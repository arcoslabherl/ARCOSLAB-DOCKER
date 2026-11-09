#!/usr/bin/env bash

# ARCoS-Lab - Stop a terminal session started from the Makefile
#
# Usage: stop_session.sh <session id>
#
# 'docker exec' does not stop its processes when the host terminal is
# closed. The Makefile calls this script when 'make sim' or 'make gello'
# ends for any reason, so nothing keeps running in the background.
# Start scripts record their process group in /tmp/ros2_session.<id>.

SESSION_FILE="/tmp/ros2_session.${1:?session id required}"

[[ -f "$SESSION_FILE" ]] || exit 0

PGID=$(cat "$SESSION_FILE")
rm -f "$SESSION_FILE"

[[ "$PGID" =~ ^[0-9]+$ ]] || exit 0

alive() {
    pgrep -g "$PGID" >/dev/null
}

# Same as Ctrl+C, then escalate if needed
for signal in INT TERM KILL; do
    alive || exit 0
    kill -"$signal" -- -"$PGID" 2>/dev/null || true

    for attempt in $(seq 1 30); do
        alive || exit 0
        sleep 0.5
    done
done
