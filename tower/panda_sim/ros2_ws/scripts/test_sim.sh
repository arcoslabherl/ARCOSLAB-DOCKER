#!/usr/bin/env bash
set -uo pipefail

# ARCoS-Lab - Run the simulator tests unattended
# ROS 2 Humble - Simulation only
#
# Usage: test_sim.sh [rbc|teleop|all]   (default: all)
#   rbc     make sim + test-all-joints + test-gripper
#   teleop  make sim-teleop + gello-fake + test-gello
#
# Starts what each test needs, waits until it is ready, runs the tests and
# stops everything again, whatever the result. Nothing else may be running.

MODE="${1:-all}"
LOG_DIR=/tmp/test_sim
SCRIPTS=/ros2_ws/scripts
READY_TIMEOUT=120   # s

mkdir -p "$LOG_DIR"

# Let the Makefile stop this session if its terminal is closed
if [[ -n "${SESSION_ID:-}" ]]; then
    ps -o pgid= -p $$ | tr -d ' ' > "/tmp/ros2_session.${SESSION_ID}"
fi

SESSIONS=()

# Start a script in the background, as its own session
start() {
    local name="$1"
    shift
    local id="test_sim_$$_${name}"

    # Job control only for this job: it gets its own process group and
    # keeps SIGINT, so it can be stopped like Ctrl+C in a terminal. The
    # tests must stay in this script's group, or a stop wouldn't reach them.
    set -m
    SESSION_ID="$id" bash "$@" < /dev/null > "${LOG_DIR}/${name}.log" 2>&1 &
    set +m

    # The job leads its group. Record it now, so stop_session.sh finds it
    # even if the script hasn't written it yet.
    echo "$!" > "/tmp/ros2_session.${id}"
    SESSIONS+=("$id")
}

stop_all() {
    for id in "${SESSIONS[@]}"; do
        bash "${SCRIPTS}/stop_session.sh" "$id"
    done
    SESSIONS=()
}

cleanup() {
    # Don't let a second signal cut the cleanup short
    trap "" INT TERM HUP
    stop_all
}

trap cleanup EXIT
trap "exit 130" INT
trap "exit 143" TERM HUP

# Wait for a line in a log, or fail if its process ended first
wait_for() {
    local name="$1"
    local pattern="$2"
    local pid="$3"

    for second in $(seq 1 "$READY_TIMEOUT"); do
        if grep -q "$pattern" "${LOG_DIR}/${name}.log"; then
            return 0
        fi

        if ! kill -0 "$pid" 2>/dev/null; then
            echo "ERROR: ${name} stopped before it was ready:"
            tail -n 15 "${LOG_DIR}/${name}.log"
            return 1
        fi

        sleep 1
    done

    echo "ERROR: ${name} not ready after ${READY_TIMEOUT} s:"
    tail -n 15 "${LOG_DIR}/${name}.log"
    return 1
}

run_rbc() {
    echo "=== Simulator with RBC ==="
    start sim "${SCRIPTS}/start_sim.sh"
    wait_for sim "Joint Reacher initialized successfully" $! || return 1

    bash "${SCRIPTS}/test_all_joints.sh" && bash "${SCRIPTS}/test_gripper.sh"
    local result=$?

    stop_all
    return "$result"
}

run_teleop() {
    echo "=== Simulator for GELLO (fake device) ==="
    start sim_teleop "${SCRIPTS}/start_sim.sh" teleop
    wait_for sim_teleop "Ready for GELLO" $! || return 1

    start gello_fake "${SCRIPTS}/start_gello.sh" fake
    wait_for gello_fake "GELLO calibrator ready" $! || return 1

    bash "${SCRIPTS}/test_gello.sh"
    local result=$?

    stop_all
    return "$result"
}

case "$MODE" in
    rbc)    run_rbc ;;
    teleop) run_teleop ;;
    all)    run_rbc && echo && run_teleop ;;
    *)      echo "Usage: test_sim.sh [rbc|teleop|all]"; exit 2 ;;
esac
RESULT=$?

echo
if (( RESULT == 0 )); then
    echo "=================================="
    echo "Simulator tests SUCCEEDED"
    echo "=================================="
else
    echo "Simulator tests FAILED. Logs: ${LOG_DIR}/ in the container."
fi

exit "$RESULT"
