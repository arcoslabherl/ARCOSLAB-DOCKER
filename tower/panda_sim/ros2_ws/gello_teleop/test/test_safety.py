import math

import pytest

from config import CALIBRATOR
from gello_teleop import safety

LOWER = CALIBRATOR["panda_joint_lower"]
UPPER = CALIBRATOR["panda_joint_upper"]


def test_panda_limits_in_the_config_are_franka_s():
    assert LOWER == [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    assert UPPER == [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]


@pytest.mark.parametrize("values", [
    [0.0] * 6,
    [0.0] * 8,
    [0.0] * 6 + [math.nan],
    [0.0] * 6 + [math.inf],
])
def test_invalid_joints_are_rejected(values):
    assert safety.valid_joints(values) is None


def test_valid_joints_returns_a_list():
    assert safety.valid_joints((0.0,) * 7) == [0.0] * 7


def test_clip_to_limits():
    inside = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
    assert safety.clip_to_limits(inside, LOWER, UPPER) == inside

    # J4 can't reach 0 and J6 can't go past 3.7525 on the Panda
    outside = [3.0, -2.0, 0.0, 0.0, 0.0, 4.0, 0.0]
    assert safety.clip_to_limits(outside, LOWER, UPPER) == [
        2.8973, -1.7628, 0.0, -0.0698, 0.0, 3.7525, 0.0]


def test_pose_differences():
    panda = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.71]
    gello = [0.1, 0.0, 0.0, -1.57, 0.0, 1.57, 0.71]
    assert safety.pose_differences(gello, panda, LOWER, UPPER) == pytest.approx(
        [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_gello_past_a_limit_matches_a_panda_at_it():
    # Otherwise control could never be enabled with a joint at its limit
    panda = [0.0, 0.0, 0.0, -0.0698, 0.0, 1.57, 0.0]
    gello = [0.0, 0.0, 0.0, 0.3, 0.0, 1.57, 0.0]
    assert safety.pose_differences(gello, panda, LOWER, UPPER) == pytest.approx(
        [0.0] * 7)


def test_joint_jump():
    previous = [0.0] * 7
    assert not safety.joint_jumped([0.0] * 7, None, 0.5)
    assert not safety.joint_jumped([0.5] + [0.0] * 6, previous, 0.5)
    assert safety.joint_jumped([0.0] * 6 + [-0.51], previous, 0.5)


def test_stale_sources():
    sources = [("GELLO", "/gello", 9.9), ("Panda state", "/state", None)]
    assert safety.stale_sources(sources, now=10.0, timeout=0.3) == [
        "Panda state (/state): no data yet"]


def test_old_data_is_stale_from_the_timeout_on():
    # Exact binary values, so the boundary itself is tested
    assert safety.stale_sources([("GELLO", "/gello", 10.0)], 10.125, 0.25) == []
    assert safety.stale_sources([("GELLO", "/gello", 10.0)], 10.25, 0.25) == [
        "GELLO (/gello): 0.25 s old"]


# --- Robot status (enabled, emergency stop) --------------------------------

def test_robot_status_in_the_config():
    assert CALIBRATOR["panda_status_topic"].endswith("/panda_arm/status")
    assert CALIBRATOR["status_timeout"] > 0.0


@pytest.mark.parametrize("enabled, emergency, problem", [
    ([True] * 8, [False] * 8, None),
    ([], [], "robot status is empty"),
    ([True] * 7 + [False], [False] * 8, "robot driver not enabled"),
    ([False] * 8, [False] * 8, "robot driver not enabled"),
    # A reflex: the driver still reports itself enabled
    ([True] * 8, [False] * 3 + [True] + [False] * 4,
     "robot in emergency stop (reflex or error)"),
    ([False] * 8, [True] * 8, "robot in emergency stop (reflex or error)"),
])
def test_robot_problem(enabled, emergency, problem):
    assert safety.robot_problem(enabled, emergency) == problem


def test_robot_ready_needs_a_status():
    ok, problem = safety.robot_ready(None, None, 10.0, 1.0, "/status")
    assert not ok
    assert problem == "no robot status on /status yet"


def test_robot_ready_needs_a_recent_status():
    assert safety.robot_ready(None, 10.0, 10.5, 1.0, "/status") == (True, None)
    assert safety.robot_ready(None, 10.0, 11.0, 1.0, "/status") == (
        False, "robot status (/status) 1.00 s old")


def test_robot_ready_reports_the_problem():
    problem = "robot in emergency stop (reflex or error)"
    assert safety.robot_ready(problem, 10.0, 10.0, 1.0, "/status") == (False, problem)
