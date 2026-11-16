"""config/panda_real.yaml, the overrides of 'make gello ROBOT=real'."""

import pytest

from config import CALIBRATOR, REAL_CALIBRATOR, REAL_OVERRIDES

# The lab driver as 'make real-driver' and the lab's real launch start it
DRIVER = "/asimilar/panda_arm"

# Robot-independent safety settings: the real robot must get the same ones
SAFETY = ["panda_joint_lower", "panda_joint_upper", "joint_limit_margin",
          "enable_tolerance", "max_sample_change", "data_timeout",
          "status_timeout", "gripper_enable_tolerance"]


def test_overrides_only_existing_parameters():
    # A misspelt name would be ignored silently and leave the sim's value
    assert set(REAL_OVERRIDES) <= set(CALIBRATOR)


@pytest.mark.parametrize("name", SAFETY)
def test_safety_settings_are_not_overridden(name):
    assert name not in REAL_OVERRIDES


def test_nothing_points_at_the_simulator():
    assert not [
        name for name, value in REAL_CALIBRATOR.items()
        if isinstance(value, str) and "sim_asimilar" in value
    ]


def test_topics_match_the_real_driver():
    assert REAL_CALIBRATOR["panda_state_topic"] == f"{DRIVER}/state"
    assert REAL_CALIBRATOR["panda_status_topic"] == f"{DRIVER}/status"
    assert REAL_CALIBRATOR["posref_topic"] == f"{DRIVER}/posref"
    assert REAL_CALIBRATOR["gripper_service"] == f"{DRIVER}/gripper"
    # joint_states is remapped out of the node's namespace
    assert REAL_CALIBRATOR["gripper_state_topic"] == "/asimilar/joint_states"
    # The driver names it <robot_name>_j7, width / 2
    assert REAL_CALIBRATOR["gripper_state_joint"] == "panda_j7"


def test_the_status_is_checked_on_the_real_robot():
    # After a reflex, only the status shows it
    assert REAL_CALIBRATOR["panda_status_topic"]


def test_grasp_only_on_the_real_robot():
    assert REAL_CALIBRATOR["gripper_grasp"] is True
    assert CALIBRATOR["gripper_grasp"] is False


def test_gripper_speed_within_the_hand_s_limit_per_finger():
    # Datasheet: 50 mm/s per finger
    assert 0.0 < REAL_CALIBRATOR["gripper_speed"] <= 0.05
