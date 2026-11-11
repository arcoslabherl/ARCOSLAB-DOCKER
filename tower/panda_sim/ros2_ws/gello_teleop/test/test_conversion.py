import math

import pytest

from config import DRIVER, HOME
from gello_teleop import conversion
from gello_teleop.backends import FakeBackend


def to_joints(raw, reference):
    """What gello_driver does on its first read."""
    joints = conversion.raw_to_joints(
        raw, DRIVER["joint_offsets"], DRIVER["joint_signs"])
    return conversion.apply_turns(joints, conversion.turns_to(joints, reference))


def raw_at(pose):
    """The lab GELLO's raw servo angles for a pose, as FakeBackend makes them."""
    backend = FakeBackend(
        rate=1000.0, home=pose, amplitude=0.0, frequency=0.0,
        joint_offsets=DRIVER["joint_offsets"],
        joint_signs=DRIVER["joint_signs"],
        gripper_open=None, gripper_closed=None)
    return backend.read()


def test_raw_to_joints_applies_offset_and_sign():
    assert conversion.raw_to_joints([1.0, 1.0], [0.5, 0.5], [1, -1]) == [0.5, -0.5]


def test_lab_calibration_maps_home():
    joints = to_joints(raw_at(HOME), DRIVER["reference_pose"])
    assert joints == pytest.approx(HOME, abs=1e-9)


@pytest.mark.parametrize("turns", [-2, -1, 1, 2])
def test_lost_turn_count_is_recovered(turns):
    # A servo powers up with its angle off by whole turns
    raw = raw_at(HOME)
    raw[3] += turns * math.tau
    raw[6] -= turns * math.tau
    joints = to_joints(raw, DRIVER["reference_pose"])
    assert joints == pytest.approx(HOME, abs=1e-9)


def test_reference_pose_covers_the_panda_range():
    # Anywhere in the Panda's range, the nearest turn is the right one
    lower = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    upper = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
    for pose in (lower, upper):
        joints = to_joints(raw_at(pose), DRIVER["reference_pose"])
        assert joints == pytest.approx(pose, abs=1e-9)


def test_turns_follow_the_last_value_after_an_error():
    # After a read error the driver unwraps against the last values: a joint
    # near +pi stays there instead of jumping to -pi
    joints = [3.1 - math.tau]
    turns = conversion.turns_to(joints, [3.0])
    assert conversion.apply_turns(joints, turns) == pytest.approx([3.1])


def test_gripper_fraction_with_the_lab_trigger():
    # The lab GELLO's servo angle goes down as the trigger closes
    open_angle = math.radians(DRIVER["gripper_open_deg"])
    closed_angle = math.radians(DRIVER["gripper_closed_deg"])
    middle = (open_angle + closed_angle) / 2.0

    assert conversion.gripper_fraction(open_angle, open_angle, closed_angle) == 0.0
    assert conversion.gripper_fraction(closed_angle, open_angle, closed_angle) == 1.0
    assert conversion.gripper_fraction(middle, open_angle, closed_angle) == pytest.approx(0.5)


def test_gripper_fraction_is_clamped():
    assert conversion.gripper_fraction(-1.0, 0.0, 1.0) == 0.0
    assert conversion.gripper_fraction(2.0, 0.0, 1.0) == 1.0


def test_smoothing():
    assert conversion.smooth([0.0, 1.0], [1.0, 3.0], 1.0) == [1.0, 3.0]
    assert conversion.smooth([0.0, 1.0], [1.0, 3.0], 0.5) == [0.5, 2.0]
