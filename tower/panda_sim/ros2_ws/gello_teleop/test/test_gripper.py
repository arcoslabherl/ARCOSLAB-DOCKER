import pytest

from config import CALIBRATOR
from gello_teleop.gripper import GripperCommand, GripperControl

MAX_WIDTH = CALIBRATOR["gripper_max_width"]


def make(grasp=False):
    return GripperControl(
        max_width=MAX_WIDTH,
        min_change=CALIBRATOR["gripper_min_change"],
        enable_tolerance=CALIBRATOR["gripper_enable_tolerance"],
        state_timeout=CALIBRATOR["gripper_state_timeout"],
        state_topic="/joint_states",
        grasp=grasp,
        grasp_threshold=CALIBRATOR["gripper_grasp_threshold"],
        grasp_release=CALIBRATOR["gripper_grasp_release"],
        grasp_force=CALIBRATOR["gripper_grasp_force"],
    )


def with_trigger(value, grasp=False):
    gripper = make(grasp)
    gripper.set_trigger(value, now=0.0)
    return gripper


def test_trigger_to_width():
    assert with_trigger(0.0).trigger_width() == pytest.approx(MAX_WIDTH)
    assert with_trigger(1.0).trigger_width() == pytest.approx(0.0)
    assert with_trigger(0.25).trigger_width() == pytest.approx(0.75 * MAX_WIDTH)


def test_trigger_is_clamped():
    assert with_trigger(-0.2).trigger == 0.0
    assert with_trigger(1.3).trigger == 1.0


def test_trigger_freshness():
    gripper = make()
    assert not gripper.trigger_fresh(now=1.0, timeout=0.3)
    gripper.set_trigger(0.5, now=1.0)
    assert gripper.trigger_fresh(now=1.125, timeout=0.25)
    assert not gripper.trigger_fresh(now=1.25, timeout=0.25)


# --- Enable check ----------------------------------------------------------

def test_enable_without_trigger():
    # A GELLO without trigger doesn't block enabling
    assert make().enable_check(now=0.0) == (True, None, None)


def test_enable_needs_the_gripper_width():
    ok, problem, hint = with_trigger(0.0).enable_check(now=0.0)
    assert not ok
    assert "/joint_states" in problem
    assert hint == "gripper state unavailable"


def test_enable_needs_a_recent_gripper_width():
    gripper = with_trigger(0.0)
    gripper.set_width(MAX_WIDTH, now=0.0)
    assert gripper.enable_check(now=0.5)[0]
    assert not gripper.enable_check(now=CALIBRATOR["gripper_state_timeout"])[0]


def test_enable_when_the_trigger_matches():
    gripper = with_trigger(0.5)
    gripper.set_width(0.04 + 0.014, now=0.0)
    assert gripper.enable_check(now=0.0) == (True, None, None)


def test_released_trigger_on_a_half_closed_gripper():
    # The trigger asks for 80 mm, the gripper is at 40 mm: enabling would
    # open it and drop what it holds
    gripper = with_trigger(0.0)
    gripper.set_width(0.04, now=0.0)
    ok, problem, hint = gripper.enable_check(now=0.0)
    assert not ok
    assert "+40 mm" in problem
    assert hint == "gripper 40 mm, press the trigger"


def test_pressed_trigger_on_an_open_gripper():
    gripper = with_trigger(1.0)
    gripper.set_width(MAX_WIDTH, now=0.0)
    ok, _, hint = gripper.enable_check(now=0.0)
    assert not ok
    assert hint == "gripper 80 mm, release the trigger"


def test_enable_in_the_grasp_zone_only_with_grasp_mode():
    for grasp, expected in ((True, True), (False, False)):
        gripper = with_trigger(0.95, grasp=grasp)
        gripper.set_width(MAX_WIDTH, now=0.0)
        assert gripper.enable_check(now=0.0)[0] is expected


# --- Commands --------------------------------------------------------------

def test_first_command_moves_to_the_trigger():
    assert with_trigger(0.5).next_command() == GripperCommand("move", 0.04)


def test_small_trigger_changes_are_not_sent():
    gripper = with_trigger(0.5)
    gripper.next_command()

    gripper.set_trigger(0.5 + 0.0005 / MAX_WIDTH, now=0.1)   # 0.5 mm
    assert gripper.next_command() is None

    gripper.set_trigger(0.5 + 0.0011 / MAX_WIDTH, now=0.2)   # 1.1 mm
    assert gripper.next_command().width == pytest.approx(0.04 - 0.0011)


def test_reset_sends_the_width_again():
    gripper = with_trigger(0.5)
    gripper.next_command()
    assert gripper.next_command() is None
    gripper.reset()
    assert gripper.next_command() == GripperCommand("move", 0.04)


def test_without_grasp_mode_a_full_squeeze_is_a_move():
    assert with_trigger(1.0).next_command() == GripperCommand("move", 0.0)


def test_grasp_cycle():
    gripper = with_trigger(0.7, grasp=True)
    assert gripper.next_command() == GripperCommand(
        "move", pytest.approx(0.3 * MAX_WIDTH))

    # Past the threshold: one grasp that accepts any final width
    gripper.set_trigger(0.92, now=0.1)
    assert gripper.next_command() == GripperCommand(
        "grasp", 0.0, force=CALIBRATOR["gripper_grasp_force"],
        epsilon_inner=0.005, epsilon_outer=MAX_WIDTH)

    # Held while the trigger stays above the release level
    for value in (1.0, 0.85, 0.8):
        gripper.set_trigger(value, now=0.2)
        assert gripper.next_command() is None

    # Released where it was before the grasp: the gripper is closed on the
    # object now, so the same width must be sent again to open it
    gripper.set_trigger(0.7, now=0.3)
    assert gripper.next_command() == GripperCommand(
        "move", pytest.approx(0.3 * MAX_WIDTH))
    assert not gripper.grasping


def test_reset_ends_a_grasp():
    gripper = with_trigger(0.95, grasp=True)
    assert gripper.next_command().cmd == "grasp"
    gripper.reset()
    assert not gripper.grasping
