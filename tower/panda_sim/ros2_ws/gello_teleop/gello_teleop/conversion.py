"""Raw GELLO servo angles to joint values. Pure functions, no ROS.

Same conversion as gello_software's DynamixelRobot:
  joint   = sign * (raw - offset)
  gripper = clamp((raw - open) / (closed - open), 0, 1)
"""

import math


def raw_to_joints(raw, offsets, signs):
    return [
        sign * (angle - offset)
        for angle, offset, sign in zip(raw, offsets, signs)
    ]


def turns_to(joints, reference):
    """Whole turns to add to each joint to land closest to its reference.

    A servo loses its turn count on power-up, so its angle can be off by
    multiples of 360 degrees.
    """
    return [
        round((target - joint) / math.tau)
        for target, joint in zip(reference, joints)
    ]


def apply_turns(joints, turns):
    return [joint + count * math.tau for joint, count in zip(joints, turns)]


def gripper_fraction(raw, open_angle, closed_angle):
    """0.0 open .. 1.0 closed, whichever way the servo turns."""
    fraction = (raw - open_angle) / (closed_angle - open_angle)
    return min(max(fraction, 0.0), 1.0)


def smooth(last, values, alpha):
    """Exponential smoothing: alpha 1.0 is no smoothing."""
    return [
        previous * (1.0 - alpha) + value * alpha
        for previous, value in zip(last, values)
    ]
