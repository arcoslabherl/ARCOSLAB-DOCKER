"""Active neck: the camera mount's 2 servos follow the operator's head.

Relative mapping: the head's pose at the first message is the reference,
the camera starts at its neutral pose (checked), and each servo moves by
the head's change in yaw or pitch, clamped to a range around neutral and
to the mount's safe ends, at a limited speed.

No ROS or serial here, so it can be unit tested.
"""

import math

TICKS_PER_REV = 4096
TICKS_PER_DEG = TICKS_PER_REV / 360.0


def ticks(deg):
    return int(round(deg * TICKS_PER_DEG))


def deg(ticks_):
    return ticks_ / TICKS_PER_DEG


def head_yaw_pitch(qx, qy, qz, qw):
    """Yaw and pitch (deg) of a ROS Z-up orientation: yaw + = head turned
    left, pitch + = looking down (ZYX Euler angles)."""
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    return math.degrees(yaw), math.degrees(math.asin(sin_pitch))


def wrap_deg(angle):
    """Angle in (-180, 180]."""
    angle = math.fmod(angle + 180.0, 360.0)
    if angle <= 0.0:
        angle += 360.0
    return angle - 180.0


class Axis:
    """One servo: neutral, safe ends and the session's range, in ticks."""

    def __init__(self, name, servo_id, neutral_deg, min_deg, max_deg, sign, range_deg):
        if not min_deg < neutral_deg < max_deg:
            raise ValueError(f'{name}: neutral {neutral_deg} not between {min_deg} and {max_deg}')
        if sign not in (1, -1):
            raise ValueError(f'{name}: sign must be 1 or -1')
        if range_deg <= 0.0:
            raise ValueError(f'{name}: range must be > 0')
        self.name = name
        self.id = servo_id
        self.sign = sign
        self.neutral = ticks(neutral_deg)
        # The session's range, never past the mount's safe ends
        self.low = max(ticks(min_deg), self.neutral - ticks(range_deg))
        self.high = min(ticks(max_deg), self.neutral + ticks(range_deg))
        # Position control mode only takes goals in 0..4095
        if self.low < 0 or self.high >= TICKS_PER_REV:
            raise ValueError(f'{name}: range {self.low}..{self.high} ticks leaves 0..4095')

    def target(self, head_change_deg):
        """Goal (ticks) for a change of the head's angle, clamped."""
        goal = self.neutral + self.sign * head_change_deg * TICKS_PER_DEG
        return int(round(min(self.high, max(self.low, goal))))


def step_toward(current, target, max_step):
    """Move current toward target by at most max_step (ticks)."""
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


class Follower:
    """The goals of both axes from the head's yaw and pitch, at a limited
    speed. Call update() at a fixed rate with the latest head angles."""

    def __init__(self, yaw_axis, pitch_axis, max_speed_deg_s, rate_hz):
        self.yaw = yaw_axis
        self.pitch = pitch_axis
        # Ticks per update, at least 1 so it can move at all
        self.max_step = max(1, int(max_speed_deg_s * TICKS_PER_DEG / rate_hz))
        self.reference = None
        self.goal = {yaw_axis.name: yaw_axis.neutral, pitch_axis.name: pitch_axis.neutral}

    def start(self, present):
        """Begin from the servos' present positions (ticks, by axis name)."""
        self.goal = dict(present)
        self.reference = None

    def update(self, head_yaw, head_pitch):
        """New goals (ticks by axis name). The first call sets the reference."""
        if self.reference is None:
            self.reference = (head_yaw, head_pitch)
        d_yaw = wrap_deg(head_yaw - self.reference[0])
        d_pitch = head_pitch - self.reference[1]
        for axis, change in ((self.yaw, d_yaw), (self.pitch, d_pitch)):
            self.goal[axis.name] = step_toward(self.goal[axis.name], axis.target(change), self.max_step)
        return dict(self.goal)
