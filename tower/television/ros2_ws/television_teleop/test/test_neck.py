import math

import pytest
from scipy.spatial.transform import Rotation

from television_teleop.neck import Axis, Follower, Settle, deg, head_yaw_pitch, step_toward, ticks, wrap_deg


def make_axes(range_deg=3.0):
    yaw = Axis('yaw', 2, 33.0, 5.0, 96.0, 1, range_deg)
    pitch = Axis('pitch', 1, 268.0, 252.0, 312.0, 1, range_deg)
    return yaw, pitch


def test_head_yaw_pitch_matches_scipy():
    for yaw, pitch in [(30.0, 0.0), (0.0, 20.0), (-45.0, -15.0), (170.0, 10.0)]:
        q = Rotation.from_euler('ZYX', [yaw, pitch, 5.0], degrees=True).as_quat()
        y, p = head_yaw_pitch(*q)
        assert y == pytest.approx(yaw, abs=1e-6)
        assert p == pytest.approx(pitch, abs=1e-6)


def test_wrap():
    assert wrap_deg(190.0) == pytest.approx(-170.0)
    assert wrap_deg(-190.0) == pytest.approx(170.0)
    assert wrap_deg(180.0) == pytest.approx(180.0)


def test_axis_clamps_to_range_and_safe_ends():
    yaw, pitch = make_axes(3.0)
    assert yaw.target(0.0) == yaw.neutral
    assert yaw.target(100.0) == yaw.neutral + ticks(3.0)
    assert yaw.target(-100.0) == yaw.neutral - ticks(3.0)
    # A wide range stops at the safe ends
    yaw, pitch = make_axes(90.0)
    assert yaw.target(-100.0) == ticks(5.0)
    assert yaw.target(100.0) == ticks(96.0)
    assert pitch.target(-100.0) == ticks(252.0)
    assert pitch.target(100.0) == ticks(312.0)


def test_axis_sign():
    reversed_axis = Axis('yaw', 2, 33.0, 5.0, 96.0, -1, 3.0)
    assert reversed_axis.target(2.0) < reversed_axis.neutral


def test_axis_rejects_bad_config():
    with pytest.raises(ValueError):
        Axis('yaw', 2, 33.0, 40.0, 96.0, 1, 3.0)       # neutral outside the ends
    with pytest.raises(ValueError):
        Axis('yaw', 2, 33.0, -59.0, 96.0, 1, 90.0)     # crosses 0
    with pytest.raises(ValueError):
        Axis('yaw', 2, 33.0, 5.0, 96.0, 0, 3.0)


def test_step_toward():
    assert step_toward(100, 110, 4) == 104
    assert step_toward(100, 90, 4) == 96
    assert step_toward(100, 102, 4) == 102


def test_follower_speed_limit_and_reference():
    yaw, pitch = make_axes(3.0)
    rate = 10.0
    f = Follower(yaw, pitch, 4.0, rate)
    f.start({'yaw': yaw.neutral, 'pitch': pitch.neutral})
    # First pose is the reference: no motion even if the head is turned
    g = f.update(40.0, -10.0)
    assert g == {'yaw': yaw.neutral, 'pitch': pitch.neutral}
    # A big jump: at most 4 deg/s
    previous = g
    for _ in range(30):
        g = f.update(80.0, 30.0)
        for name in g:
            assert abs(deg(g[name] - previous[name])) <= 4.0 / rate + 1e-9
        previous = g
    assert g['yaw'] == yaw.neutral + ticks(3.0)       # head left -> up
    assert g['pitch'] == pitch.neutral + ticks(3.0)   # head down -> up


def test_follower_starts_from_present():
    yaw, pitch = make_axes(3.0)
    f = Follower(yaw, pitch, 4.0, 10.0)
    off = yaw.neutral + ticks(4.0)
    f.start({'yaw': off, 'pitch': pitch.neutral})
    g = f.update(0.0, 0.0)
    # Back toward neutral, one step at a time
    assert off - g['yaw'] == f.max_step
    assert math.isclose(deg(f.max_step), 0.35, abs_tol=0.05)


def test_settle_needs_a_still_steady_second():
    s = Settle()
    t = 0.0
    # A jump (tracking starting), then still
    s.add(t, -32.0, -51.5)
    for k in range(1, 30):
        s.add(t + k / 60.0, 0.0, 0.0)
    assert not s.still()                 # the jump is within the last second
    for k in range(30, 100):
        s.add(t + k / 60.0, 0.3, -0.2)
    assert s.still()


def test_settle_rejects_slow_or_moving():
    slow = Settle()
    for k in range(15):                  # 15 Hz: too few poses
        slow.add(k / 15.0, 0.0, 0.0)
    assert not slow.still()
    moving = Settle()
    for k in range(80):
        moving.add(k / 60.0, k * 0.1, 0.0)   # 6 deg/s
    assert not moving.still()
    wrapping = Settle()
    for k in range(80):
        wrapping.add(k / 60.0, 179.5 if k % 2 else -179.5, 0.0)
    assert wrapping.still()              # 1 deg apart across +-180
