#!/usr/bin/env python3
"""Test the neck node (make neck) against fake servos and a fake head,
no hardware: refuses to start away from neutral, dry run writes nothing,
live stays within range and speed with the right signs, and turns the
torque off when the head pose stops or a servo is pushed away.

Own ROS domain, so a running 'make stream' is not disturbed.
"""

import os
import signal
import subprocess
import sys
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '87'

import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_dxl import TICKS_PER_DEG, FakeServos  # noqa: E402

CONFIG = '/ros2_ws/install/television_teleop/share/television_teleop/config/neck.yaml'
PORT = '/tmp/fake_u2d2'
PITCH_ID, YAW_ID = 1, 2

def _neutral():
    import yaml
    with open(CONFIG.replace('/install/television_teleop/share/television_teleop/', '/television_teleop/')) as f:
        p = yaml.safe_load(f)['television_neck']['ros__parameters']
    return ({PITCH_ID: round(p['pitch_neutral_deg'] * TICKS_PER_DEG),
             YAW_ID: round(p['yaw_neutral_deg'] * TICKS_PER_DEG)},
            None if p['pitch_level_deg'] < 0 else p['pitch_level_deg'] - p['pitch_neutral_deg'])


NEUTRAL, LEVEL_FROM_NEUTRAL = _neutral()   # from neck.yaml


def expected_pitch(head_pitch, reference_pitch):
    """Pitch servo, deg from neutral (clamped): absolute or relative, as in neck.yaml."""
    change = head_pitch - reference_pitch if LEVEL_FROM_NEUTRAL is None else LEVEL_FROM_NEUTRAL + head_pitch
    return max(-3.0, min(3.0, change))
RANGE_TICKS = round(3.0 * TICKS_PER_DEG)
MAX_STEP = int(4.0 * TICKS_PER_DEG / 10.0)    # 4 deg/s at 10 Hz
ADDR_TORQUE, ADDR_GOAL = 64, 116

failures = []


def check(ok, what):
    print(('PASS ' if ok else 'FAIL ') + what, flush=True)
    if not ok:
        failures.append(what)


class Head:
    """Publishes a head pose at 60 Hz; angles None = stop publishing."""

    def __init__(self):
        self.node = rclpy.create_node('fake_head')
        self.pub = self.node.create_publisher(PoseStamped, 'television/head_pose', 10)
        self.angles = None
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            if self.angles is not None:
                yaw, pitch = self.angles
                q = Rotation.from_euler('ZYX', [yaw, pitch, 0.0], degrees=True).as_quat()
                msg = PoseStamped()
                msg.pose.orientation.x, msg.pose.orientation.y = float(q[0]), float(q[1])
                msg.pose.orientation.z, msg.pose.orientation.w = float(q[2]), float(q[3])
                msg.pose.position.z = 1.2
                self.pub.publish(msg)
            time.sleep(1.0 / 60.0)


def start_node(live, *extra):
    return subprocess.Popen(
        ['ros2', 'run', 'television_teleop', 'television_neck', '--ros-args',
         '--params-file', CONFIG, '-p', f'port:={PORT}', '-p', f'live:={str(live).lower()}'] +
        [a for e in extra for a in ('-p', e)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)


def finish(proc, timeout):
    try:
        out, _ = proc.communicate(timeout=timeout)
        return proc.returncode, out
    except subprocess.TimeoutExpired:
        proc.terminate()
        out, _ = proc.communicate(timeout=5)
        return None, out


def goals(servos, servo_id):
    return [v for _, i, a, v in servos.writes if i == servo_id and a == ADDR_GOAL]


def torque_writes(servos, servo_id):
    return [v for _, i, a, v in servos.writes if i == servo_id and a == ADDR_TORQUE]


def test_refuses_away_from_neutral(head):
    print('-- refuses to start 10 deg from neutral')
    servos = FakeServos({PITCH_ID: NEUTRAL[PITCH_ID], YAW_ID: NEUTRAL[YAW_ID] + round(10 * TICKS_PER_DEG)})
    head.angles = (0.0, 0.0)
    code, out = finish(start_node(live=True), 15)
    servos.close()
    check(code not in (0, None), 'exits with an error')
    check('from neutral' in out, 'says the camera is away from neutral')
    check(not servos.writes, 'writes nothing')


def test_dry_run(head):
    print('-- dry run')
    servos = FakeServos(dict(NEUTRAL))
    head.angles = (0.0, 0.0)
    proc = start_node(live=False)
    time.sleep(4.0)
    head.angles = (20.0, 20.0)
    time.sleep(2.0)
    head.angles = None
    code, out = finish(proc, 10)
    servos.close()
    check(not servos.writes, 'writes nothing')
    check('dry run' in out, 'prints the goals it would send')
    check(code not in (0, None) and 'no head pose' in out, 'exits when the head pose stops')


def test_live(head):
    print('-- live: follows within range and speed, torque off when the head stops')
    servos = FakeServos(dict(NEUTRAL))
    head.angles = (5.0, -3.0)          # the reference: any pose
    proc = start_node(True, 'position_p_gain:=800')
    time.sleep(4.0)
    # Head 20 deg left and 20 deg down, in 1 s
    for k in range(1, 21):
        head.angles = (5.0 + k, -3.0 + k)
        time.sleep(0.05)
    time.sleep(4.0)
    final = {i: servos.position(i) for i in NEUTRAL}
    torque_on_during = all(servos.torque(i) for i in NEUTRAL)
    head.angles = None
    stopped_at = time.monotonic()
    code, out = finish(proc, 10)
    exit_after = time.monotonic() - stopped_at
    servos.close()

    check(torque_on_during, 'torque on while following')
    for i in NEUTRAL:
        w = [(a, v) for _, j, a, v in servos.writes if j == i]
        p_at = next((k for k, (a, v) in enumerate(w) if a == 84 and v == 800), None)
        t_at = next((k for k, (a, v) in enumerate(w) if a == ADDR_TORQUE and v == 1), None)
        check(p_at is not None and t_at is not None and p_at < t_at,
              f'ID {i}: P gain 800 written before torque on')
    for i, name in ((YAW_ID, 'yaw (ID 2)'), (PITCH_ID, 'pitch (ID 1)')):
        g = goals(servos, i)
        check(bool(g) and g[0] == NEUTRAL[i], f'{name}: first goal = where it is (no jump at torque on)')
        check(all(abs(v - NEUTRAL[i]) <= RANGE_TICKS for v in g), f'{name}: every goal within 3 deg of neutral')
        steps = [abs(b - a) for a, b in zip(g, g[1:])]
        check(max(steps, default=0) <= MAX_STEP, f'{name}: at most {MAX_STEP} ticks per goal (4 deg/s)')
        check(abs(final[i] - (NEUTRAL[i] + RANGE_TICKS)) <= 2,
              f'{name}: head left / 17 deg down -> +3 deg ({(final[i] - NEUTRAL[i]) / TICKS_PER_DEG:+.1f})')
        check(torque_writes(servos, i)[-1:] == [0], f'{name}: torque off at the end')
    check(code not in (0, None) and 'no head pose' in out, 'stops when the head pose stops')
    check(exit_after < 4.0, f'stops within 4 s ({exit_after:.1f} s; timeout 2 s)')


def test_reference_waits_for_still_head(head):
    print('-- live: the reference waits for the head to be still (not a tracking jump)')
    servos = FakeServos(dict(NEUTRAL))
    head.angles = None
    proc = start_node(live=True)
    time.sleep(3.0)
    # Tracking starts: one wild pose, then the head still looking forward
    head.angles = (-32.0, -51.5)
    time.sleep(0.2)
    head.angles = (0.0, 0.0)
    time.sleep(2.5)
    pitch_move = 1.0 if LEVEL_FROM_NEUTRAL is None else 8.0
    head.angles = (1.0, pitch_move)     # yaw: 1 deg from the still pose
    time.sleep(3.5)
    moved = {i: (servos.position(i) - NEUTRAL[i]) / TICKS_PER_DEG for i in NEUTRAL}
    head.angles = None
    code, out = finish(proc, 10)
    servos.close()
    check('Reference set' in out, 'says when the reference is set')
    check(abs(moved[YAW_ID] - 1.0) < 0.2, f'yaw (ID 2): +1 deg from the still pose ({moved[YAW_ID]:+.1f}), '
          'not saturated by the wild one')
    want = expected_pitch(pitch_move, 0.0)
    mode = 'relative' if LEVEL_FROM_NEUTRAL is None else 'absolute'
    check(abs(moved[PITCH_ID] - want) < 0.2,
          f'pitch (ID 1, {mode}): head {pitch_move:g} deg down -> {want:+.1f} from neutral ({moved[PITCH_ID]:+.1f})')


def test_ctrl_c_mid_packet(head):
    print('-- live: Ctrl+C (twice, like make) on a slow bus -> torque off, checked')
    for attempt in range(4):
        # ~ the real bus: FTDI's 16 ms latency timer per answer, so a Ctrl+C
        # usually lands inside a transaction (what happened on the tower)
        servos = FakeServos(dict(NEUTRAL), latency=0.016)
        head.angles = (0.0, 0.0)
        proc = start_node(live=True)
        time.sleep(4.0 + 0.013 * attempt)
        head.angles = (3.0, 3.0)
        time.sleep(1.0)
        os.killpg(proc.pid, signal.SIGINT)
        time.sleep(0.05)
        os.killpg(proc.pid, signal.SIGINT)
        code, out = finish(proc, 15)
        servos.close()
        torque = [servos.torque(i) for i in NEUTRAL]
        check(torque == [0, 0], f'run {attempt + 1}: torque off on both (now {torque})')
        check('Could not' not in out, f'run {attempt + 1}: no "Could not turn ... torque off"')


def test_pushed(head):
    print('-- live: a servo pushed away -> torque off')
    servos = FakeServos(dict(NEUTRAL))
    head.angles = (0.0, 0.0)
    proc = start_node(live=True)
    time.sleep(4.0)
    # Hold the camera 10 deg away, against the servo
    end = time.monotonic() + 3.0
    while time.monotonic() < end and proc.poll() is None:
        servos.push(PITCH_ID, NEUTRAL[PITCH_ID] + round(10 * TICKS_PER_DEG))
        time.sleep(0.02)
    code, out = finish(proc, 10)
    servos.close()
    check(code not in (0, None) and 'from its goal' in out, 'stops: servo far from its goal')
    check(all(torque_writes(servos, i)[-1:] == [0] for i in NEUTRAL), 'torque off on both')


def main():
    rclpy.init()
    head = Head()
    try:
        test_refuses_away_from_neutral(head)
        test_dry_run(head)
        test_live(head)
        test_reference_waits_for_still_head(head)
        test_ctrl_c_mid_packet(head)
        test_pushed(head)
    finally:
        head.running = False
        rclpy.try_shutdown()
    print('RESULT: ' + ('PASS' if not failures else f'FAIL ({len(failures)})'))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
