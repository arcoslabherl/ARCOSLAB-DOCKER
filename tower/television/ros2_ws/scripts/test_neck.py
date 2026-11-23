#!/usr/bin/env python3
"""Test the neck node (make neck) against fake servos and a fake head,
no hardware: refuses to start away from neutral, dry run writes nothing,
live stays within range and speed with the right signs, and turns the
torque off when the head pose stops or a servo is pushed away.

Own ROS domain, so a running 'make stream' is not disturbed.
"""

import os
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
    return {PITCH_ID: round(p['pitch_neutral_deg'] * TICKS_PER_DEG),
            YAW_ID: round(p['yaw_neutral_deg'] * TICKS_PER_DEG)}


NEUTRAL = _neutral()   # from neck.yaml
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


def start_node(live):
    return subprocess.Popen(
        ['ros2', 'run', 'television_teleop', 'television_neck', '--ros-args',
         '--params-file', CONFIG, '-p', f'port:={PORT}', '-p', f'live:={str(live).lower()}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


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
    proc = start_node(live=True)
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
    for i, name in ((YAW_ID, 'yaw (ID 2)'), (PITCH_ID, 'pitch (ID 1)')):
        g = goals(servos, i)
        check(bool(g) and g[0] == NEUTRAL[i], f'{name}: first goal = where it is (no jump at torque on)')
        check(all(abs(v - NEUTRAL[i]) <= RANGE_TICKS for v in g), f'{name}: every goal within 3 deg of neutral')
        steps = [abs(b - a) for a, b in zip(g, g[1:])]
        check(max(steps, default=0) <= MAX_STEP, f'{name}: at most {MAX_STEP} ticks per goal (4 deg/s)')
        check(abs(final[i] - (NEUTRAL[i] + RANGE_TICKS)) <= 2,
              f'{name}: head left/down -> +3 deg ({(final[i] - NEUTRAL[i]) / TICKS_PER_DEG:+.1f})')
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
    head.angles = (1.0, 1.0)            # a small move: the camera follows 1 deg
    time.sleep(2.0)
    moved = {i: (servos.position(i) - NEUTRAL[i]) / TICKS_PER_DEG for i in NEUTRAL}
    head.angles = None
    code, out = finish(proc, 10)
    servos.close()
    check('Reference set' in out, 'says when the reference is set')
    for i, name in ((YAW_ID, 'yaw (ID 2)'), (PITCH_ID, 'pitch (ID 1)')):
        check(abs(moved[i] - 1.0) < 0.2, f'{name}: +1 deg from the still pose ({moved[i]:+.1f}), '
              'not saturated by the wild one')


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
        test_pushed(head)
    finally:
        head.running = False
        rclpy.try_shutdown()
    print('RESULT: ' + ('PASS' if not failures else f'FAIL ({len(failures)})'))
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
