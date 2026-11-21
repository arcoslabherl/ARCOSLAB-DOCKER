#!/usr/bin/env python3
"""GELLO joint mask (make gello ROBOT=real JOINTS=7 RANGE=10), against the
driver on the fake libfranka.

Expects the driver (run_driver.sh) and the GELLO (fake backend, every joint
swinging 0.3 rad, joints:=7 range:=10) to be running; test_joint_mask.sh
starts both.

Checks:
  1. GELLO enables with only J7 matching (the other joints are far).
  2. While enabled, posref holds J1..J6 at the pose at enable and keeps J7
     within 10 degrees of it, reaching that bound (the GELLO goes past it).
  3. The arm's J1..J6 don't move.
"""

import math
import re
import sys
import time

import rclpy
from impedance_control_msgs.msg import PosRef, State
from impedance_control_msgs.srv import Modes
from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_srvs.srv import SetBool

NS = "/asimilar"
RANGE = math.radians(10.0)
HOLD_TOLERANCE = 1e-5     # rad: held joints get the enable pose, unchanged
MAX_ARM_MOTION = 0.005    # rad, measured motion allowed on the held joints
WATCH = 12.0              # s, more than one GELLO swing (0.1 Hz)


class Test(Node):
    def __init__(self):
        super().__init__("fake_franka_test_joint_mask")
        self.state = None
        self.commanded = None
        self.posrefs = []
        self.gello_pose = None
        self.create_subscription(State, f"{NS}/panda_arm/state", self.on_state, 10)
        self.create_subscription(PosRef, f"{NS}/panda_arm/posref", self.on_posref, 10)
        self.create_subscription(JointState, "/gello/joint_states", self.on_gello, 10)
        self.control = self.create_client(Modes, f"{NS}/panda_arm/control")
        self.gello = self.create_client(SetBool, "/gello/enable")

    def on_state(self, msg):
        self.state = list(msg.position)
        self.commanded = list(msg.cmdedposition)

    def on_gello(self, msg):
        self.gello_pose = list(msg.position)

    def on_posref(self, msg):
        self.posrefs.append(list(msg.position))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {client.srv_name} unavailable")
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise RuntimeError(f"service {client.srv_name} timed out")
            rclpy.spin_once(self, timeout_sec=0.01)
        return future.result()

    def driver(self, enable):
        request = Modes.Request()
        request.mode = 0
        request.enable = [enable] * 8
        return self.call(self.control, request).result

    def gello_enable(self, data=True):
        request = SetBool.Request()
        request.data = data
        return self.call(self.gello, request)


def main():
    rclpy.init()
    node = Test()
    failures = []

    def check(ok, text):
        print(f"  {'PASS' if ok else 'FAIL'}  {text}")
        if not ok:
            failures.append(text)

    try:
        node.spin_for(2.0)
        if node.state is None:
            print("ERROR: no driver state. Is run_driver.sh running?")
            return 1

        check(node.driver(True) == "Success", "driver enabled")
        node.spin_for(1.5)

        print("1. Enable with only J7 matching")
        # J7 swings 0.3 rad around the arm's pose: it's within the 2 degree
        # tolerance only near each crossing; the other joints are far then
        end = time.monotonic() + 20.0
        refusals = set()
        response = None
        while time.monotonic() < end:
            hold = list(node.commanded)
            arm = list(node.state)
            gello = list(node.gello_pose)
            response = node.gello_enable()
            if response.success:
                break
            refusals.add(response.message)
            node.spin_for(0.05)
        check(response.success, f"GELLO enabled: '{response.message}'")
        others = [text for text in refusals if re.search(r"\bJ[1-6]\b", text)]
        check(not others, f"no refusal names J1..J6 ({len(refusals)} refusals): {others[:2]}")
        far = max(abs(g - a) for g, a in zip(gello[:6], arm[:6]))
        check(far > math.radians(5.0),
              f"J1..J6 of the GELLO were far at enable ({math.degrees(far):.1f} deg)")
        if not response.success:
            return 1

        print("2. posref while enabled")
        node.posrefs.clear()
        moved = 0.0
        end = time.monotonic() + WATCH
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            moved = max(moved, max(abs(a - b) for a, b in zip(node.state[:6], arm[:6])))
        node.gello_enable(False)

        check(len(node.posrefs) > 100, f"{len(node.posrefs)} posref messages")
        held = max(
            abs(ref[i] - hold[i]) for ref in node.posrefs for i in range(6))
        check(held < HOLD_TOLERANCE,
              f"J1..J6 held at the pose at enable (largest change {held:.2e} rad)")
        j7 = max(abs(ref[6] - hold[6]) for ref in node.posrefs)
        check(j7 <= RANGE + 1e-6,
              f"J7 within 10 deg of the pose at enable ({math.degrees(j7):.2f} deg)")
        check(j7 >= RANGE - 1e-3,
              f"J7 reached the range limit (the GELLO goes 17 deg): {math.degrees(j7):.2f} deg")

        print("3. The arm")
        check(moved < MAX_ARM_MOTION,
              f"J1..J6 measured motion {moved:.4f} rad (limit {MAX_ARM_MOTION})")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print("RESULT:", "PASS" if not failures else f"FAIL ({len(failures)})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
