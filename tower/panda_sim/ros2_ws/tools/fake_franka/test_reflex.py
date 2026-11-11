#!/usr/bin/env python3
"""Reflex during teleop, against the driver on the fake libfranka.

Expects the driver (run_driver.sh) and the GELLO (fake backend, config
pointed at /asimilar) to be running; test_reflex.sh starts both.

Checks:
  1. GELLO enable is refused while the driver is disabled.
  2. After enabling the driver, GELLO enables and the arm follows.
  3. An injected reflex disables GELLO within 0.5 s.
  4. Re-enabling the driver doesn't move the arm towards the GELLO
     (GELLO stays disabled), and GELLO enable works again once matched.
"""

import os
import sys
import time

import rclpy
from impedance_control_msgs.msg import State
from impedance_control_msgs.srv import Modes
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

NS = "/asimilar"
REFLEX_FILE = os.environ.get("FAKE_FRANKA_REFLEX_FILE", "/tmp/fake_franka_reflex")
MAX_DISABLE_DELAY = 0.5   # s, reflex -> GELLO disabled
MAX_MOTION = 0.02         # rad, arm motion allowed after the driver re-enable


class Test(Node):
    def __init__(self):
        super().__init__("fake_franka_test_reflex")
        self.state = None
        self.gello_enabled = None
        self.create_subscription(State, f"{NS}/panda_arm/state", self.on_state, 10)
        self.create_subscription(Bool, "/gello/enabled", self.on_enabled, 10)
        self.control = self.create_client(Modes, f"{NS}/panda_arm/control")
        self.gello = self.create_client(SetBool, "/gello/enable")

    def on_state(self, msg):
        self.state = list(msg.position)

    def on_enabled(self, msg):
        self.gello_enabled = msg.data

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

    def gello_enable_retry(self, seconds=15.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            response = self.gello_enable()
            if response.success:
                return response
            self.spin_for(0.3)
        return response


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

        print("1. Driver disabled")
        response = node.gello_enable_retry(seconds=3.0)
        check(not response.success, f"GELLO enable refused: '{response.message}'")

        print("2. Driver enabled")
        check(node.driver(True) == "Success", "driver enabled")
        node.spin_for(1.5)
        response = node.gello_enable_retry()
        check(response.success, f"GELLO enabled: '{response.message}'")
        node.spin_for(2.0)
        check(node.gello_enabled is True, "/gello/enabled is true")

        print("3. Reflex")
        open(REFLEX_FILE, "w").close()
        start = time.monotonic()
        while node.gello_enabled and time.monotonic() - start < 2.0:
            rclpy.spin_once(node, timeout_sec=0.005)
        delay = time.monotonic() - start
        check(node.gello_enabled is False and delay < MAX_DISABLE_DELAY,
              f"GELLO disabled {delay * 1000:.0f} ms after the reflex "
              f"(limit {MAX_DISABLE_DELAY * 1000:.0f} ms)")

        # The GELLO keeps moving while the arm is stopped
        node.spin_for(4.0)
        frozen = list(node.state)

        print("4. Driver re-enabled")
        check(node.driver(True) == "Success", "driver re-enabled")
        moved = 0.0
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            moved = max(moved, max(abs(a - b) for a, b in zip(node.state, frozen)))
        check(moved < MAX_MOTION,
              f"arm moved {moved:.4f} rad after the re-enable (limit {MAX_MOTION})")
        check(node.gello_enabled is False, "GELLO still disabled")
        response = node.gello_enable_retry()
        check(response.success, f"GELLO enables again once matched: '{response.message}'")
        node.gello_enable(False)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print("RESULT:", "PASS" if not failures else f"FAIL ({len(failures)})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
