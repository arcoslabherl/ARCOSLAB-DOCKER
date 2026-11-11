#!/usr/bin/env python3
"""Driver behavior on the fake libfranka (test_driver.sh starts the driver).

  1. First enable: does panda_arm/state keep publishing during the call?
  2. Disable while J1 moves fast: how far and how long until it stops.
  3. Re-enable: does the arm move by itself?
  4. posref at ~1 kHz with a moving target, for the try_lock stall.

Command-stream problems (violations, stalls) are reported by the fake in
the driver's output; test_driver.sh counts them.
"""

import math
import sys
import time

import rclpy
from impedance_control_msgs.msg import PosRef, State
from impedance_control_msgs.srv import Modes
from rclpy.node import Node

NS = "/asimilar"
HOME = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.71]
MAX_STATE_GAP = 0.1      # s, during the enable call
MAX_SELF_MOTION = 0.005  # rad, after re-enabling at rest


class Test(Node):
    def __init__(self):
        super().__init__("fake_franka_test_driver")
        self.state = None
        self.state_time = None
        self.max_gap = 0.0
        self.create_subscription(State, f"{NS}/panda_arm/state", self.on_state, 100)
        self.pub = self.create_publisher(PosRef, f"{NS}/panda_arm/posref", 10)
        self.control = self.create_client(Modes, f"{NS}/panda_arm/control")

    def on_state(self, msg):
        now = time.monotonic()
        if self.state_time is not None:
            self.max_gap = max(self.max_gap, now - self.state_time)
        self.state_time = now
        self.state = list(msg.position)

    def spin_for(self, seconds, posref=None, rate=30.0):
        end = time.monotonic() + seconds
        next_pub = 0.0
        while time.monotonic() < end:
            if posref is not None and time.monotonic() >= next_pub:
                self.send(posref)
                next_pub = time.monotonic() + 1.0 / rate
            rclpy.spin_once(self, timeout_sec=0.002)

    def send(self, position):
        msg = PosRef()
        msg.position = [float(x) for x in position]
        self.pub.publish(msg)

    def driver(self, enable, timeout=10.0):
        request = Modes.Request()
        request.mode = 0
        request.enable = [enable] * 8
        if not self.control.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("driver control service unavailable")
        future = self.control.call_async(request)
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise RuntimeError("driver control call timed out")
            rclpy.spin_once(self, timeout_sec=0.002)
        return future.result().result


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

        print("1. First enable")
        node.max_gap = 0.0
        check(node.driver(True) == "Success", "driver enabled")
        node.spin_for(0.5)
        check(node.max_gap < MAX_STATE_GAP,
              f"largest state gap during the enable: {node.max_gap * 1000:.0f} ms "
              f"(limit {MAX_STATE_GAP * 1000:.0f} ms)")

        print("2. Disable while moving")
        node.spin_for(1.0, posref=HOME)
        far = list(HOME)
        far[0] = 1.5
        node.spin_for(1.0, posref=far)  # J1 accelerates for ~1 s
        q0 = node.state[0]
        start = time.monotonic()
        result = node.driver(False)
        call_time = time.monotonic() - start
        # Wait until J1 stops moving
        last, still_since = node.state[0], None
        while time.monotonic() - start < 5.0:
            node.spin_for(0.02)
            if abs(node.state[0] - last) < 1e-5:
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since > 0.2:
                    break
            else:
                still_since = None
            last = node.state[0]
        stop_time = (still_since or time.monotonic()) - start
        print(f"  info  disable call {result}, took {call_time * 1000:.0f} ms; "
              f"J1 went {node.state[0] - q0:+.3f} rad more and stopped "
              f"after {stop_time * 1000:.0f} ms")

        print("3. Re-enable at rest")
        node.spin_for(1.0)
        rest = list(node.state)
        check(node.driver(True) == "Success", "driver re-enabled")
        moved = 0.0
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)
            moved = max(moved, max(abs(a - b) for a, b in zip(node.state, rest)))
        check(moved < MAX_SELF_MOTION,
              f"arm moved {moved:.4f} rad by itself after the re-enable "
              f"(limit {MAX_SELF_MOTION})")

        print("4. posref at ~1 kHz, moving target, 20 s")
        node.spin_for(5.0, posref=HOME)
        t0 = time.monotonic()
        count = 0
        while time.monotonic() - t0 < 20.0:
            t = time.monotonic() - t0
            node.send([h + 0.4 * math.sin(2 * math.pi * 0.3 * t + i)
                       for i, h in enumerate(HOME)])
            count += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.0008)
        print(f"  info  {count} posref in 20 s ({count / 20:.0f} Hz)")
        node.spin_for(5.0, posref=HOME)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print("ROS-side checks:", "PASS" if not failures else f"FAIL ({len(failures)})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
