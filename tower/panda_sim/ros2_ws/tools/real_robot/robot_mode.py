#!/usr/bin/env python3
"""Enables or disables the lab driver (its 'control' service).

  robot_mode.py enable|disable [--ns /asimilar/panda_arm] [--yes]

The driver reads 8 enable values (7 joints and the gripper) without checking
how many came, so this always sends 8, with mode 0.

disable refuses while the GELLO controls the arm or while the arm moves. With
the lab's original driver a disable freezes the command in one cycle (a
reflex on the real robot), and the next enable starts with the stale velocity:
the arm then moves by itself, up to 0.32 rad at 0.8 rad/s. Both found with the
fake libfranka.

Works the same on the real robot and on the fake libfranka.
"""

import argparse
import sys
import time

import rclpy
from impedance_control_msgs.msg import State, Status
from impedance_control_msgs.srv import Modes
from rclpy.node import Node
from std_msgs.msg import Bool

ENABLE_VALUES = 8          # 7 joints + gripper, as the driver reads them
STILL = 0.002              # rad, largest change for the arm to count as still
WATCH = 0.5                # s, how long the arm is watched
GELLO_WATCH = 0.5          # s (the calibrator publishes /gello/enabled at 20 Hz)
CALL_TIMEOUT = 10.0        # s


class RobotMode(Node):
    def __init__(self, ns):
        super().__init__("robot_mode")
        self.ns = ns
        self.control = self.create_client(Modes, f"{ns}/control")
        self.gello_enabled = []
        self.states = []
        self.status = None
        self.create_subscription(
            Bool, "/gello/enabled", lambda m: self.gello_enabled.append(m.data), 10)
        self.create_subscription(
            State, f"{ns}/state", lambda m: self.states.append(m), 10)
        self.create_subscription(
            Status, f"{ns}/status", self.on_status, 10)

    def on_status(self, msg):
        self.status = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def arm_motion(self):
        """Largest change of the measured or commanded position while
        watching, or None without enough state messages."""
        self.states = []
        self.spin_for(WATCH)
        if len(self.states) < 2:
            return None
        motion = 0.0
        for field in ("position", "cmdedposition"):
            columns = zip(*[list(getattr(s, field))[:7] for s in self.states])
            motion = max([motion] + [max(c) - min(c) for c in columns])
        return motion

    def call(self, enable):
        request = Modes.Request()
        request.enable = [enable] * ENABLE_VALUES
        request.mode = 0
        future = self.control.call_async(request)
        end = time.monotonic() + CALL_TIMEOUT
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None


def refuse(message):
    print(f"NOT DONE: {message}")
    return 1


def confirm(lines, assume_yes):
    for line in lines:
        print(line)
    if assume_yes:
        return True
    try:
        return input("Type 'yes' to continue: ").strip() == "yes"
    except EOFError:
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["enable", "disable"])
    parser.add_argument("--ns", default="/asimilar/panda_arm")
    parser.add_argument("--yes", action="store_true",
                        help="don't ask (for tests)")
    args = parser.parse_args()

    rclpy.init()
    node = RobotMode(args.ns)
    try:
        if not node.control.wait_for_service(timeout_sec=3.0):
            return refuse(f"no driver: {args.ns}/control doesn't answer. "
                          "Start it with 'make real-driver'.")

        if args.action == "disable":
            node.spin_for(GELLO_WATCH)
            if any(node.gello_enabled):
                return refuse("the GELLO controls the arm. "
                              "Run 'make gello-disable' first.")
            motion = node.arm_motion()
            if motion is None:
                return refuse(f"no state on {args.ns}/state: can't tell "
                              "whether the arm moves.")
            if motion > STILL:
                return refuse(f"the arm moves ({motion:.4f} rad in "
                              f"{WATCH} s). Wait until it's still.")
        else:
            if not confirm([
                "The driver starts controlling the arm: it holds its current",
                "pose (no motion expected). E-stop in hand.",
            ], args.yes):
                return refuse("cancelled.")

        response = node.call(args.action == "enable")
        if response is None:
            return refuse(f"no answer from {args.ns}/control in "
                          f"{CALL_TIMEOUT:.0f} s.")
        print(f"Driver: {response.result}, enabled={list(response.enabled)}")

        node.status = None
        node.spin_for(0.5)
        if node.status is not None:
            print(f"Status: enabled={list(node.status.enabled)}, "
                  f"emergency stop={list(node.status.emergencystop)}")
        if args.action == "enable":
            print("Next: match the GELLO's pose, then 'make gello-enable'.")
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
