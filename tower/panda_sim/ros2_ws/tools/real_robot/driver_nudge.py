#!/usr/bin/env python3
"""Moves one joint a few degrees from where it is, and back, through the lab
driver (posref), as the GELLO will. The step after 'nudge' (libfranka alone).

  driver_nudge.py <joint 1-7> <degrees> [--ns /asimilar/panda_arm] [--yes]

Needs the driver running and enabled ('make real-driver', 'make
robot-enable'), and the GELLO disabled: both publish posref. The driver's
ruckig (0.8 rad/s, 1.0 rad/s^2, 50 rad/s^3) shapes the motion. At most 10
degrees, and only inside the joint limits minus 0.1 rad.
"""

import argparse
import math
import sys
import time

import rclpy
from impedance_control_msgs.msg import PosRef, State, Status
from rclpy.node import Node
from std_msgs.msg import Bool

MAX_DEGREES = 10.0
MARGIN = 0.1          # rad, as the calibrator's joint_limit_margin
STILL = 0.002         # rad, largest change for the arm to count as still
REACHED = 0.005       # rad, measured position close enough to the goal
MOVE_TIMEOUT = 10.0   # s, each way
HOLD = 1.0            # s, at the target
RATE = 20.0           # Hz, posref
GELLO_WATCH = 1.0     # s, listening for /gello/enabled after the driver shows up

# Franka Panda joint position limits (rad), as in the URDF and the calibrator
LOW = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
HIGH = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]


class DriverNudge(Node):
    def __init__(self, ns):
        super().__init__("driver_nudge")
        self.ns = ns
        # Created only after the checks: the calibrator disables the GELLO as
        # soon as another posref publisher appears, which would hide it here
        self.posref = None
        self.state = None
        self.status = None
        self.gello_enabled = False
        self.create_subscription(State, f"{ns}/state", self.on_state, 10)
        self.create_subscription(Status, f"{ns}/status", self.on_status, 10)
        self.create_subscription(Bool, "/gello/enabled", self.on_gello, 10)

    def on_state(self, msg):
        self.state = msg

    def on_status(self, msg):
        self.status = msg

    def on_gello(self, msg):
        self.gello_enabled = self.gello_enabled or msg.data

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def measured(self):
        return list(self.state.position)[:7]

    def commanded(self):
        return list(self.state.cmdedposition)[:7]

    def problem(self):
        """Why the nudge must stop now, or None."""
        if self.gello_enabled:
            return "the GELLO was enabled."
        if self.status is not None:
            if any(list(self.status.emergencystop)[:7]):
                return "the driver reports an emergency stop (reflex)."
            if not all(list(self.status.enabled)[:7]):
                return "the driver was disabled."
        return None

    def go(self, goal, j, seconds):
        """Publishes goal until the arm gets there and is still. Returns the
        largest move of the other joints, or a string with the problem."""
        start = self.measured()
        others = 0.0
        reached_at = None
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = PosRef()
            msg.position = [float(x) for x in goal]
            self.posref.publish(msg)
            self.spin_for(1.0 / RATE)
            problem = self.problem()
            if problem is not None:
                return problem
            q = self.measured()
            others = max([others] + [abs(q[i] - start[i]) for i in range(7) if i != j])
            if abs(q[j] - goal[j]) < REACHED:
                reached_at = reached_at or time.monotonic()
                if time.monotonic() - reached_at > HOLD:
                    return others
            else:
                reached_at = None
        return f"J{j + 1} didn't reach {math.degrees(goal[j]):.1f} deg in {seconds:.0f} s."


def refuse(message):
    print(f"NOT DONE: {message}")
    return 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("joint", type=int, choices=range(1, 8), metavar="joint")
    parser.add_argument("degrees", type=float)
    parser.add_argument("--ns", default="/asimilar/panda_arm")
    parser.add_argument("--yes", action="store_true", help="don't ask (for tests)")
    args = parser.parse_args()
    if args.degrees == 0.0 or abs(args.degrees) > MAX_DEGREES:
        return refuse(f"the angle must be between -{MAX_DEGREES:.0f} and "
                      f"{MAX_DEGREES:.0f} degrees, not 0.")
    j = args.joint - 1

    rclpy.init()
    node = DriverNudge(args.ns)
    try:
        # Discovery: wait for the driver, then listen a while longer for the
        # calibrator's /gello/enabled (20 Hz), which may arrive later
        end = time.monotonic() + 3.0
        while (node.state is None or node.status is None) and time.monotonic() < end:
            node.spin_for(0.1)
        node.spin_for(GELLO_WATCH)
        if node.gello_enabled:
            return refuse("the GELLO controls the arm. Run 'make gello-disable' first.")
        if node.state is None or node.status is None:
            return refuse(f"no driver on {args.ns}. Start it with 'make real-driver'.")
        problem = node.problem()
        if problem is not None:
            return refuse(problem.replace("was disabled", "is disabled: "
                                          "run 'make robot-enable' first"))

        first = node.measured()
        node.spin_for(0.5)
        moved = max(abs(a - b) for a, b in zip(first, node.measured()))
        if moved > STILL:
            return refuse(f"the arm moves ({moved:.4f} rad in 0.5 s). Wait until it's still.")

        start = node.commanded()   # what the driver holds now
        goal = list(start)
        goal[j] += math.radians(args.degrees)
        if not LOW[j] + MARGIN <= goal[j] <= HIGH[j] - MARGIN:
            return refuse(f"J{args.joint} would end at {math.degrees(goal[j]):.1f} deg, "
                          f"past its limit minus {math.degrees(MARGIN):.1f} deg.")

        print("Pose now (deg): " + " ".join(f"{math.degrees(x):6.1f}" for x in start))
        print(f"J{args.joint}: {math.degrees(start[j]):.1f} -> "
              f"{math.degrees(goal[j]):.1f} deg and back, through the driver. "
              "E-stop in hand.")
        if not args.yes:
            try:
                if input("Type 'yes' to start: ").strip() != "yes":
                    return refuse("cancelled.")
            except EOFError:
                return refuse("cancelled.")

        node.posref = node.create_publisher(PosRef, f"{args.ns}/posref", 10)
        node.spin_for(0.5)   # discovery, before the first message
        for name, target in (("out", goal), ("back", start)):
            result = node.go(target, j, MOVE_TIMEOUT)
            if isinstance(result, str):
                if name == "out" and node.problem() is None:
                    node.go(start, j, MOVE_TIMEOUT)   # try to return
                return refuse(result)
            print(f"{name}: reached; other joints moved at most "
                  f"{math.degrees(result):.2f} deg.")
        back = max(abs(a - b) for a, b in zip(node.measured(), start))
        print(f"Done. Back at the start within {math.degrees(back):.2f} deg (largest joint).")
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
