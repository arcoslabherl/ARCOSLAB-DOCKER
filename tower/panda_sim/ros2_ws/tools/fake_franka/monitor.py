#!/usr/bin/env python3
"""Watches the driver for N seconds: gaps in panda_arm/state, changes of
the reported position, of status.enabled / emergencystop and of
/gello/enabled.

  python3 monitor.py <seconds> [--ns /asimilar]
"""

import argparse
import time

import rclpy
from impedance_control_msgs.msg import State, Status
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("duration", type=float)
    parser.add_argument("--ns", default="/asimilar")
    args = parser.parse_args()

    rclpy.init()
    node = Node("fake_franka_monitor")
    t0 = time.monotonic()
    state = {"last": None, "gaps": [], "pos": None, "events": [], "count": 0,
             "status": None, "gello": None}

    def now():
        return round(time.monotonic() - t0, 3)

    def on_state(msg):
        t = time.monotonic()
        if state["last"] is not None:
            state["gaps"].append((t - state["last"], now()))
        state["last"] = t
        state["count"] += 1
        pos = tuple(round(x, 3) for x in msg.position)
        if pos != state["pos"]:
            if state["pos"] is None or all(x == 0.0 for x in pos) or \
                    all(x == 0.0 for x in state["pos"]):
                state["events"].append((now(), "state position", pos))
            state["pos"] = pos

    def on_status(msg):
        value = (all(msg.enabled), any(msg.emergencystop))
        if value != state["status"]:
            state["events"].append(
                (now(), "status", f"enabled={value[0]} emergency={value[1]}"))
            state["status"] = value

    def on_gello(msg):
        if msg.data != state["gello"]:
            state["events"].append((now(), "/gello/enabled", msg.data))
            state["gello"] = msg.data

    node.create_subscription(State, f"{args.ns}/panda_arm/state", on_state, 100)
    node.create_subscription(Bool, "/gello/enabled", on_gello, 10)
    node.create_subscription(Status, f"{args.ns}/panda_arm/status", on_status, 100)
    while time.monotonic() - t0 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    gaps = sorted(state["gaps"], reverse=True)[:3]
    print(f"[monitor] state messages: {state['count']}, largest gaps: "
          + ", ".join(f"{g * 1000:.0f} ms at t={t}" for g, t in gaps))
    for t, kind, value in state["events"]:
        print(f"[monitor] t={t:7.3f}  {kind}: {value}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
