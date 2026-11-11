#!/usr/bin/env python3
"""Publishes one posref target for a while (default 10 Hz).

  python3 posref.py "<q1..q7>" <seconds> [--rate 10] [--ns /asimilar]
"""
import argparse
import time

import rclpy
from impedance_control_msgs.msg import PosRef
from rclpy.node import Node

parser = argparse.ArgumentParser()
parser.add_argument("pose")
parser.add_argument("duration", type=float)
parser.add_argument("--rate", type=float, default=10.0)
parser.add_argument("--ns", default="/asimilar")
args = parser.parse_args()
pose = [float(x) for x in args.pose.split()]
assert len(pose) == 7, "7 joint values"

rclpy.init()
node = Node("fake_franka_posref")
pub = node.create_publisher(PosRef, f"{args.ns}/panda_arm/posref", 10)
time.sleep(0.5)  # discovery
end = time.monotonic() + args.duration
while time.monotonic() < end:
    msg = PosRef()
    msg.position = pose
    pub.publish(msg)
    time.sleep(1.0 / args.rate)
node.destroy_node()
rclpy.try_shutdown()
