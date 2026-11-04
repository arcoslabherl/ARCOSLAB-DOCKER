#!/usr/bin/env python3
"""Compute joint offsets for a GELLO unit (port of gello_software's
scripts/gello_get_offset.py).

Hold the GELLO in a known pose (--start-joints, robot joint angles in rad),
then run this tool. For each joint it finds the offset, a multiple of pi/2,
that maps the raw servo angle to that pose. It also measures the gripper's
open and closed angles.

Prints a YAML block to paste into config/gello_panda.yaml.
"""

import argparse
import math

from gello_teleop.backends import DynamixelBackend


NUM_JOINTS = 7
DEFAULT_START_JOINTS = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]

# Offsets are searched in pi/2 steps within +/- 8 pi
OFFSET_CANDIDATES = [
    -8.0 * math.pi + k * (math.pi / 2.0) for k in range(8 * 4 + 1)
]


def best_offset(raw, sign, start):
    return min(
        OFFSET_CANDIDATES,
        key=lambda offset: abs(sign * (raw - offset) - start),
    )


def read_average(backend, samples):
    readings = [backend.read() for _ in range(samples)]
    return [sum(values) / samples for values in zip(*readings)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=57600)
    parser.add_argument(
        "--joint-signs", type=int, nargs=NUM_JOINTS, required=True,
        help="1 or -1 per joint, e.g. 1 -1 1 1 1 -1 1")
    parser.add_argument(
        "--start-joints", type=float, nargs=NUM_JOINTS,
        default=DEFAULT_START_JOINTS,
        help="Pose the GELLO is held in, robot joint angles in rad")
    parser.add_argument(
        "--joint-ids", type=int, nargs=NUM_JOINTS,
        default=list(range(1, NUM_JOINTS + 1)))
    parser.add_argument(
        "--gripper-id", type=int, default=8,
        help="Gripper servo id, -1 if there is no gripper")
    args = parser.parse_args()

    if any(sign not in (-1, 1) for sign in args.joint_signs):
        parser.error("--joint-signs must be 1 or -1")

    servo_ids = list(args.joint_ids)
    if args.gripper_id >= 0:
        servo_ids.append(args.gripper_id)

    backend = DynamixelBackend(args.port, args.baudrate, servo_ids)

    try:
        for _ in range(10):  # warm up
            backend.read()
        raw = read_average(backend, 10)
    finally:
        backend.close()

    offsets = [
        best_offset(raw[i], args.joint_signs[i], args.start_joints[i])
        for i in range(NUM_JOINTS)
    ]

    print(f"Start joints : {args.start_joints}")
    print("Residual error per joint (rad, should be small):")
    for i, offset in enumerate(offsets):
        error = args.joint_signs[i] * (raw[i] - offset) - args.start_joints[i]
        print(f"  J{i + 1}: {error:+.3f}")

    print("\nPaste into config/gello_panda.yaml (gello_driver parameters):\n")
    print(f"    joint_ids: {args.joint_ids}")
    print("    joint_offsets: [" + ", ".join(
        f"{offset:.6f}" for offset in offsets) + "]")
    print("    # in pi/2 units: [" + ", ".join(
        str(round(offset / (math.pi / 2))) for offset in offsets) + "]")
    print(f"    joint_signs: {args.joint_signs}")

    if args.gripper_id >= 0:
        gripper_deg = math.degrees(raw[NUM_JOINTS])
        # Same convention as gello_software: the GELLO trigger must be
        # released (open) while running this tool.
        print(f"    gripper_id: {args.gripper_id}")
        print(f"    gripper_open_deg: {gripper_deg - 0.2:.2f}")
        print(f"    gripper_closed_deg: {gripper_deg - 42.0:.2f}")
    else:
        print("    gripper_id: -1")


if __name__ == "__main__":
    main()
