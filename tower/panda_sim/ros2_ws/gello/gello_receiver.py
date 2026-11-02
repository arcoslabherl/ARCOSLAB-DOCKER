#!/usr/bin/env python3

import json
import math
import socket
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32


UDP_HOST = "127.0.0.1"
UDP_PORT = 15010


class GelloReceiver(Node):
    def __init__(self):
        super().__init__("gello_receiver")

        self.joint_pub = self.create_publisher(
            JointState,
            "/gello/joint_states",
            10,
        )

        self.gripper_pub = self.create_publisher(
            Float32,
            "/gello/gripper",
            10,
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_HOST, UDP_PORT))
        self.sock.setblocking(False)

        self.received = 0
        self.discarded = 0
        self.latencies_ms = []

        # Check for new packets every 5 ms
        self.timer = self.create_timer(0.005, self.receive_data)

        self.get_logger().info("GELLO receiver ready.")

    def receive_data(self):
        latest_data = None
        pending = 0

        # Drain queued UDP packets, keeping only the newest
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break

            latest_data = data
            pending += 1

        if latest_data is None:
            return

        self.discarded += max(0, pending - 1)

        try:
            payload = json.loads(latest_data.decode("utf-8"))

            joints = [float(value) for value in payload["joints"]]
            gripper = float(payload["gripper"])
            sample_time_ns = int(payload["sample_time_ns"])

            if len(joints) != 7:
                return

            if not all(math.isfinite(value) for value in joints):
                return

            if not math.isfinite(gripper) or not 0.0 <= gripper <= 1.0:
                return

        except (ValueError, TypeError, KeyError):
            self.get_logger().warning("Invalid GELLO packet.")
            return

        age_ms = (time.monotonic_ns() - sample_time_ns) / 1_000_000

        if age_ms < 0:
            self.get_logger().warning("Invalid packet timestamp.")
            return

        self.latencies_ms.append(age_ms)
        self.received += 1

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [f"gello_j{i}" for i in range(1, 8)]
        joint_msg.position = joints

        self.joint_pub.publish(joint_msg)

        gripper_msg = Float32()
        gripper_msg.data = gripper
        self.gripper_pub.publish(gripper_msg)

        if self.received % 100 == 0:
            samples = sorted(self.latencies_ms)
            p95_index = int(0.95 * (len(samples) - 1))

            avg_ms = sum(samples) / len(samples)
            p95_ms = samples[p95_index]

            self.get_logger().info(
                f"Received: {self.received} | "
                f"Average age: {avg_ms:.2f} ms | "
                f"P95: {p95_ms:.2f} ms | "
                f"Discarded: {self.discarded}"
            )

            self.latencies_ms.clear()

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = GelloReceiver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()