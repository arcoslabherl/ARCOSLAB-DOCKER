#!/usr/bin/env python3
"""GELLO driver: reads the GELLO servos and publishes joint angles.

Publishes:
  /gello/joint_states  sensor_msgs/JointState  (gello_j1..gello_j7, rad)
  /gello/gripper       std_msgs/Float32        (0.0 open .. 1.0 closed)

Conversion (same as gello_software's DynamixelRobot):
  joint   = sign * (raw - offset)
  gripper = clamp((raw - open) / (closed - open), 0, 1)
followed by exponential smoothing.
"""

import math
import threading
import time

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

from gello_teleop.backends import DynamixelBackend, FakeBackend


NUM_JOINTS = 7
JOINT_NAMES = [f"gello_j{i}" for i in range(1, NUM_JOINTS + 1)]

RATE_LOG_INTERVAL = 10.0  # seconds
READ_ERROR_LOG_INTERVAL = 2.0  # seconds


class GelloDriver(Node):
    def __init__(self):
        super().__init__("gello_driver")

        self.declare_parameter("backend", "dynamixel")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("joint_ids", list(range(1, NUM_JOINTS + 1)))
        self.declare_parameter("joint_offsets", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("joint_signs", [1] * NUM_JOINTS)
        self.declare_parameter("gripper_id", 8)
        self.declare_parameter("gripper_open_deg", 0.0)
        self.declare_parameter("gripper_closed_deg", 0.0)
        self.declare_parameter("smoothing_alpha", 0.99)
        self.declare_parameter("fake_rate", 200.0)
        self.declare_parameter(
            "fake_home", [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.71])
        self.declare_parameter("fake_amplitude", 0.1)
        self.declare_parameter("fake_frequency", 0.1)

        backend_name = self.get_parameter("backend").value
        port = self.get_parameter("port").value
        joint_ids = list(self.get_parameter("joint_ids").value)
        signs = list(self.get_parameter("joint_signs").value)
        gripper_id = self.get_parameter("gripper_id").value
        self.alpha = self.get_parameter("smoothing_alpha").value

        try:
            offsets = list(self.get_parameter("joint_offsets").value)
        except ParameterUninitializedException:
            offsets = None

        if len(joint_ids) != NUM_JOINTS or len(signs) != NUM_JOINTS:
            raise ValueError("joint_ids and joint_signs need 7 values")

        if any(sign not in (-1, 1) for sign in signs):
            raise ValueError("joint_signs must be 1 or -1")

        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in (0, 1]")

        if offsets is None:
            if backend_name != "fake":
                raise ValueError(
                    "joint_offsets not configured for this GELLO. "
                    "Set them in config/gello_panda.yaml "
                    "(see 'make gello-offsets')."
                )
            offsets = [0.0] * NUM_JOINTS

        if len(offsets) != NUM_JOINTS:
            raise ValueError("joint_offsets needs 7 values")

        self.offsets = offsets
        self.signs = signs

        self.has_gripper = gripper_id >= 0
        if self.has_gripper:
            self.gripper_open = math.radians(
                self.get_parameter("gripper_open_deg").value)
            self.gripper_closed = math.radians(
                self.get_parameter("gripper_closed_deg").value)
            if self.gripper_open == self.gripper_closed:
                if backend_name != "fake":
                    raise ValueError(
                        "gripper_open_deg and gripper_closed_deg not "
                        "configured. Set them in config/gello_panda.yaml "
                        "(see 'make gello-offsets') or set gripper_id: -1.")
                # Same 42 deg travel gello_get_offset assumes
                self.gripper_open = 0.0
                self.gripper_closed = math.radians(-42.0)
        else:
            self.gripper_open = self.gripper_closed = None

        servo_ids = joint_ids + ([gripper_id] if self.has_gripper else [])

        if backend_name == "dynamixel":
            baudrate = self.get_parameter("baudrate").value
            self.get_logger().info(
                f"Connecting to GELLO on {port} at {baudrate} baud...")
            self.backend = DynamixelBackend(port, baudrate, servo_ids)

        elif backend_name == "fake":
            self.get_logger().warning(
                "Using FAKE GELLO backend (no hardware).")
            self.backend = FakeBackend(
                rate=self.get_parameter("fake_rate").value,
                home=self.get_parameter("fake_home").value,
                amplitude=self.get_parameter("fake_amplitude").value,
                frequency=self.get_parameter("fake_frequency").value,
                joint_offsets=offsets,
                joint_signs=signs,
                gripper_open=self.gripper_open,
                gripper_closed=self.gripper_closed,
            )

        else:
            raise ValueError(
                f"Unknown backend '{backend_name}' (use dynamixel or fake)")

        self.joint_pub = self.create_publisher(
            JointState, "/gello/joint_states", 10)
        self.gripper_pub = self.create_publisher(
            Float32, "/gello/gripper", 10)

        self.last_values = None
        self.running = True

        # Serial reads block, so they run outside the ROS executor
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

        self.get_logger().info("GELLO driver ready.")

    def convert(self, raw):
        values = [
            sign * (angle - offset)
            for angle, offset, sign in zip(raw, self.offsets, self.signs)
        ]

        if self.has_gripper:
            gripper = (raw[NUM_JOINTS] - self.gripper_open) / (
                self.gripper_closed - self.gripper_open)
            values.append(min(max(gripper, 0.0), 1.0))

        if self.last_values is not None:
            values = [
                last * (1.0 - self.alpha) + value * self.alpha
                for last, value in zip(self.last_values, values)
            ]

        self.last_values = values
        return values

    def read_loop(self):
        published = 0
        rate_start = time.monotonic()
        last_error_log = 0.0

        while self.running and rclpy.ok():
            try:
                raw = self.backend.read()
            except (IOError, OSError) as error:
                # Stop publishing: the calibrator disables control on timeout
                now = time.monotonic()
                if now - last_error_log >= READ_ERROR_LOG_INTERVAL:
                    self.get_logger().warning(f"GELLO read failed: {error}")
                    last_error_log = now
                time.sleep(0.01)
                continue

            values = self.convert(raw)

            if not all(math.isfinite(value) for value in values):
                continue

            try:
                self.publish(values)
            except Exception:
                # Ctrl+C can shut ROS down between rclpy.ok() and publish
                if not rclpy.ok():
                    break
                raise

            published += 1
            elapsed = time.monotonic() - rate_start
            if elapsed >= RATE_LOG_INTERVAL:
                self.get_logger().info(
                    f"Publishing at {published / elapsed:.1f} Hz")
                published = 0
                rate_start = time.monotonic()

    def publish(self, values):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = JOINT_NAMES
        joint_msg.position = values[:NUM_JOINTS]
        self.joint_pub.publish(joint_msg)

        if self.has_gripper:
            gripper_msg = Float32()
            gripper_msg.data = float(values[NUM_JOINTS])
            self.gripper_pub.publish(gripper_msg)

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.backend.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = None

    try:
        node = GelloDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
