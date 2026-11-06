#!/usr/bin/env python3
"""GELLO calibrator: turns GELLO joint angles into Panda commands.

Absolute mapping: each joint's target is the GELLO's angle, clipped to the
Panda's joint limits. Control only starts when the GELLO matches the Panda's
pose, so the robot doesn't jump. While disabled, it logs how to move each
joint that doesn't match yet, to guide the operator.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from impedance_control_msgs.msg import PosRef, State
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


JOINT_NAMES = [f"panda_j{i}" for i in range(7)]

# How to move each GELLO joint to make its angle (grow, shrink). Checked
# against the URDF. "From above" assumes the upper arm is roughly vertical.
MOVE_HINTS = [
    ("counterclockwise from above", "clockwise from above"),
    ("tilt the arm forward", "tilt the arm backward"),
    ("counterclockwise from above", "clockwise from above"),
    ("open the elbow", "close the elbow"),
    ("clockwise from the elbow", "counterclockwise from the elbow"),
    ("swing the hand away from the elbow", "swing the hand toward the elbow"),
    ("clockwise from the wrist", "counterclockwise from the wrist"),
]

# All tunable values (topics, Panda joint limits, tolerances, timeouts) are
# ROS parameters, set in config/gello_panda.yaml.


class GelloCalibrator(Node):
    def __init__(self):
        super().__init__("gello_calibrator")

        self.enabled = False

        self.panda_position = None
        self.panda_commanded = None
        self.panda_time = None

        self.gello_position = None
        self.gello_time = None

        self.load_parameters()

        self.target_pub = self.create_publisher(
            JointState, self.target_topic, 10
        )

        self.posref_pub = self.create_publisher(
            PosRef, self.posref_topic, 10
        )

        self.enabled_pub = self.create_publisher(
            Bool, "/gello/enabled", 10
        )

        self.create_subscription(
            JointState,
            self.gello_topic,
            self.gello_callback,
            10,
        )

        self.create_subscription(
            State,
            self.panda_state_topic,
            self.panda_callback,
            10,
        )

        self.create_service(
            SetBool,
            "/gello/enable",
            self.enable_callback,
        )

        self.create_timer(0.05, self.check_safety)
        self.create_timer(self.status_interval, self.log_status)

        self.get_logger().info(
            "GELLO calibrator ready. Control DISABLED. Move the GELLO to the "
            "Panda's pose, then enable."
        )

    def load_parameters(self):
        defaults = {
            "gello_topic": "/gello/joint_states",
            "panda_state_topic": "/sim_asimilar_01/panda_arm/state",
            "target_topic": "/gello/target_joint_states",
            "posref_topic": "/sim_asimilar_01/panda_arm/posref",
            # Franka Panda joint position limits (rad), J1..J7
            "panda_joint_lower":
                [-2.8973, -1.7628, -2.8973, -3.0718,
                 -2.8973, -0.0175, -2.8973],
            "panda_joint_upper":
                [2.8973, 1.7628, 2.8973, -0.0698,
                 2.8973, 3.7525, 2.8973],
            "enable_tolerance": 0.1,
            "max_sample_change": 0.5,
            "data_timeout": 0.3,
            "status_interval": 0.5,
        }

        params = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            params[name] = self.get_parameter(name).value

        self.gello_topic = params["gello_topic"]
        self.panda_state_topic = params["panda_state_topic"]
        self.target_topic = params["target_topic"]
        self.posref_topic = params["posref_topic"]
        self.panda_lower = list(params["panda_joint_lower"])
        self.panda_upper = list(params["panda_joint_upper"])
        self.enable_tolerance = params["enable_tolerance"]
        self.max_sample_change = params["max_sample_change"]
        self.data_timeout = params["data_timeout"]
        self.status_interval = params["status_interval"]

        for name in ("panda_joint_lower", "panda_joint_upper"):
            if len(params[name]) != len(JOINT_NAMES):
                raise ValueError(f"{name} needs {len(JOINT_NAMES)} values")

        if any(low >= high
               for low, high in zip(self.panda_lower, self.panda_upper)):
            raise ValueError("panda_joint_lower must be below upper")

        if self.enable_tolerance <= 0.0:
            raise ValueError("enable_tolerance must be positive")

    def panda_callback(self, msg):
        if len(msg.position) != 7:
            return

        positions = list(msg.position)

        if not all(math.isfinite(x) for x in positions):
            return

        commanded = list(msg.cmdedposition)
        if len(commanded) != 7 or not all(math.isfinite(x) for x in commanded):
            commanded = positions

        self.panda_position = positions
        self.panda_commanded = commanded
        self.panda_time = time.monotonic()

    def gello_callback(self, msg):
        if len(msg.position) != 7:
            return

        positions = list(msg.position)

        if not all(math.isfinite(x) for x in positions):
            return

        jumped = self.gello_position is not None and any(
            abs(current - previous) > self.max_sample_change
            for current, previous in zip(positions, self.gello_position)
        )

        self.gello_position = positions
        self.gello_time = time.monotonic()

        if jumped:
            self.get_logger().warning("Unexpected GELLO joint jump.")
            if self.enabled:
                self.disable()
                self.get_logger().error(
                    "GELLO disabled. Match the Panda's pose to enable again."
                )
            return

        if not self.enabled:
            return

        targets = self.clip_to_limits(positions)

        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = JOINT_NAMES
        output.position = targets

        self.target_pub.publish(output)
        self.publish_posref(targets)

    def clip_to_limits(self, positions):
        return [
            max(low, min(high, position))
            for position, low, high in zip(
                positions, self.panda_lower, self.panda_upper
            )
        ]

    def pose_differences(self):
        # What the Panda would be commanded minus where it is, per joint
        return [
            target - actual
            for target, actual in zip(
                self.clip_to_limits(self.gello_position), self.panda_position
            )
        ]

    def publish_posref(self, position):
        command = PosRef()
        command.position = [float(value) for value in position]
        self.posref_pub.publish(command)

    def other_commander(self):
        # count_publishers includes this node's own publisher. Two sources
        # on posref would fight over the robot's goal.
        return self.count_publishers(self.posref_topic) > 1

    def disable(self):
        self.enabled = False

        # The robot keeps moving to the last goal it received, so make it
        # stop where it is now. Its trajectory generator brakes smoothly.
        if self.panda_commanded is not None:
            self.publish_posref(self.panda_commanded)

    def data_fresh(self):
        now = time.monotonic()

        if self.gello_time is None or self.panda_time is None:
            return False

        return (
            now - self.gello_time < self.data_timeout
            and now - self.panda_time < self.data_timeout
        )

    def enable_callback(self, request, response):
        if not request.data:
            if self.enabled:
                self.disable()

            response.success = True
            response.message = "GELLO control disabled."
            return response

        if not self.data_fresh():
            response.success = False
            response.message = "GELLO or Panda data unavailable."
            return response

        if self.other_commander():
            response.success = False
            response.message = (
                f"Another node publishes {self.posref_topic}. "
                "Start the simulator with 'make sim-teleop' (no RBC)."
            )
            return response

        far = [
            f"J{index} {math.degrees(difference):+.1f} deg"
            for index, difference in enumerate(self.pose_differences(), start=1)
            if abs(difference) > self.enable_tolerance
        ]
        if far:
            response.success = False
            response.message = (
                f"GELLO doesn't match the Panda: {', '.join(far)} "
                f"(tolerance {math.degrees(self.enable_tolerance):.1f} deg)."
            )
            return response

        self.enabled = True

        response.success = True
        response.message = "GELLO control enabled."
        return response

    def check_safety(self):
        if self.enabled and not self.data_fresh():
            self.disable()

            self.get_logger().warning(
                "Communication timeout. GELLO disabled."
            )

        if self.enabled and self.other_commander():
            self.disable()

            self.get_logger().error(
                f"Another node publishes {self.posref_topic}. GELLO disabled."
            )

        status = Bool()
        status.data = self.enabled
        self.enabled_pub.publish(status)

    def log_status(self):
        if not self.data_fresh():
            return

        if self.enabled:
            at_limit = [
                f"J{index}"
                for index, (position, low, high) in enumerate(zip(
                    self.gello_position, self.panda_lower, self.panda_upper
                ), start=1)
                if not low <= position <= high
            ]
            if at_limit:
                self.get_logger().warning(
                    f"At the Panda's joint limit: {', '.join(at_limit)}"
                )
            return

        moves = []
        for index, difference in enumerate(self.pose_differences()):
            if abs(difference) > self.enable_tolerance:
                # A positive difference: the GELLO's angle must shrink
                hint = MOVE_HINTS[index][1 if difference > 0 else 0]
                moves.append(
                    f"J{index + 1} {abs(math.degrees(difference)):.0f} deg, "
                    f"{hint}"
                )

        if moves:
            self.get_logger().info(f"Move the GELLO: {' | '.join(moves)}")
        else:
            self.get_logger().info("GELLO matches the Panda: READY to enable")


def main():
    rclpy.init()
    node = GelloCalibrator()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
