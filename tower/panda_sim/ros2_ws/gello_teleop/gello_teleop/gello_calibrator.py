#!/usr/bin/env python3
"""GELLO calibrator: turns GELLO joint angles into Panda commands.

Absolute mapping: each joint's target is the GELLO's angle, clipped to the
Panda's joint limits. Control only starts when the GELLO matches the Panda's
pose, so the robot doesn't jump. While disabled, it logs how to move each
joint that doesn't match yet, to guide the operator.

While enabled, the gripper follows the GELLO's trigger (0 open, 1 closed) as a
width command. The gripper service blocks until the move ends and queues
calls, so one call is sent at a time, and only when the width changed enough.
Optionally, pressing the trigger past a threshold sends a 'grasp' instead, to
hold an object with force. Enabling also needs the trigger to match the
gripper's width, so the gripper doesn't open and drop what it holds.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from impedance_control_msgs.msg import PosRef, State
from panda_topics_msgs_srvs.srv import Pandagripper
from std_msgs.msg import Bool, Float32
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

        self.gripper_value = None
        self.gripper_time = None
        self.gripper_width = None
        self.gripper_width_time = None
        self.gripper_width_sent = None
        self.gripper_grasping = False
        self.gripper_future = None
        self.gripper_cmd_sent = None
        self.gripper_call_time = None
        self.gripper_warned = False

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

        if self.gripper_service:
            self.create_subscription(
                Float32,
                self.gripper_topic,
                self.gripper_callback,
                10,
            )

            self.create_subscription(
                JointState,
                self.gripper_state_topic,
                self.gripper_state_callback,
                10,
            )

            self.gripper_client = self.create_client(
                Pandagripper, self.gripper_service
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
            # Empty gripper_service: the gripper isn't controlled
            "gripper_topic": "/gello/gripper",
            "gripper_service": "/sim_asimilar_01/panda_arm/gripper",
            "gripper_max_width": 0.08,
            "gripper_min_change": 0.005,
            "gripper_speed": 0.05,
            "gripper_call_timeout": 5.0,
            # Both the sim and the real driver publish the gripper as one
            # joint_states joint, at half the width (one finger)
            "gripper_state_topic": "/sim_asimilar_01/joint_states",
            "gripper_state_joint": "panda_j7",
            "gripper_state_timeout": 1.0,
            "gripper_enable_tolerance": 0.015,
            "gripper_grasp": False,
            "gripper_grasp_threshold": 0.9,
            "gripper_grasp_release": 0.8,
            "gripper_grasp_force": 20.0,
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
        self.gripper_topic = params["gripper_topic"]
        self.gripper_service = params["gripper_service"]
        self.gripper_max_width = params["gripper_max_width"]
        self.gripper_min_change = params["gripper_min_change"]
        self.gripper_speed = params["gripper_speed"]
        self.gripper_call_timeout = params["gripper_call_timeout"]
        self.gripper_state_topic = params["gripper_state_topic"]
        self.gripper_state_joint = params["gripper_state_joint"]
        self.gripper_state_timeout = params["gripper_state_timeout"]
        self.gripper_enable_tolerance = params["gripper_enable_tolerance"]
        self.gripper_grasp = params["gripper_grasp"]
        self.gripper_grasp_threshold = params["gripper_grasp_threshold"]
        self.gripper_grasp_release = params["gripper_grasp_release"]
        self.gripper_grasp_force = params["gripper_grasp_force"]

        for name in ("panda_joint_lower", "panda_joint_upper"):
            if len(params[name]) != len(JOINT_NAMES):
                raise ValueError(f"{name} needs {len(JOINT_NAMES)} values")

        if any(low >= high
               for low, high in zip(self.panda_lower, self.panda_upper)):
            raise ValueError("panda_joint_lower must be below upper")

        if self.enable_tolerance <= 0.0:
            raise ValueError("enable_tolerance must be positive")

        for name in ("gripper_max_width", "gripper_min_change",
                     "gripper_speed", "gripper_call_timeout",
                     "gripper_state_timeout", "gripper_enable_tolerance",
                     "gripper_grasp_force"):
            if params[name] <= 0.0:
                raise ValueError(f"{name} must be positive")

        if not (0.0 < self.gripper_grasp_release
                < self.gripper_grasp_threshold <= 1.0):
            raise ValueError(
                "Need 0 < gripper_grasp_release < gripper_grasp_threshold <= 1"
            )

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

    def gripper_callback(self, msg):
        if not math.isfinite(msg.data):
            return

        self.gripper_value = min(max(float(msg.data), 0.0), 1.0)
        self.gripper_time = time.monotonic()

    def gripper_state_callback(self, msg):
        # The sim shares this topic with the platform: look the joint up
        if self.gripper_state_joint not in msg.name:
            return

        index = msg.name.index(self.gripper_state_joint)
        if index >= len(msg.position) or not math.isfinite(msg.position[index]):
            return

        self.gripper_width = 2.0 * msg.position[index]
        self.gripper_width_time = time.monotonic()

    def trigger_width(self):
        return self.gripper_max_width * (1.0 - self.gripper_value)

    def gripper_check(self):
        """Whether the trigger lets control start: (ok, problem, hint)."""
        # No gripper control, or a GELLO without trigger
        if not self.gripper_service or self.gripper_time is None:
            return True, None, None

        if (self.gripper_width_time is None
                or time.monotonic() - self.gripper_width_time
                >= self.gripper_state_timeout):
            return (False,
                    f"No gripper width on {self.gripper_state_topic}.",
                    "gripper state unavailable")

        # A grasp only closes the gripper, so it can't drop anything
        if (self.gripper_grasp
                and self.gripper_value >= self.gripper_grasp_threshold):
            return True, None, None

        difference = self.trigger_width() - self.gripper_width
        if abs(difference) <= self.gripper_enable_tolerance:
            return True, None, None

        # A positive difference: the trigger asks for a wider gripper
        action = "press the trigger" if difference > 0 else "release the trigger"
        return (False,
                f"Trigger doesn't match the gripper: "
                f"{difference * 1000:+.0f} mm (tolerance "
                f"{self.gripper_enable_tolerance * 1000:.0f} mm).",
                f"gripper {abs(difference) * 1000:.0f} mm, {action}")

    def update_gripper(self):
        if not self.gripper_service:
            return

        now = time.monotonic()

        if self.gripper_future is not None:
            if self.gripper_future.done():
                self.gripper_done(self.gripper_future)
            elif now - self.gripper_call_time > self.gripper_call_timeout:
                # Never answered (gripper node gone?). Allow a new call.
                self.get_logger().warning("Gripper command timed out.")
                self.gripper_future = None
            else:
                return

        if not self.enabled or self.gripper_time is None:
            return

        if now - self.gripper_time >= self.data_timeout:
            return

        if not self.gripper_client.service_is_ready():
            if not self.gripper_warned:
                self.get_logger().warning(
                    f"Gripper service {self.gripper_service} unavailable."
                )
                self.gripper_warned = True
            return

        self.gripper_warned = False

        if self.gripper_grasp:
            if self.gripper_grasping:
                # Hold until the trigger is released past the hysteresis
                if self.gripper_value >= self.gripper_grasp_release:
                    return
                self.gripper_grasping = False
                self.gripper_width_sent = None
            elif self.gripper_value >= self.gripper_grasp_threshold:
                # Close on whatever is between the fingers: with width 0 and
                # epsilon_outer at full width, any final width succeeds
                self.send_gripper(
                    "grasp", 0.0, force=self.gripper_grasp_force,
                    epsilon_inner=0.005, epsilon_outer=self.gripper_max_width)
                self.gripper_grasping = True
                return

        width = self.trigger_width()

        if (self.gripper_width_sent is not None
                and abs(width - self.gripper_width_sent)
                < self.gripper_min_change):
            return

        self.send_gripper("move", width)
        self.gripper_width_sent = width

    def send_gripper(self, cmd, width, force=0.0, epsilon_inner=0.0,
                     epsilon_outer=0.0):
        request = Pandagripper.Request()
        request.cmd = cmd
        request.width = float(width)
        request.speed = float(self.gripper_speed)
        request.force = float(force)
        request.epsilon_inner = float(epsilon_inner)
        request.epsilon_outer = float(epsilon_outer)

        self.gripper_future = self.gripper_client.call_async(request)
        self.gripper_cmd_sent = cmd
        self.gripper_call_time = time.monotonic()

    def gripper_done(self, future):
        self.gripper_future = None

        # Not retried: on the real Panda 'move' fails when the fingers hit an
        # object, and retrying would repeat it. The next trigger change sends
        # a new command.
        response = future.result()
        if response is None or response.result != "Success":
            result = "no response" if response is None else response.result
            self.get_logger().warning(
                f"Gripper {self.gripper_cmd_sent} failed: {result}"
            )

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

        gripper_ok, gripper_problem, _ = self.gripper_check()
        if not gripper_ok:
            response.success = False
            response.message = gripper_problem
            return response

        self.enabled = True
        # The gripper may have moved while disabled: command it again
        self.gripper_width_sent = None
        self.gripper_grasping = False

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

        self.update_gripper()

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

        _, _, gripper_hint = self.gripper_check()
        if gripper_hint:
            moves.append(gripper_hint)

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
