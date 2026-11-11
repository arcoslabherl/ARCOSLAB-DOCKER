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

It also watches the robot's status (enabled, emergency stop): control only
runs while the robot reports itself enabled and without emergency. After a
reflex, the real driver keeps publishing the frozen state, so data timeouts
alone don't notice it, and re-enabling the driver would move the arm straight
to wherever the GELLO is by then.
"""

import math
import time

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import JointState
from impedance_control_msgs.msg import PosRef, State, Status
from panda_topics_msgs_srvs.srv import Pandagripper
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

from gello_teleop import safety
from gello_teleop.gripper import GripperControl


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

        self.robot_problem = None
        self.robot_status_time = None

        # Gripper service calls in progress (the decisions are in GripperControl)
        self.gripper_future = None
        self.gripper_cmd_sent = None
        self.gripper_call_time = None
        self.gripper_warned = False

        self.load_parameters()

        self.gripper = GripperControl(
            max_width=self.gripper_max_width,
            min_change=self.gripper_min_change,
            enable_tolerance=self.gripper_enable_tolerance,
            state_timeout=self.gripper_state_timeout,
            state_topic=self.gripper_state_topic,
            grasp=self.gripper_grasp,
            grasp_threshold=self.gripper_grasp_threshold,
            grasp_release=self.gripper_grasp_release,
            grasp_force=self.gripper_grasp_force,
        )

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

        if self.panda_status_topic:
            self.create_subscription(
                Status,
                self.panda_status_topic,
                self.status_callback,
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
        # Values come only from config/gello_panda.yaml (no defaults here,
        # so there's a single place to change them).
        STRING = Parameter.Type.STRING
        DOUBLE = Parameter.Type.DOUBLE
        DOUBLES = Parameter.Type.DOUBLE_ARRAY
        types = {
            "gello_topic": STRING,
            "panda_state_topic": STRING,
            "panda_status_topic": STRING,   # "": status not checked
            "status_timeout": DOUBLE,
            "target_topic": STRING,
            "posref_topic": STRING,
            "panda_joint_lower": DOUBLES,
            "panda_joint_upper": DOUBLES,
            "enable_tolerance": DOUBLE,
            "max_sample_change": DOUBLE,
            "data_timeout": DOUBLE,
            "status_interval": DOUBLE,
            "gripper_topic": STRING,
            "gripper_service": STRING,
            "gripper_max_width": DOUBLE,
            "gripper_min_change": DOUBLE,
            "gripper_speed": DOUBLE,
            "gripper_call_timeout": DOUBLE,
            "gripper_state_topic": STRING,
            "gripper_state_joint": STRING,
            "gripper_state_timeout": DOUBLE,
            "gripper_enable_tolerance": DOUBLE,
            "gripper_grasp": Parameter.Type.BOOL,
            "gripper_grasp_threshold": DOUBLE,
            "gripper_grasp_release": DOUBLE,
            "gripper_grasp_force": DOUBLE,
        }

        params = {}
        for name, param_type in types.items():
            self.declare_parameter(name, param_type)
            try:
                params[name] = self.get_parameter(name).value
            except ParameterUninitializedException:
                raise ValueError(
                    f"Parameter '{name}' not set. Launch with "
                    "config/gello_panda.yaml (make gello / gello-fake)."
                ) from None

        self.gello_topic = params["gello_topic"]
        self.panda_state_topic = params["panda_state_topic"]
        self.panda_status_topic = params["panda_status_topic"]
        self.status_timeout = params["status_timeout"]
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

        for name in ("status_timeout", "gripper_max_width",
                     "gripper_min_change",
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
        positions = safety.valid_joints(msg.position)
        if positions is None:
            return

        commanded = safety.valid_joints(msg.cmdedposition) or positions

        self.panda_position = positions
        self.panda_commanded = commanded
        self.panda_time = time.monotonic()

    def status_callback(self, msg):
        self.robot_problem = safety.robot_problem(
            list(msg.enabled), list(msg.emergencystop))
        self.robot_status_time = time.monotonic()

    def robot_check(self):
        """Whether the robot's status lets control run: (ok, problem)."""
        if not self.panda_status_topic:
            return True, None

        return safety.robot_ready(
            self.robot_problem, self.robot_status_time, time.monotonic(),
            self.status_timeout, self.panda_status_topic)

    def gello_callback(self, msg):
        positions = safety.valid_joints(msg.position)
        if positions is None:
            return

        jumped = safety.joint_jumped(
            positions, self.gello_position, self.max_sample_change)

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

        targets = safety.clip_to_limits(
            positions, self.panda_lower, self.panda_upper)

        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = JOINT_NAMES
        output.position = targets

        self.target_pub.publish(output)
        self.publish_posref(targets)

    def gripper_callback(self, msg):
        if not math.isfinite(msg.data):
            return

        self.gripper.set_trigger(msg.data, time.monotonic())

    def gripper_state_callback(self, msg):
        # The sim shares this topic with the platform: look the joint up
        if self.gripper_state_joint not in msg.name:
            return

        index = msg.name.index(self.gripper_state_joint)
        if index >= len(msg.position) or not math.isfinite(msg.position[index]):
            return

        # One finger: half the width
        self.gripper.set_width(2.0 * msg.position[index], time.monotonic())

    def gripper_check(self):
        """Whether the trigger lets control start: (ok, problem, hint)."""
        if not self.gripper_service:
            return True, None, None
        return self.gripper.enable_check(time.monotonic())

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

        if not self.enabled or not self.gripper.trigger_fresh(
                now, self.data_timeout):
            return

        if not self.gripper_client.service_is_ready():
            if not self.gripper_warned:
                self.get_logger().warning(
                    f"Gripper service {self.gripper_service} unavailable."
                )
                self.gripper_warned = True
            return

        self.gripper_warned = False

        command = self.gripper.next_command()
        if command is not None:
            self.send_gripper(command)

    def send_gripper(self, command):
        request = Pandagripper.Request()
        request.cmd = command.cmd
        request.width = float(command.width)
        request.speed = float(self.gripper_speed)
        request.force = float(command.force)
        request.epsilon_inner = float(command.epsilon_inner)
        request.epsilon_outer = float(command.epsilon_outer)

        self.gripper_future = self.gripper_client.call_async(request)
        self.gripper_cmd_sent = command.cmd
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

    def pose_differences(self):
        return safety.pose_differences(
            self.gello_position, self.panda_position,
            self.panda_lower, self.panda_upper)

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

    def stale_sources(self):
        """Which inputs are missing or older than data_timeout, and why."""
        return safety.stale_sources(
            [("GELLO", self.gello_topic, self.gello_time),
             ("Panda state", self.panda_state_topic, self.panda_time)],
            time.monotonic(), self.data_timeout)

    def data_fresh(self):
        return not self.stale_sources()

    def enable_callback(self, request, response):
        if not request.data:
            if self.enabled:
                self.disable()

            response.success = True
            response.message = "GELLO control disabled."
            return response

        stale = self.stale_sources()
        if stale:
            response.success = False
            response.message = f"Data unavailable: {'; '.join(stale)}."
            return response

        robot_ok, robot_problem = self.robot_check()
        if not robot_ok:
            response.success = False
            response.message = (
                f"Panda not ready: {robot_problem}. Enable the robot first."
            )
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
        self.gripper.reset()

        response.success = True
        response.message = "GELLO control enabled."
        return response

    def check_safety(self):
        stale = self.stale_sources() if self.enabled else []
        if stale:
            self.disable()

            self.get_logger().warning(
                f"Communication timeout: {'; '.join(stale)}. GELLO disabled."
            )

        robot_ok, robot_problem = self.robot_check()
        if self.enabled and not robot_ok:
            self.disable()

            self.get_logger().error(
                f"Panda not ready: {robot_problem}. GELLO disabled. "
                "Re-enable the robot, match its pose, then enable again."
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

        # Until the robot is enabled its pose may be meaningless (the real
        # driver reports zeros before the first enable)
        robot_ok, robot_problem = self.robot_check()
        if not robot_ok:
            self.get_logger().info(
                f"Panda not ready: {robot_problem}. Enable the robot first."
            )
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
