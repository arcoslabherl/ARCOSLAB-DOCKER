#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from impedance_control_msgs.msg import State
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool


JOINT_NAMES = [f"panda_j{i}" for i in range(7)]

# All tunable values (topics, travel limits, directions, Panda joint
# limits, timeouts) are ROS parameters, set in config/gello_panda.yaml.


class GelloCalibrator(Node):
    def __init__(self):
        super().__init__("gello_calibrator")

        self.enabled = False
        self.calibrated = False

        self.panda_position = None
        self.panda_time = None
        self.panda_stable_since = None

        self.gello_previous = None
        self.gello_unwrapped = None
        self.gello_time = None

        self.gello_initial = None
        self.panda_initial = None
        self.last_debug_time = 0.0

        self.load_parameters()

        self.target_pub = self.create_publisher(
            JointState, self.target_topic, 10
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
            Trigger,
            "/gello/recenter",
            self.recenter_callback,
        )

        self.create_service(
            SetBool,
            "/gello/enable",
            self.enable_callback,
        )

        self.create_timer(0.05, self.check_safety)

        self.get_logger().info(
            "GELLO calibrator ready. Control DISABLED."
        )
        for index, limit in enumerate(self.max_displacements, start=1):
            self.get_logger().info(
                f"J{index}: +/-{limit:.3f} rad "
                f"(+/-{math.degrees(limit):.1f} deg, "
                f"{2 * math.degrees(limit):.1f} deg total), "
                f"direction={self.joint_directions[index - 1]:+d}"
            )

    def load_parameters(self):
        defaults = {
            "gello_topic": "/gello/joint_states",
            "panda_state_topic": "/sim_asimilar_01/panda_arm/state",
            "target_topic": "/gello/target_joint_states",
            # Symmetric travel allowed from the recenter pose, per joint
            "max_joint_displacement_deg":
                [55.0, 45.0, 80.0, 14.3239, 80.0, 80.0, 270.0],
            # 1 or -1 per joint; flip a joint that moves the wrong way
            "joint_directions": [1, 1, 1, 1, 1, 1, 1],
            # Franka Panda joint position limits (rad), J1..J7
            "panda_joint_lower":
                [-2.8973, -1.7628, -2.8973, -3.0718,
                 -2.8973, -0.0175, -2.8973],
            "panda_joint_upper":
                [2.8973, 1.7628, 2.8973, -0.0698,
                 2.8973, 3.7525, 2.8973],
            # 1-7 logs that joint once per debug_interval, 0 disables
            "debug_joint": 7,
            "debug_interval": 1.0,
            "max_sample_change": 0.5,
            "data_timeout": 0.3,
            "panda_stable_time": 0.5,
            "panda_stable_tolerance": 0.01,
        }

        params = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            params[name] = self.get_parameter(name).value

        self.gello_topic = params["gello_topic"]
        self.panda_state_topic = params["panda_state_topic"]
        self.target_topic = params["target_topic"]

        self.max_displacements = [
            math.radians(value)
            for value in params["max_joint_displacement_deg"]
        ]
        self.joint_directions = list(params["joint_directions"])
        self.panda_lower = list(params["panda_joint_lower"])
        self.panda_upper = list(params["panda_joint_upper"])

        self.debug_joint = params["debug_joint"]
        self.debug_interval = params["debug_interval"]
        self.max_sample_change = params["max_sample_change"]
        self.data_timeout = params["data_timeout"]
        self.panda_stable_time = params["panda_stable_time"]
        self.panda_stable_tolerance = params["panda_stable_tolerance"]

        for name in ("max_joint_displacement_deg", "joint_directions",
                     "panda_joint_lower", "panda_joint_upper"):
            if len(params[name]) != len(JOINT_NAMES):
                raise ValueError(f"{name} needs {len(JOINT_NAMES)} values")

        if not all(math.isfinite(x) and x > 0
                   for x in self.max_displacements):
            raise ValueError(
                "max_joint_displacement_deg must be finite positive values")

        if any(d not in (-1, 1) for d in self.joint_directions):
            raise ValueError("joint_directions must be 1 or -1")

        if any(low >= high
               for low, high in zip(self.panda_lower, self.panda_upper)):
            raise ValueError("panda_joint_lower must be below upper")

    def panda_callback(self, msg):
        if len(msg.position) != 7:
            return

        positions = list(msg.position)

        if not all(math.isfinite(x) for x in positions):
            return

        now = time.monotonic()

        if self.panda_position is None:
            self.panda_stable_since = now
        else:
            difference = max(
                abs(a - b)
                for a, b in zip(positions, self.panda_position)
            )

            if difference > self.panda_stable_tolerance:
                self.panda_stable_since = now

        self.panda_position = positions
        self.panda_time = now

    def gello_callback(self, msg):
        if len(msg.position) != 7:
            return

        positions = list(msg.position)

        if not all(math.isfinite(x) for x in positions):
            return

        if self.gello_previous is None:
            self.gello_previous = positions.copy()
            self.gello_unwrapped = positions.copy()

        else:
            deltas = []

            for current, previous in zip(
                positions, self.gello_previous
            ):
                raw_delta = current - previous

                delta = math.atan2(
                    math.sin(raw_delta),
                    math.cos(raw_delta),
                )

                if abs(delta) > self.max_sample_change:
                    self.get_logger().warning(
                        "Unexpected GELLO joint jump."
                    )
                    return

                deltas.append(delta)

            self.gello_unwrapped = [
                previous + delta
                for previous, delta in zip(
                    self.gello_unwrapped, deltas
                )
            ]

            self.gello_previous = positions.copy()

        self.gello_time = time.monotonic()

        if not self.enabled or not self.calibrated:
            return

        targets = []
        debug_info = None

        for index, (panda_home, current, gello_home) in enumerate(zip(
            self.panda_initial,
            self.gello_unwrapped,
            self.gello_initial,
        )):
            # Relative teleoperation: keep the recenter behavior unchanged.
            displacement = (current - gello_home) * self.joint_directions[index]

            # An independent, symmetric travel limit for every joint.
            limit = self.max_displacements[index]
            limited_displacement = max(-limit, min(limit, displacement))

            # Also respect the Panda's absolute joint position bounds.
            desired = panda_home + limited_displacement
            target = max(
                self.panda_lower[index],
                min(self.panda_upper[index], desired),
            )
            targets.append(target)

            if self.debug_joint == index + 1:
                debug_info = (
                    index, displacement, target,
                    abs(displacement - limited_displacement) > 1e-6,
                    abs(desired - target) > 1e-6,
                )

        # Display one joint at a time, at a low frequency (not every sample).
        now = time.monotonic()
        if debug_info is not None and now - self.last_debug_time >= self.debug_interval:
            index, displacement, target, travel_clipped, physical_clipped = debug_info
            actual = self.panda_position[index]
            limits = []
            if travel_clipped:
                limits.append("GELLO_TRAVEL_LIMIT")
            if physical_clipped:
                limits.append("PANDA_JOINT_LIMIT")
            status = ",".join(limits) if limits else "OK"
            self.get_logger().info(
                f"J{index + 1} | GELLO delta={math.degrees(displacement):+.1f} deg "
                f"| target={math.degrees(target):+.1f} deg "
                f"| sim={math.degrees(actual):+.1f} deg "
                f"| status={status}"
            )
            self.last_debug_time = now

        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = JOINT_NAMES
        output.position = targets

        self.target_pub.publish(output)

    def data_fresh(self):
        now = time.monotonic()

        if self.gello_time is None or self.panda_time is None:
            return False

        return (
            now - self.gello_time < self.data_timeout
            and now - self.panda_time < self.data_timeout
        )

    def recenter_callback(self, request, response):
        if self.enabled:
            response.success = False
            response.message = "Disable control before recalibration."
            return response

        if not self.data_fresh():
            response.success = False
            response.message = "GELLO or Panda data unavailable."
            return response

        if (
            self.panda_stable_since is None
            or time.monotonic() - self.panda_stable_since
            < self.panda_stable_time
        ):
            response.success = False
            response.message = "Panda position is not stable yet."
            return response

        self.gello_initial = self.gello_unwrapped.copy()
        self.panda_initial = self.panda_position.copy()

        self.calibrated = True

        response.success = True
        response.message = "Calibration reference captured."

        self.get_logger().info(response.message)

        return response

    def enable_callback(self, request, response):
        if not request.data:
            self.enabled = False

            response.success = True
            response.message = "GELLO control disabled."
            return response

        if not self.calibrated:
            response.success = False
            response.message = "Recalibration required."
            return response

        if not self.data_fresh():
            response.success = False
            response.message = "GELLO or Panda data unavailable."
            return response

        self.enabled = True

        response.success = True
        response.message = "GELLO control enabled."
        return response

    def check_safety(self):
        if self.enabled and not self.data_fresh():
            self.enabled = False
            self.calibrated = False

            self.get_logger().warning(
                "Communication timeout. GELLO disabled."
            )

        status = Bool()
        status.data = self.enabled
        self.enabled_pub.publish(status)


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
