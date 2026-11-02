#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from impedance_control_msgs.msg import State
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool


GELLO_TOPIC = "/gello/joint_states"
PANDA_TOPIC = "/sim_asimilar_01/panda_arm/state"
TARGET_TOPIC = "/gello/target_joint_states"

JOINT_NAMES = [f"panda_j{i}" for i in range(7)]

# ============================================================================
# EDITABLE CONFIGURATION — GELLO -> PANDA SIM
# ============================================================================
MAX_JOINT_J1 = math.radians(55) 
MAX_JOINT_J2 = math.radians(45) 
MAX_JOINT_J3 = math.radians(80) 
MAX_JOINT_J4 = 0.25  
MAX_JOINT_J5 = math.radians(80)  
MAX_JOINT_J6 = math.radians(80)  
MAX_JOINT_J7 = math.radians(270)  

MAX_JOINT_DISPLACEMENTS = (
    MAX_JOINT_J1, MAX_JOINT_J2, MAX_JOINT_J3, MAX_JOINT_J4,
    MAX_JOINT_J5, MAX_JOINT_J6, MAX_JOINT_J7,
)

# Preserve existing motion directions. Only J7 is reversed.
# If a joint moves in the opposite direction, change its entry to -1.
JOINT_DIRECTIONS = (1, 1, 1, 1, 1, 1, 1)

# Franka Panda's real joint position limits (radians), in J1..J7 order.
# Safety bounds are separate from the editable GELLO travel limits.
# Verify against the local simulation URDF before using another robot model.
PANDA_JOINT_LOWER = (-2.8973, -1.7628, -2.8973, -3.0718,
                     -2.8973, -0.0175, -2.8973)
PANDA_JOINT_UPPER = ( 2.8973,  1.7628,  2.8973, -0.0698,
                      2.8973,  3.7525,  2.8973)

# Optional console diagnostic to compare GELLO, target and actual Panda.
# Set DEBUG_JOINT to 1-7 for the joint to track, or 0 to disable logging.
DEBUG_JOINT = 7
DEBUG_INTERVAL = 1.0  # seconds

MAX_SAMPLE_CHANGE = 0.5

DATA_TIMEOUT = 0.3
PANDA_STABLE_TIME = 0.5
PANDA_STABLE_TOLERANCE = 0.01


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

        if not all(math.isfinite(x) and x > 0 for x in MAX_JOINT_DISPLACEMENTS):
            raise ValueError("MAX_JOINT_J1..J7 must be finite positive radians")

        self.target_pub = self.create_publisher(
            JointState, TARGET_TOPIC, 10
        )

        self.enabled_pub = self.create_publisher(
            Bool, "/gello/enabled", 10
        )

        self.create_subscription(
            JointState,
            GELLO_TOPIC,
            self.gello_callback,
            10,
        )

        self.create_subscription(
            State,
            PANDA_TOPIC,
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
        for index, limit in enumerate(MAX_JOINT_DISPLACEMENTS, start=1):
            self.get_logger().info(
                f"J{index}: +/-{limit:.3f} rad "
                f"(+/-{math.degrees(limit):.1f} deg, "
                f"{2 * math.degrees(limit):.1f} deg total), "
                f"direction={JOINT_DIRECTIONS[index - 1]:+d}"
            )

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

            if difference > PANDA_STABLE_TOLERANCE:
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

                if abs(delta) > MAX_SAMPLE_CHANGE:
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
            displacement = (current - gello_home) * JOINT_DIRECTIONS[index]

            # An independent, symmetric travel limit for every joint.
            limit = MAX_JOINT_DISPLACEMENTS[index]
            limited_displacement = max(-limit, min(limit, displacement))

            # Also respect the Panda's absolute joint position bounds.
            desired = panda_home + limited_displacement
            target = max(
                PANDA_JOINT_LOWER[index],
                min(PANDA_JOINT_UPPER[index], desired),
            )
            targets.append(target)

            if DEBUG_JOINT == index + 1:
                debug_info = (
                    index, displacement, target,
                    abs(displacement - limited_displacement) > 1e-6,
                    abs(desired - target) > 1e-6,
                )

        # Display one joint at a time, at a low frequency (not every sample).
        now = time.monotonic()
        if debug_info is not None and now - self.last_debug_time >= DEBUG_INTERVAL:
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
            now - self.gello_time < DATA_TIMEOUT
            and now - self.panda_time < DATA_TIMEOUT
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
            < PANDA_STABLE_TIME
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
