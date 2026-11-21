"""Launch the GELLO driver and calibrator.

Arguments:
  backend:=dynamixel|fake   (default dynamixel)
  port:=/dev/ttyUSB0        GELLO serial device (dynamixel backend)
  robot:=sim|real           which Panda to command (default sim). real adds
                            config/panda_real.yaml on top of the config.
  config:=<yaml>            parameter file (default config/gello_panda.yaml)
  gripper:=on|off           off: the calibrator leaves the gripper alone
                            (default on)
  joints:=all|<list>        joint mask: only these joints (1..7, e.g. 7 or
                            6,7) follow the GELLO, the rest hold the pose at
                            enable (default all)
  range:=<deg>              with a joint mask, how far the active joints may
                            go from the pose at enable (default 10)
"""

import math
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG_DIR = os.path.join(
    get_package_share_directory("gello_teleop"), "config")


def nodes(context):
    backend = LaunchConfiguration("backend").perform(context)
    port = LaunchConfiguration("port").perform(context)
    robot = LaunchConfiguration("robot").perform(context)
    config = LaunchConfiguration("config").perform(context)
    gripper = LaunchConfiguration("gripper").perform(context)
    joints = LaunchConfiguration("joints").perform(context).strip()
    range_deg = LaunchConfiguration("range").perform(context)

    if robot not in ("sim", "real"):
        raise RuntimeError(f"robot:={robot}: use robot:=sim or robot:=real")
    if gripper not in ("on", "off"):
        raise RuntimeError(f"gripper:={gripper}: use gripper:=on or gripper:=off")

    # Later files override earlier ones
    calibrator_params = [config]
    if robot == "real":
        calibrator_params.append(os.path.join(CONFIG_DIR, "panda_real.yaml"))
    if gripper == "off":
        calibrator_params.append({"gripper_service": ""})
    if joints != "all":
        try:
            active = [int(joint) for joint in re.split(r"[,\s]+", joints)]
            max_range = math.radians(float(range_deg))
        except ValueError:
            raise RuntimeError(
                f"joints:={joints} range:={range_deg}: use e.g. joints:=7 or "
                "joints:=6,7 and range:=10 (degrees)") from None
        # The calibrator checks the joints (1..7, no repeats) and the range
        calibrator_params.append(
            {"active_joints": active, "active_range": max_range})

    return [
        Node(
            package="gello_teleop",
            executable="gello_driver",
            name="gello_driver",
            output="screen",
            parameters=[config, {"backend": backend, "port": port}],
        ),
        Node(
            package="gello_teleop",
            executable="gello_calibrator",
            name="gello_calibrator",
            output="screen",
            parameters=calibrator_params,
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="dynamixel"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("robot", default_value="sim"),
        DeclareLaunchArgument("gripper", default_value="on"),
        DeclareLaunchArgument("joints", default_value="all"),
        DeclareLaunchArgument("range", default_value="10"),
        DeclareLaunchArgument(
            "config", default_value=os.path.join(CONFIG_DIR, "gello_panda.yaml")),
        OpaqueFunction(function=nodes),
    ])
