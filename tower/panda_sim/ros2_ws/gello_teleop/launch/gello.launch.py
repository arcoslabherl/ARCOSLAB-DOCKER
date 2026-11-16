"""Launch the GELLO driver and calibrator.

Arguments:
  backend:=dynamixel|fake   (default dynamixel)
  port:=/dev/ttyUSB0        GELLO serial device (dynamixel backend)
  robot:=sim|real           which Panda to command (default sim). real adds
                            config/panda_real.yaml on top of the config.
  config:=<yaml>            parameter file (default config/gello_panda.yaml)
"""

import os

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

    if robot not in ("sim", "real"):
        raise RuntimeError(f"robot:={robot}: use robot:=sim or robot:=real")

    # Later files override earlier ones
    calibrator_params = [config]
    if robot == "real":
        calibrator_params.append(os.path.join(CONFIG_DIR, "panda_real.yaml"))

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
        DeclareLaunchArgument(
            "config", default_value=os.path.join(CONFIG_DIR, "gello_panda.yaml")),
        OpaqueFunction(function=nodes),
    ])
