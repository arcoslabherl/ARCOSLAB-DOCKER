"""Launch the GELLO driver and calibrator.

Arguments:
  backend:=dynamixel|fake   (default dynamixel)
  port:=/dev/ttyUSB0        GELLO serial device (dynamixel backend)
  config:=<yaml>            parameter file (default config/gello_panda.yaml)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("gello_teleop"),
        "config", "gello_panda.yaml")

    backend = LaunchConfiguration("backend")
    port = LaunchConfiguration("port")
    config = LaunchConfiguration("config")

    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="dynamixel"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("config", default_value=default_config),

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
            parameters=[config],
        ),
    ])
