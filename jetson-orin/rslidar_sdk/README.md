# RoboSense E1R - Jetson AGX Orin

Docker environment for RoboSense E1R using ROS 2 Humble,
RSLiDAR SDK and RViz2.

## Requirements

- NVIDIA Jetson AGX Orin with JetPack 6.1
- Docker with NVIDIA Container Runtime
- Git and Make
- X11 graphical environment
- RoboSense E1R connected via Ethernet

## 1. Build the base Docker image

From the repository root:

    cd common/base-jetson/base-zedsdk-jetson-jp-ros2-humble
    make .build

Return to the RSLiDAR directory:

    cd ../../../jetson-orin/rslidar_sdk

## 2. Download dependencies

    make .pull

This downloads the pinned RSLiDAR SDK and message packages
and applies the local E1R and RViz2 configurations.

## 3. Build the Docker image

    make .build

## 4. Configure Ethernet

Example configuration used on the Jetson:

    sudo nmcli connection modify "Wired connection 1" \
      ipv4.method manual \
      ipv4.addresses 192.168.1.102/16 \
      ipv4.gateway "" \
      ipv4.never-default yes

    sudo nmcli connection up "Wired connection 1"

The IP address, subnet and connection name must be adapted
to the local network and LiDAR configuration.

## 5. Start Docker

    make run
    make debug

## 6. Build the ROS 2 workspace

Inside the Docker container, run:

    source /ros2_ws/build_ws.sh

This script automatically builds the ROS 2 workspace and
loads the required environment.

The workspace only needs to be rebuilt when the source
code changes.

For subsequent sessions, load the existing workspace with:

    source /ros2_ws/install/setup.bash

## 7. Launch RSLiDAR and RViz2

    ros2 launch rslidar_sdk humble_start.py

RViz2 should display the RoboSense E1R point cloud.

## 8. Stop

Press Ctrl+C to stop the launch process.

Exit the container:

    exit

From the Jetson host:

    make stop

## Configuration

- Sensor: RoboSense E1R
- ROS distribution: Humble
- DDS middleware: Cyclone DDS
- ROS_DOMAIN_ID: 42
- Point cloud topic: /rslidar_points
- Fixed frame: rslidar
- MSOP port: 6699
- DIFOP port: 7788

## Project structure

- Dockerfile: Docker environment and dependencies
- Makefile: Setup, build and execution commands
- config/rse1.yaml: RoboSense E1R configuration
- config/rviz2.rviz: RViz2 visualization settings
- ros_entrypoint.sh: ROS 2 environment initialization
- ros2_ws/src/: ROS 2 workspace source directory

External dependencies, Docker images and ROS 2 build
artifacts are generated locally and excluded from Git.
