# Ouster OS1-128 - Jetson AGX Orin

Docker environment for the Ouster OS1-128 LiDAR using ROS 2 Humble,
the official Ouster ROS driver and RViz2.

## Requirements

- NVIDIA Jetson AGX Orin with JetPack 6.1
- Docker with NVIDIA Container Runtime
- Git and Make
- X11 graphical environment
- Ouster OS1-128 connected via Ethernet

## 1. Build the base Docker image

From the ARCOSLAB-DOCKER repository root:

    cd common/base-jetson/base-zedsdk-jetson-jp-ros2-humble
    make .build

Return to the Ouster directory:

    cd ../../../jetson-orin/ouster_ros

## 2. Download dependencies

    make .pull

This downloads the official Ouster ROS driver at the pinned commit
and initializes its submodules.

It also automatically applies the local WINDOW compatibility patch
required for the tested OS1-128 configuration.

The patch is safe to apply repeatedly.

## 3. Build the Docker image

    make .build

## 4. Configure Ethernet

The Jetson uses the same Ethernet interface (eno1) for RoboSense
and Ouster, with two IPv4 addresses:

- RoboSense: 192.168.1.102/16
- Ouster host IP: 169.254.168.100/16
- Ouster sensor IP: 169.254.168.178

If the RoboSense address is already configured, add the Ouster
address to the existing NetworkManager profile:

    sudo nmcli connection modify "Wired connection 1" \
      +ipv4.addresses "169.254.168.100/16"

    sudo nmcli device reapply eno1

Verify both addresses and routing:

    ip -4 addr show eno1
    ip -4 route get 169.254.168.178
    ip -4 route get 192.168.1.1
    ping -I eno1 -c 4 169.254.168.178

The Ouster route should use eno1 with source 169.254.168.100.

The connection profile is saved by NetworkManager. Verify
persistence after reboot.

Adapt IP addresses and connection names to the target hardware.
For the initial RoboSense Ethernet configuration, see the
rslidar_sdk README.

## 5. Start Docker

From the Jetson host:

    make run

Open an interactive terminal:

    make debug

## 6. Build the ROS 2 workspace

From the Jetson host, after starting the container:

    make .colcon_build

Alternatively, inside the container:

    source /ros2_ws/build_ws.sh

For subsequent sessions, the entrypoint loads the existing
compiled workspace automatically.

Rebuild only when the ROS 2 source code changes.

## 7. Launch Ouster and RViz2

From the Jetson host, start the Docker container:

    make run
    make debug

Inside the container, launch the Ouster driver and RViz2:

    ./start_ouster.sh

The script automatically:
- Loads the ROS 2 Humble environment.
- Sets the ROS domain and DDS middleware.
- Starts the Ouster driver using the sensor configuration.
- Opens RViz2 for point cloud visualization.

Sensor initialization may take approximately 10-15 seconds.

## 8. RViz2 configuration

The visualization uses the following settings:

- Fixed Frame: os_sensor
- PointCloud2 Topic: /ouster/points
- Reliability Policy: Best Effort

The point cloud should appear automatically once the sensor
initialization is complete.

## 9. Verify ROS 2 data

In another container terminal:

    ros2 lifecycle get /ouster/os_driver
    ros2 topic info /ouster/points
    ros2 topic hz /ouster/points

Expected results:

- Driver lifecycle state: active
- At least one publisher on /ouster/points
- Point cloud frequency: approximately 10 Hz at 1024x10

## 10. Stop

Press Ctrl+C in the terminal running start_ouster.sh.

This stops the Ouster driver and RViz2.

Exit the container:

    exit

From the Jetson host:

    make stop

## Configuration

- Sensor: Ouster OS1-128
- Tested firmware: 3.1.0
- ROS distribution: Humble
- DDS middleware: Cyclone DDS
- ROS_DOMAIN_ID: 42
- Point cloud topic: /ouster/points
- Fixed frame: os_sensor
- LiDAR mode: 1024x10
- Container: ouster-os1-128
- Docker image: arcoslab-ouster:os1-128

## Project structure

- Dockerfile: Docker environment and dependencies
- Makefile: Download, build and execution commands
- config/driver_params.yaml: Ouster sensor parameters
- patches/: Versioned driver compatibility fixes
- ros_entrypoint.sh: ROS 2 environment initialization
- ros2_ws/build_ws.sh: ROS 2 build helper
- ros2_ws/start_ouster.sh: Launches Ouster and RViz2
- ros2_ws/src/: ROS 2 workspace source directory

The external Ouster driver, ROS 2 build artifacts and generated
sensor metadata are excluded from Git.

## Notes

The tested Ouster ROS revision requires a compatibility patch
for the WINDOW field in the RNG19_RFL8_SIG16_NIR16 UDP profile.

The patch is applied automatically by make .pull.
Do not modify the downloaded vendor code manually.

Docker and containerd storage reside on the Jetson NVMe SSD.
