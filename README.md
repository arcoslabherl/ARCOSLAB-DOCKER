# ARCOSLAB-DOCKER

Reproducible Docker environments for robotics development at
**ARCoS Lab — University of Costa Rica.**

This repository provides containerized development environments
for NVIDIA Jetson platforms, ROS 2, robotics sensors and robot
simulation.

Each integration has its own Docker environment, ROS 2 workspace
and configuration files.

## Supported Integrations

| Platform | Sensor | Integration | Documentation |
| --- | --- | --- | --- |
| NVIDIA Jetson AGX Orin | RoboSense E1R | RSLiDAR SDK, ROS 2 Humble, RViz2 | [RoboSense E1R](jetson-orin/rslidar_sdk/README.md) |
| NVIDIA Jetson AGX Orin | Ouster OS1-128 | Official Ouster ROS driver, ROS 2 Humble, RViz2 | [Ouster OS1-128](jetson-orin/ouster_ros/README.md) |
| Dell Precision 5860 Tower | Franka Panda Simulator | ROS 2 Humble, RBC, Joint Reacher, RViz2 | Franka Panda Simulator |

Both sensors have been tested with live point cloud visualization in RViz2.

## Repository Structure

```text
ARCOSLAB-DOCKER/
├── common/
│   └── base-jetson/
│       └── base-zedsdk-jetson-jp-ros2-humble/
├── jetson-orin/
│   ├── rslidar_sdk/
│   └── ouster_ros/
├── tower/
│   └── panda_sim/
│       ├── Dockerfile
│       ├── Makefile
│       ├── patches/
│       └── ros2_ws/
└── README.md
```

- **`common/`** contains the shared Docker base environment.
- **`jetson-orin/rslidar_sdk/`** contains the RoboSense E1R integration.
- **`jetson-orin/ouster_ros/`** contains the Ouster OS1-128 integration, including a versioned compatibility patch.
- **`tower/panda_sim/`** contains the Franka Panda simulation
  environment for x86_64 workstations, including ROS 2 Humble,
  Robot Body Coupler, Joint Reacher and RViz2.

## Technology Stack

- NVIDIA Jetson AGX Orin and JetPack 6.1
- Docker with NVIDIA Container Runtime
- ROS 2 Humble
- RViz2
- C++ and Python
- Ethernet-based LiDAR communication

## Getting Started

### 1. Clone the repository

```bash
git clone -b develop https://github.com/arcoslabherl/ARCOSLAB-DOCKER.git
cd ARCOSLAB-DOCKER
```

### 2. Build the shared Jetson base image

```bash
cd common/base-jetson/base-zedsdk-jetson-jp-ros2-humble
make .build
```

### 3. Select a sensor integration

**RoboSense E1R**

Follow the [RoboSense setup guide](jetson-orin/rslidar_sdk/README.md) for dependency installation, network configuration, ROS 2 compilation, and RViz2 visualization.

**Ouster OS1-128**

Follow the [Ouster setup guide](jetson-orin/ouster_ros/README.md) for the pinned driver, compatibility patch, network configuration, and RViz2 visualization.

-----
### Franka Panda Simulator

From the repository root:

```bash
cd tower/panda_sim
make .pull
make .build
make run
make .colcon_build
make sim
```

For subsequent sessions, rebuilding is not necessary unless source code or dependencies have changed.

## Development Guidelines

- Keep each sensor integration in its own directory and ROS 2 workspace.
- Use pinned upstream driver versions for reproducible builds.
- Store local configuration files, startup scripts, and compatibility patches in Git.
- Do not commit downloaded vendor repositories, compiled workspaces, generated sensor metadata, or container data.
- Review each sensor's README before configuring Ethernet or launching its driver.

## Documentation

- [RoboSense E1R — Setup and Usage](jetson-orin/rslidar_sdk/README.md)
- [Ouster OS1-128 — Setup and Usage](jetson-orin/ouster_ros/README.md)

## About

Developed as part of robotics research and development activities at **ARCoS Lab, University of Costa Rica**.
