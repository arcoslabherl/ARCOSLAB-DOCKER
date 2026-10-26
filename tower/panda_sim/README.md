# Franka Panda Simulator - Dell Precision Tower

Docker environment for the ARCoS-Lab Franka Panda simulator
using ROS 2 Humble, Robot Body Coupler (RBC), Joint Reacher
and RViz2.

## Requirements

- Dell Precision Tower or compatible x86_64 workstation
- Ubuntu with Docker installed
- Git and Make
- X11 graphical environment
- Access to the required ARCoS-Lab GitLab repositories

## 1. Download dependencies

From the ARCOSLAB-DOCKER repository root:

    cd tower/panda_sim
    make .pull

This downloads five ARCoS-Lab repositories at pinned commits
and applies the required compatibility patches.

## 2. Build Docker

    make .build

Docker image:

    arcoslab-panda:sim-humble

## 3. Start Docker

    make run

Open an interactive terminal:

    make debug

## 4. Build the ROS 2 workspace

    make .colcon_build

The workspace uses ROS 2 Humble.

The physical Panda driver packages are excluded from
the simulation build.

## 5. Launch the simulator

    make sim

This starts:

- Franka Panda virtual robot
- Robot Body Coupler (RBC)
- Robot State Publisher
- Joint Reacher
- RViz2

Joint Reacher is initialized automatically.

## 6. Test joint movement

With the simulator running, open another terminal:

    make test-j1

This test:

1. Moves Panda J1 to 0.1 rad.
2. Waits for successful action completion.
3. Returns J1 to 0.0 rad.
4. Verifies that both ROS 2 actions succeed.

Expected result:

    Goal finished with status: SUCCEEDED

    Franka Panda J1 test completed successfully.

## 7. Verify ROS 2

Enter the container:

    make debug

List available nodes:

    ros2 node list

Check the Panda joint state:

    ros2 topic echo --once /sim_asimilar_01/panda_arm/state

Check the Joint Reacher action:

    ros2 action list -t

## 8. Stop

Press Ctrl+C in the terminal running the simulator.

Then:

    make stop

## Configuration

- Platform: x86_64 workstation
- ROS distribution: Humble
- Simulator: ARCoS-Lab Virtual Robot
- Control architecture: Robot Body Coupler
- Joint controller: Joint Reacher
- Visualization: RViz2
- ROS namespace: /sim_asimilar_01
- Docker container: panda-sim-tower
- Docker image: arcoslab-panda:sim-humble

## Project structure

- Dockerfile: ROS 2 environment and dependencies
- Makefile: Build and execution commands
- ros_entrypoint.sh: ROS 2 environment initialization
- ros2_ws/build_ws.sh: Workspace compilation
- ros2_ws/start_sim.sh: Simulator startup
- ros2_ws/setup_jr.sh: Joint Reacher initialization
- ros2_ws/test_joint.sh: J1 movement test
- ros2_ws/panda_sim.rviz: RViz2 configuration
- patches/: Compatibility fixes
- ros2_ws/src/: External ROS 2 repositories

## Notes

The external ARCoS-Lab repositories are not committed
to ARCOSLAB-DOCKER.

GitLab access is required to download these dependencies.

This environment has been tested with the simulated
Franka Panda. It does not require connecting the physical robot.

Currently, automatic movement tests cover J1 only.
The remaining joints and GELLO integration are future work.