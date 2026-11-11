# Franka Panda Simulator - Dell Precision Tower

Docker environment for the ARCoS-Lab Franka Panda simulator
using ROS 2 Humble and RViz2. It runs in two modes:

- `make sim`: with the Robot Body Coupler (RBC) and a Joint
  Reacher (sections 5 to 7).
- `make sim-teleop`: without RBC, for GELLO teleoperation
  (section 8).

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

## 6. Test the simulated Panda

With the simulator running, open another terminal:

    make test-all-joints   # moves each joint 0.1 rad and back
    make test-gripper      # opens and closes the gripper
    make test-panda        # both

Expected result:

    Full Panda test SUCCEEDED

To run every simulator test unattended, with nothing else running:

    make test-sim                 # all tests, about 2.5 min
    make test-sim ONLY=rbc        # only this section's tests
    make test-sim ONLY=teleop     # only the GELLO test (fake device)
    make test-unit                # GELLO unit tests, no simulator needed

It starts the simulator (and the fake GELLO) itself, waits until they
are ready, runs the tests and stops everything again, also on Ctrl+C.
Expected result:

    Simulator tests SUCCEEDED

## 7. Verify ROS 2

Enter the container:

    make debug

List available nodes:

    ros2 node list

Check the Panda joint state:

    ros2 topic echo --once /sim_asimilar_01/panda_arm/state

Check the Joint Reacher action:

    ros2 action list -t

## 8. GELLO teleoperation

With the GELLO plugged in before `make run`, in separate
terminals:

    make sim-teleop       # simulator without RBC
    make gello            # or 'make gello-fake' without hardware
    make gello-enable     # once the GELLO shows 'READY to enable'

Setup, calibration, safety checks, gripper, tests and delay
measurements: see the [GELLO README](ros2_ws/gello_teleop/README.md).

## 9. Stop

Press Ctrl+C in the terminal running the simulator (and GELLO).
Closing that terminal also stops everything it started.

`make sim`, `make sim-teleop` and `make gello` refuse to start a
second copy while one is already running.

To stop the container:

    make stop

## Project structure

- `Dockerfile`, `ros_entrypoint.sh`: ROS 2 environment
- `Makefile`: all commands (`make help`)
- `patches/`: changes to the external repositories, applied by
  `make .pull`
- `ros2_ws/src/`: external ARCoS-Lab repositories, at pinned
  commits (not committed; GitLab access required)
- `ros2_ws/gello_teleop/`: GELLO ROS 2 package (driver,
  calibrator, offset tool, configuration, its own README)
- `ros2_ws/scripts/`: workspace build, startup, Joint Reacher
  setup, session cleanup and tests (run by the Makefile)
- `ros2_ws/panda_sim.rviz`: RViz2 configuration
- `ros2_ws/tools/`: delay measurement tool and recordings

Inside the container: ROS namespace `/sim_asimilar_01`, container
`panda-sim-tower`, image `arcoslab-panda:sim-humble`.

This environment runs only the simulated Panda. The physical
Panda driver is downloaded but excluded from the build.
