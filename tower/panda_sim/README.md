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

## 8. GELLO teleoperation

GELLO is read directly from its Dynamixel servos inside the
container. No host software is needed.

### Connect the device

Plug in the GELLO **before** starting the container:

    make run

`make run` passes `/dev/ttyUSB0` to the container if it exists.
For another device:

    make run GELLO_PORT=/dev/serial/by-id/usb-FTDI_...

### Configure the unit (once per GELLO)

Each physical GELLO has its own joint offsets, signs and gripper
angles, stored in:

    ros2_ws/gello_teleop/config/gello_panda.yaml

Copy them from the unit's `PORT_CONFIG_MAP` entry in
gello_software, or measure them. Hold the GELLO in the Panda pose
`0 0 0 -1.57 0 1.57 0` with the gripper trigger released and run:

    make gello-offsets GELLO_SIGNS="1 -1 1 1 1 -1 1"

Paste the printed block into the YAML file. The driver refuses
to start with real hardware until offsets are set.

The tool only finds offsets in steps of 90 degrees. If a servo is
mounted at another angle, set that offset by hand. The YAML file
explains the case of J7 on the lab's GELLO.

### Run

Start the simulator without RBC, because RBC publishes `posref`
all the time and GELLO would fight with it:

    make sim-teleop

It uses the same arm limits as the real robot's driver (0.8 rad/s,
1.0 rad/s^2), so the response matches what the real robot would
have. To try other limits in the simulator only:

    make sim-teleop ARM_VEL=1.5 ARM_ACC=5

In another terminal:

    make gello            # real GELLO
    make gello-fake       # no hardware: simulated GELLO motion

Then:

    make gello-recenter   # capture GELLO/Panda reference pose
    make gello-enable     # the Panda follows the GELLO
    make gello-disable    # the Panda stops where it is

Topics:

- `/gello/joint_states`: GELLO joint angles (rad)
- `/gello/gripper`: 0.0 open, 1.0 closed
- `/gello/target_joint_states`: Panda joint targets
- `/sim_asimilar_01/panda_arm/posref`: commands to the Panda
- `/gello/enabled`: control state

Control disables itself and stops the arm if:

- GELLO or Panda data stops for 0.3 s
- another node starts publishing `posref`

Enabling is refused if another node publishes `posref`, or if the
GELLO moved more than 0.1 rad since `make gello-recenter`.

### Test

With `make sim-teleop` and `make gello-fake` running:

    make test-gello

Expected result:

    GELLO pipeline test SUCCEEDED

## 9. Stop

Press Ctrl+C in the terminal running the simulator (and GELLO).
Closing that terminal also stops everything it started.

`make sim`, `make sim-teleop` and `make gello` refuse to start a
second copy while one is already running.

To stop the container:

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
- ros2_ws/start_gello.sh: GELLO startup
- ros2_ws/test_gello.sh: GELLO pipeline test
- ros2_ws/gello_teleop/: GELLO ROS 2 package (driver,
  calibrator, offset tool, configuration)
- ros2_ws/stop_session.sh: Stops a session when its terminal closes
- patches/: Compatibility fixes (including RBC process cleanup)
- ros2_ws/src/: External ROS 2 repositories

## Notes

The external ARCoS-Lab repositories are not committed
to ARCOSLAB-DOCKER.

GitLab access is required to download these dependencies.

This environment has been tested with the simulated
Franka Panda. It does not require connecting the physical robot.

Automatic tests cover all seven joints, the gripper and
the GELLO pipeline, including the Panda following the GELLO.
GELLO does not control the gripper yet.