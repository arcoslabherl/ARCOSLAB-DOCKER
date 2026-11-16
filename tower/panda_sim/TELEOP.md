# GELLO teleoperation, step by step

The commands to start and end a session, in order: first in the
simulator, then on the real Panda. Run every `make` command from
`tower/panda_sim/`. Setup and details are in the [README](README.md),
the [GELLO README](ros2_ws/gello_teleop/README.md) and the
[real robot README](ros2_ws/tools/real_robot/README.md).

**The real Panda part has not been run on the robot yet.**

## Two switches

The GELLO's commands pass through two switches before they reach the
arm:

    GELLO -> calibrator (gello-enable/disable) -> driver (robot-enable/disable) -> arm

- `make gello-enable` / `make gello-disable`: whether the GELLO's
  motion is sent to the driver. On disable the arm stops where it is,
  held by the driver. **This is the stop button.**
- `make robot-enable` / `make robot-disable`: whether the real driver
  controls the arm. Only on the real Panda: the simulator is always
  enabled. **Never use `robot-disable` to stop:** with the lab's driver
  it freezes the command at once, which triggers a reflex if the arm
  is moving. That's why `make robot-disable` refuses while the GELLO
  is enabled or the arm moves.

To stop in an emergency: `make gello-disable`, or the e-stop.

## Simulator

Plug in the GELLO's USB cable and power its servos (before or after
`make run`). No GELLO at hand: use `make gello-fake` instead of
`make gello`. The fake starts already in the Panda's pose.

### Start

    # Terminal 1
    make run                # "name already in use": make stop && make run
    make sim-teleop         # simulator and RViz; wait for the Panda to appear

    # Terminal 2
    make gello              # finds the GELLO by itself

Move the GELLO to the Panda's pose in RViz, gripper trigger released,
until terminal 2 prints `GELLO matches the Panda: READY to enable`.
Until then it says which joints to move.

    # Terminal 3
    make gello-enable       # the Panda follows the GELLO

### End

    # Terminal 3
    make gello-disable      # the Panda stops where it is

    # Terminal 2: Ctrl+C    (GELLO)
    # Terminal 1: Ctrl+C    (simulator)
    make stop               # the container

### If something happens

- **The GELLO disables itself** (lost data, a jump, a timeout): match
  the pose again until `READY to enable`, then `make gello-enable`.
- **The USB cable came out:** Ctrl+C in terminal 2 and `make gello`
  again. No need to restart the container.
- **Faster response:** `make sim-teleop ARM_ACC=4.0 ARM_JERK=500`. The
  defaults are the real driver's limits, so the simulator responds as
  the robot would.

## Real Panda

### Once per computer (at home, with internet)

    make setup REAL=1       # or: make .pull, make .build-real, make run REAL=1, make .colcon_build
    make real-driver-build  # the driver and rt_check
    make rt-check           # must print: libfranka's real-time checks pass.

The computer needs a PREEMPT_RT kernel: boot it before `make run REAL=1`.

### Before each session, at the robot

1. Cable from the computer to the **Control box** LAN port (not the
   arm's base). Static address and Energy-Efficient Ethernet off, as in
   the [real robot README](ros2_ws/tools/real_robot/README.md)
   ("Network"). Check: `ping -c 5 172.16.0.2`.
2. Desk (`https://172.16.0.2`): unlock the joints and activate FCI.
   **Unlocking moves the arm slightly and initializes the gripper:**
   area clear, nothing in the gripper.
3. External activation device (the white button) released: the light
   is blue.
4. Nobody within the arm's reach. E-stop in hand.

**First visit only**, before the driver: `make robot-echo` (no motion;
check the flange load) and `make robot-comm-test` (**moves the arm**),
with someone who has operated the Panda. See the real robot README
("At the lab").

### Start

    # Terminal 1
    make run REAL=1         # "name already in use": make stop && make run REAL=1
    make rt-check
    make real-driver        # asks for 'yes'; the gripper's fingers open and close (homing)

    # Terminal 2
    make robot-enable       # asks for 'yes'; the driver holds the arm's current pose

    # Terminal 3
    make gello ROBOT=real

Move the GELLO to the **arm's** pose until terminal 3 prints
`GELLO matches the Panda: READY to enable`.

    # Terminal 2
    make gello-enable       # the arm follows the GELLO

The driver's limits are fixed at 0.8 rad/s, 1.0 rad/s², 50 rad/s³
(`ARM_ACC`/`ARM_JERK` only exist in the simulator). There is no RViz:
`make real-driver` starts only the driver.

### End

    # Terminal 2
    make gello-disable      # the arm stops; the driver holds it
    make robot-disable      # only with the arm still (the command checks)

    # Terminal 3: Ctrl+C    (GELLO)
    # Terminal 1: Ctrl+C    (driver)
    make stop

Then, as the lab wiki says: press the external activation device, lock
the joints in Desk, and shut the robot down.

### If something happens

- **To stop the arm:** `make gello-disable`, or the e-stop. Never
  `make robot-disable`.
- **A reflex** (the robot stops itself): the GELLO disables itself.
  The driver runs libfranka's automatic error recovery before it
  controls again (read from its code, not tried on the robot). Then:
  `make robot-enable`, match the pose, `make gello-enable`. If it
  doesn't recover, check Desk.
  **With the original driver, the arm can move by itself on that
  `robot-enable`** (up to ~19° in the fake libfranka tests): it
  restarts with the velocity it had before the reflex. Area clear and
  e-stop in hand before re-enabling. The patched driver starts from
  rest.
- **The GELLO disconnects:** it disables itself; the driver keeps
  holding the arm. Ctrl+C in terminal 3, plug it back, `make gello
  ROBOT=real`, match the pose, `make gello-enable`.

### Pending the supervisor

- `make real-driver PATCHED=1` (with `patches/panda_driver_safety.patch`,
  which fixes the driver's braking on disable and its restart after a
  disable or reflex). Until approved, use the
  original driver, as above.
- Teleoperation without RBC, as here.
