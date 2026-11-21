# GELLO teleoperation, step by step

How to start and end a session, in the simulator and on the real Panda.
Run every `make` command from `tower/panda_sim/`.

**The real Panda part has not been run on the robot yet.**

## Two switches

- `make gello-enable` / `make gello-disable`: the arm follows the GELLO,
  or stops where it is. **`gello-disable` is the stop button.**
- `make robot-enable` / `make robot-disable`: the real driver takes or
  releases the arm (real Panda only). **Never use `robot-disable` to
  stop:** it can trigger a reflex. It refuses while the GELLO is enabled
  or the arm moves.

In an emergency on the real Panda: the **white button** (external
activation device). In the simulator: `make gello-disable`.

## Simulator

Plug in the GELLO's USB cable and power its servos. Without a GELLO,
use `make gello-fake` instead of `make gello`.

### Start

    # Terminal 1
    make run                # "name already in use": make stop && make run
    make sim-teleop         # wait for the Panda to appear in RViz

    # Terminal 2
    make gello

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
    make stop

### If something happens

- **The GELLO disables itself:** match the pose again until
  `READY to enable`, then `make gello-enable`.
- **The USB cable came out:** Ctrl+C in terminal 2 and `make gello`
  again.
- **Faster response:** `make sim-teleop ARM_ACC=4.0 ARM_JERK=500`.

## Real Panda

### Once per computer (with internet)

    make setup REAL=1
    make real-driver-build
    make rt-check           # must print: libfranka's real-time checks pass.

The computer needs a PREEMPT_RT kernel: boot it before `make run REAL=1`.

### Before each session, at the robot

1. Cable from the computer to the **Control box** LAN port (not the
   arm's base), set up as in the
   [real robot README](ros2_ws/tools/real_robot/README.md) ("Network").
   Check: `ping -c 5 172.16.0.2`.
2. Desk (`https://172.16.0.2`): unlock the joints and activate FCI.
   **Unlocking moves the arm slightly and initializes the gripper:**
   area clear, nothing in the gripper.
3. External activation device (the white button) released: the light
   is blue.
4. Nobody within the arm's reach. Someone holds the white button the
   whole time, watching the arm (with the GELLO: a second person).

**First visits only: small tests first**, with someone who has operated
the Panda. See the real robot README ("At the lab"):

    make robot-echo                         # no motion
    make robot-nudge JOINT=7 DEG=2 NOLIMIT=1    # one joint 2° and back (driver stopped)
    make robot-nudge JOINT=7 DEG=2          # the same with libfranka's rate limiter
    # then, with the driver running and enabled (below):
    make robot-nudge VIA=driver JOINT=7 DEG=2

### Start

    # Terminal 1
    make run REAL=1         # "name already in use": make stop && make run REAL=1
    make rt-check
    make real-driver        # asks for 'yes'; the gripper's fingers open and close

    # Terminal 2
    make robot-enable       # asks for 'yes'; the driver holds the arm's pose

    # Terminal 3
    make gello ROBOT=real GRIPPER=off JOINTS=7   # first sessions: J7 only, ±10°

Move the GELLO to the **arm's** pose until terminal 3 prints
`GELLO matches the Panda: READY to enable` (within 2°; with `JOINTS`,
only those joints must match, the others hold their pose). First sessions:
one joint at a time (`JOINTS=7`, then 6, 5...), small slow moves; the
gripper stays still (`GRIPPER=off`). Then all joints, still with a range
(`JOINTS=1,2,3,4,5,6,7`), before normal teleop (no `JOINTS`).

    # Terminal 2
    make gello-enable       # the arm follows the GELLO

### End

    # Terminal 2
    make gello-disable      # the arm stops; the driver holds it
    make robot-disable      # only with the arm still

    # Terminal 3: Ctrl+C    (GELLO)
    # Terminal 1: Ctrl+C    (driver)
    make stop

Then press the external activation device, lock the joints in Desk,
and shut the robot down.

### If something happens

- **To stop the arm:** the white button. `make gello-disable` stops it
  too (it brakes smoothly). Never `make robot-disable`.
- **A reflex** (the robot stops itself): the GELLO disables itself.
  Area clear and white button in hand: **the arm can move by itself (up to
  ~19°) on the next `robot-enable`.** Then `make robot-enable`, match
  the pose, `make gello-enable`. If the driver doesn't recover, check
  Desk.
- **The GELLO disconnects:** it disables itself and the driver holds
  the arm. Ctrl+C in terminal 3, plug it back, `make gello ROBOT=real`,
  match the pose, `make gello-enable`.

Use the original driver (`make real-driver`, without `PATCHED=1`): the
patch hasn't been tested on the robot.
