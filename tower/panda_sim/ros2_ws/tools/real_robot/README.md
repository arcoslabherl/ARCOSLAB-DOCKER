# Real Panda: image, real-time check and driver

Everything needed on the computer wired to the real Panda. Only the first
checks have been run (on a laptop, without the robot). Nothing here has
been run against the robot yet.

## Requirements

- **libfranka 0.8.0.** The lab's Panda runs System Version 4.0.4. libfranka
  0.9.x needs 4.2.1 or newer. `make .build-real` builds 0.8.0 from source
  with `patches/libfranka_0.8.0_gcc11.patch`, which adds two `#include`s that
  Ubuntu 22.04's GCC 11 needs.
- **A PREEMPT_RT kernel on the host.** The lab driver creates the robot with
  libfranka's default `RealtimeConfig::kEnforce`. Before controlling the robot,
  libfranka sets its thread to the highest `SCHED_FIFO` priority and checks
  `/sys/kernel/realtime`. If either fails, it refuses to control the robot.
- **Real-time limits in the container.** `make run REAL=1` adds
  `--ulimit rtprio=99 --ulimit memlock=-1`. These are enough even though the
  container runs as the host user.

## Setup (from `tower/panda_sim/`)

    make .build-real            # sim image + libfranka 0.8.0 (~2 GB, a few minutes)
    make stop && make run REAL=1
    make real-driver-build      # rt_check, the driver, and the driver with
                                # patches/panda_driver_safety.patch
    make rt-check               # libfranka's two real-time checks, no robot needed

`make rt-check` must print `libfranka's real-time checks pass.` On a kernel
without PREEMPT_RT it prints `Real-time kernel ... NO`. Without the container
limits it prints `Highest SCHED_FIFO priority: FAILED`.

The real image also runs everything the simulator image does (`make sim`,
`make gello`, tests).

## At the lab

Follow the checklist in the project notes (network, Desk, the flange load).
libfranka's examples are in the image:

    make debug
    /opt/libfranka/build/examples/echo_robot_state 172.16.0.2     # read only
    /opt/libfranka/build/examples/communication_test 172.16.0.2   # MOVES the arm

Then the driver, named as in the lab's real launch (`/asimilar/panda_arm`):

    make real-driver [PATCHED=1]

It asks for confirmation first: at startup the driver connects and homes the
gripper, so the fingers open and close. The arm moves only after the driver's
`control` service enables it.
