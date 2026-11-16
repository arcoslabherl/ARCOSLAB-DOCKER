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

## The computer wired to the Panda

- **Kernel:** boot the PREEMPT_RT kernel (on CachyOS:
  `linux-cachyos-rt-bore`). Check with `uname -v` (`PREEMPT_RT`) and
  `cat /sys/kernel/realtime` (`1`), then `make rt-check`.
- **Latency:** `sudo cyclictest -m -p 99 -i 1000 -t -a -D 2m -q` (from
  `rt-tests`), plugged in, `powerprofilesctl set performance`. The robot
  expects an answer every 1 ms. A laptop can stall for a few ms even with
  PREEMPT_RT: one measured about 3 ms on all cores every ~20 s, so
  `make robot-comm-test`'s success rate has to decide.
- **Network:** a cable from the computer to the **Control's** LAN port (not
  the arm's base). A static address on that interface (here `eno1`), and
  Energy-Efficient Ethernet off:

      nmcli con add type ethernet ifname eno1 con-name franka \
          ipv4.method manual ipv4.addresses 172.16.0.1/24
      nmcli con up franka
      sudo ethtool --set-eee eno1 eee off
      ping -c 5 172.16.0.2

- **Desk** (`https://172.16.0.2`, Chromium): unlock the joints. Unlocking
  moves the arm slightly and initializes the gripper.

## At the lab

Follow the checklist in the project notes. In order:

    make run REAL=1
    make rt-check
    make robot-echo        # reads the state 100 times, no motion
    make robot-comm-test   # MOVES the arm (see below)

`make robot-echo` shows what libfranka knows about the robot, including the
end-effector load configured in Desk (`m_ee`, `F_x_Cee`, `F_T_EE`) and the
estimated external force `O_F_ext_hat_K`, which should be near 0 with the arm
still.

`make robot-comm-test` asks for confirmation first. libfranka 0.8.0's
example **moves the arm** to the joint pose (0, -45, 0, -135, 0, 90, 45)
degrees at half speed. Then it runs about 10 s with zero torque: the arm
floats, compensated for gravity only, so it sags if the load in Desk is
wrong. At the end it prints the average, minimum and maximum
`control_command_success_rate` and how many robot states were lost.

Then the driver, named as in the lab's real launch (`/asimilar/panda_arm`):

    make real-driver [PATCHED=1]

It asks for confirmation first: at startup the driver connects and homes the
gripper, so the fingers open and close. The arm moves only after the driver's
`control` service enables it.
