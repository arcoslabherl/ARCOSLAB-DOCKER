# Fake libfranka: test the real Panda driver without the robot

A small stand-in for libfranka. It lets the lab's real driver
(`panda_driver_topics`, from `panda-driver-topics`) build and run unchanged
on a machine without a robot. Use it to check the driver, and the GELLO
pipeline on top of it, before touching the real Panda.

What the fake does:

- **`Robot::control()`** calls the driver's motion generator at 1 kHz. It
  checks every command the way the Panda's controller would: velocity,
  acceleration and jerk (finite differences over 1 ms) and position, against
  the Panda's limits. It reports each violation with libfranka's error names.
  It also reports **stalls** (the same command twice while moving) and slow
  callbacks (over 300 µs).
  - The reported "measured" position follows the command with a first-order
    lag (`FAKE_FRANKA_TAU`). It is not the arm's dynamics.
  - The real libfranka is called with `limit_rate = true`, so it would clip
    the commands before sending them. The fake doesn't clip: it shows the raw
    stream the driver produces. Any violation is a driver problem. On the
    real robot it means either a reflex or a hard, rate-limited jerk.
- **`Gripper`** is blocking, like libfranka: `homing`, `move`, `grasp` (with
  an optional object) and `stop`.

The Panda limits in `src/robot.cpp` are taken from libfranka's
`rate_limiting.h` and Franka's datasheet. Check them against the libfranka
version that will run on the tower.

## Tests (from `tower/panda_sim/`)

```
make fake-driver-build           # fake lib + driver + patched driver
make test-driver [PATCHED=1]     # the driver alone
make test-reflex [PATCHED=1]     # a reflex during teleop, with the fake GELLO
```

- **`test_driver`:** state gaps during the first enable; disabling while J1
  moves fast (how far and how long it takes to stop); whether the arm moves by
  itself after re-enabling; posref at ~1 kHz for the `try_lock` stall. It
  fails if the fake saw any violation or stall.
- **`test_reflex`:** GELLO enable is refused while the driver is disabled. An
  injected reflex must disable GELLO within 0.5 s. Re-enabling the driver must
  not move the arm towards the GELLO.
- `PATCHED=1` uses the driver with `patches/panda_driver_safety.patch`. The
  patch fixes: braking with ruckig on disable instead of freezing the command;
  starting ruckig from rest after a reflex (no stale velocity); ruckig running
  every cycle from a local copy of the goal (the `try_lock` stall); and the
  `control` service in its own callback group (state kept publishing). It is
  **not** applied by `make .pull`.

Results on 2026-11-09 (laptop, no RT kernel):

| | Original driver | Patched |
|---|---|---|
| Arm moving by itself after re-enable | 0.328 rad (19°) | 0 |
| Command-stream violations (`test_driver`) | 41, up to 800 rad/s² | 0 (peaks = ruckig's 0.8/1.0/50) |
| `try_lock` stalls at 1 kHz posref | 2 | 0 |
| State gap during the first enable | 0–1 s (60–804 ms seen) | 4 ms |
| Disable while moving at 0.8 rad/s | freezes in 1 ms (reflex) | brakes in 0.86 s, 0.34 rad more |
| `gello_01` replay, cmd delay slow/fast | 168 / 500 ms | 166 / 510 ms |

## Build and run by hand (inside the container)

```
/ros2_ws/tools/fake_franka/build.sh [<patch>]     # in /ros2_ws/build/_fake_franka
/ros2_ws/tools/fake_franka/run_driver.sh [--patched]
```

- `build.sh` doesn't touch the colcon workspace or the driver's `COLCON_IGNORE`.
  With a patch, it builds a patched copy in `driver_patched/` (`src/` stays at
  the pinned commit). `patches/` isn't mounted in the container: `docker cp` it
  first (as `make fake-driver-build` does).
- The driver runs in namespace `/asimilar`, as node `panda_arm`, with
  `joint_states` remapped to `/asimilar/joint_states`. Only the driver
  starts: no platform, RViz or `robot_state_publisher`.
- Enable it like the real one:
  `ros2 service call /asimilar/panda_arm/control impedance_control_msgs/srv/Modes "{mode: 0, enable: [true,true,true,true,true,true,true,true]}"`.
- GELLO against it: point a copy of `gello_panda.yaml` at `/asimilar`:
  `sed "s#/sim_asimilar_01/#/asimilar/#g; s#gripper_grasp: false#gripper_grasp: true#" /ros2_ws/gello_teleop/config/gello_panda.yaml > /tmp/gello_asimilar.yaml`,
  then `ros2 launch gello_teleop gello.launch.py backend:=fake config:=/tmp/gello_asimilar.yaml`.
- Replay a recording through it: `python3 /ros2_ws/tools/latency.py --ns /asimilar replay gello_01 <name>`.

## Settings (environment variables)

| Variable | Default | Meaning |
|---|---|---|
| `FAKE_FRANKA_Q0` | `0 0 0 -1.57 0 1.57 0.71` | start pose (rad) |
| `FAKE_FRANKA_LOG` | none | CSV with every cycle: `seg,k,t,step_ms,cb_us,c0..c6,q0..q6` |
| `FAKE_FRANKA_STRICT` | `0` | `1`: throw a reflex on the first violation, like the robot without rate limiting |
| `FAKE_FRANKA_TAU` | `0.01` | s, lag of the reported position |
| `FAKE_FRANKA_REPORT_MAX` | `5` | violations printed per type and segment (the rest are counted) |
| `FAKE_FRANKA_UNREACHABLE` | `0` | `1`: the constructor fails, as with no robot on the network |
| `FAKE_FRANKA_REFLEX_FILE` | `/tmp/fake_franka_reflex` | `touch` it to inject a reflex (`control()` throws) |
| `FAKE_FRANKA_SUMMARY_FILE` | `/tmp/fake_franka_summary` | `touch` it to print the summary so far |
| `FAKE_GRIPPER_WIDTH0` | `0.04` | m, width before homing |
| `FAKE_GRIPPER_OBJECT` | `0` | m, object between the fingers (0 = none) |
| `FAKE_GRIPPER_MAX_SPEED` | `0.1` | m/s |
| `FAKE_GRIPPER_HOMING_TIME` | `2.0` | s |

A summary is also printed whenever `control()` ends: on a reflex, or when
the motion finishes.

## Helpers

- `monitor.py <s>`: gaps in `panda_arm/state`, and changes of `status`
  (enabled / emergency) and of `/gello/enabled`.
- `posref.py "<q1..q7>" <s>`: holds one `posref` target for a while.

## Known limits of the fake

- No dynamics, contact, payload or collision reflexes: only the command
  stream is checked. The real robot can still reflex on contact.
- No rate limiting (see above), no real-time scheduling, no packet loss
  except the laptop's own jitter (`missed` in the summary).
- The driver aborts with a core dump on Ctrl+C. Most likely it's the driver
  (it `exit()`s from its signal handler while its threads run); not
  investigated, harmless for the tests.
