# Fake libfranka: test the real Panda driver without the robot

A small stand-in for libfranka. It lets the lab's real driver
(`panda_driver_topics`, from `panda-driver-topics`) build and run unchanged
on a machine without a robot. Use it to check the driver, and the GELLO
pipeline on top of it, before touching the real Panda.

What the fake does:

- **`Robot::control()`** calls the driver's motion generator at 1 kHz. It
  checks every raw command against the robot's `q_d`, `dq_d`, `ddq_d`:
  velocity, acceleration and jerk (finite differences over 1 ms) and
  position, against the Panda's limits. It reports each violation with
  libfranka's error names: a violation is a caller (driver) problem. It also
  reports **stalls** (the same command twice while moving) and slow
  callbacks (over 300 µs).
  - **Rate limiter:** with `limit_rate = true` (the driver and the nudge) the
    command then goes through libfranka 0.8.0's own rate limiter
    (`src/rate_limiting.h`, copied from libfranka), as the real library does,
    and the robot accepts the limited command. The summary says how many
    commands the limiter changed and in how many cycles `|ddq_d|` sat at
    ≥ 90 % of the limit: both should be 0; many saturated cycles is the
    2026-11-17 oscillation (the old 1 ms-per-call nudge reproduces it here).
    With `cutoff_frequency` 1000 the real low-pass filter is off; below 1000
    it isn't modelled.
  - **Lost packets:** the robot goes on by itself, with constant
    acceleration ("When a packet is lost, FCI assumes a constant
    acceleration model", libfranka 0.8.0 `rate_limiting.h`), so `q_d` moves
    ahead and the next command is judged and limited from there.
  - The reported "measured" position follows `q_d` with a first-order lag
    (`FAKE_FRANKA_TAU`). It is not the arm's dynamics.
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
make test-robot-mode [PATCHED=1] # make robot-enable / robot-disable
make test-nudge [PATCHED=1]      # make robot-nudge, libfranka alone and via the driver
make test-fake-robot [PATCHED=1] # the build, then the four tests
```

With the original driver, `test-driver` is expected to fail (results
below). With `PATCHED=1` all four pass.

`test-reflex` and `test-robot-mode` start the fake GELLO configured for the real robot
(`robot:=real`, `config/panda_real.yaml`).

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
| `FAKE_FRANKA_LOG` | none | CSV with every cycle: `seg,k,t,step_ms,cb_us,c0..c6,q0..q6,d0..d6` (`c` raw command, `q` measured, `d` accepted `q_d`) |
| `FAKE_FRANKA_STRICT` | `0` | `1`: throw a reflex on the first violation, like the robot without rate limiting |
| `FAKE_FRANKA_LIMIT_RATE` | as the caller | `0`/`1`: force libfranka's rate limiter off/on (A/B without changing the caller). With `0` and `STRICT=1` a violation stops the robot, as the real one would |
| `FAKE_FRANKA_LOSE_EVERY` | `0` | `N`: drop 2 packets every N cycles; meanwhile the robot extrapolates `q_d` |
| `FAKE_FRANKA_EXTRAPOLATE` | `acceleration` | how the robot extrapolates during lost packets: `acceleration` (constant, as libfranka documents) or `velocity` (our first model, 2026-11-17) |
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
- No real-time scheduling, no packet loss except the forced one and the
  laptop's own jitter (`missed` in the summary), no low-pass filter below
  1000 Hz, no 20-packet communication limit.
- The real robot can still reject a rate-limited command (2026-11-17 F5:
  discontinuity errors with the limiter on), probably from state/command
  timing the fake doesn't model: no violations here doesn't rule that out.
- Not yet checked against a real log number by number: the 2026-11-17 23:55
  CSV (`nudge_J7_2deg_20261117_235529.csv`, on `arcospanda`) is the reference.
- The driver aborts with a core dump on Ctrl+C. Most likely it's the driver
  (it `exit()`s from its signal handler while its threads run); not
  investigated, harmless for the tests.
