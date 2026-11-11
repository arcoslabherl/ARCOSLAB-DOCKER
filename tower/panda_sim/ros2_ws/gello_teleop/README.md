# GELLO teleoperation

ROS 2 package that lets a GELLO (a 1:1 replica arm made of
Dynamixel servos) move the simulated Franka Panda. The GELLO is
read directly from its servos inside the container. No host
software is needed.

- `gello_driver`: reads the servos and publishes the GELLO's
  joint angles and trigger.
- `gello_calibrator`: turns them into Panda commands, with the
  safety checks below.
- `gello_get_offset`: measures a new GELLO unit's calibration.
- `config/gello_panda.yaml`: all parameters.

The nodes only do the ROS part. The logic they rely on is in modules
without ROS, covered by the unit tests in `test/`: `conversion.py`
(servo angles to joints), `safety.py` (limits, pose match, jumps,
timeouts) and `gripper.py` (trigger to gripper commands).

Run every `make` command from `tower/panda_sim/`, with the
container set up as in the [simulator README](../../README.md)
(sections 1 to 4).

## Connect the device

Plug in the GELLO **before** starting the container:

    make run

`make run` passes `/dev/ttyUSB0` to the container if it exists.
For another device:

    make run GELLO_PORT=/dev/serial/by-id/usb-FTDI_...

Pass the same `GELLO_PORT` to `make gello`.

## Configure the unit (once per GELLO)

Each physical GELLO has its own joint offsets, signs and gripper
angles, stored in `config/gello_panda.yaml`.

The lab's GELLO is already calibrated there. For another unit,
hold it in the Panda pose `0 0 0 -1.57 0 1.57 0` with the gripper
trigger released and run, with that unit's joint signs:

    make gello-offsets GELLO_SIGNS="1 1 1 1 1 -1 1"

Paste the printed block into the YAML file. The driver refuses
to start with real hardware until offsets are set.

The tool only finds offsets in steps of 90 degrees and assumes a
42 degree trigger travel. If a servo is mounted at another angle,
or the trigger travels more, set those values by hand. The YAML
file explains the case of J7 on the lab's GELLO.

## Run

Start the simulator without RBC, because RBC publishes `posref`
all the time and GELLO would fight with it:

    make sim-teleop

It uses the same arm limits as the real robot's driver (0.8 rad/s,
1.0 rad/s^2, 50 rad/s^3), so the response matches what the real
robot would have. To try faster limits in the simulator only,
keeping the real robot's 0.8 rad/s:

    make sim-teleop ARM_ACC=4.0 ARM_JERK=500

In slow moves, the delay drops from about 210 ms to about 115 ms.
The real robot's driver still uses 1.0 rad/s^2 and 50 rad/s^3.

In another terminal:

    make gello            # real GELLO
    make gello-fake       # no hardware: simulated GELLO motion

The Panda takes the GELLO's pose, so they must match before control
starts. While disabled, the GELLO terminal shows how far and which
way to move each joint that doesn't match yet, for example
`J2 8 deg, tilt the arm forward`. Follow it until it shows
`READY to enable`. Then:

    make gello-enable     # the Panda follows the GELLO
    make gello-disable    # the Panda brakes and stays where it is

After disabling, or if control stops by itself, match the Panda's
pose again before enabling.

To stop, press Ctrl+C in the GELLO and simulator terminals.

## Gripper

While enabled, the gripper follows the GELLO's trigger: released is
open (0.08 m), pressed is closed, at 0.1 m/s. Each gripper move
must finish before the next one starts, so it follows in 1 mm steps.
A full squeeze takes about 0.8 s to follow.
The trigger must also match the gripper's width to enable (within
15 mm, the hint says `press the trigger` or `release the trigger`),
so enabling doesn't open the gripper and drop what it holds.

For the real Panda, `gripper_grasp: true` in
`config/gello_panda.yaml` makes pressing the trigger past 90% grasp
with force, and hold until it goes below 80%. It's off in the
simulator: there, a grasp with nothing to hold never finishes and
blocks the gripper.

## Safety

Control disables itself and stops the arm if:

- GELLO or Panda data stops for 0.3 s (the message says which one,
  and how old its last data is)
- another node starts publishing `posref`
- a GELLO joint jumps more than 0.5 rad between two readings

Enabling is refused if another node publishes `posref`, if any
joint differs from the Panda by more than 0.1 rad (5.7 degrees), or
if the trigger doesn't match the gripper.

## Topics

- `/gello/joint_states`: GELLO joint angles (rad)
- `/gello/gripper`: 0.0 open, 1.0 closed
- `/gello/target_joint_states`: Panda joint targets
- `/sim_asimilar_01/panda_arm/posref`: commands to the Panda
- `/gello/enabled`: control state
- `/gello/enable` (`std_srvs/SetBool`): service behind
  `make gello-enable` and `make gello-disable`
- `/sim_asimilar_01/panda_arm/gripper`: gripper service, called
  with `move`

## Test

Unit tests, in a second, with only the container running:

    make test-unit

They also read `config/gello_panda.yaml`, so a broken calibration or
wrong joint limits there fail too. Run them after changing the
package's logic.

The whole pipeline, with `make sim-teleop` and `make gello-fake`
running:

    make test-gello

Expected result:

    GELLO pipeline test SUCCEEDED

Or, with nothing running, `make test-sim ONLY=teleop` starts the
simulator and the fake GELLO, runs the test and stops them.

## Measure the delay

`ros2_ws/tools/latency.py` measures how far the Panda lags behind
the GELLO. Record a session, with `make sim-teleop` and
`make gello` running and control enabled (Ctrl+C stops early):

    make latency-record REC=my_session DURATION=60

Then replay its targets to the simulator, with the original timing.
GELLO control must be disabled. Restart `make sim-teleop` with other
limits to compare them on the same motion:

    make latency-replay REC=my_session RUN=my_run

And compare the results:

    make latency-analyze FILES="my_session my_run"

It shows per joint the delay in slow and fast moves, the largest
lag and the overshoot, for ruckig's output and for the arm.
The delay doesn't include the GELLO's own read time (about 30 ms).
Files go to `ros2_ws/tools/data/`, which is not committed.
`ros2_ws/tools/recordings/gello_01.csv` is a 60 s session with the
lab's GELLO, ready to replay without hardware:

    make latency-replay REC=gello_01 RUN=my_run
