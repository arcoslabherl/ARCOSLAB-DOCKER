#!/usr/bin/env python3
"""Teleoperation latency tool: record, replay and analyze.

  record   Records the GELLO targets and the Panda's state during a live
           session (GELLO enabled).
  replay   Publishes a recording's targets to posref with their original
           timing, and records the Panda's state. Lets different arm
           settings be compared on exactly the same motion.
  analyze  Per joint: delay (slow and fast moves), largest lag and
           overshoot, for the commanded position (ruckig's output) and the
           arm's position.

Files are CSV, one row per message:
  kind,t,q0..q6,c0..c6
  kind  "target" (GELLO target, or the posref a replay published) or
        "state" (panda_arm/state)
  t     seconds since the file started. Targets: their header stamp
        (publish time); replayed targets: publish time; state: receive time
        (State has no timestamp)
  q     positions (rad). For a state row, the arm's position
  c     state rows only: the commanded position (ruckig's output)
"""

import argparse
import csv
import math
import os
import sys
import threading
import time

NUM_JOINTS = 7
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(TOOLS_DIR, "data")              # gitignored
RECORDINGS_DIR = os.path.join(TOOLS_DIR, "recordings")  # committed
TARGET_TOPIC = "/gello/target_joint_states"

# Analysis settings
GRID_STEP = 0.001          # s
VELOCITY_SMOOTHING = 0.05  # s, moving average of the target's velocity
MOVE_VELOCITY = 0.05       # rad/s, above this the target is moving
MOVE_MERGE_GAP = 0.1       # s, closer motion segments are merged
MOVE_MIN_DURATION = 0.15   # s
MOVE_MIN_RANGE = 0.03      # rad
FAST_VELOCITY = 1.0        # rad/s, a segment peaking above this is "fast"
MAX_DELAY = 1.5            # s, longest delay searched for
DELAY_STEP = 0.002         # s
TURN_HYSTERESIS = 0.02     # rad, smaller reversals are ignored (noise)
PAUSE_MIN_DURATION = 0.2   # s, a stop shorter than this isn't a pause
PAUSE_MAX_WINDOW = 2.0     # s
OVERSHOOT_REPORT = 0.005   # rad, overshoots above this are counted


# ---------------------------------------------------------------------------
# Files
# ---------------------------------------------------------------------------

def resolve(name, existing=False):
    """A bare name means <tools>/data/<name>.csv. To read one, the
    committed <tools>/recordings/<name>.csv is used if data/ lacks it."""
    if not name.endswith(".csv"):
        name += ".csv"
    if os.sep in name:
        return name
    path = os.path.join(DATA_DIR, name)
    committed = os.path.join(RECORDINGS_DIR, name)
    if existing and not os.path.exists(path) and os.path.exists(committed):
        return committed
    return path


def give_to_host_user(path):
    # Files written in the container are root's; hand them to the host user
    uid = os.environ.get("HOST_UID")
    gid = os.environ.get("HOST_GID")
    if uid and gid:
        try:
            os.chown(path, int(uid), int(gid))
        except OSError:
            pass


def write_rows(path, rows):
    directory = os.path.dirname(path)
    if not os.path.isdir(directory):
        os.makedirs(directory)
        give_to_host_user(directory)

    header = (["kind", "t"] + [f"q{i}" for i in range(NUM_JOINTS)]
              + [f"c{i}" for i in range(NUM_JOINTS)])
    with open(path, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for kind, t, position, commanded in rows:
            row = [kind, f"{t:.6f}"] + [f"{x:.6f}" for x in position]
            row += ([f"{x:.6f}" for x in commanded] if commanded
                    else [""] * NUM_JOINTS)
            writer.writerow(row)
    give_to_host_user(path)


def read_rows(path):
    targets, states = [], []
    with open(path, newline="") as handle:
        reader = csv.reader(handle)
        next(reader)
        for row in reader:
            t = float(row[1])
            position = [float(x) for x in row[2:2 + NUM_JOINTS]]
            if row[0] == "target":
                targets.append((t, position))
            else:
                commanded = [float(x) for x in row[2 + NUM_JOINTS:]]
                states.append((t, position, commanded))
    return targets, states


# ---------------------------------------------------------------------------
# ROS: record and replay
# ---------------------------------------------------------------------------

def make_node(name, namespace, record_targets):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import JointState
    from impedance_control_msgs.msg import State

    class LatencyNode(Node):
        def __init__(self):
            super().__init__(name)
            # Wall clock, the same one the calibrator stamps targets with
            self.t0 = time.time()
            self.lock = threading.Lock()
            self.rows = []
            self.last_state = None
            qos = QoSProfile(depth=1000)
            self.create_subscription(
                State, f"{namespace}/panda_arm/state", self.on_state, qos)
            if record_targets:
                self.create_subscription(
                    JointState, TARGET_TOPIC, self.on_target, qos)

        def now(self):
            return time.time() - self.t0

        def add(self, row):
            with self.lock:
                self.rows.append(row)

        def on_target(self, msg):
            if len(msg.position) != NUM_JOINTS:
                return
            # The stamp is when it was published: receive times bunch up
            # whenever this node falls behind
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            t = stamp - self.t0 if stamp > 0 else self.now()
            self.add(("target", t, list(msg.position), None))

        def on_state(self, msg):
            if (len(msg.position) != NUM_JOINTS
                    or len(msg.cmdedposition) != NUM_JOINTS):
                return
            row = ("state", self.now(), list(msg.position),
                   list(msg.cmdedposition))
            self.last_state = row
            self.add(row)

        def count(self, kind):
            with self.lock:
                return sum(1 for row in self.rows if row[0] == kind)

    rclpy.init()
    return rclpy, LatencyNode()


def spin_in_background(rclpy, node):
    from rclpy.executors import SingleThreadedExecutor

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except Exception:
            pass  # Ctrl+C shuts the context down; the main thread reports it

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    return executor


def report_rates(rows, duration):
    for kind in ("target", "state"):
        n = sum(1 for row in rows if row[0] == kind)
        rate = n / duration if duration > 0 else 0.0
        print(f"  {kind:6s}: {n:6d} messages, {rate:7.1f} Hz")


def cmd_record(args):
    path = resolve(args.name)
    rclpy, node = make_node("latency_recorder", args.ns, record_targets=True)
    executor = spin_in_background(rclpy, node)

    try:
        # Messages queued while starting arrive in a burst: drop them
        deadline = time.monotonic() + 10.0
        while node.last_state is None:
            if time.monotonic() > deadline:
                print(f"ERROR: no {args.ns}/panda_arm/state. "
                      "Is 'make sim-teleop' running?")
                return 1
            time.sleep(0.1)
        time.sleep(1.0)
        with node.lock:
            node.rows.clear()
        start = node.now()

        print(f"Recording {TARGET_TOPIC} and {args.ns}/panda_arm/state "
              f"for {args.duration:.0f} s (Ctrl+C stops early).")
        print("Move the GELLO: slow and fast moves, reversals, sudden stops.")

        while node.now() - start < args.duration:
            time.sleep(min(5.0, args.duration - (node.now() - start)))
            targets = node.count("target")
            print(f"  {node.now() - start:5.1f} s  {targets} targets")
            if targets == 0 and node.now() - start >= 5.0:
                print("  WARNING: no targets yet. Is the GELLO enabled "
                      "(make gello-enable)?")
    except KeyboardInterrupt:
        print("\nStopped.")

    duration = node.now() - start
    executor.shutdown(timeout_sec=1.0)
    with node.lock:
        rows = [(kind, t - start, q, c) for kind, t, q, c in node.rows
                if t >= start]
    rclpy.try_shutdown()
    rows.sort(key=lambda row: row[1])

    report_rates(rows, duration)
    if not any(row[0] == "target" for row in rows):
        print("ERROR: no targets recorded, nothing saved.")
        return 1

    write_rows(path, rows)
    print(f"Saved {path}")
    return 0


def cmd_replay(args):
    source = resolve(args.recording, existing=True)
    path = resolve(args.name)
    targets, _ = read_rows(source)
    if not targets:
        print(f"ERROR: {source} has no targets.")
        return 1

    from impedance_control_msgs.msg import PosRef

    rclpy, node = make_node("latency_replay", args.ns, record_targets=False)
    posref_topic = f"{args.ns}/panda_arm/posref"
    publisher = node.create_publisher(PosRef, posref_topic, 10)
    executor = spin_in_background(rclpy, node)

    def publish(position):
        msg = PosRef()
        msg.position = [float(x) for x in position]
        publisher.publish(msg)

    try:
        # Wait for the arm's state
        deadline = time.monotonic() + 10.0
        while node.last_state is None:
            if time.monotonic() > deadline:
                print(f"ERROR: no {args.ns}/panda_arm/state. "
                      "Is 'make sim-teleop' running?")
                return 1
            time.sleep(0.1)

        time.sleep(0.5)  # let discovery see the other publishers
        others = node.count_publishers(posref_topic) - 1
        if others > 0:
            print(f"NOTE: {others} other node(s) publish {posref_topic}. "
                  "The GELLO calibrator disables itself when it sees this "
                  "replay; anything else must be stopped.")

        # Bring the arm to the first target and wait until it settles there
        first = targets[0][1]
        print("Moving the arm to the recording's first pose...")
        settled_since = None
        deadline = time.monotonic() + 60.0
        while True:
            publish(first)
            _, _, position, commanded = node.last_state
            error = max(abs(a - b) for a, b in zip(position, first))
            cmd_error = max(abs(a - b) for a, b in zip(commanded, first))
            if error < 0.01 and cmd_error < 1e-3:
                settled_since = settled_since or time.monotonic()
                if time.monotonic() - settled_since > 0.5:
                    break
            else:
                settled_since = None
            if time.monotonic() > deadline:
                print(f"ERROR: the arm did not reach the first pose "
                      f"(error {error:.3f} rad).")
                return 1
            time.sleep(0.02)

        # Replay with the original timing
        duration = targets[-1][0] - targets[0][0]
        print(f"Replaying {len(targets)} targets ({duration:.1f} s)...")
        with node.lock:
            node.rows.clear()
        start = time.monotonic()
        t_first = targets[0][0]
        late = []
        for t, position in targets:
            due = start + (t - t_first)
            wait = due - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            late.append(time.monotonic() - due)
            publish(position)
            node.add(("target", node.now(), position, None))

        # Keep recording while the arm catches up
        time.sleep(args.tail)
    except KeyboardInterrupt:
        print("\nStopped, nothing saved.")
        return 1
    finally:
        executor.shutdown(timeout_sec=1.0)

    with node.lock:
        rows = list(node.rows)
    rclpy.try_shutdown()

    # Times relative to the first replayed target
    t_start = next(row[1] for row in rows if row[0] == "target")
    rows = [(kind, t - t_start, q, c) for kind, t, q, c in rows
            if t >= t_start]

    report_rates(rows, rows[-1][1])
    late.sort()
    print(f"  publish timing: {late[len(late) // 2] * 1000:.2f} ms late "
          f"(median), {late[int(len(late) * 0.99)] * 1000:.2f} ms (99%), "
          f"{late[-1] * 1000:.1f} ms (worst)")
    write_rows(path, rows)
    print(f"Saved {path}")
    return 0


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def moving_average(np, x, width):
    if width <= 1:
        return x
    kernel = np.ones(width) / width
    padded = np.concatenate([np.full(width // 2, x[0]), x,
                             np.full(width - 1 - width // 2, x[-1])])
    return np.convolve(padded, kernel, mode="valid")


def runs(np, mask):
    """[start, end) index pairs of the True runs in mask."""
    edges = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def motion_segments(np, target, velocity):
    """Moves in one direction: (start, end, peak |velocity|). A reversal
    starts a new segment."""
    merge = int(MOVE_MERGE_GAP / GRID_STEP)
    segments = []
    for sign in (1, -1):
        same = []
        for a, b in runs(np, sign * velocity > MOVE_VELOCITY):
            if same and a - same[-1][1] < merge and np.all(
                    sign * velocity[same[-1][1]:a] > -MOVE_VELOCITY):
                same[-1] = (same[-1][0], b)
            else:
                same.append((a, b))
        segments += same
    segments.sort()

    result = []
    for a, b in segments:
        if (b - a) * GRID_STEP < MOVE_MIN_DURATION:
            continue
        if np.ptp(target[a:b]) < MOVE_MIN_RANGE:
            continue
        peak = float(np.max(np.abs(velocity[a:b])))
        result.append((a, b, peak))
    return result


def best_delay(np, target, signal, a, b):
    """Shift that best overlays signal (later) on target, over [a, b)."""
    reference = target[a:b]
    step = max(1, int(DELAY_STEP / GRID_STEP))
    best, best_error = None, math.inf
    for d in range(0, int(MAX_DELAY / GRID_STEP) + 1, step):
        if b + d > len(signal):
            break
        error = float(np.mean((signal[a + d:b + d] - reference) ** 2))
        if error < best_error:
            best, best_error = d, error
    return None if best is None else best * GRID_STEP


def turning_points(target, hysteresis):
    """Reversals of the target: (index, direction it moved in before)."""
    events = []
    high = low = 0
    direction = 0
    extreme = 0
    for i in range(1, len(target)):
        if direction == 0:
            if target[i] > target[high]:
                high = i
            if target[i] < target[low]:
                low = i
            if target[high] - target[low] > hysteresis:
                direction = 1 if high > low else -1
                extreme = high if direction == 1 else low
        elif direction == 1:
            if target[i] > target[extreme]:
                extreme = i
            elif target[extreme] - target[i] > hysteresis:
                events.append((extreme, 1))
                direction, extreme = -1, i
        else:
            if target[i] < target[extreme]:
                extreme = i
            elif target[i] - target[extreme] > hysteresis:
                events.append((extreme, -1))
                direction, extreme = 1, i
    return events


def crossed(np, past):
    """Largest positive value of past after it was <= 0 at least once.

    The signal must come from behind the target and then cross it. If it
    starts beyond (still lagging from an earlier move the other way), it is
    behind, not past."""
    behind = np.flatnonzero(past <= 0)
    if len(behind) == 0:
        return 0.0
    return max(0.0, float(np.max(past[behind[0]:])))


def overshoots(np, target, signal, segments):
    """How far signal goes past the target where the target reverses or
    pauses. Returns a list of overshoots (rad, >= 0), one per event."""
    result = []

    # Reversals: from the turn to the next turn
    turns = turning_points(target, TURN_HYSTERESIS)
    for k, (i, direction) in enumerate(turns):
        end = turns[k + 1][0] if k + 1 < len(turns) else len(target)
        past = direction * (signal[i:end] - target[i])
        result.append(crossed(np, past))

    # Pauses: the target stops after a move, then stays still for a while
    band = TURN_HYSTERESIS / 2
    pause = int(PAUSE_MIN_DURATION / GRID_STEP)
    window = int(PAUSE_MAX_WINDOW / GRID_STEP)
    for a, b, _ in segments:
        if b + pause > len(target):
            continue
        held = target[b]
        if np.max(np.abs(target[b:b + pause] - held)) > band:
            continue
        direction = 1 if target[b] > target[a] else -1
        end = b
        limit = min(len(target), b + window)
        while end < limit and abs(target[end] - held) <= band:
            end += 1
        # Furthest the target has been so far: if it moves on, so may the arm
        reach = np.maximum.accumulate(direction * target[b:end])
        past = direction * signal[b:end] - reach
        result.append(crossed(np, past))

    return result


def analyze_file(np, path):
    targets, states = read_rows(path)
    if len(targets) < 2 or len(states) < 2:
        print(f"{path}: not enough data (a recording with targets only "
              "is analyzed by replaying it)")
        return None

    tt = np.array([t for t, _ in targets])
    tq = np.array([q for _, q in targets])
    st = np.array([t for t, _, _ in states])
    sq = np.array([q for _, q, _ in states])
    sc = np.array([c for _, _, c in states])

    start = max(tt[0], st[0])
    stop = min(tt[-1] + 2.0, st[-1])
    grid = np.arange(start, stop, GRID_STEP)
    hold = np.clip(np.searchsorted(tt, grid, side="right") - 1,
                   0, len(tt) - 1)
    smoothing = int(VELOCITY_SMOOTHING / GRID_STEP)

    print(f"== {os.path.basename(path)}: {tt[-1] - tt[0]:.1f} s, "
          f"targets {len(tt) / (tt[-1] - tt[0]):.1f} Hz, "
          f"state {len(st) / (st[-1] - st[0]):.0f} Hz")
    # The sim publishes state every 1 ms; a much shorter gap means the
    # recorder fell behind and received a burst (its times are off)
    bunched = float(np.mean(np.diff(st) < 0.0002))
    if bunched > 0.01:
        print(f"  WARNING: {bunched:.0%} of the state messages arrived in "
              "bursts; delays may be off by a few ms.")

    summary = {}
    for label, signals in (("cmd", sc), ("arm", sq)):
        print(f"  {label}: {'commanded position (ruckig output)' if label == 'cmd' else 'arm position'}")
        print("    joint   slow delay        fast delay        "
              "largest lag   overshoot (events > 0.005 rad)")
        all_slow, all_fast, all_lag, all_over = [], [], [], []
        for j in range(NUM_JOINTS):
            target = np.interp(grid, tt, tq[:, j])
            target_held = tq[hold, j]
            signal = np.interp(grid, st, signals[:, j])
            velocity = moving_average(
                np, np.gradient(target, GRID_STEP), smoothing)

            segments = motion_segments(np, target, velocity)
            slow, fast = [], []
            for a, b, peak in segments:
                delay = best_delay(np, target, signal, a, b)
                if delay is not None:
                    (fast if peak > FAST_VELOCITY else slow).append(delay)

            lag = float(np.max(np.abs(target_held - signal)))
            over = overshoots(np, target, signal, segments)
            counted = sum(1 for x in over if x > OVERSHOOT_REPORT)
            worst = max(over) if over else 0.0

            all_slow += slow
            all_fast += fast
            all_lag.append(lag)
            all_over += over

            print(f"    J{j + 1}     {fmt_delay(np, slow):17s} "
                  f"{fmt_delay(np, fast):17s} {lag:6.3f} rad    "
                  f"{worst:6.3f} rad ({counted}/{len(over)})")

        counted = sum(1 for x in all_over if x > OVERSHOOT_REPORT)
        worst = max(all_over) if all_over else 0.0
        print(f"    all    {fmt_delay(np, all_slow):17s} "
              f"{fmt_delay(np, all_fast):17s} {max(all_lag):6.3f} rad    "
              f"{worst:6.3f} rad ({counted}/{len(all_over)})")
        summary[label] = (all_slow, all_fast, max(all_lag), worst,
                          counted, len(all_over))
    print()
    return summary


def fmt_delay(np, delays):
    if not delays:
        return "-"
    return f"{np.median(delays) * 1000:4.0f} ms (n={len(delays)})"


def cmd_analyze(args):
    import numpy as np

    summaries = []
    for name in args.files:
        path = resolve(name, existing=True)
        if not os.path.exists(path):
            print(f"ERROR: {path} not found")
            return 1
        summaries.append((os.path.basename(path), analyze_file(np, path)))

    if len(summaries) > 1:
        print("== Summary (all joints; delay = median, lag and overshoot "
              "= max)")
        for label in ("cmd", "arm"):
            print(f"  {label}:")
            print(f"    {'file':32s} {'slow':>8s} {'fast':>8s} "
                  f"{'lag':>9s} {'overshoot':>10s}")
            for name, summary in summaries:
                if summary is None:
                    continue
                slow, fast, lag, worst, counted, total = summary[label]
                slow_ms = f"{np.median(slow) * 1000:.0f} ms" if slow else "-"
                fast_ms = f"{np.median(fast) * 1000:.0f} ms" if fast else "-"
                print(f"    {name:32s} {slow_ms:>8s} {fast_ms:>8s} "
                      f"{lag:5.3f} rad {worst:6.3f} rad")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ns", default="/sim_asimilar_01",
                        help="robot namespace (default /sim_asimilar_01)")
    sub = parser.add_subparsers(dest="command", required=True)

    record = sub.add_parser("record", help="record a live GELLO session")
    record.add_argument("name", help="output name (in tools/data/)")
    record.add_argument("--duration", type=float, default=60.0)

    replay = sub.add_parser("replay", help="replay a recording to posref")
    replay.add_argument("recording", help="recording to replay")
    replay.add_argument("name", help="output name (in tools/data/)")
    replay.add_argument("--tail", type=float, default=3.0,
                        help="seconds recorded after the last target")

    analyze = sub.add_parser("analyze", help="delay, lag and overshoot")
    analyze.add_argument("files", nargs="+")

    args = parser.parse_args()
    handler = {"record": cmd_record, "replay": cmd_replay,
               "analyze": cmd_analyze}[args.command]
    return handler(args)


if __name__ == "__main__":
    sys.exit(main())
