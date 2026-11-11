"""Gripper logic of the GELLO calibrator: trigger to gripper commands.

No ROS here: the calibrator feeds it the trigger and the gripper width, and
sends the commands it returns. The gripper service blocks until a move ends
and queues calls, so the calibrator sends one at a time and asks for the
next command only when it can send it.
"""

from dataclasses import dataclass


@dataclass
class GripperCommand:
    cmd: str              # "move" or "grasp"
    width: float          # m
    force: float = 0.0    # N
    epsilon_inner: float = 0.0
    epsilon_outer: float = 0.0


class GripperControl:
    def __init__(self, max_width, min_change, enable_tolerance, state_timeout,
                 state_topic, grasp, grasp_threshold, grasp_release,
                 grasp_force):
        self.max_width = max_width
        self.min_change = min_change
        self.enable_tolerance = enable_tolerance
        self.state_timeout = state_timeout
        self.state_topic = state_topic
        self.grasp = grasp
        self.grasp_threshold = grasp_threshold
        self.grasp_release = grasp_release
        self.grasp_force = grasp_force

        self.trigger = None         # 0.0 open .. 1.0 closed
        self.trigger_time = None
        self.width = None           # m, measured
        self.width_time = None
        self.width_sent = None      # m, last 'move' sent
        self.grasping = False

    def set_trigger(self, value, now):
        self.trigger = min(max(float(value), 0.0), 1.0)
        self.trigger_time = now

    def set_width(self, width, now):
        self.width = width
        self.width_time = now

    def trigger_fresh(self, now, timeout):
        return self.trigger_time is not None and now - self.trigger_time < timeout

    def trigger_width(self):
        return self.max_width * (1.0 - self.trigger)

    def reset(self):
        """On enable: the gripper may have moved, so command it again."""
        self.width_sent = None
        self.grasping = False

    def enable_check(self, now):
        """Whether the trigger lets control start: (ok, problem, hint)."""
        # A GELLO without trigger
        if self.trigger_time is None:
            return True, None, None

        if (self.width_time is None
                or now - self.width_time >= self.state_timeout):
            return (False,
                    f"No gripper width on {self.state_topic}.",
                    "gripper state unavailable")

        # A grasp only closes the gripper, so it can't drop anything
        if self.grasp and self.trigger >= self.grasp_threshold:
            return True, None, None

        difference = self.trigger_width() - self.width
        if abs(difference) <= self.enable_tolerance:
            return True, None, None

        # A positive difference: the trigger asks for a wider gripper
        action = "press the trigger" if difference > 0 else "release the trigger"
        return (False,
                f"Trigger doesn't match the gripper: "
                f"{difference * 1000:+.0f} mm (tolerance "
                f"{self.enable_tolerance * 1000:.0f} mm).",
                f"gripper {abs(difference) * 1000:.0f} mm, {action}")

    def next_command(self):
        """The command to send now, or None. Assumes it will be sent."""
        if self.grasp:
            if self.grasping:
                # Hold until the trigger is released past the hysteresis
                if self.trigger >= self.grasp_release:
                    return None
                self.grasping = False
                self.width_sent = None
            elif self.trigger >= self.grasp_threshold:
                # Close on whatever is between the fingers: with width 0 and
                # epsilon_outer at full width, any final width succeeds
                self.grasping = True
                return GripperCommand(
                    "grasp", 0.0, force=self.grasp_force,
                    epsilon_inner=0.005, epsilon_outer=self.max_width)

        width = self.trigger_width()
        if (self.width_sent is not None
                and abs(width - self.width_sent) < self.min_change):
            return None

        self.width_sent = width
        return GripperCommand("move", width)
