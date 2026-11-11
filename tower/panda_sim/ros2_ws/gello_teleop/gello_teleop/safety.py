"""Safety checks of the GELLO calibrator. Pure functions, no ROS."""

import math


def valid_joints(values, count=7):
    """The values as a list if there are `count` finite ones, else None."""
    values = list(values)
    if len(values) != count or not all(math.isfinite(x) for x in values):
        return None
    return values


def clip_to_limits(positions, lower, upper):
    return [
        max(low, min(high, position))
        for position, low, high in zip(positions, lower, upper)
    ]


def pose_differences(gello, panda, lower, upper):
    """What the Panda would be commanded minus where it is, per joint."""
    return [
        target - actual
        for target, actual in zip(clip_to_limits(gello, lower, upper), panda)
    ]


def joint_jumped(current, previous, max_change):
    """Whether any joint moved more than max_change since the last sample."""
    if previous is None:
        return False
    return any(
        abs(now - before) > max_change
        for now, before in zip(current, previous)
    )


def stale_sources(sources, now, timeout):
    """Which inputs are missing or too old, as messages.

    sources: (name, topic, time of the last message or None).
    """
    stale = []
    for name, topic, stamp in sources:
        if stamp is None:
            stale.append(f"{name} ({topic}): no data yet")
        elif now - stamp >= timeout:
            stale.append(f"{name} ({topic}): {now - stamp:.2f} s old")
    return stale
