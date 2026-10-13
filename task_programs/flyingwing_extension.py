"""Timed wing extension demonstration; coefficient order follows flyingwing modes."""

from math import cos, pi


EXTENSION_SPEED_MPS = 0.2
EXTENSION_LEG_S = 2.0
EXTENSION_PAUSE_S = 1.0
DEMO_START_S = 5.0
WINGTIP_DELAY_S = 0.5
EXTENSION_RAMP_S = 0.25


def _leg_speed(elapsed_s):
    """Return a 0.2 m/s plateau with cosine ramps inside a two-second leg."""

    if elapsed_s <= 0.0 or elapsed_s >= EXTENSION_LEG_S:
        return 0.0
    ramp_s = min(elapsed_s, EXTENSION_LEG_S - elapsed_s)
    if ramp_s < EXTENSION_RAMP_S:
        return EXTENSION_SPEED_MPS * 0.5 * (
            1.0 - cos(pi * ramp_s / EXTENSION_RAMP_S)
        )
    return EXTENSION_SPEED_MPS


def _extension_speed(time_s, start_s):
    """Return a smooth extension/pause/retraction in effective control time."""

    elapsed_s = time_s - start_s
    if elapsed_s < 0.0:
        return 0.0
    if elapsed_s < EXTENSION_LEG_S:
        return _leg_speed(elapsed_s)
    if elapsed_s < EXTENSION_LEG_S + EXTENSION_PAUSE_S:
        return 0.0
    if elapsed_s < 2.0 * EXTENSION_LEG_S + EXTENSION_PAUSE_S:
        return -_leg_speed(elapsed_s - EXTENSION_LEG_S - EXTENSION_PAUSE_S)
    return 0.0


def extension_16_speed(time_s):
    return _extension_speed(time_s, DEMO_START_S)


def extension_28_speed(time_s):
    return _extension_speed(time_s, DEMO_START_S)


def extension_56_speed(time_s):
    return _extension_speed(time_s, DEMO_START_S + WINGTIP_DELAY_S)


def extension_78_speed(time_s):
    return _extension_speed(time_s, DEMO_START_S + WINGTIP_DELAY_S)


# Order: +x, 16, 28, 78, 56, +y.
COEFFICIENTS = (
    0.0, extension_16_speed, extension_28_speed,
    extension_78_speed, extension_56_speed, 0.0,
)
