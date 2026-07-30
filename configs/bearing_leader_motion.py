"""Single-leader experiment for testing configurable motion references.

Set ``REFERENCE_MODE`` to ``"time_velocity"``, ``"time_position"``, or
``"xy_curve"`` before starting the experiment. Time-position coordinates and
the static curve are expressed in the motion-capture world frame.
"""

from __future__ import annotations

from math import cos, sin

from swarm.domain.config import (
    ExperimentConfig,
    LeaderTargetVelocity,
    PIDConfig,
    TimePositionReference,
    TimeVelocityReference,
    TopologyConfig,
    VehicleConfig,
    XYCurveReference,
)


# Select one input reference for the active experiment.
REFERENCE_MODE = "xy_curve"

# Shared outer position-loop parameters for time-position and XY-curve modes.
POSITION_PID_KP = 0.8
POSITION_PID_KI = 0.0
POSITION_PID_KD = 0.0
POSITION_PID_INTEGRAL_LIMIT = 0.5

# Time-velocity input parameters use metres, seconds, and radians.
FORWARD_SPEED_MPS = -0.1
LATERAL_SPEED_AMPLITUDE_MPS = 0.2
TIME_FREQUENCY_RAD_S = 2

# Time-position input parameters define an absolute world-frame circle.
# Set the absolute position origin to the measured start point before using
# time-position mode.
POSITION_ORIGIN_X_M = 0.0
POSITION_ORIGIN_Y_M = 0.0
POSITION_RADIUS_M = 0.10
POSITION_ANGULAR_SPEED_RAD_S = 0.25

# XY-curve input parameters define a static world-frame curve.
# Before using XY-curve mode, set the offset so that curve_y(start_x) is
# approximately the measured start_y.
# The static world-frame curve is y = offset + amplitude * sin(k * x).
CURVE_Y_OFFSET_M = 0.0
CURVE_AMPLITUDE_M = 0.10
CURVE_WAVE_NUMBER_RAD_PER_M = 3.0
CURVE_SPEED_MPS = 0.2


def leader_time_velocity(time_s: float):
    """Return the world-frame velocity reference at elapsed control time."""

    return (
        FORWARD_SPEED_MPS,
        LATERAL_SPEED_AMPLITUDE_MPS * sin(TIME_FREQUENCY_RAD_S * time_s),
        0.0,
    )


def leader_time_position(time_s: float):
    """Return an absolute world-frame circular position reference."""

    phase = POSITION_ANGULAR_SPEED_RAD_S * time_s
    return (
        POSITION_ORIGIN_X_M + POSITION_RADIUS_M * sin(phase),
        POSITION_ORIGIN_Y_M + POSITION_RADIUS_M * (1.0 - cos(phase)),
    )


def leader_xy_curve(x_m: float) -> float:
    """Return the static world-frame curve y coordinate for a given x."""

    return CURVE_Y_OFFSET_M + CURVE_AMPLITUDE_M * sin(
        CURVE_WAVE_NUMBER_RAD_PER_M * x_m
    )


def build_leader_reference(mode: str) -> LeaderTargetVelocity:
    """Build one configured reference without changing planner interfaces."""

    references = {
        "time_velocity": TimeVelocityReference(leader_time_velocity),
        "time_position": TimePositionReference(leader_time_position),
        "xy_curve": XYCurveReference(leader_xy_curve, CURVE_SPEED_MPS),
    }
    try:
        return references[mode]
    except KeyError:
        raise ValueError(
            "unsupported REFERENCE_MODE %r; expected one of: %s"
            % (mode, ", ".join(sorted(references)))
        ) from None


CONFIG = ExperimentConfig(
    name="bearing_leader_motion",
    algorithm="bearing",
    control_space="planar_xy",
    vehicles=(
        VehicleConfig(
            "car5",
            "leader",
            "ground_vehicle",
            "10.1.1.85",
        ),
    ),
    topology=TopologyConfig(
        adjacency_matrix=((0,),),
        bearing_matrix=(((0.0, 0.0, 0.0),),),
        leader_mask=(True,),
    ),
    leader_target_velocities={
        "car5": build_leader_reference(REFERENCE_MODE),
    },
    leader_position_pid=PIDConfig(
        POSITION_PID_KP,
        POSITION_PID_KI,
        POSITION_PID_KD,
        POSITION_PID_INTEGRAL_LIMIT,
        None,
    ),
)
