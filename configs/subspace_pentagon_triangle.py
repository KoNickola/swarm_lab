"""Pentagon and triangle connected by two vertical constraints."""

from math import cos, pi, sin

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    SubspaceControlConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    VehicleConfig,
)


SIN_36 = sin(pi / 5.0)
COS_36 = cos(pi / 5.0)
SIN_72 = sin(2.0 * pi / 5.0)
COS_72 = cos(2.0 * pi / 5.0)
SIN_60 = sin(pi / 3.0)
SIDE_M = 1.0

# Nominal coordinates define polygon scaling about each horizontal base center.
_NOMINAL_POSITIONS = (
    (0.0, 0.0),
    (SIDE_M, 0.0),
    (SIDE_M * (1.0 + COS_72), SIDE_M * SIN_72),
    (SIDE_M * 0.5, SIDE_M * (SIN_72 + SIN_36)),
    (SIDE_M * (0.5 - COS_36), SIDE_M * SIN_72),
    (SIDE_M, -SIDE_M),
    (0.0, -SIDE_M),
    (SIDE_M * 0.5, SIDE_M * (-1.0 - SIN_60)),
)
_SCALE_CENTERS = ((SIDE_M * 0.5, 0.0),) * 5 + (
    (SIDE_M * 0.5, -SIDE_M),
) * 3
_SCALE_MODE = tuple(
    tuple((position[axis] - center[axis]) / SIDE_M for axis in range(2))
    for position, center in zip(_NOMINAL_POSITIONS, _SCALE_CENTERS)
)
def expand_speed(time_s):
    if time_s < 5:
        return 0.0
    if time_s < 8:
        return -0.2
    if time_s < 15:
        return 0.0
    return 0
def extend_speed(time_s):
    if time_s < 5:
        return 0.0
    if time_s < 9:
        return -0.2
    return 0
def xspeed(time_s):
    if time_s < 7:
        return 0.0
    if time_s < 10:
        return -0.2
    return 0
def yspeed(time_s):
    if time_s < 7:
        return 0.0
    if time_s < 10:
        return -0.2
    return 0
CONFIG = ExperimentConfig(
    name="subspace_pentagon_triangle",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "peer",
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in range(1, 9)
    ),
    topology=SubspaceTopologyConfig(
        edges=(
            # Pentagon perimeter; car1-car2 is the horizontal reference edge.
            SubspaceEdgeConfig("car1", "car2", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car2", "car3", free_direction=(COS_72, SIN_72)),
            SubspaceEdgeConfig("car3", "car4", free_direction=(-COS_36, SIN_36)),
            SubspaceEdgeConfig("car4", "car5", free_direction=(-COS_36, -SIN_36)),
            SubspaceEdgeConfig("car5", "car1", free_direction=(COS_72, -SIN_72)),
            # Pentagon triangulation preserves its edge directions.
            SubspaceEdgeConfig("car1", "car3", free_direction=(COS_36, SIN_36)),
            SubspaceEdgeConfig("car1", "car4", free_direction=(COS_72, SIN_72)),
            # Triangle; car8 lies on the +y side of edge car6-car7.
            SubspaceEdgeConfig("car6", "car7", free_direction=(-1.0, 0.0)),
            SubspaceEdgeConfig("car7", "car8", free_direction=(0.5, -SIN_60)),
            SubspaceEdgeConfig("car8", "car6", free_direction=(0.5, SIN_60)),
            # Vertical links align car1 with car7 and car2 with car6 in x.
            SubspaceEdgeConfig("car1", "car7", free_direction=(0.0, 1.0)),
            SubspaceEdgeConfig("car2", "car6", free_direction=(0.0, 1.0)),
        ),
    ),
    obstacle_avoidance=ObstacleAvoidanceConfig(
        enabled=True,
        avoid_other_vehicles=True,
    ),
    leader_target_velocities={},
    subspace_control=SubspaceControlConfig(
        k=1.0,
        modes=(
            # Common world-frame x translation; rows follow car1 through car8.
            tuple((1.0, 0.0) for _ in range(8)),
            # Common world-frame y translation.
            tuple((0.0, 1.0) for _ in range(8)),
            # Move the triangle +y while moving the pentagon -y.
            (
                (0.0, -1.0),
                (0.0, -1.0),
                (0.0, -1.0),
                (0.0, -1.0),
                (0.0, -1.0),
                (0.0, 1.0),
                (0.0, 1.0),
                (0.0, 1.0),
            ),
            # Positive coefficient expands both polygons about their base centers.
            _SCALE_MODE,
        ),
        coefficients=(xspeed, yspeed, extend_speed, expand_speed),
    ),
)
