"""Four-vehicle square with center contraction followed by translation."""

from math import sqrt

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    SubspaceControlConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    VehicleConfig,
)


TASK_START_DELAY_S = 5.0
DIAGONAL_COMPONENT = 1.0 / sqrt(2.0)


def center_contraction_speed(time_s):
    """Return the inward center-symmetric speed in m/s."""

    if TASK_START_DELAY_S <= time_s < TASK_START_DELAY_S + 3.0:
        return -0.2
    if TASK_START_DELAY_S + 3.0 <= time_s < TASK_START_DELAY_S + 6.0:
        return 0.2
    return 0.0


def translate_x_speed(time_s):
    """Return the scheduled world-frame x translation speed in m/s."""

    if TASK_START_DELAY_S + 6.0 <= time_s < TASK_START_DELAY_S + 9.0:
        return -0.2
    if TASK_START_DELAY_S + 9.0 <= time_s < TASK_START_DELAY_S + 12.0:
        return 0.2 * DIAGONAL_COMPONENT
    return 0.0


def translate_y_speed(time_s):
    """Return the scheduled world-frame y translation speed in m/s."""

    if TASK_START_DELAY_S + 9.0 <= time_s < TASK_START_DELAY_S + 12.0:
        return 0.2 * DIAGONAL_COMPONENT
    return 0.0


CONFIG = ExperimentConfig(
    name="subspace_four_square",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=(
        VehicleConfig("car1", "peer", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car4", "peer", "ground_vehicle", "10.1.1.84"),
        VehicleConfig("car6", "peer", "ground_vehicle", "10.1.1.86"),
        VehicleConfig("car7", "peer", "ground_vehicle", "10.1.1.87"),
    ),
    topology=SubspaceTopologyConfig(
        edges=(
            SubspaceEdgeConfig("car1", "car4", free_direction=(1.0, 1.0)),
            SubspaceEdgeConfig("car4", "car6", free_direction=(1.0, -1.0)),
            SubspaceEdgeConfig("car6", "car7", free_direction=(-1.0, -1.0)),
            SubspaceEdgeConfig("car7", "car1", free_direction=(-1.0, 1.0)),
            SubspaceEdgeConfig("car1", "car6", free_direction=(1.0, 0)),
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
            # Positive coefficient moves each corner toward the square center.
            (
                (-1,0),
                (0,1),
                (1,0),
                (0,-1),
            ),
            # Common world-frame x translation; rows follow car1, car4, car6, car7.
            ((1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)),
            # Common world-frame y translation.
            ((0.0, 1.0), (0.0, 1.0), (0.0, 1.0), (0.0, 1.0)),
        ),
        coefficients=(
            center_contraction_speed,
            translate_x_speed,
            translate_y_speed,
        ),
    ),
)
