"""Four-vehicle rectangle with translation and center-symmetric scale tasks."""

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    SubspaceControlConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    VehicleConfig,
)


TASK_START_DELAY_S = 5.0


def translate_x_speed(time_s):
    """Return the scheduled world-frame x translation speed in m/s."""

    if time_s < 18:
        return 0.0
    if time_s < 21:
        return 0.2
    return 0.0


def translate_y_speed(time_s):
    """Return the scheduled world-frame y translation speed in m/s."""

    time_s -= TASK_START_DELAY_S
    if 15.0 <= time_s < 18.0:
        return -0.2
    return 0.0





def x_stretch_speed(time_s):
    """Return signed center-symmetric x stretch speed in m/s."""
    if time_s < 3:
        return 0.0
    if 3.0 <= time_s < 8.0:
        return 0.2
    if 8.0 <= time_s < 12.0:
        return -0.2
    return 0.0
    return _axis_stretch_speed(time_s)


def y_stretch_speed(time_s):
    if time_s < 6:
        return 0.0
    if 6.0 <= time_s < 11.0:
        return -0.2
    if 11.0 <= time_s < 15.0:
        return 0.2
    return 0.0

CONFIG = ExperimentConfig(
    name="subspace_four_rectangle",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=(
        VehicleConfig("car1" , "peer", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car4" , "peer", "ground_vehicle", "10.1.1.84"),
        VehicleConfig("car6" , "peer", "ground_vehicle", "10.1.1.86"),
        VehicleConfig("car7" , "peer", "ground_vehicle", "10.1.1.87")
    ),
    topology=SubspaceTopologyConfig(
        edges=(
            SubspaceEdgeConfig("car1", "car4", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car4", "car6", free_direction=(0.0, 1.0)),
            SubspaceEdgeConfig("car6", "car7", free_direction=(-1.0, 0.0)),
            SubspaceEdgeConfig("car7", "car1", free_direction=(0.0, -1.0)),
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
            # Common world-frame x translation; rows follow car1, car4, car6, car7.
            (
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
            ),
            # Common world-frame y translation.
            (
                (0.0, 1.0),
                (0.0, 1.0),
                (0.0, 1.0),
                (0.0, 1.0),
            ),
            # Positive speed stretches the rectangle symmetrically along x.
            (
                (-1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (-1.0, 0.0),
            ),
            # Positive speed stretches the rectangle symmetrically along y.
            (
                (0.0, 1.0),
                (0.0, 1.0),
                (0.0, -1.0),
                (0.0, -1.0),
            ),
        ),
        coefficients=(
            translate_x_speed,
            translate_y_speed,
            x_stretch_speed,
            y_stretch_speed,
        ),
    ),
)
