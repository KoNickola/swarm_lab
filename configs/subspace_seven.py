"""Seven-vehicle square with a vertically extensible lower section.

Vehicle mapping follows the diagram: I/J/K/L/M/N/O map to car1/car2/.../car7.
The square is fixed by full relative-position constraints. M and N connect
below the square with vertical free edges and a horizontal free edge, so the
lower segment can extend or contract along y. O is offset from J by 0.3 m in
x and remains free along y.
"""

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    SubspaceControlConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    RuntimeConfig,
    VehicleConfig,
)


SIDE_M = 0.40


def mn_extension_speed(time_s):
    """Return the piecewise M-N vertical task speed in m/s."""

    if time_s < 8.0:
        return 0.0
    if time_s < 11.0:
        return -0.2
    if time_s < 16.0:
        return 0.0
    if time_s < 20.0:
        return 0.2
    return 0.0

def xspeed(time_s):
    if time_s < 5.0:
        return 0.0
    return 0.2
def square_scale_speed(time_s):
    """Return the independent square-scale mode coefficient."""

    _ = time_s
    return 0.0


CONFIG = ExperimentConfig(
    name="subspace_seven",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "peer",
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in range(1, 8)
    ),
    obstacle_avoidance=ObstacleAvoidanceConfig(
        enabled=True),
    topology=SubspaceTopologyConfig(
        edges=(
            # I-J-K-L: all edges use the same subspace form; dimensions are free.
            SubspaceEdgeConfig(
                "car1", "car2", free_direction=(1.0, 0.0), bias=(SIDE_M, 0.0)
            ),
            SubspaceEdgeConfig(
                "car1", "car3", free_direction=(0.0, 1.0), bias=(0.0, -SIDE_M)
            ),
            SubspaceEdgeConfig(
                "car2", "car4", free_direction=(0.0, 1.0), bias=(0.0, -SIDE_M)
            ),
            SubspaceEdgeConfig(
                "car3", "car4", free_direction=(1.0, 0.0), bias=(SIDE_M, 0.0)
            ),
            SubspaceEdgeConfig(
                "car1", "car4", free_direction=(1.0, -1.0), bias=(SIDE_M, -SIDE_M)
            ),
            # K-M-N-L: common vertical extension below the square.
            SubspaceEdgeConfig(
                "car3", "car5", free_direction=(0.0, 1.0), bias=(0.0, -SIDE_M)
            ),
            SubspaceEdgeConfig(
                "car4", "car6", free_direction=(0.0, 1.0), bias=(0.0, -SIDE_M)
            ),
            SubspaceEdgeConfig(
                "car5", "car6", free_direction=(1.0, 0.0), bias=(SIDE_M, 0.0)
            ),
            # O is 0.3 m to the right of J and free along y.
            SubspaceEdgeConfig(
                "car2", "car7", free_direction=(0.0, 1.0), bias=(0.3, 0.0)
            ),
        ),

    ),
    leader_target_velocities={},
    runtime=RuntimeConfig(duration_s=25),
    subspace_control=SubspaceControlConfig(
        k=2,
        modes=(
            # Mode 1: common +x translation of all seven vehicles.
            (
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
            ),
            # Mode 2: move the M-N edge up/down together.
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 1.0),
                (0.0, 1.0),
                (0.0, 0.0),
            ),
            # Mode 3: scale the square while preserving the 34-56 gap.
            # M/N follow K/L respectively; O follows J in x to preserve J-O.
            (
                (0.0, 0.0),
                (1.0, 0.0),
                (0.0, -1.0),
                (1.0, -1.0),
                (0.0, -1.0),
                (1.0, -1.0),
                (1.0, 0.0),
            ),
            # Mode 4: move O/car7 up and down along the J-O free direction.
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 1.0),
            ),
        ),
        # Each mode has an independent scalar or time-dependent scalar.
        # coefficients=(-0.0, 0, 0, 0.0),
        coefficients=(xspeed, mn_extension_speed, 0, 0.0),
    ),
)
