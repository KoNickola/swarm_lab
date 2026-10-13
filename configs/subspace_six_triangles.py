"""Six-vehicle subspace formation with two side-by-side equilateral triangles.

World-frame target geometry:

        car3 ●                 ● car6
             ╱ ╲               ╱ ╲
            ╱   ╲             ╱   ╲
    car1 ●───────● car2  car4 ●───────● car5  → +x

Each triangle has three line constraints. Its edge lengths remain free.
"""

from math import sqrt

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    SubspaceControlConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    VehicleConfig,
)


SIN_60 = sqrt(3.0) / 2.0
def x_speed(time_s):

    if time_s<5:
        return 0
    if time_s < 8.0:
        return 0.2
    if time_s <12:
        return 0
    if time_s <18:
        return 0.2
    return 0
def change_speed(time_s):

    if time_s < 8.0:
        return 0.0
    if time_s < 12.0:
        return 0.2

    return 0.0
CONFIG = ExperimentConfig(
    name="subspace_six_triangles",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "peer",
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in range(1, 7)
    ),
    topology=SubspaceTopologyConfig(
        edges=(
            # Left equilateral triangle: car1, car2, car3.
            SubspaceEdgeConfig("car1", "car2", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car1", "car3", free_direction=(0.5, SIN_60)),
            SubspaceEdgeConfig("car2", "car3", free_direction=(-0.5, SIN_60)),
            # Right equilateral triangle: car4, car5, car6.
            SubspaceEdgeConfig("car4", "car5", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car4", "car6", free_direction=(0.5, SIN_60)),
            SubspaceEdgeConfig("car5", "car6", free_direction=(-0.5, SIN_60)),
            SubspaceEdgeConfig("car3", "car6", bias=(-0.5,0.5), free_direction=(1.0, 1.0)),
        ),
    ),
    leader_target_velocities={},
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True,avoid_other_vehicles=True),
    subspace_control=SubspaceControlConfig(
        k=1.0,
        modes=(
            # Common +x translation; rows follow car1 through car6.
            (
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
            ),

            (
                (1,1),(1,1),(1,1),(-1,-1),(-1,-1),(-1,-1),
            ),
        ),
        coefficients=(x_speed,change_speed,),
        # coefficients=(0,0,),

    ),
)
