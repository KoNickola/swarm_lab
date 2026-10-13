"""Equilateral three-vehicle subspace formation.

The three edge constraints place the vehicles on the lines joining the
reference vertices.  Each edge permits motion along its own free direction;
the intersection of all three constraints preserves the triangle up to a
common translation.
"""

from swarm.domain.config import (
    ExperimentConfig,
    SubspaceControlConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    VehicleConfig,
)


SIDE_M = 0.40
HEIGHT_M = 0.3464101615


CONFIG = ExperimentConfig(
    name="subspace_triangle",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=(
        VehicleConfig("car3", "peer", "ground_vehicle", "10.1.1.83"),
        VehicleConfig("car6", "peer", "ground_vehicle", "10.1.1.86"),
        VehicleConfig("car7", "peer", "ground_vehicle", "10.1.1.87"),
    ),
    topology=SubspaceTopologyConfig(
        edges=(
            SubspaceEdgeConfig(
                "car3",
                "car6",
                free_direction=(0, 1.0),
                bias=(0.3, 0.0),
            ),
            SubspaceEdgeConfig(
                "car3",
                "car7",
                free_direction=(1,0),
                bias=(0,0.4),
            ),
            SubspaceEdgeConfig(
                "car6",
                "car7",
                free_direction=(-0.3, 0.4),
                bias=(0,0),
            ),
        ),
    ),
    leader_target_velocities={},
    subspace_control=SubspaceControlConfig(
        k=5,
        modes=(
            # Mode 1: common +x translation.
            (
                (1.0, 0.0),  # car3
                (1.0, 0.0),  # car6
                (1.0, 0.0),  # car7
            ),
            # # Mode 2: equilateral-triangle scale about car3.
            # (
            #     (0.0, 0.0),
            #     (SIDE_M, 0.0),
            #     (SIDE_M / 2.0, HEIGHT_M),
            # ),
        ),
        coefficients=(-0,),
    ),
)
