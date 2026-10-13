"""Eight-vehicle flying wing with free edge lengths.

World +x points right and +y points down in this diagram:

    5                 7
    |                 |
    6     1-----2     8
           ╲   ╱
             3
             |
             4

The central triangle has edge directions 0, 60 and 120 degrees from +x.
Subspace constraints preserve edge lines, not signed ordering or distances.
Task modes provide x/y translation and independent wing-joint extensions.
"""

from math import sqrt

from swarm.domain.config import (
    ExperimentConfig,
    SubspaceControlConfig,
    ObstacleAvoidanceConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    VehicleConfig,
)
from swarm.domain.tasks import TaskFeedbackConfig
from task_programs.flyingwing_extension import COEFFICIENTS
from task_programs.flyingwing_goals import PROGRAM


SIN_60 = sqrt(3.0) / 2.0
# Enable after setting world-frame targets in task_programs/flyingwing_goals.py.
USE_TASK_PROGRAM = True


CONFIG = ExperimentConfig(
    name="flyingwing",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "peer",
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in (1, 2, 3, 5, 6, 7, 8)
    ),
    topology=SubspaceTopologyConfig(
        edges=(
            # Central equilateral triangle; its common scale remains free.
            SubspaceEdgeConfig("car1", "car2", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car1", "car3", free_direction=(0.5, SIN_60)),
            SubspaceEdgeConfig("car2", "car3", free_direction=(-0.5, SIN_60)),
            # Wings attach horizontally; each outer pair is vertical.
            SubspaceEdgeConfig("car1", "car6", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car5", "car6", free_direction=(0.0, 1.0)),
            SubspaceEdgeConfig("car2", "car8", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car7", "car8", free_direction=(0.0, 1.0)),
        ),
    ),
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True,avoid_other_vehicles=True),
    leader_target_velocities={},
    subspace_control=SubspaceControlConfig(
        k=1.0,
        modes=(
            # Mode 1: common +x translation; rows follow configured vehicle order.
            (
                (1.0, 0.0),  # car1
                (1.0, 0.0),  # car2
                (1.0, 0.0),  # car3
                (1.0, 0.0),  # car5
                (1.0, 0.0),  # car6
                (1.0, 0.0),  # car7
                (1.0, 0.0),  # car8
            ),
            # Mode 2: extend 1-6 along -x; car5 follows car6.
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (1.0, 0.0),
                (1.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ),
            # Mode 3: extend 2-8 along +x; car7 follows car8.
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (-1.0, 0.0),
                (-1.0, 0.0),
            ),
            # Mode 4: extend 7-8 by moving car7 along -y.
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 1.0),
                (0.0, 0.0),
            ),
            # Mode 5: extend 5-6 by moving car5 along -y.
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 1.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ),
            # Mode 6: common +y translation.
            tuple((0.0, 1.0) for _ in range(7)),
        ),
        coefficients=() if USE_TASK_PROGRAM else COEFFICIENTS,
        task_feedback=TaskFeedbackConfig(PROGRAM) if USE_TASK_PROGRAM else None,
    ),
)
