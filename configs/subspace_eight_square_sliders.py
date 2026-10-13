"""Eight-vehicle square with four independently sliding arms."""

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    SubspaceControlConfig,
    SubspaceEdgeConfig,
    SubspaceTopologyConfig,
    VehicleConfig,
)


_VEHICLE_IDS = tuple("car%d" % index for index in range(1, 9))


CONFIG = ExperimentConfig(
    name="subspace_eight_square_sliders",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=tuple(
        VehicleConfig(vehicle_id, "peer", "ground_vehicle", "10.1.1.%d" % (80 + index))
        for index, vehicle_id in enumerate(_VEHICLE_IDS, start=1)
    ),
    topology=SubspaceTopologyConfig(
        edges=(
            # The four corner vehicles retain the square geometry.
            SubspaceEdgeConfig("car1", "car2", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car2", "car3", free_direction=(0.0, -1.0)),
            SubspaceEdgeConfig("car3", "car4", free_direction=(-1.0, 0.0)),
            SubspaceEdgeConfig("car4", "car1", free_direction=(0.0, 1.0)),
            SubspaceEdgeConfig("car1", "car3", free_direction=(1.0, -1.0)),
            # Each outer vehicle slides along its arm direction.
            SubspaceEdgeConfig("car1", "car5", free_direction=(0.0, 1.0)),
            SubspaceEdgeConfig("car2", "car6", free_direction=(1.0, 0.0)),
            SubspaceEdgeConfig("car3", "car7", free_direction=(0.0, -1.0)),
            SubspaceEdgeConfig("car4", "car8", free_direction=(-1.0, 0.0)),
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
            tuple((1.0, 0.0) for _ in _VEHICLE_IDS),
            # Common world-frame y translation.
            tuple((0.0, 1.0) for _ in _VEHICLE_IDS),
            # Independent arm extension modes, in car5 through car8 order.
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 1.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ),
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (1.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
            ),
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, -1.0),
                (0.0, 0.0),
            ),
            (
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (0.0, 0.0),
                (-1.0, 0.0),
            ),
            ((1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),(1,0),),
            ((-1,1), (1,1), (1,-1), (-1,-1), (-1,0), (0,1), (1,0), (0,-1)),
        ),
        coefficients=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,-0.2,0),
    ),
)
