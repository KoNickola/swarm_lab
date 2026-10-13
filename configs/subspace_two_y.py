"""Two peers maintain x_car6-x_car3=0.5 m and permit relative y motion."""

from swarm.domain.config import (
    ExperimentConfig, SubspaceControlConfig, SubspaceEdgeConfig,
    SubspaceTopologyConfig, VehicleConfig,
)
def y_speed(time_s):
    if time_s < 3.0:
        return 0.0
    if time_s<6:
        return 0.2
    if time_s<9:
        return -0.2
    return 0
CONFIG = ExperimentConfig(
    name="subspace_two",
    algorithm="subspace",
    control_space="planar_xy",
    vehicles=(
        VehicleConfig("car1", "peer", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car4", "peer", "ground_vehicle", "10.1.1.84"),
    ),
    topology=SubspaceTopologyConfig(edges=(
        SubspaceEdgeConfig(
            "car1",
            "car4",
            bias=(0, 0.0),  
            free_direction=(0, 1.0),
        ),
    )),
    leader_target_velocities={},
    subspace_control=SubspaceControlConfig(k=0.5,modes=(((0,1),(0,-1)),),coefficients=(y_speed,)),
)
