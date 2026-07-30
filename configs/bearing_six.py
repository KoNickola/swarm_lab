"""原项目中保留的六车柔性编队实验。

世界坐标系与期望构型：

          +y
           ↑
    car1 ●────● car2  → +x
         │  ╲ │
    car3 ●────● car4
         │    │
    car5 ●────● car6

连线用于辅助理解构型；实际有效边及方向以拓扑矩阵为准。
"""


from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    TimeVelocityReference,
    XYCurveReference,
    TimePositionReference,
    TopologyConfig,
    VehicleConfig,
    FormationControlConfig
)


# _BEARINGS[source][target] independently defines one directed edge.
# Vehicle indices 0 through 5 correspond to car1 through car6.
_BEARINGS = [
    [(0.0, 0.0, 0.0) for _ in range(6)]
    for _ in range(6)
]


_BEARINGS[1][0] = (-1.0, 0.0, 0.0)
_BEARINGS[2][0] = (0.0, 1.0, 0.0)
_BEARINGS[2][3] = (1.0, 0.0, 0.0)
_BEARINGS[3][0] = (-1.0, 1.0, 0.0)
_BEARINGS[3][1] = (0.0, 1.0, 0.0)
_BEARINGS[4][2] = (0.0, 1.0, 0.0)
_BEARINGS[4][5] = (1.0, 0.0, 0.0)
_BEARINGS[5][3] = (0.0, 1.0, 0.0)

_ADJACENCY = tuple(
    tuple(int(any(abs(value) > 0.0 for value in vector)) for vector in row)
    for row in _BEARINGS
)

def car2_time_velocity(time_s: float):
    """Alternate between +x and -x every two seconds."""

    phase_s = time_s % (2.0 * 2)
    vx = 0.2 if phase_s < 2 else -0.2
    return vx, 0.0, 0.0

def car6_time_velocity(time_s: float):
    """Alternate between +x and -x every two seconds."""

    phase_s = time_s % (3.0 * 2)
    vy = 0.3 if phase_s < 3 else -0.3
    return 0.0, vy, 0.0

def car1_target_position(time_s: float):

    return -0.2, 0.8, 0.0
CONFIG = ExperimentConfig(
    name="bearing_six",
    algorithm="bearing",
    control_space="planar_xy",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "leader" if index in (1, 2, 6) else "follower",
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in range(1, 7)
    ),
    topology=TopologyConfig(
        adjacency_matrix=_ADJACENCY,
        bearing_matrix=tuple(tuple(row) for row in _BEARINGS),
        leader_mask=(True, True, False, False, False, True),
    ),
    leader_target_velocities={
        # "car1": (-0.2, 0.0, 0.0),
        "car1": TimePositionReference(car1_target_position),
        # "car2": TimeVelocityReference(car2_time_velocity),
        # "car6": TimeVelocityReference(car6_time_velocity),
        "car2": (0.0, 0.0, 0.0),
        "car6": (0.0, 0.0, 0.0)
    },
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
    formation_control=FormationControlConfig(leader_bearing_enabled=True)

)
