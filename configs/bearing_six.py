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
    TopologyConfig,
    VehicleConfig,
)


_BEARINGS = [[[0.0, 0.0, 0.0] for _ in range(6)] for _ in range(6)]
for source, target, vector in (
    (0, 1, (1.0, 0.0, 0.0)),
    (0, 2, (0.0, -1.0, 0.0)),
    (0, 3, (1.0, -1.0, 0.0)),
    (1, 3, (0.0, -1.0, 0.0)),
    (2, 3, (1.0, 0.0, 0.0)),
    (2, 4, (0.0, -1.0, 0.0)),
    (3, 5, (0.0, -1.0, 0.0)),
    (4, 5, (1.0, 0.0, 0.0)),
):
    _BEARINGS[source][target] = list(vector)
    _BEARINGS[target][source] = [-value for value in vector]

_ADJACENCY = tuple(
    tuple(int(any(abs(value) > 0.0 for value in _BEARINGS[i][j])) for j in range(6))
    for i in range(6)
)

CONFIG = ExperimentConfig(
    name="bearing_six",
    algorithm="bearing",
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
        bearing_matrix=tuple(tuple(tuple(vector) for vector in row) for row in _BEARINGS),
        leader_mask=(True, True, False, False, False, True),
    ),
    leader_target_velocities={
        "car1": (0, 0.0, 0.0),
        "car2": (0, 0.0, 0.0),
        "car6": (0, 0.0, 0.0),
    },
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
)
