"""八车构型：中间刚性正方形，左右各两车构成平行边。

世界坐标系 +x 向右，+y 向上：

    car1 ●────● car2 ────● car3 ────● car4
         │    │    ╲    │         │
    car5 ●────● car6 ────● car7 ────● car8

中间正方形包含 car2-car7 对角线；两侧不添加对角线，保留侧边
沿 x 方向伸缩的自由度。bearing_only 不指定绝对边长。
"""

from swarm.domain.config import ExperimentConfig, ObstacleAvoidanceConfig, TopologyConfig, VehicleConfig, FormationControlConfig


# Indices 0 through 7 correspond to car1 through car8.
_BEARINGS = [[(0.0, 0.0, 0.0) for _ in range(8)] for _ in range(8)]

# Each directed edge is explicit; reverse edges are not generated automatically.
_BEARINGS[0][1] = (1.0, 0.0, 0.0)
_BEARINGS[1][0] = (-1.0, 0.0, 0.0)
_BEARINGS[0][4] = (0.0, -1.0, 0.0)
_BEARINGS[4][0] = (0.0, 1.0, 0.0)
_BEARINGS[4][5] = (1.0, 0.0, 0.0)
_BEARINGS[5][4] = (-1.0, 0.0, 0.0)

_BEARINGS[1][2] = (1.0, 0.0, 0.0)
_BEARINGS[2][1] = (-1.0, 0.0, 0.0)
_BEARINGS[1][5] = (0.0, -1.0, 0.0)
_BEARINGS[5][1] = (0.0, 1.0, 0.0)
_BEARINGS[1][6] = (1.0, -1.0, 0.0)
_BEARINGS[6][1] = (-1.0, 1.0, 0.0)
_BEARINGS[2][6] = (0.0, -1.0, 0.0)
_BEARINGS[6][2] = (0.0, 1.0, 0.0)
_BEARINGS[5][6] = (1.0, 0.0, 0.0)
_BEARINGS[6][5] = (-1.0, 0.0, 0.0)

_BEARINGS[2][3] = (1.0, 0.0, 0.0)
_BEARINGS[3][2] = (-1.0, 0.0, 0.0)
_BEARINGS[3][7] = (0.0, -1.0, 0.0)
_BEARINGS[7][3] = (0.0, 1.0, 0.0)
_BEARINGS[6][7] = (1.0, 0.0, 0.0)
_BEARINGS[7][6] = (-1.0, 0.0, 0.0)

CONFIG = ExperimentConfig(
    name="bearing_eight",
    algorithm="bearing",
    control_space="planar_xy",
    vehicles=tuple(
        VehicleConfig(
            "car%d" % index,
            "leader" if index == 2 else "follower",
            "ground_vehicle",
            "10.1.1.%d" % (80 + index),
        )
        for index in range(1, 9)
    ),
    topology=TopologyConfig(
        adjacency_matrix=tuple(
            tuple(int(any(vector)) for vector in row) for row in _BEARINGS
        ),
        bearing_matrix=tuple(tuple(row) for row in _BEARINGS),
        leader_mask=(False, True, False, False, False, False, False, False),
    ),
    leader_target_velocities={"car2": (0.0, -0.2, 0.0)},
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
    formation_control=FormationControlConfig(leader_bearing_enabled=False)
)
