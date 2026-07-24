"""四车正方形编队实验。

世界坐标系与期望构型：

          +y
           ↑
    car1 ●────● car2  → +x
         │  ╲ │
    car3 ●────● car4

该构型取自六车构型的上部 ``2×2`` 子构型。car1 和 car2 为静止
leader，car3 和 car4 为 follower。bearing-only 控制约束相对方向
和构型比例，不单独指定正方形边长。
"""

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="bearing_four",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car1", "leader", "ground_vehicle", "10.1.1.81"),
        VehicleConfig("car2", "leader", "ground_vehicle", "10.1.1.82"),
        VehicleConfig("car3", "follower", "ground_vehicle", "10.1.1.83"),
        VehicleConfig("car4", "follower", "ground_vehicle", "10.1.1.84"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=(
            (0, 1, 1, 1),
            (1, 0, 0, 1),
            (1, 0, 0, 1),
            (1, 1, 1, 0),
        ),
        bearing_matrix=(
            (
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
                (1.0, -1.0, 0.0),
            ),
            (
                (-1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
            ),
            (
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
            ),
            (
                (-1.0, 1.0, 0.0),
                (0.0, 1.0, 0.0),
                (-1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        leader_mask=(True, True, False, False),
    ),
    leader_target_velocities={
        "car1": (0.0, 0.0, 0.0),
        "car2": (0.0, 0.0, 0.0),
    },
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
)
