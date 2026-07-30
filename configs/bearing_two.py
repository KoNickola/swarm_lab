"""当前两车 leader/follower 实验。

世界坐标系与期望构型：

    car2 ●
         ↑ +y
         │
    car1 ●────────→ +x

期望 bearing 为 car1 指向 car2 的世界坐标 +y 方向。bearing-only
控制只约束相对方向，不指定图中两车间的绝对距离。
"""

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="bearing_two",
    algorithm="bearing",
    control_space="planar_xy",
    vehicles=(
        VehicleConfig("car1", "leader", "ground_vehicle", "10.1.1.81"),
        # VehicleConfig("car2", "follower", "ground_vehicle", "10.1.1.82"),
        VehicleConfig("car4", "follower", "ground_vehicle", "10.1.1.84"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=((0, 1), (1, 0)),
        bearing_matrix=(
            ((0.0, 0.0, 0.0), (0, 1.0, 0.0)),
            ((0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            # ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            # ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ),
        leader_mask=(True, False),
    ),
    leader_target_velocities={"car1": (0.0, 0.0, 0.0)},
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=False),
    static_obstacles=(),
)
