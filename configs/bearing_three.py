"""三车等腰直角三角形编队实验。

世界坐标系与期望构型：

    car6 ● (0, +d)
         │╲
      +y ↑ ╲
         │  ╲
    car3 ●───● car5  → +x
        (0, 0) (+d, 0)

其中 ``d > 0``。car5 位于 car3 的 +x 方向，car6 位于 car3 的 +y
方向，两条直角边等长。bearing-only 控制确定相对方向和构型比例，
不指定绝对尺度 ``d``。
"""

from swarm.domain.config import (
    ExperimentConfig,
    ObstacleAvoidanceConfig,
    TopologyConfig,
    VehicleConfig,
)


CONFIG = ExperimentConfig(
    name="bearing_three",
    algorithm="bearing",
    vehicles=(
        VehicleConfig("car3", "leader", "ground_vehicle", "10.1.1.83"),
        VehicleConfig("car5", "follower", "ground_vehicle", "10.1.1.85"),
        VehicleConfig("car6", "follower", "ground_vehicle", "10.1.1.86"),
    ),
    topology=TopologyConfig(
        adjacency_matrix=(
            (0, 1, 1),
            (1, 0, 1),
            (1, 1, 0),
        ),
        bearing_matrix=(
            (
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
            ),
            (
                (-1.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
                (-1.0, 1.0, 0.0),
            ),
            (
                (0.0, -1.0, 0.0),
                (1.0, -1.0, 0.0),
                (0.0, 0.0, 0.0),
            ),
        ),
        leader_mask=(True, False, False),
    ),
    leader_target_velocities={"car3": (0.0, 0.0, 0.0)},
    obstacle_avoidance=ObstacleAvoidanceConfig(enabled=True),
    static_obstacles=(),
)
