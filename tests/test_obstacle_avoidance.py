"""Tests for the basic artificial-potential-field velocity filter."""

from __future__ import annotations

from dataclasses import replace
import math
import unittest

from swarm.algorithms.obstacle_avoidance import ArtificialPotentialField
from swarm.domain.config import ObstacleAvoidanceConfig, StaticObstacleConfig
from swarm.domain.models import MocapSnapshot, PlannerResult, VehicleState, VelocityCommand
from configs.bearing_two import CONFIG


def _result(*vehicle_ids: str) -> PlannerResult:
    return PlannerResult(
        timestamp=1.0,
        commands={
            vehicle_id: VelocityCommand(vehicle_id, valid=True)
            for vehicle_id in vehicle_ids
        },
        ready=True,
    )


def _config(**overrides) -> ObstacleAvoidanceConfig:
    values = {
        "enabled": True,
        "vehicle_radius_m": 0.10,
        "influence_distance_m": 0.50,
        "repulsive_gain": 0.01,
        "min_distance_epsilon_m": 0.01,
        "avoid_other_vehicles": False,
    }
    values.update(overrides)
    return ObstacleAvoidanceConfig(**values)


class ArtificialPotentialFieldTest(unittest.TestCase):
    def test_disabled_filter_preserves_result(self) -> None:
        result = _result("car1")
        result.commands["car1"].vx = 0.2
        snapshot = MocapSnapshot(
            1.0,
            {"car1": VehicleState("car1", valid=True)},
        )
        obstacle = StaticObstacleConfig("obs1", -0.3, 0.0, 0.1)
        field = ArtificialPotentialField(
            ObstacleAvoidanceConfig(enabled=False),
            command_speed_limit_mps=10.0,
            static_obstacles=(obstacle,),
        )

        filtered = field.apply(snapshot, result)

        self.assertIs(filtered, result)
        self.assertEqual(filtered.commands["car1"].vx, 0.2)

    def test_obstacle_outside_influence_preserves_velocity(self) -> None:
        result = _result("car1")
        result.commands["car1"].vx = 0.1
        result.commands["car1"].vy = -0.05
        snapshot = MocapSnapshot(
            1.0,
            {"car1": VehicleState("car1", x=0.0, y=0.0, valid=True)},
        )
        obstacle = StaticObstacleConfig("obs1", 2.0, 0.0, 0.1)

        ArtificialPotentialField(
            _config(),
            command_speed_limit_mps=10.0,
            static_obstacles=(obstacle,),
        ).apply(snapshot, result)

        self.assertEqual(result.commands["car1"].vx, 0.1)
        self.assertEqual(result.commands["car1"].vy, -0.05)

    def test_repulsion_points_away_and_grows_at_shorter_distance(self) -> None:
        snapshot = MocapSnapshot(
            1.0,
            {"car1": VehicleState("car1", x=0.0, y=0.0, valid=True)},
        )
        near_result = _result("car1")
        far_result = _result("car1")
        near = StaticObstacleConfig("near", -0.4, 0.0, 0.1)
        far = StaticObstacleConfig("far", -0.6, 0.0, 0.1)

        filtered_near = ArtificialPotentialField(
            _config(),
            command_speed_limit_mps=10.0,
            static_obstacles=(near,),
        ).apply(snapshot, near_result)
        filtered_far = ArtificialPotentialField(
            _config(),
            command_speed_limit_mps=10.0,
            static_obstacles=(far,),
        ).apply(snapshot, far_result)

        self.assertIsNot(filtered_near, near_result)
        self.assertIsNot(filtered_near.commands["car1"], near_result.commands["car1"])
        self.assertEqual(near_result.commands["car1"].vx, 0.0)
        self.assertGreater(filtered_near.commands["car1"].vx, 0.0)
        self.assertAlmostEqual(filtered_near.commands["car1"].vy, 0.0)
        self.assertGreater(
            filtered_near.commands["car1"].vx,
            filtered_far.commands["car1"].vx,
        )

    def test_other_vehicles_repel_each_other(self) -> None:
        result = _result("car1", "car2")
        snapshot = MocapSnapshot(
            1.0,
            {
                "car1": VehicleState("car1", x=0.0, y=0.0, valid=True),
                "car2": VehicleState("car2", x=0.4, y=0.0, valid=True),
            },
        )
        field = ArtificialPotentialField(
            _config(avoid_other_vehicles=True),
            command_speed_limit_mps=10.0,
        )

        filtered = field.apply(snapshot, result)

        self.assertLess(filtered.commands["car1"].vx, 0.0)
        self.assertGreater(filtered.commands["car2"].vx, 0.0)
        self.assertEqual(result.commands["car1"].vx, 0.0)
        self.assertEqual(result.commands["car2"].vx, 0.0)

    def test_final_planar_speed_is_limited(self) -> None:
        result = _result("car1")
        snapshot = MocapSnapshot(
            1.0,
            {"car1": VehicleState("car1", x=0.0, y=0.0, valid=True)},
        )
        obstacle = StaticObstacleConfig("obs1", -0.3, 0.0, 0.1)

        filtered = ArtificialPotentialField(
            _config(),
            command_speed_limit_mps=0.2,
            static_obstacles=(obstacle,),
        ).apply(snapshot, result)

        command = filtered.commands["car1"]
        self.assertAlmostEqual(math.hypot(command.vx, command.vy), 0.2)

    def test_coincident_centers_remain_finite(self) -> None:
        result = _result("car1")
        snapshot = MocapSnapshot(
            1.0,
            {"car1": VehicleState("car1", x=0.0, y=0.0, valid=True)},
        )
        obstacle = StaticObstacleConfig("obs1", 0.0, 0.0, 0.1)

        filtered = ArtificialPotentialField(
            _config(),
            command_speed_limit_mps=0.2,
            static_obstacles=(obstacle,),
        ).apply(snapshot, result)

        command = filtered.commands["car1"]
        self.assertTrue(math.isfinite(command.vx))
        self.assertTrue(math.isfinite(command.vy))
        self.assertLessEqual(math.hypot(command.vx, command.vy), 0.2)

    def test_configuration_rejects_epsilon_at_influence_distance(self) -> None:
        invalid_avoidance = replace(
            CONFIG.obstacle_avoidance,
            influence_distance_m=0.5,
            min_distance_epsilon_m=0.5,
        )

        with self.assertRaisesRegex(ValueError, "min_distance_epsilon_m"):
            replace(CONFIG, obstacle_avoidance=invalid_avoidance).validate()

    def test_configuration_rejects_command_limit_above_vehicle_limit(self) -> None:
        invalid_runtime = replace(
            CONFIG.runtime,
            command_speed_limit_mps=CONFIG.ground_vehicle_control.max_speed_mps + 0.01,
        )

        with self.assertRaisesRegex(ValueError, "command_speed_limit_mps"):
            replace(CONFIG, runtime=invalid_runtime).validate()


if __name__ == "__main__":
    unittest.main()
