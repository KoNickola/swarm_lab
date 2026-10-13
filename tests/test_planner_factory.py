"""Tests for configuration-driven swarm planner selection."""

from __future__ import annotations

from dataclasses import replace
import unittest

from swarm.algorithms.bearing import BearingOnlyFormationController
from swarm.algorithms.bearing_projection import BearingProjectionFormationController
from swarm.application.planner_factory import (
    available_algorithms,
    build_swarm_planner,
)
from configs.bearing_two import CONFIG


class PlannerFactoryTest(unittest.TestCase):
    def test_bearing_config_builds_bearing_planner(self) -> None:
        planner = build_swarm_planner(CONFIG)

        self.assertIsInstance(planner, BearingOnlyFormationController)
        self.assertIn("bearing", available_algorithms())

    def test_projection_config_builds_projection_planner(self) -> None:
        planner = build_swarm_planner(
            replace(CONFIG, algorithm="bearing_projection")
        )

        self.assertIsInstance(planner, BearingProjectionFormationController)
        self.assertIn("bearing_projection", available_algorithms())

    def test_unknown_algorithm_reports_available_choices(self) -> None:
        config = replace(CONFIG, algorithm="boids")

        with self.assertRaisesRegex(
            ValueError,
            "unsupported swarm algorithm 'boids'; available algorithms: "
            "bearing, bearing_projection",
        ):
            build_swarm_planner(config)

    def test_empty_algorithm_is_rejected_during_validation(self) -> None:
        with self.assertRaisesRegex(ValueError, "algorithm must be a non-empty string"):
            replace(CONFIG, algorithm="").validate()

    def test_invalid_projection_gain_is_rejected_during_validation(self) -> None:
        control = replace(CONFIG.formation_control, projection_k_p=-1.0)

        with self.assertRaisesRegex(ValueError, "projection_k_p"):
            replace(CONFIG, formation_control=control).validate()

    def test_invalid_bearing_mode_is_rejected_during_validation(self) -> None:
        control = replace(CONFIG.formation_control, bearing_mode="invalid")

        with self.assertRaisesRegex(ValueError, "bearing_mode"):
            replace(CONFIG, formation_control=control).validate()


if __name__ == "__main__":
    unittest.main()
