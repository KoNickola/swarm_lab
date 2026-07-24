"""Tests for configuration-driven swarm planner selection."""

from __future__ import annotations

from dataclasses import replace
import unittest

from swarm.algorithms.bearing import BearingOnlyFormationController
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

    def test_unknown_algorithm_reports_available_choices(self) -> None:
        config = replace(CONFIG, algorithm="boids")

        with self.assertRaisesRegex(
            ValueError,
            "unsupported swarm algorithm 'boids'; available algorithms: bearing",
        ):
            build_swarm_planner(config)

    def test_empty_algorithm_is_rejected_during_validation(self) -> None:
        with self.assertRaisesRegex(ValueError, "algorithm must be a non-empty string"):
            replace(CONFIG, algorithm="").validate()


if __name__ == "__main__":
    unittest.main()
