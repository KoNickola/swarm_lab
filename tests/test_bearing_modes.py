"""Tests for normalized bearing-only and distance-aware bearing-based modes."""

from __future__ import annotations

import unittest

from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.algorithms.bearing_projection import BearingProjectionFormationController
from swarm.domain.config import (
    BEARING_MODE_BASED,
    BEARING_MODE_ONLY,
    FormationControlConfig,
)
from swarm.domain.models import MocapSnapshot, VehicleState


def _topology(desired):
    return build_topology_from_matrices(
        ("follower", "leader"),
        ((0, 1), (0, 0)),
        (
            ((0.0, 0.0, 0.0), desired),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ),
        (False, True),
    )


def _snapshot(actual) -> MocapSnapshot:
    return MocapSnapshot(
        1.0,
        {
            "follower": VehicleState("follower", valid=True),
            "leader": VehicleState(
                "leader",
                x=actual[0],
                y=actual[1],
                z=actual[2],
                valid=True,
            ),
        },
    )


class BearingModeTest(unittest.TestCase):
    @staticmethod
    def _pi_planner(mode: str) -> BearingOnlyFormationController:
        return BearingOnlyFormationController(
            _topology((2.0, 0.0, 0.0)),
            {"leader": (0.0, 0.0, 0.0)},
            FormationControlConfig(
                k_p=1.0,
                k_i=0.0,
                output_limit=10.0,
                bearing_mode=mode,
            ),
            command_speed_limit_mps=10.0,
        )

    @staticmethod
    def _projection_planner(mode: str) -> BearingProjectionFormationController:
        return BearingProjectionFormationController(
            _topology((2.0, 0.0, 0.0)),
            {"leader": (0.0, 0.0, 0.0)},
            FormationControlConfig(
                projection_k_p=1.0,
                output_limit=10.0,
                bearing_mode=mode,
            ),
            command_speed_limit_mps=10.0,
        )

    def test_pi_only_normalizes_while_based_preserves_distance(self) -> None:
        snapshot = _snapshot((4.0, 0.0, 0.0))

        only_result = self._pi_planner(BEARING_MODE_ONLY).step(snapshot, 0.02)
        based_result = self._pi_planner(BEARING_MODE_BASED).step(snapshot, 0.02)
        only_command = only_result.commands["follower"]
        based_command = based_result.commands["follower"]

        self.assertEqual(
            (only_command.vx, only_command.vy, only_command.vz),
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(only_command.error_vector, (0.0, 0.0, 0.0))
        self.assertEqual(only_result.bearing_samples[0].actual, (1.0, 0.0, 0.0))
        self.assertEqual(only_result.bearing_samples[0].desired, (1.0, 0.0, 0.0))

        self.assertEqual(
            (based_command.vx, based_command.vy, based_command.vz),
            (2.0, 0.0, 0.0),
        )
        self.assertEqual(based_command.error_vector, (2.0, 0.0, 0.0))
        self.assertEqual(based_result.bearing_samples[0].actual, (4.0, 0.0, 0.0))
        self.assertEqual(based_result.bearing_samples[0].desired, (2.0, 0.0, 0.0))

    def test_projection_only_and_based_use_the_selected_vectors(self) -> None:
        snapshot = _snapshot((1.0, 1.0, 0.0))

        only_result = self._projection_planner(BEARING_MODE_ONLY).step(
            snapshot,
            0.02,
        )
        based_result = self._projection_planner(BEARING_MODE_BASED).step(
            snapshot,
            0.02,
        )
        only_command = only_result.commands["follower"]
        based_command = based_result.commands["follower"]

        self.assertAlmostEqual(only_command.vx, -0.5)
        self.assertAlmostEqual(only_command.vy, 0.5)
        self.assertAlmostEqual(only_command.vz, 0.0)
        self.assertAlmostEqual(based_command.vx, -1.0)
        self.assertAlmostEqual(based_command.vy, 1.0)
        self.assertAlmostEqual(based_command.vz, 0.0)
        self.assertEqual(based_result.bearing_samples[0].actual, (1.0, 1.0, 0.0))
        self.assertEqual(based_result.bearing_samples[0].desired, (2.0, 0.0, 0.0))

        scaled_result = self._projection_planner(BEARING_MODE_BASED).step(
            _snapshot((2.0, 2.0, 0.0)),
            0.02,
        )
        scaled_command = scaled_result.commands["follower"]
        self.assertAlmostEqual(scaled_command.vx, based_command.vx)
        self.assertAlmostEqual(scaled_command.vy, based_command.vy)

    def test_projection_based_keeps_desired_raw_vector_as_equilibrium(self) -> None:
        command = self._projection_planner(BEARING_MODE_BASED).step(
            _snapshot((2.0, 0.0, 0.0)),
            0.02,
        ).commands["follower"]

        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))

    def test_default_mode_remains_bearing_only(self) -> None:
        self.assertEqual(FormationControlConfig().bearing_mode, BEARING_MODE_ONLY)


if __name__ == "__main__":
    unittest.main()
