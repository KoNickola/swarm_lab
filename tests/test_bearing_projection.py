"""Tests for projection-matrix bearing formation control."""

from __future__ import annotations

from math import cos, radians, sin
from typing import Optional
import unittest

from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.algorithms.bearing_projection import BearingProjectionFormationController
from swarm.domain.config import FormationControlConfig
from swarm.domain.models import MocapSnapshot, VehicleState


def _single_edge_planner(
    desired=(1.0, 0.0, 0.0),
    control: Optional[FormationControlConfig] = None,
    speed_limit: float = 10.0,
) -> BearingProjectionFormationController:
    topology = build_topology_from_matrices(
        ("follower", "leader"),
        ((0, 1), (0, 0)),
        (
            ((0.0, 0.0, 0.0), desired),
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ),
        (False, True),
    )
    return BearingProjectionFormationController(
        topology=topology,
        leader_target_velocities={"leader": (0.0, 0.0, 0.0)},
        control=control
        or FormationControlConfig(
            projection_k_p=1.0,
            output_limit=10.0,
        ),
        command_speed_limit_mps=speed_limit,
    )


def _single_edge_snapshot(actual) -> MocapSnapshot:
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


class BearingProjectionTest(unittest.TestCase):
    def test_aligned_bearing_produces_zero_correction(self) -> None:
        planner = _single_edge_planner()

        result = planner.step(_single_edge_snapshot((2.0, 0.0, 0.0)), 0.02)
        command = result.commands["follower"]

        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))
        self.assertEqual(command.error_vector, (0.0, 0.0, 0.0))
        self.assertTrue(result.ready)

    def test_projects_actual_minus_desired_error_with_positive_gain(self) -> None:
        planner = _single_edge_planner(
            control=FormationControlConfig(
                k_p=0.25,
                k_i=8.0,
                projection_k_p=2.0,
                output_limit=10.0,
            )
        )

        first = planner.step(_single_edge_snapshot((0.0, 1.0, 0.0)), 0.02)
        second = planner.step(_single_edge_snapshot((0.0, 1.0, 0.0)), 1.0)

        for result in (first, second):
            command = result.commands["follower"]
            self.assertAlmostEqual(command.vx, -2.0)
            self.assertAlmostEqual(command.vy, 0.0)
            self.assertAlmostEqual(command.vz, 0.0)
            self.assertEqual(command.error_vector, (-1.0, 1.0, 0.0))

    def test_pi_gain_does_not_change_projection_output(self) -> None:
        low_pi_gain = _single_edge_planner(
            control=FormationControlConfig(
                k_p=0.0,
                projection_k_p=1.5,
                output_limit=10.0,
            )
        )
        high_pi_gain = _single_edge_planner(
            control=FormationControlConfig(
                k_p=100.0,
                projection_k_p=1.5,
                output_limit=10.0,
            )
        )
        snapshot = _single_edge_snapshot((0.0, 1.0, 0.0))

        low_command = low_pi_gain.step(snapshot, 0.02).commands["follower"]
        high_command = high_pi_gain.step(snapshot, 0.02).commands["follower"]

        self.assertEqual(
            (low_command.vx, low_command.vy, low_command.vz),
            (high_command.vx, high_command.vy, high_command.vz),
        )
        self.assertAlmostEqual(low_command.vx, -1.5)

    def test_projection_gain_does_not_change_pi_output(self) -> None:
        topology = build_topology_from_matrices(
            ("follower", "leader"),
            ((0, 1), (0, 0)),
            (
                ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            (False, True),
        )
        planner = BearingOnlyFormationController(
            topology,
            {"leader": (0.0, 0.0, 0.0)},
            FormationControlConfig(
                k_p=2.0,
                k_i=0.0,
                projection_k_p=100.0,
                output_limit=10.0,
            ),
            command_speed_limit_mps=10.0,
        )

        command = planner.step(
            _single_edge_snapshot((0.0, 1.0, 0.0)),
            0.02,
        ).commands["follower"]

        self.assertAlmostEqual(command.vx, -2.0)
        self.assertAlmostEqual(command.vy, 2.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_nonorthogonal_projection_is_perpendicular_to_actual_bearing(self) -> None:
        component = 2.0 ** -0.5
        planner = _single_edge_planner()

        command = planner.step(
            _single_edge_snapshot((component, component, 0.0)),
            0.02,
        ).commands["follower"]

        self.assertAlmostEqual(command.vx, -0.5)
        self.assertAlmostEqual(command.vy, 0.5)
        self.assertAlmostEqual(command.vz, 0.0)
        self.assertAlmostEqual(
            command.vx * component + command.vy * component,
            0.0,
        )

    def test_uniform_weights_keep_command_independent_of_neighbor_count(self) -> None:
        topology = build_topology_from_matrices(
            ("follower", "leader_a", "leader_b"),
            ((0, 1, 1), (0, 0, 0), (0, 0, 0)),
            (
                (
                    (0.0, 0.0, 0.0),
                    (1.0, 0.0, 0.0),
                    (1.0, 0.0, 0.0),
                ),
                ((0.0, 0.0, 0.0),) * 3,
                ((0.0, 0.0, 0.0),) * 3,
            ),
            (False, True, True),
        )
        planner = BearingProjectionFormationController(
            topology,
            {
                "leader_a": (0.0, 0.0, 0.0),
                "leader_b": (0.0, 0.0, 0.0),
            },
            FormationControlConfig(
                k_p=9.0,
                projection_k_p=1.0,
                output_limit=10.0,
            ),
            command_speed_limit_mps=10.0,
        )
        snapshot = MocapSnapshot(
            1.0,
            {
                "follower": VehicleState("follower", valid=True),
                "leader_a": VehicleState("leader_a", y=1.0, valid=True),
                "leader_b": VehicleState("leader_b", y=-1.0, valid=True),
            },
        )

        command = planner.step(snapshot, 0.02).commands["follower"]

        self.assertAlmostEqual(command.vx, -1.0)
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_spatial_projection_generates_vertical_correction(self) -> None:
        planner = _single_edge_planner(desired=(0.0, 0.0, 1.0))

        command = planner.step(
            _single_edge_snapshot((1.0, 0.0, 0.0)),
            0.02,
        ).commands["follower"]

        self.assertAlmostEqual(command.vx, 0.0)
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertAlmostEqual(command.vz, -1.0)

    def test_global_vector_speed_limit_is_preserved(self) -> None:
        planner = _single_edge_planner(
            control=FormationControlConfig(
                projection_k_p=2.0,
                output_limit=10.0,
            ),
            speed_limit=0.3,
        )

        command = planner.step(
            _single_edge_snapshot((0.0, 1.0, 0.0)),
            0.02,
        ).commands["follower"]

        self.assertAlmostEqual(command.vx, -0.3)
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_component_output_limit_is_preserved(self) -> None:
        planner = _single_edge_planner(
            control=FormationControlConfig(
                projection_k_p=2.0,
                output_limit=0.4,
            )
        )

        command = planner.step(
            _single_edge_snapshot((0.0, 1.0, 0.0)),
            0.02,
        ).commands["follower"]

        self.assertAlmostEqual(command.vx, -0.4)
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_leader_reference_and_projection_share_one_vector_sum(self) -> None:
        topology = build_topology_from_matrices(
            ("leader", "follower"),
            ((0, 1), (0, 0)),
            (
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            (True, False),
        )
        planner = BearingProjectionFormationController(
            topology,
            {"leader": (1.0, 0.0, 0.0)},
            FormationControlConfig(
                projection_k_p=1.0,
                output_limit=10.0,
                leader_bearing_enabled=True,
            ),
            command_speed_limit_mps=10.0,
        )
        snapshot = MocapSnapshot(
            1.0,
            {
                "leader": VehicleState("leader", valid=True),
                "follower": VehicleState("follower", x=1.0, valid=True),
            },
        )

        command = planner.step(snapshot, 0.02).commands["leader"]

        self.assertAlmostEqual(command.vx, 1.0)
        self.assertAlmostEqual(command.vy, -1.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_deadband_and_reset_preserve_hysteresis_behavior(self) -> None:
        planner = _single_edge_planner(
            control=FormationControlConfig(
                projection_k_p=1.0,
                output_limit=10.0,
                bearing_deadband_enabled=True,
                bearing_deadband_enter_deg=2.0,
                bearing_deadband_exit_deg=5.0,
            )
        )

        def snapshot(angle_deg: float) -> MocapSnapshot:
            angle = radians(angle_deg)
            return _single_edge_snapshot((cos(angle), sin(angle), 0.0))

        entered = planner.step(snapshot(1.0), 0.02).commands["follower"]
        held = planner.step(snapshot(4.0), 0.02).commands["follower"]
        planner.reset()
        active = planner.step(snapshot(4.0), 0.02).commands["follower"]

        self.assertEqual((entered.vx, entered.vy, entered.vz), (0.0, 0.0, 0.0))
        self.assertEqual((held.vx, held.vy, held.vz), (0.0, 0.0, 0.0))
        self.assertGreater(abs(active.vx) + abs(active.vy), 0.0)

    def test_zero_length_edge_is_recorded_but_not_controlled(self) -> None:
        planner = _single_edge_planner()

        result = planner.step(_single_edge_snapshot((0.0, 0.0, 0.0)), 0.02)
        command = result.commands["follower"]

        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))
        self.assertEqual(command.error_vector, (0.0, 0.0, 0.0))
        self.assertEqual(len(result.bearing_samples), 1)
        self.assertEqual(result.bearing_samples[0].actual, (0.0, 0.0, 0.0))

    def test_invalid_neighbor_is_not_used_for_projection(self) -> None:
        planner = _single_edge_planner()
        snapshot = MocapSnapshot(
            1.0,
            {
                "follower": VehicleState("follower", valid=True),
                "leader": VehicleState("leader", y=1.0, valid=False),
            },
        )

        result = planner.step(snapshot, 0.02)
        command = result.commands["follower"]

        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))
        self.assertEqual(result.bearing_samples, [])
        self.assertFalse(result.ready)


if __name__ == "__main__":
    unittest.main()
