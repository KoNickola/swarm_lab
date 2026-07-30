"""Tests for the angular hysteresis deadband in bearing control."""

from __future__ import annotations

from dataclasses import replace
from math import cos, radians, sin
from typing import Optional
import unittest

from configs.bearing_two import CONFIG
from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.domain.config import FormationControlConfig
from swarm.domain.models import MocapSnapshot, VehicleState


def _follower_planner(
    control: Optional[FormationControlConfig] = None,
) -> BearingOnlyFormationController:
    topology = build_topology_from_matrices(
        ("leader", "follower"),
        ((0, 0), (1, 0)),
        (
            ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ((-1.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ),
        (True, False),
    )
    return BearingOnlyFormationController(
        topology=topology,
        leader_target_velocities={"leader": (0.0, 0.0, 0.0)},
        control=control or FormationControlConfig(),
        command_speed_limit_mps=1.0,
    )


def _snapshot(angle_deg: float) -> MocapSnapshot:
    angle_rad = radians(angle_deg)
    return MocapSnapshot(
        timestamp=1.0,
        states={
            "leader": VehicleState("leader", x=0.0, y=0.0, valid=True),
            # The follower-to-leader bearing is rotated by angle_deg from -x.
            "follower": VehicleState(
                "follower",
                x=cos(angle_rad),
                y=-sin(angle_rad),
                valid=True,
            ),
        },
    )


class BearingDeadbandTest(unittest.TestCase):
    def test_hysteresis_holds_zero_between_enter_and_exit_thresholds(self) -> None:
        planner = _follower_planner()

        active = planner.step(_snapshot(6.0), 0.02).commands["follower"]
        entered = planner.step(_snapshot(1.0), 0.02).commands["follower"]
        held = planner.step(_snapshot(4.0), 0.02).commands["follower"]
        released = planner.step(_snapshot(6.0), 0.02).commands["follower"]
        active_between = planner.step(_snapshot(4.0), 0.02).commands["follower"]

        self.assertGreater(abs(active.vx) + abs(active.vy), 0.0)
        self.assertEqual((entered.vx, entered.vy, entered.vz), (0.0, 0.0, 0.0))
        self.assertEqual((held.vx, held.vy, held.vz), (0.0, 0.0, 0.0))
        self.assertGreater(abs(released.vx) + abs(released.vy), 0.0)
        self.assertGreater(abs(active_between.vx) + abs(active_between.vy), 0.0)

    def test_entering_deadband_resets_bearing_integral(self) -> None:
        planner = _follower_planner(
            FormationControlConfig(k_p=1.0, k_i=1.0),
        )

        planner.step(_snapshot(10.0), 1.0)
        self.assertNotEqual(planner._controllers["follower"].integral, ())

        command = planner.step(_snapshot(1.0), 1.0).commands["follower"]

        self.assertEqual((command.vx, command.vy, command.vz), (0.0, 0.0, 0.0))
        self.assertEqual(planner._controllers["follower"].integral, ())

    def test_reset_clears_settled_state(self) -> None:
        planner = _follower_planner()
        planner.step(_snapshot(1.0), 0.02)

        planner.reset()
        command = planner.step(_snapshot(4.0), 0.02).commands["follower"]

        self.assertGreater(abs(command.vx) + abs(command.vy), 0.0)

    def test_disabled_deadband_preserves_nonzero_small_correction(self) -> None:
        planner = _follower_planner(
            FormationControlConfig(bearing_deadband_enabled=False),
        )

        command = planner.step(_snapshot(1.0), 0.02).commands["follower"]

        self.assertGreater(abs(command.vx) + abs(command.vy), 0.0)

    def test_leader_preset_remains_active_inside_deadband(self) -> None:
        topology = build_topology_from_matrices(
            ("leader", "follower"),
            ((0, 1), (0, 0)),
            (
                ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            (True, False),
        )
        planner = BearingOnlyFormationController(
            topology=topology,
            leader_target_velocities={"leader": (0.1, 0.0, 0.0)},
            control=FormationControlConfig(leader_bearing_enabled=True),
            command_speed_limit_mps=1.0,
        )
        angle_rad = radians(1.0)
        snapshot = MocapSnapshot(
            timestamp=1.0,
            states={
                "leader": VehicleState("leader", valid=True),
                "follower": VehicleState(
                    "follower",
                    x=cos(angle_rad),
                    y=sin(angle_rad),
                    valid=True,
                ),
            },
        )

        command = planner.step(snapshot, 0.02).commands["leader"]

        self.assertAlmostEqual(command.vx, 0.1)
        self.assertAlmostEqual(command.vy, 0.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_invalid_deadband_thresholds_are_rejected(self) -> None:
        invalid_control = replace(
            CONFIG.formation_control,
            bearing_deadband_enter_deg=5.0,
            bearing_deadband_exit_deg=2.0,
        )
        invalid_config = replace(CONFIG, formation_control=invalid_control)

        with self.assertRaisesRegex(ValueError, "deadband"):
            invalid_config.validate()


if __name__ == "__main__":
    unittest.main()
