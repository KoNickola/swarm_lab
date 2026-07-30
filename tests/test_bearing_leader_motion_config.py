"""Checks for the single-leader motion-reference experiment."""

from __future__ import annotations

from dataclasses import replace
from math import hypot, isfinite
import unittest

from configs.bearing_leader_motion import (
    CONFIG,
    POSITION_PID_INTEGRAL_LIMIT,
    POSITION_PID_KD,
    POSITION_PID_KI,
    POSITION_PID_KP,
    build_leader_reference,
)
from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.config import (
    RuntimeConfig,
    TimePositionReference,
    TimeVelocityReference,
    XYCurveReference,
)
from swarm.domain.models import MocapSnapshot, VehicleState


class SingleLeaderMotionConfigurationTest(unittest.TestCase):
    def test_single_vehicle_topology_and_position_pid(self) -> None:
        CONFIG.validate()

        self.assertEqual(CONFIG.vehicle_ids, ("car3",))
        self.assertEqual(CONFIG.topology.leader_mask, (True,))
        self.assertEqual(CONFIG.topology.adjacency_matrix, ((0,),))
        self.assertEqual(
            CONFIG.topology.bearing_matrix,
            (((0.0, 0.0, 0.0),),),
        )
        self.assertEqual(
            CONFIG.runtime.command_speed_limit_mps,
            RuntimeConfig().command_speed_limit_mps,
        )
        self.assertEqual(CONFIG.leader_position_pid.kp, POSITION_PID_KP)
        self.assertEqual(CONFIG.leader_position_pid.ki, POSITION_PID_KI)
        self.assertEqual(CONFIG.leader_position_pid.kd, POSITION_PID_KD)
        self.assertEqual(
            CONFIG.leader_position_pid.integral_limit,
            POSITION_PID_INTEGRAL_LIMIT,
        )
        self.assertIsNone(CONFIG.leader_position_pid.output_limit)

    def test_reference_selector_builds_all_three_supported_types(self) -> None:
        self.assertIsInstance(
            build_leader_reference("time_velocity"),
            TimeVelocityReference,
        )
        self.assertIsInstance(
            build_leader_reference("time_position"),
            TimePositionReference,
        )
        self.assertIsInstance(
            build_leader_reference("xy_curve"),
            XYCurveReference,
        )

    def test_each_reference_runs_through_the_bearing_planner(self) -> None:
        leader_id = CONFIG.vehicle_ids[0]
        state = VehicleState(
            leader_id,
            x=0.0,
            y=0.0,
            z=0.5,
            valid=True,
            pose_ready=True,
            twist_ready=True,
        )
        snapshot = MocapSnapshot(1.0, {leader_id: state})

        for mode in ("time_velocity", "time_position", "xy_curve"):
            with self.subTest(mode=mode):
                reference = build_leader_reference(mode)
                config = replace(
                    CONFIG,
                    leader_target_velocities={leader_id: reference},
                )
                config.validate()
                planner = build_swarm_planner(config)
                initial_result = planner.step(snapshot, 0.02)
                result = planner.step(snapshot, 0.02)
                command = result.commands[leader_id]
                components = (command.vx, command.vy, command.vz)

                self.assertTrue(initial_result.ready)
                self.assertTrue(result.ready)
                self.assertEqual(result.bearing_samples, [])
                self.assertEqual(command.error_vector, (0.0, 0.0, 0.0))
                self.assertTrue(all(isfinite(value) for value in components))
                self.assertLessEqual(
                    hypot(*components),
                    config.runtime.command_speed_limit_mps + 1e-12,
                )

    def test_unknown_reference_mode_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported REFERENCE_MODE"):
            build_leader_reference("unknown")


if __name__ == "__main__":
    unittest.main()
