"""Tests for configurable leader references and shared command saturation."""

from __future__ import annotations

from dataclasses import asdict, replace
import json
from math import sqrt
import unittest

from configs.bearing_two import CONFIG
from swarm.algorithms.bearing import (
    BearingOnlyFormationController,
    build_topology_from_matrices,
)
from swarm.application.bootstrap import _metadata_value
from swarm.application.control_loop import ControlLoop
from swarm.domain.config import (
    FormationControlConfig,
    PIDConfig,
    TimePositionReference,
    TimeVelocityReference,
    XYCurveReference,
)
from swarm.domain.models import (
    ActuationResult,
    MocapSnapshot,
    VehicleState,
)


def _single_leader_planner(
    reference,
    position_pid: PIDConfig = PIDConfig(1.0, 0.0, 0.0, 1.0, None),
    speed_limit: float = 10.0,
) -> BearingOnlyFormationController:
    topology = build_topology_from_matrices(
        ("leader",),
        ((0,),),
        (((0.0, 0.0, 0.0),),),
        (True,),
    )
    return BearingOnlyFormationController(
        topology=topology,
        leader_target_velocities={"leader": reference},
        control=FormationControlConfig(),
        command_speed_limit_mps=speed_limit,
        leader_position_pid=position_pid,
    )


def _state(
    vehicle_id: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
) -> VehicleState:
    return VehicleState(
        vehicle_id,
        x=x,
        y=y,
        z=z,
        valid=True,
        pose_ready=True,
        twist_ready=True,
    )


class LeaderReferenceTest(unittest.TestCase):
    def test_time_velocity_uses_elapsed_time_once_per_step_and_resets(self) -> None:
        planner = _single_leader_planner(
            TimeVelocityReference(lambda elapsed_s: (elapsed_s, 2.0 * elapsed_s))
        )
        snapshot = MocapSnapshot(1.0, {"leader": _state("leader")})

        first = planner.step(snapshot, 0.1).commands["leader"]
        second = planner.step(snapshot, 0.1).commands["leader"]
        planner.reset()
        after_reset = planner.step(snapshot, 0.1).commands["leader"]

        self.assertEqual((first.vx, first.vy, first.vz), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(second.vx, 0.1)
        self.assertAlmostEqual(second.vy, 0.2)
        self.assertEqual(
            (after_reset.vx, after_reset.vy, after_reset.vz),
            (0.0, 0.0, 0.0),
        )

    def test_time_position_uses_outer_pid_and_ignores_z_for_2d_reference(self) -> None:
        planner = _single_leader_planner(
            TimePositionReference(lambda _elapsed_s: (1.0, 2.0)),
            position_pid=PIDConfig(0.5, 0.0, 0.0, 1.0, None),
        )
        snapshot = MocapSnapshot(
            1.0,
            {"leader": _state("leader", x=0.0, y=0.0, z=0.8)},
        )

        command = planner.step(snapshot, 0.02).commands["leader"]

        self.assertAlmostEqual(command.vx, 0.5)
        self.assertAlmostEqual(command.vy, 1.0)
        self.assertAlmostEqual(command.vz, 0.0)

    def test_reset_clears_position_pid_integral(self) -> None:
        planner = _single_leader_planner(
            TimePositionReference(lambda _elapsed_s: (1.0, 0.0)),
            position_pid=PIDConfig(0.0, 1.0, 0.0, 10.0, None),
        )
        snapshot = MocapSnapshot(1.0, {"leader": _state("leader")})

        first = planner.step(snapshot, 1.0).commands["leader"]
        second = planner.step(snapshot, 1.0).commands["leader"]
        planner.reset()
        after_reset = planner.step(snapshot, 1.0).commands["leader"]

        self.assertAlmostEqual(first.vx, 1.0)
        self.assertAlmostEqual(second.vx, 2.0)
        self.assertAlmostEqual(after_reset.vx, 1.0)

    def test_static_xy_curve_combines_tangent_speed_and_position_pid(self) -> None:
        planner = _single_leader_planner(
            XYCurveReference(lambda _x: 1.0, speed_mps=0.2),
            position_pid=PIDConfig(0.5, 0.0, 0.0, 1.0, None),
        )
        snapshot = MocapSnapshot(1.0, {"leader": _state("leader")})

        command = planner.step(snapshot, 0.02).commands["leader"]

        self.assertAlmostEqual(command.vx, 0.2, places=9)
        self.assertAlmostEqual(command.vy, 0.5, places=9)
        self.assertAlmostEqual(command.vz, 0.0, places=9)

    def test_reference_rejects_non_finite_function_output(self) -> None:
        planner = _single_leader_planner(
            TimeVelocityReference(lambda _elapsed_s: (float("nan"), 0.0))
        )
        snapshot = MocapSnapshot(1.0, {"leader": _state("leader")})

        with self.assertRaisesRegex(ValueError, "finite"):
            planner.step(snapshot, 0.02)

    def test_curve_rejects_non_finite_numerical_slope(self) -> None:
        def steep_curve(x: float) -> float:
            return 1e308 if x >= 0.0 else -1e308

        planner = _single_leader_planner(
            XYCurveReference(steep_curve, speed_mps=0.1)
        )
        snapshot = MocapSnapshot(1.0, {"leader": _state("leader")})

        with self.assertRaisesRegex(ValueError, "slope must be finite"):
            planner.step(snapshot, 0.02)

    def test_multiple_leaders_keep_independent_reference_controllers(self) -> None:
        topology = build_topology_from_matrices(
            ("leader_a", "leader_b"),
            ((0, 0), (0, 0)),
            (
                ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
                ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            (True, True),
        )
        planner = BearingOnlyFormationController(
            topology=topology,
            leader_target_velocities={
                "leader_a": TimePositionReference(
                    lambda _elapsed_s: (1.0, 0.0)
                ),
                "leader_b": TimePositionReference(
                    lambda _elapsed_s: (0.5, 0.0)
                ),
            },
            control=FormationControlConfig(),
            command_speed_limit_mps=10.0,
            leader_position_pid=PIDConfig(0.0, 1.0, 0.0, 10.0, None),
        )
        initial_snapshot = MocapSnapshot(
            1.0,
            {
                "leader_a": _state("leader_a"),
                "leader_b": _state("leader_b"),
            },
        )
        second_snapshot = MocapSnapshot(
            2.0,
            {
                "leader_a": _state("leader_a", x=1.0),
                "leader_b": _state("leader_b"),
            },
        )

        first = planner.step(initial_snapshot, 1.0)
        second = planner.step(second_snapshot, 1.0)
        planner.reset()
        after_reset = planner.step(initial_snapshot, 1.0)

        self.assertAlmostEqual(first.commands["leader_a"].vx, 1.0)
        self.assertAlmostEqual(first.commands["leader_b"].vx, 0.5)
        self.assertAlmostEqual(second.commands["leader_a"].vx, 1.0)
        self.assertAlmostEqual(second.commands["leader_b"].vx, 1.0)
        self.assertAlmostEqual(after_reset.commands["leader_a"].vx, 1.0)
        self.assertAlmostEqual(after_reset.commands["leader_b"].vx, 0.5)


class LeaderBearingCompositionTest(unittest.TestCase):
    @staticmethod
    def _planner(
        leader_bearing_enabled: bool,
        speed_limit: float = 0.3,
    ) -> BearingOnlyFormationController:
        topology = build_topology_from_matrices(
            ("leader", "follower"),
            ((0, 1), (1, 0)),
            (
                ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
                ((0.0, -1.0, 0.0), (0.0, 0.0, 0.0)),
            ),
            (True, False),
        )
        return BearingOnlyFormationController(
            topology=topology,
            leader_target_velocities={"leader": (1.0, 0.0, 0.0)},
            control=FormationControlConfig(
                k_p=1.0,
                k_i=0.0,
                integral_limit=1.0,
                output_limit=0.4,
                leader_bearing_enabled=leader_bearing_enabled,
            ),
            command_speed_limit_mps=speed_limit,
        )

    @staticmethod
    def _snapshot() -> MocapSnapshot:
        return MocapSnapshot(
            1.0,
            {
                "leader": _state("leader", x=0.0, y=0.0),
                "follower": _state("follower", x=1.0, y=0.0),
            },
        )

    def test_leader_components_are_added_before_one_vector_limit(self) -> None:
        planner = self._planner(True)

        command = planner.step(self._snapshot(), 0.02).commands["leader"]

        raw_x = 1.0 + 0.4
        raw_y = -0.4
        raw_norm = sqrt(raw_x * raw_x + raw_y * raw_y)
        self.assertAlmostEqual(command.vx, 0.3 * raw_x / raw_norm, places=12)
        self.assertAlmostEqual(command.vy, 0.3 * raw_y / raw_norm, places=12)
        self.assertAlmostEqual(
            sqrt(command.vx * command.vx + command.vy * command.vy),
            0.3,
            places=12,
        )

    def test_disabling_leader_bearing_preserves_legacy_leader_command(self) -> None:
        planner = self._planner(False)

        command = planner.step(self._snapshot(), 0.02).commands["leader"]

        self.assertAlmostEqual(command.vx, 0.3)
        self.assertAlmostEqual(command.vy, 0.0)

    def test_follower_command_is_unchanged_when_leader_bearing_is_enabled(self) -> None:
        disabled = self._planner(False)
        enabled = self._planner(True)

        disabled_command = disabled.step(self._snapshot(), 0.02).commands["follower"]
        enabled_command = enabled.step(self._snapshot(), 0.02).commands["follower"]

        self.assertAlmostEqual(disabled_command.vx, enabled_command.vx, places=12)
        self.assertAlmostEqual(disabled_command.vy, enabled_command.vy, places=12)
        self.assertAlmostEqual(disabled_command.vz, enabled_command.vz, places=12)


class LeaderReferenceConfigurationTest(unittest.TestCase):
    def test_callable_reference_metadata_is_json_serializable(self) -> None:
        config = replace(
            CONFIG,
            leader_target_velocities={
                "car1": TimeVelocityReference(self._velocity_function)
            },
        )

        metadata = _metadata_value(asdict(config))
        encoded = json.dumps(metadata)

        self.assertIn("_velocity_function", encoded)

    def test_position_pid_cannot_apply_a_separate_output_limit(self) -> None:
        config = replace(
            CONFIG,
            leader_position_pid=PIDConfig(1.0, 0.0, 0.0, 1.0, 0.1),
        )

        with self.assertRaisesRegex(ValueError, "must be None"):
            config.validate()

    def test_configuration_rejects_non_callable_reference(self) -> None:
        config = replace(
            CONFIG,
            leader_target_velocities={
                "car1": TimeVelocityReference(None),  # type: ignore[arg-type]
            },
        )

        with self.assertRaisesRegex(ValueError, "must be callable"):
            config.validate()

    def test_configuration_rejects_non_finite_curve_speed(self) -> None:
        config = replace(
            CONFIG,
            leader_target_velocities={
                "car1": XYCurveReference(
                    lambda _x: 0.0,
                    speed_mps=float("inf"),
                ),
            },
        )

        with self.assertRaisesRegex(ValueError, "speed must be finite"):
            config.validate()

    @staticmethod
    def _velocity_function(_elapsed_s: float):
        return 0.1, 0.0, 0.0


class _Clock:
    def __init__(self) -> None:
        self.now = 1.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class _Source:
    def __init__(self) -> None:
        self.valid = True
        self.state = _state("leader")

    def start(self) -> None:
        return None

    def stop(self) -> None:
        return None

    def get_snapshot(self) -> MocapSnapshot:
        return MocapSnapshot(self.state.timestamp, {"leader": self.state})

    def all_valid(self) -> bool:
        return self.valid


class _Actuator:
    def execute(self, command, state, dt):
        return ActuationResult("leader", "fake", {"dt": dt}, True)

    def stop(self) -> bool:
        return True


class _Telemetry:
    def emit(self, frame) -> None:
        return None

    def close(self) -> None:
        return None


class MocapRecoveryTimingTest(unittest.TestCase):
    def test_recovery_uses_nominal_dt_instead_of_the_outage_duration(self) -> None:
        clock = _Clock()
        source = _Source()
        loop = ControlLoop(
            control_hz=50.0,
            stop_on_loss_of_mocap=True,
            state_source=source,
            planner=_single_leader_planner((0.0, 0.0, 0.0)),
            actuators={"leader": _Actuator()},
            telemetry=_Telemetry(),
            shutdown_requested=lambda: False,
            clock=clock,
        )
        first = loop.run_cycle()
        source.valid = False
        loop.run_cycle()
        clock.now += 5.0
        source.valid = True

        recovered = loop.run_cycle()

        self.assertIsNotNone(first)
        self.assertIsNotNone(recovered)
        self.assertAlmostEqual(recovered.dt, 0.02)


if __name__ == "__main__":
    unittest.main()
