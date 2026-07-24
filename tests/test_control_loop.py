"""Application orchestration tests using in-memory adapters."""

from __future__ import annotations

from dataclasses import replace
import unittest

from swarm.algorithms.bearing import BearingOnlyFormationController, build_topology_from_matrices
from swarm.application.control_loop import ControlLoop
from swarm.domain.models import ActuationResult, MocapSnapshot, VehicleState
from configs.bearing_two import CONFIG


class FakeClock:
    def __init__(self) -> None:
        self.now = 10.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class FakeSource:
    def __init__(self, valid: bool = True) -> None:
        self.valid = valid
        self.started = False
        self.stopped = False
        self.states = {
            "car1": VehicleState("car1", x=0.0, y=0.0, valid=valid),
            "car2": VehicleState("car2", x=0.5, y=0.2, valid=valid),
        }

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def get_snapshot(self) -> MocapSnapshot:
        return MocapSnapshot(1.0, self.states)

    def all_valid(self) -> bool:
        return self.valid


class FakeActuator:
    def __init__(self, vehicle_id: str) -> None:
        self.vehicle_id = vehicle_id
        self.executed = 0
        self.stopped = 0
        self.target_velocities = []

    def execute(self, command, state, dt):
        self.executed += 1
        self.target_velocities.append((command.vx, command.vy, command.vz))
        return ActuationResult(self.vehicle_id, "fake", {"front_left": command.vx}, True)

    def stop(self) -> bool:
        self.stopped += 1
        return True


class MemoryTelemetry:
    def __init__(self) -> None:
        self.frames = []

    def emit(self, frame) -> None:
        self.frames.append(frame)

    def close(self) -> None:
        return None


class FakeVelocityFilter:
    def __init__(self) -> None:
        self.calls = 0

    def apply(self, snapshot, nominal):
        self.calls += 1
        commands = {
            vehicle_id: replace(command, vx=0.123)
            for vehicle_id, command in nominal.commands.items()
        }
        return replace(nominal, commands=commands)


def _planner() -> BearingOnlyFormationController:
    topology = build_topology_from_matrices(
        CONFIG.vehicle_ids,
        CONFIG.topology.adjacency_matrix,
        CONFIG.topology.bearing_matrix,
        CONFIG.topology.leader_mask,
    )
    return BearingOnlyFormationController(
        topology,
        CONFIG.leader_target_velocities,
        CONFIG.formation_control,
        CONFIG.runtime.command_speed_limit_mps,
    )


class ControlLoopTest(unittest.TestCase):
    def test_valid_cycle_plans_executes_records_and_sleeps(self) -> None:
        clock = FakeClock()
        source = FakeSource(valid=True)
        actuators = {vehicle_id: FakeActuator(vehicle_id) for vehicle_id in CONFIG.vehicle_ids}
        telemetry = MemoryTelemetry()
        loop = ControlLoop(50.0, True, source, _planner(), actuators, telemetry, lambda: False, clock)

        frame = loop.run_cycle()

        self.assertIsNotNone(frame)
        self.assertAlmostEqual(frame.dt, 0.02)
        self.assertEqual(loop.cycle, 1)
        self.assertEqual(len(telemetry.frames), 1)
        self.assertEqual([actuator.executed for actuator in actuators.values()], [1, 1])
        self.assertAlmostEqual(clock.now, 10.02)

    def test_invalid_mocap_only_sends_stop(self) -> None:
        clock = FakeClock()
        source = FakeSource(valid=False)
        actuators = {vehicle_id: FakeActuator(vehicle_id) for vehicle_id in CONFIG.vehicle_ids}
        telemetry = MemoryTelemetry()
        loop = ControlLoop(50.0, True, source, _planner(), actuators, telemetry, lambda: False, clock)

        frame = loop.run_cycle()

        self.assertIsNone(frame)
        self.assertEqual([actuator.executed for actuator in actuators.values()], [0, 0])
        self.assertEqual([actuator.stopped for actuator in actuators.values()], [1, 1])
        self.assertEqual(telemetry.frames, [])
        self.assertAlmostEqual(clock.now, 10.02)

    def test_velocity_filter_runs_after_planning_and_before_actuation(self) -> None:
        clock = FakeClock()
        source = FakeSource(valid=True)
        actuators = {vehicle_id: FakeActuator(vehicle_id) for vehicle_id in CONFIG.vehicle_ids}
        telemetry = MemoryTelemetry()
        velocity_filter = FakeVelocityFilter()
        loop = ControlLoop(
            50.0,
            True,
            source,
            _planner(),
            actuators,
            telemetry,
            lambda: False,
            clock,
            velocity_filter,
        )

        frame = loop.run_cycle()

        self.assertIsNotNone(frame)
        self.assertEqual(velocity_filter.calls, 1)
        self.assertIsNot(frame.nominal_planner, frame.planner)
        for actuator in actuators.values():
            self.assertEqual(actuator.target_velocities[0][0], 0.123)
        for command in frame.planner.commands.values():
            self.assertEqual(command.vx, 0.123)
        for command in frame.nominal_planner.commands.values():
            self.assertNotEqual(command.vx, 0.123)


if __name__ == "__main__":
    unittest.main()
