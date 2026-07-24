"""Minimal interfaces separating control orchestration from external systems."""

from __future__ import annotations

from typing import Protocol

from swarm.domain.models import (
    ActuationResult,
    ControlFrame,
    MocapSnapshot,
    PlannerResult,
    VehicleState,
    VelocityCommand,
)


class SwarmPlanner(Protocol):
    """Planner extension point for bearing-only, boids, or future algorithms."""

    def step(self, snapshot: MocapSnapshot, dt: float) -> PlannerResult: ...

    def reset(self) -> None: ...


class VelocityCommandFilter(Protocol):
    """Post-process planner velocity commands before platform actuation."""

    def apply(self, snapshot: MocapSnapshot, nominal: PlannerResult) -> PlannerResult: ...


class StateSource(Protocol):
    def start(self) -> None: ...

    def stop(self) -> None: ...

    def get_snapshot(self) -> MocapSnapshot: ...

    def all_valid(self) -> bool: ...


class VehicleActuator(Protocol):
    """Platform extension point for ground vehicles or future aircraft."""

    def execute(
        self,
        command: VelocityCommand,
        state: VehicleState,
        dt: float,
    ) -> ActuationResult: ...

    def stop(self) -> bool: ...


class TelemetrySink(Protocol):
    def emit(self, frame: ControlFrame) -> None: ...

    def close(self) -> None: ...


class Clock(Protocol):
    def monotonic(self) -> float: ...

    def sleep(self, seconds: float) -> None: ...
