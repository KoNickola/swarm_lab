"""Deterministic control-loop orchestration independent of ROS and UDP."""

from __future__ import annotations

from dataclasses import replace
import time
from typing import Callable, Dict, Mapping, Optional

from swarm.application.interfaces import (
    Clock,
    StateSource,
    SwarmPlanner,
    TelemetrySink,
    VehicleActuator,
    VelocityCommandFilter,
)
from swarm.domain.config import (
    CONTROL_SPACE_PLANAR_XY,
    CONTROL_SPACE_SPATIAL_XYZ,
    SUPPORTED_CONTROL_SPACES,
)
from swarm.domain.models import ActuationResult, ControlFrame, MocapSnapshot


def _project_snapshot(
    snapshot: MocapSnapshot,
    control_space: str,
) -> MocapSnapshot:
    """Project one stable mocap snapshot into the configured control space."""

    if control_space != CONTROL_SPACE_PLANAR_XY:
        return snapshot
    states = {
        vehicle_id: replace(state, z=0.0, vz=0.0)
        for vehicle_id, state in snapshot.states.items()
    }
    return replace(snapshot, states=states)


class SystemClock:
    """Production monotonic clock adapter."""

    @staticmethod
    def monotonic() -> float:
        return time.monotonic()

    @staticmethod
    def sleep(seconds: float) -> None:
        time.sleep(seconds)


class ControlLoop:
    """Run sensing, planning, platform execution, and telemetry in order."""

    def __init__(
        self,
        control_hz: float,
        stop_on_loss_of_mocap: bool,
        state_source: StateSource,
        planner: SwarmPlanner,
        actuators: Mapping[str, VehicleActuator],
        telemetry: TelemetrySink,
        shutdown_requested: Callable[[], bool],
        clock: Optional[Clock] = None,
        velocity_filter: Optional[VelocityCommandFilter] = None,
        control_space: str = CONTROL_SPACE_SPATIAL_XYZ,
    ) -> None:
        if control_hz <= 0.0:
            raise ValueError("control_hz must be positive")
        if not actuators:
            raise ValueError("actuators must not be empty")
        if control_space not in SUPPORTED_CONTROL_SPACES:
            raise ValueError(
                "unsupported control_space %r; expected one of: %s"
                % (control_space, ", ".join(SUPPORTED_CONTROL_SPACES))
            )
        self.nominal_dt = 1.0 / control_hz
        self.stop_on_loss_of_mocap = stop_on_loss_of_mocap
        self.state_source = state_source
        self.planner = planner
        self.actuators = dict(actuators)
        self.telemetry = telemetry
        self.shutdown_requested = shutdown_requested
        self.clock = clock or SystemClock()
        self.velocity_filter = velocity_filter
        self.control_space = control_space
        self.previous_control_time: Optional[float] = None
        self.cycle = 0

    def _compute_control_dt(self, current_time: float) -> float:
        if self.previous_control_time is None:
            return self.nominal_dt
        measured_dt = current_time - self.previous_control_time
        return measured_dt if measured_dt > 0.0 else self.nominal_dt

    def _sleep_remaining_period(self, cycle_started_at: float) -> None:
        remaining = self.nominal_dt - (self.clock.monotonic() - cycle_started_at)
        if remaining > 0.0:
            self.clock.sleep(remaining)

    def _stop_all(self) -> None:
        for actuator in self.actuators.values():
            try:
                actuator.stop()
            except Exception:
                pass

    def run_cycle(self) -> Optional[ControlFrame]:
        """Run one cycle and return its frame, or ``None`` while mocap is unready."""

        cycle_started_at = self.clock.monotonic()
        snapshot = self.state_source.get_snapshot()
        snapshot = _project_snapshot(snapshot, self.control_space)
        if self.stop_on_loss_of_mocap and not self.state_source.all_valid():
            # The next valid cycle restarts with nominal dt, so time-based
            # references and controller derivatives exclude the outage.
            self.previous_control_time = None
            self._stop_all()
            self._sleep_remaining_period(cycle_started_at)
            return None

        control_dt = self._compute_control_dt(cycle_started_at)
        self.previous_control_time = cycle_started_at
        nominal_planner_result = self.planner.step(snapshot, control_dt)
        planner_result = nominal_planner_result
        if self.velocity_filter is not None:
            planner_result = self.velocity_filter.apply(snapshot, planner_result)
        actuations: Dict[str, ActuationResult] = {}
        for vehicle_id, actuator in self.actuators.items():
            command = planner_result.commands[vehicle_id]
            state = snapshot.states.get(vehicle_id)
            if state is None:
                continue
            actuations[vehicle_id] = actuator.execute(command, state, control_dt)

        frame = ControlFrame(
            monotonic_time=cycle_started_at,
            dt=control_dt,
            snapshot=snapshot,
            planner=planner_result,
            actuations=actuations,
            nominal_planner=nominal_planner_result,
        )
        self.telemetry.emit(frame)
        self.cycle += 1
        self._sleep_remaining_period(cycle_started_at)
        return frame

    def run(self) -> None:
        """Run until shutdown and always issue final stop commands."""

        self.state_source.start()
        try:
            while not self.shutdown_requested():
                self.run_cycle()
        except KeyboardInterrupt:
            pass
        finally:
            self._stop_all()
            self.telemetry.close()
            self.state_source.stop()
