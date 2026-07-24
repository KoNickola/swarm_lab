"""Rate-limited console telemetry for live experiment tuning."""

from __future__ import annotations

from typing import Optional

from swarm.domain.models import ControlFrame


class ConsoleTelemetrySink:
    """Print compact planner, velocity, heading, wheel, and delivery state."""

    def __init__(self, output_hz: float) -> None:
        self.interval = 1.0 / max(output_hz, 1e-6)
        self._next_output_time: Optional[float] = None

    def emit(self, frame: ControlFrame) -> None:
        if self._next_output_time is not None and frame.monotonic_time < self._next_output_time:
            return
        self._next_output_time = frame.monotonic_time + self.interval
        print(
            "control | t=%.3f ready=%s mean_error=%.4f max_error=%.4f dt=%.4f"
            % (
                frame.snapshot.timestamp,
                frame.planner.ready,
                frame.planner.mean_error,
                frame.planner.max_error,
                frame.dt,
            )
        )
        for vehicle_id, planner_command in frame.planner.commands.items():
            nominal_command = planner_command
            if frame.nominal_planner is not None:
                nominal_command = frame.nominal_planner.commands.get(
                    vehicle_id,
                    planner_command,
                )
            state = frame.snapshot.states.get(vehicle_id)
            actuation = frame.actuations.get(vehicle_id)
            if state is None or actuation is None:
                print("vehicle | %s state_or_actuation=NA" % vehicle_id)
                continue
            values = actuation.values
            print(
                "vehicle | %s formation=(%.3f,%.3f,%.3f) "
                "apf_target=(%.3f,%.3f,%.3f) apf_delta=(%.3f,%.3f,%.3f) "
                "measured=(%.3f,%.3f,%.3f) "
                "heading=%.3f/%.3f speed=%.3f/%.3f cmd=(%d,%d,%d,%d) sent=%s"
                % (
                    vehicle_id,
                    nominal_command.vx,
                    nominal_command.vy,
                    nominal_command.vz,
                    planner_command.vx,
                    planner_command.vy,
                    planner_command.vz,
                    planner_command.vx - nominal_command.vx,
                    planner_command.vy - nominal_command.vy,
                    planner_command.vz - nominal_command.vz,
                    state.vx,
                    state.vy,
                    state.vz,
                    state.yaw,
                    values.get("target_heading", 0.0),
                    values.get("measured_speed", 0.0),
                    values.get("target_speed", 0.0),
                    int(values.get("front_left", 0.0)),
                    int(values.get("front_right", 0.0)),
                    int(values.get("rear_left", 0.0)),
                    int(values.get("rear_right", 0.0)),
                    actuation.sent,
                )
            )
        for sample in frame.planner.bearing_samples:
            print(
                "bearing | %s->%s desired=(%.3f,%.3f,%.3f) actual=(%.3f,%.3f,%.3f)"
                % (
                    sample.source_id,
                    sample.target_id,
                    *sample.desired,
                    *sample.actual,
                )
            )

    @staticmethod
    def close() -> None:
        return None
