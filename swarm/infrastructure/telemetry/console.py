"""Rate-limited console telemetry for live experiment tuning."""

from __future__ import annotations

from math import degrees
from typing import Optional

from swarm.domain.models import ControlFrame
from swarm.infrastructure.telemetry.bearing_metrics import bearing_angle_error_rad


class ConsoleTelemetrySink:
    """Print compact planner, velocity, heading, wheel, and delivery state."""

    def __init__(self, output_hz: float) -> None:
        self.interval = 1.0 / max(output_hz, 1e-6)
        self._next_output_time: Optional[float] = None

    def emit(self, frame: ControlFrame) -> None:
        if self._next_output_time is not None and frame.monotonic_time < self._next_output_time:
            return
        self._next_output_time = frame.monotonic_time + self.interval
        edge_errors_rad = [
            angle
            for sample in frame.planner.bearing_samples
            for angle in (bearing_angle_error_rad(sample),)
            if angle is not None
        ]
        mean_edge_error_deg = (
            degrees(sum(edge_errors_rad) / len(edge_errors_rad))
            if edge_errors_rad
            else 0.0
        )
        max_edge_error_deg = (
            degrees(max(edge_errors_rad)) if edge_errors_rad else 0.0
        )
        print(
            "control | t=%.3f ready=%s mean_edge_error_deg=%.3f "
            "max_edge_error_deg=%.3f valid_edges=%d dt=%.4f"
            % (
                frame.snapshot.timestamp,
                frame.planner.ready,
                mean_edge_error_deg,
                max_edge_error_deg,
                len(edge_errors_rad),
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
            angle_rad = bearing_angle_error_rad(sample)
            angle_text = (
                "%.3f" % degrees(angle_rad)
                if angle_rad is not None
                else "NA"
            )
            print(
                "bearing | %s->%s desired=(%.3f,%.3f,%.3f) "
                "actual=(%.3f,%.3f,%.3f) error_deg=%s"
                % (
                    sample.source_id,
                    sample.target_id,
                    *sample.desired,
                    *sample.actual,
                    angle_text,
                )
            )

    @staticmethod
    def close() -> None:
        return None
