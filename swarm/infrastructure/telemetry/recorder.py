"""Rate-limited CSV recording with compact metadata and run summary."""

from __future__ import annotations

import csv
import json
from math import degrees
from pathlib import Path
from typing import Any, Dict, Optional, TextIO

from swarm.domain.models import ControlFrame
from swarm.infrastructure.telemetry.bearing_metrics import bearing_angle_error_rad


CSV_COLUMNS = (
    "time_s",
    "mocap_time_s",
    "vehicle_id",
    "x_m",
    "y_m",
    "z_m",
    "yaw_rad",
    "target_vx_mps",
    "target_vy_mps",
    "target_vz_mps",
    "measured_vx_mps",
    "measured_vy_mps",
    "measured_vz_mps",
    "target_heading_rad",
    "target_speed_mps",
    "measured_speed_mps",
    "speed_command_mps",
    "angle_command",
    "front_left_command",
    "front_right_command",
    "rear_left_command",
    "rear_right_command",
    "command_sent",
)

BEARING_EDGE_COLUMNS = (
    "time_s",
    "mocap_time_s",
    "source_id",
    "target_id",
    "desired_x",
    "desired_y",
    "desired_z",
    "actual_x",
    "actual_y",
    "actual_z",
    "bearing_valid",
    "bearing_error_rad",
    "bearing_error_deg",
)


class ExperimentRecorder:
    """Persist a small analysis-ready subset instead of every 50 Hz frame."""

    def __init__(
        self,
        run_directory: Path,
        metadata: Dict[str, Any],
        record_hz: float,
        flush_interval_s: float,
    ) -> None:
        run_directory.mkdir(parents=True, exist_ok=False)
        self.run_directory = run_directory
        self.interval = 1.0 / max(record_hz, 1e-6)
        self.flush_interval_s = max(0.1, flush_interval_s)
        self._first_time: Optional[float] = None
        self._next_record_time: Optional[float] = None
        self._last_flush_time: Optional[float] = None
        self._recorded_frames = 0
        self._recorded_rows = 0
        self._recorded_bearing_edges = 0
        self._valid_bearing_edges = 0
        self._bearing_error_sum_rad = 0.0
        self._bearing_error_sq_sum_rad = 0.0
        self._max_bearing_error_rad = 0.0
        self._file: TextIO = (run_directory / "trajectory.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._bearing_file: TextIO = (
            run_directory / "bearing_edges.csv"
        ).open("w", newline="", encoding="utf-8")
        self._bearing_writer = csv.DictWriter(
            self._bearing_file,
            fieldnames=BEARING_EDGE_COLUMNS,
        )
        self._bearing_writer.writeheader()
        with (run_directory / "metadata.json").open("w", encoding="utf-8") as metadata_file:
            json.dump(metadata, metadata_file, indent=2, ensure_ascii=False)

    def emit(self, frame: ControlFrame) -> None:
        if self._first_time is None:
            self._first_time = frame.monotonic_time
            self._next_record_time = frame.monotonic_time
            self._last_flush_time = frame.monotonic_time
        if self._next_record_time is not None and frame.monotonic_time < self._next_record_time:
            return
        self._next_record_time = frame.monotonic_time + self.interval
        elapsed = frame.monotonic_time - self._first_time
        for vehicle_id, state in frame.snapshot.states.items():
            if not state.valid:
                continue
            command = frame.planner.commands.get(vehicle_id)
            actuation = frame.actuations.get(vehicle_id)
            if command is None or actuation is None:
                continue
            values = actuation.values
            self._writer.writerow(
                {
                    "time_s": "%.6f" % elapsed,
                    "mocap_time_s": "%.9f" % frame.snapshot.timestamp,
                    "vehicle_id": vehicle_id,
                    "x_m": "%.6f" % state.x,
                    "y_m": "%.6f" % state.y,
                    "z_m": "%.6f" % state.z,
                    "yaw_rad": "%.6f" % state.yaw,
                    "target_vx_mps": "%.6f" % command.vx,
                    "target_vy_mps": "%.6f" % command.vy,
                    "target_vz_mps": "%.6f" % command.vz,
                    "measured_vx_mps": "%.6f" % state.vx,
                    "measured_vy_mps": "%.6f" % state.vy,
                    "measured_vz_mps": "%.6f" % state.vz,
                    "target_heading_rad": "%.6f" % values.get("target_heading", 0.0),
                    "target_speed_mps": "%.6f" % values.get("target_speed", 0.0),
                    "measured_speed_mps": "%.6f" % values.get("measured_speed", 0.0),
                    "speed_command_mps": "%.6f" % values.get("speed_mps", 0.0),
                    "angle_command": "%.6f" % values.get("steer_rad", 0.0),
                    "front_left_command": "%.6f" % values.get("front_left", 0.0),
                    "front_right_command": "%.6f" % values.get("front_right", 0.0),
                    "rear_left_command": "%.6f" % values.get("rear_left", 0.0),
                    "rear_right_command": "%.6f" % values.get("rear_right", 0.0),
                    "command_sent": int(actuation.sent),
                }
            )
            self._recorded_rows += 1
        for sample in frame.planner.bearing_samples:
            angle_rad = bearing_angle_error_rad(sample)
            valid = angle_rad is not None
            self._bearing_writer.writerow(
                {
                    "time_s": "%.6f" % elapsed,
                    "mocap_time_s": "%.9f" % frame.snapshot.timestamp,
                    "source_id": sample.source_id,
                    "target_id": sample.target_id,
                    "desired_x": "%.9f" % sample.desired[0],
                    "desired_y": "%.9f" % sample.desired[1],
                    "desired_z": "%.9f" % sample.desired[2],
                    "actual_x": "%.9f" % sample.actual[0],
                    "actual_y": "%.9f" % sample.actual[1],
                    "actual_z": "%.9f" % sample.actual[2],
                    "bearing_valid": int(valid),
                    "bearing_error_rad": "%.9f" % angle_rad if valid else "",
                    "bearing_error_deg": "%.6f" % degrees(angle_rad) if valid else "",
                }
            )
            self._recorded_bearing_edges += 1
            if angle_rad is not None:
                self._valid_bearing_edges += 1
                self._bearing_error_sum_rad += angle_rad
                self._bearing_error_sq_sum_rad += angle_rad * angle_rad
                self._max_bearing_error_rad = max(
                    self._max_bearing_error_rad,
                    angle_rad,
                )
        self._recorded_frames += 1
        if self._last_flush_time is not None and frame.monotonic_time - self._last_flush_time >= self.flush_interval_s:
            self._file.flush()
            self._bearing_file.flush()
            self._last_flush_time = frame.monotonic_time

    def close(self) -> None:
        if self._file.closed:
            return
        self._file.flush()
        self._bearing_file.flush()
        self._file.close()
        self._bearing_file.close()
        if self._valid_bearing_edges:
            mean_error_rad: Optional[float] = (
                self._bearing_error_sum_rad / self._valid_bearing_edges
            )
            rms_error_rad: Optional[float] = (
                self._bearing_error_sq_sum_rad / self._valid_bearing_edges
            ) ** 0.5
            max_error_rad: Optional[float] = self._max_bearing_error_rad
        else:
            mean_error_rad = None
            rms_error_rad = None
            max_error_rad = None
        summary = {
            "recorded_frames": self._recorded_frames,
            "recorded_rows": self._recorded_rows,
            "bearing_error_metric": "directed_edge_angular_error_3d",
            "recorded_bearing_edges": self._recorded_bearing_edges,
            "valid_bearing_edges": self._valid_bearing_edges,
            "invalid_bearing_edges": (
                self._recorded_bearing_edges - self._valid_bearing_edges
            ),
            "mean_bearing_error_rad": mean_error_rad,
            "rms_bearing_error_rad": rms_error_rad,
            "max_bearing_error_rad": max_error_rad,
            "mean_bearing_error_deg": (
                degrees(mean_error_rad) if mean_error_rad is not None else None
            ),
            "rms_bearing_error_deg": (
                degrees(rms_error_rad) if rms_error_rad is not None else None
            ),
            "max_bearing_error_deg": (
                degrees(max_error_rad) if max_error_rad is not None else None
            ),
        }
        with (self.run_directory / "summary.json").open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, indent=2, ensure_ascii=False)
