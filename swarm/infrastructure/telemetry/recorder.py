"""Rate-limited CSV recording with compact metadata and run summary."""

from __future__ import annotations

import csv
import json
from math import sqrt
from pathlib import Path
from typing import Any, Dict, Optional, TextIO

from swarm.domain.models import ControlFrame


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
    "bearing_error",
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
        self._error_sum = 0.0
        self._max_error = 0.0
        self._file: TextIO = (run_directory / "trajectory.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
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
            error = command.error_vector
            error_magnitude = sqrt(error[0] * error[0] + error[1] * error[1] + error[2] * error[2])
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
                    "bearing_error": "%.6f" % error_magnitude,
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
        self._recorded_frames += 1
        self._error_sum += frame.planner.mean_error
        self._max_error = max(self._max_error, frame.planner.max_error)
        if self._last_flush_time is not None and frame.monotonic_time - self._last_flush_time >= self.flush_interval_s:
            self._file.flush()
            self._last_flush_time = frame.monotonic_time

    def close(self) -> None:
        if self._file.closed:
            return
        self._file.flush()
        self._file.close()
        summary = {
            "recorded_frames": self._recorded_frames,
            "recorded_rows": self._recorded_rows,
            "mean_bearing_error": (
                self._error_sum / self._recorded_frames if self._recorded_frames else 0.0
            ),
            "max_bearing_error": self._max_error,
        }
        with (self.run_directory / "summary.json").open("w", encoding="utf-8") as summary_file:
            json.dump(summary, summary_file, indent=2, ensure_ascii=False)
