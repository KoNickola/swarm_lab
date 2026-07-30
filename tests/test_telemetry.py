"""Tests for bounded plotting history and rate-limited local recording."""

from __future__ import annotations

from contextlib import redirect_stdout
import csv
import io
import tempfile
import unittest
from pathlib import Path

from swarm.domain.models import (
    ActuationResult,
    ControlFrame,
    MocapSnapshot,
    PlannerResult,
    VehicleState,
    VelocityCommand,
)
from swarm.infrastructure.telemetry.live_plot import _append_sample
from swarm.infrastructure.telemetry.console import ConsoleTelemetrySink
from swarm.infrastructure.telemetry.recorder import ExperimentRecorder
from tools.postprocess import _metrics, _plot, _read_rows


class TelemetryTest(unittest.TestCase):
    def _frame(self, timestamp: float) -> ControlFrame:
        state = VehicleState("car1", x=timestamp, y=timestamp * 2.0, valid=True)
        command = VelocityCommand("car1", vx=0.1, error_vector=(0.2, 0.0, 0.0), valid=True)
        planner = PlannerResult(
            timestamp=timestamp,
            commands={"car1": command},
            mean_error=0.2,
            max_error=0.2,
            ready=True,
        )
        nominal_planner = PlannerResult(
            timestamp=timestamp,
            commands={
                "car1": VelocityCommand(
                    "car1",
                    vx=0.05,
                    error_vector=(0.2, 0.0, 0.0),
                    valid=True,
                )
            },
            mean_error=0.2,
            max_error=0.2,
            ready=True,
        )
        actuation = ActuationResult(
            "car1",
            "ground_vehicle",
            {
                "target_heading": 0.0,
                "target_speed": 0.1,
                "measured_speed": 0.08,
                "speed_mps": 0.02,
                "steer_rad": 0.0,
                "front_left": 10.25,
                "front_right": 11.5,
                "rear_left": 10.25,
                "rear_right": 11.5,
            },
            True,
        )
        return ControlFrame(
            timestamp,
            0.02,
            MocapSnapshot(timestamp, {"car1": state}),
            planner,
            {"car1": actuation},
            nominal_planner,
        )

    def test_plot_history_is_bounded(self) -> None:
        histories = {"car1": {"x": [], "y": []}}
        for index in range(10):
            _append_sample(histories, {"car1": (float(index), -float(index))}, 3)
        self.assertEqual(histories["car1"]["x"], [7.0, 8.0, 9.0])
        self.assertEqual(histories["car1"]["y"], [-7.0, -8.0, -9.0])

    def test_console_prints_formation_and_apf_target_velocity(self) -> None:
        output = io.StringIO()

        with redirect_stdout(output):
            ConsoleTelemetrySink(10.0).emit(self._frame(1.0))

        text = output.getvalue()
        self.assertIn("formation=(0.050,0.000,0.000)", text)
        self.assertIn("apf_target=(0.100,0.000,0.000)", text)
        self.assertNotIn("apf_delta=", text)
        self.assertIn("speed_cmd_mps=0.020", text)
        self.assertIn("angle_cmd_rps=0.000", text)

    def test_recorder_samples_five_hz_from_fifty_hz_frames(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run"
            recorder = ExperimentRecorder(run_directory, {"test": True}, 5.0, 1.0)
            for index in range(50):
                recorder.emit(self._frame(index * 0.02))
            recorder.close()

            with (run_directory / "trajectory.csv").open("r", encoding="utf-8") as source:
                rows = list(csv.DictReader(source))
            self.assertGreaterEqual(len(rows), 4)
            self.assertLessEqual(len(rows), 6)
            self.assertEqual(rows[0]["front_left_command"], "10.250000")
            self.assertEqual(rows[0]["front_right_command"], "11.500000")
            self.assertEqual(rows[0]["rear_left_command"], "10.250000")
            self.assertEqual(rows[0]["rear_right_command"], "11.500000")
            self.assertTrue((run_directory / "metadata.json").is_file())
            self.assertTrue((run_directory / "bearing_edges.csv").is_file())
            self.assertFalse((run_directory / "summary.json").exists())

            vehicles = _read_rows(run_directory / "trajectory.csv")
            metrics = _metrics(vehicles)
            _plot(run_directory, vehicles)
            self.assertIn("car1", metrics["vehicles"])
            self.assertTrue((run_directory / "analysis.png").is_file())


if __name__ == "__main__":
    unittest.main()
