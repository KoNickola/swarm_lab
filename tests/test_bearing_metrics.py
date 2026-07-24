"""Tests for directed-edge bearing performance metrics."""

from __future__ import annotations

import csv
import json
from math import pi
from pathlib import Path
import tempfile
import unittest

from swarm.domain.models import (
    ActuationResult,
    BearingSample,
    ControlFrame,
    MocapSnapshot,
    PlannerResult,
    VehicleState,
    VelocityCommand,
)
from swarm.infrastructure.telemetry.bearing_metrics import bearing_angle_error_rad
from swarm.infrastructure.telemetry.recorder import ExperimentRecorder
from tools.postprocess import _metrics, _read_bearing_rows, _read_rows


class BearingMetricsTest(unittest.TestCase):
    def test_angle_error_uses_each_directed_edge(self) -> None:
        aligned = BearingSample(
            "car1",
            "car2",
            (1.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
        )
        perpendicular = BearingSample(
            "car1",
            "car3",
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
        )

        self.assertAlmostEqual(bearing_angle_error_rad(aligned), 0.0)
        self.assertAlmostEqual(bearing_angle_error_rad(perpendicular), pi / 2.0)

    def test_zero_length_bearing_is_invalid(self) -> None:
        sample = BearingSample(
            "car1",
            "car2",
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

        self.assertIsNone(bearing_angle_error_rad(sample))

    def test_recorder_and_postprocess_report_per_edge_angles(self) -> None:
        samples = [
            BearingSample(
                "car1",
                "car2",
                (1.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
            ),
            BearingSample(
                "car1",
                "car3",
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
            ),
        ]
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run"
            recorder = ExperimentRecorder(
                run_directory,
                {"schema_version": 2},
                record_hz=5.0,
                flush_interval_s=1.0,
            )
            state = VehicleState("car1", valid=True)
            planner = PlannerResult(
                timestamp=1.0,
                commands={"car1": VelocityCommand("car1", valid=True)},
                bearing_samples=samples,
                ready=True,
            )
            actuation = ActuationResult("car1", "ground_vehicle", {}, True)
            recorder.emit(
                ControlFrame(
                    1.0,
                    0.02,
                    MocapSnapshot(1.0, {"car1": state}),
                    planner,
                    {"car1": actuation},
                )
            )
            recorder.close()

            with (run_directory / "trajectory.csv").open(
                "r",
                encoding="utf-8",
            ) as source:
                self.assertNotIn("bearing_error", next(csv.reader(source)))
            with (run_directory / "summary.json").open(
                "r",
                encoding="utf-8",
            ) as source:
                summary = json.load(source)
            self.assertEqual(summary["recorded_bearing_edges"], 2)
            self.assertAlmostEqual(summary["mean_bearing_error_deg"], 45.0)

            metrics = _metrics(
                _read_rows(run_directory / "trajectory.csv"),
                _read_bearing_rows(run_directory / "bearing_edges.csv"),
            )
            edge_metrics = metrics["bearing_edges"]
            self.assertAlmostEqual(
                edge_metrics["car1->car2"]["mean_error_deg"],
                0.0,
            )
            self.assertAlmostEqual(
                edge_metrics["car1->car3"]["mean_error_deg"],
                90.0,
                places=6,
            )


if __name__ == "__main__":
    unittest.main()
