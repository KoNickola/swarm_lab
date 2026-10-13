"""Subspace telemetry serialization and shared RMS presentation."""

from contextlib import redirect_stdout
import csv
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import tempfile
import unittest

from configs.subspace_two import CONFIG
from swarm.application.bootstrap import _metadata_value
from swarm.application.planner_factory import build_swarm_planner
from swarm.domain.config import SubspaceControlConfig
from swarm.domain.models import ControlFrame, MocapSnapshot, PlannerResult
from swarm.infrastructure.telemetry.console import ConsoleTelemetrySink
from swarm.infrastructure.telemetry.live_plot import _consume_sample, _subspace_title
from swarm.infrastructure.telemetry.recorder import ExperimentRecorder
from tools.postprocess import _plot, _read_rows
from swarm.algorithms.obstacle_avoidance import ArtificialPotentialField
from test_subspace import snapshot


def coefficients(time_s):
    return (.1,)


class SubspaceTelemetryTest(unittest.TestCase):
    def frame(self, points):
        state = snapshot(points)
        result = build_swarm_planner(CONFIG).step(state, .02)
        return ControlFrame(0, .02, state, result, {})

    def test_csv_records_raw_and_normalized_values_and_empty_rms(self):
        frame = self.frame({"car3": (0, 0), "car6": (1, 0)})
        with tempfile.TemporaryDirectory() as root:
            directory = Path(root) / "run"
            recorder = ExperimentRecorder(directory, {}, 5, 1)
            recorder.emit(frame)
            invalid = self.frame({"car3": (0, 0)})
            recorder.emit(replace(invalid, monotonic_time=1))
            recorder.close()
            recorder.close()
            with (directory / "subspace_edges.csv").open() as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(len(rows), 2)
            self.assertEqual(float(rows[0]["residual_x"]), .5)
            self.assertEqual(float(rows[0]["normalized_error"]), .5)
            self.assertEqual(rows[1]["normalized_error"], "")
            self.assertEqual(rows[1]["invalid_reason"], "invalid_state")
            with (directory / "subspace_summary.csv").open() as source:
                rows = list(csv.DictReader(source))
            self.assertEqual(float(rows[0]["subspace_edge_rms"]), .5)
            self.assertEqual(rows[0]["valid_edges"], "1")
            self.assertEqual(rows[1]["subspace_edge_rms"], "")
            self.assertEqual(rows[1]["valid_edges"], "0")

    def test_bearing_frames_do_not_create_subspace_files(self):
        with tempfile.TemporaryDirectory() as root:
            directory = Path(root) / "run"
            recorder = ExperimentRecorder(directory, {}, 5, 1)
            recorder.emit(ControlFrame(0, .02, MocapSnapshot(0), PlannerResult(0), {}))
            recorder.close()
            self.assertFalse((directory / "subspace_edges.csv").exists())
            self.assertFalse((directory / "subspace_summary.csv").exists())

    def test_console_and_plot_share_rms(self):
        frame = self.frame({"car3": (0, 0), "car6": (1, 0)})
        output = io.StringIO()
        with redirect_stdout(output):
            ConsoleTelemetrySink(10).emit(frame)
        self.assertIn("subspace_edge_rms=0.500000 valid_edges=1", output.getvalue())
        self.assertNotIn("mean_edge_error_deg", output.getvalue())
        self.assertIn("RMS=0.500000", _subspace_title(.5, 1))
        self.assertIn("RMS= |", _subspace_title(None, 0))
        class Axes:
            def set_title(self, title):
                self.title = title
        axes = Axes()
        histories = {"car3": {"x": [], "y": []}}
        _consume_sample(histories, "subspace_sample", ({"car3": (1, 2)}, .5, 1), 10, axes)
        self.assertEqual(histories["car3"]["x"], [1])
        self.assertIn("RMS=0.500000", axes.title)

    def test_metadata_serializes_modes_and_function(self):
        config = replace(CONFIG, subspace_control=SubspaceControlConfig(
            modes=((1,), (0,), (1,), (0,)), coefficients=coefficients))
        config.validate()
        metadata = json.loads(json.dumps(_metadata_value(asdict(config))))
        self.assertEqual(metadata["subspace_control"]["modes"], [[1], [0], [1], [0]])
        self.assertTrue(metadata["subspace_control"]["coefficients"].endswith(".coefficients"))

    def test_apf_preserves_subspace_metrics(self):
        frame = self.frame({"car3": (0, 0), "car6": (1, 0)})
        apf = ArtificialPotentialField(replace(CONFIG.obstacle_avoidance, enabled=True), .3, ())
        result = apf.apply(frame.snapshot, frame.planner)
        self.assertEqual(result.subspace_edge_rms, .5)
        self.assertEqual(result.subspace_samples, frame.planner.subspace_samples)

    def test_postprocess_subspace_analysis_is_only_edge_rms_plot(self):
        frame = self.frame({"car3": (0, 0), "car6": (1, 0)})
        with tempfile.TemporaryDirectory() as root:
            directory = Path(root) / "run"
            recorder = ExperimentRecorder(directory, {}, 5, 1)
            recorder.emit(frame)
            recorder.close()
            _plot(directory, _read_rows(directory / "trajectory.csv"))
            self.assertTrue((directory / "analysis.png").is_file())


if __name__ == "__main__":
    unittest.main()
