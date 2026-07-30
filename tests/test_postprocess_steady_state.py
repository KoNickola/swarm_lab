"""Tests for tail-estimated bearing steady-state analysis."""

from __future__ import annotations

from collections import defaultdict
from math import radians, sqrt
import unittest

from tools.postprocess import (
    BearingEdgeRows,
    VehicleRows,
    _edges_by_source,
    _metrics,
)


def _row(time_s: float, error_deg: float) -> dict:
    return {
        "time_s": time_s,
        "bearing_valid": 1.0,
        "bearing_error_rad": radians(error_deg),
        "bearing_error_deg": error_deg,
    }


class SteadyStatePostprocessTest(unittest.TestCase):
    def test_plot_edges_are_grouped_by_source_vehicle(self) -> None:
        edges: BearingEdgeRows = defaultdict(list)
        edges[("car1", "car2")].append(_row(0.0, 1.0))
        edges[("car1", "car3")].append(_row(0.0, 2.0))
        edges[("car2", "car3")].append(_row(0.0, 3.0))

        grouped = _edges_by_source(edges)

        self.assertEqual(list(grouped), ["car1", "car2"])
        self.assertEqual(
            [target_id for target_id, _ in grouped["car1"]],
            ["car2", "car3"],
        )
        self.assertEqual(
            [target_id for target_id, _ in grouped["car2"]],
            ["car3"],
        )

    def test_tail_estimate_finds_edge_and_global_steady_intervals(self) -> None:
        vehicles: VehicleRows = defaultdict(list)
        edges: BearingEdgeRows = defaultdict(list)
        errors_deg = (20.0, 12.0, 8.0, 5.0, 3.0, 2.2, 2.05, 1.95, 2.0, 2.0)
        for time_s, error_deg in enumerate(errors_deg):
            edges[("car3", "car1")].append(
                _row(float(time_s), error_deg)
            )
            edges[("car3", "car2")].append(
                _row(float(time_s), error_deg)
            )

        result = _metrics(
            vehicles,
            edges,
            steady_tail_fraction=0.20,
            steady_tolerance_fraction=0.05,
        )

        edge = result["bearing_edges"]["car3->car1"]
        steady = edge["steady_state"]
        self.assertTrue(steady["available"])
        self.assertEqual(steady["tail_samples"], 2)
        self.assertAlmostEqual(
            steady["estimated_steady_error_deg"],
            2.0,
        )
        self.assertNotIn("lower_bound_deg", steady)
        self.assertNotIn("upper_bound_deg", steady)
        self.assertEqual(steady["convergence_time_s"], 6.0)
        self.assertEqual(steady["steady_samples"], 4)
        self.assertAlmostEqual(steady["steady_mean_error_deg"], 2.0)
        expected_edge_rms_deg = sqrt(
            (2.05 ** 2 + 1.95 ** 2 + 2.0 ** 2 + 2.0 ** 2) / 4.0
        )
        self.assertAlmostEqual(
            steady["steady_rms_error_deg"],
            expected_edge_rms_deg,
        )
        self.assertNotIn("mean_error_deg", edge)
        self.assertNotIn("rms_error_deg", edge)
        self.assertNotIn("p95_error_deg", edge)
        self.assertNotIn("max_error_rad", edge)
        self.assertNotIn("estimated_steady_error_rad", steady)
        self.assertLess(
            list(result).index("bearing_global"),
            list(result).index("bearing_edges"),
        )

        global_metrics = result["bearing_global"]
        global_steady = global_metrics["steady_state"]
        self.assertTrue(global_steady["available"])
        self.assertEqual(global_steady["convergence_time_s"], 6.0)
        self.assertAlmostEqual(
            global_steady["steady_mean_error_deg"],
            2.0 * sqrt(2.0),
        )
        self.assertAlmostEqual(
            global_steady["steady_rms_error_deg"],
            expected_edge_rms_deg * sqrt(2.0),
        )
        self.assertEqual(
            result["global_error_metric"],
            "l2_norm_of_complete_directed_edge_errors",
        )
        self.assertNotIn("convergence", result)
        self.assertNotIn("terminal_window", result)

    def test_final_sample_outside_band_has_no_steady_interval(self) -> None:
        vehicles: VehicleRows = defaultdict(list)
        edges: BearingEdgeRows = defaultdict(list)
        errors_deg = (10.0, 8.0, 6.0, 4.0, 3.0, 2.0, 2.0, 2.0, 2.0, 4.0)
        for time_s, error_deg in enumerate(errors_deg):
            edges[("car3", "car1")].append(
                _row(float(time_s), error_deg)
            )

        result = _metrics(vehicles, edges)

        edge = result["bearing_edges"]["car3->car1"]
        self.assertAlmostEqual(edge["max_error_deg"], 10.0)
        steady = edge["steady_state"]
        self.assertFalse(steady["available"])
        self.assertEqual(steady["reason"], "final_sample_outside_band")
        self.assertIsNone(steady["convergence_time_s"])

    def test_global_l2_uses_only_complete_edge_frames(self) -> None:
        vehicles: VehicleRows = defaultdict(list)
        edges: BearingEdgeRows = defaultdict(list)
        for time_s in (0.0, 1.0, 2.0):
            edges[("car3", "car1")].append(_row(time_s, 3.0))
        for time_s in (0.0, 2.0):
            edges[("car3", "car2")].append(_row(time_s, 4.0))

        result = _metrics(
            vehicles,
            edges,
            steady_tail_fraction=1.0,
        )

        global_metrics = result["bearing_global"]
        self.assertEqual(global_metrics["observed_frames"], 3)
        self.assertEqual(global_metrics["valid_samples"], 2)
        self.assertEqual(global_metrics["invalid_or_missing_samples"], 1)
        self.assertAlmostEqual(global_metrics["max_error_deg"], 5.0)
        self.assertEqual(
            global_metrics["steady_state"]["convergence_time_s"],
            2.0,
        )

        quality = result["frame_quality"]
        self.assertEqual(quality["expected_edges"], 2)
        self.assertEqual(quality["observed_frames"], 3)
        self.assertEqual(quality["complete_frames"], 2)
        self.assertAlmostEqual(quality["complete_frame_rate"], 2.0 / 3.0)
        self.assertEqual(quality["maximum_observed_gap_s"], 1.0)

    def test_vehicle_metrics_keep_maximum_speed_error(self) -> None:
        vehicles: VehicleRows = defaultdict(list)
        vehicles["car1"].extend(
            (
                {
                    "time_s": 0.0,
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "target_speed_mps": 0.3,
                    "measured_speed_mps": 0.1,
                    "command_sent": 1.0,
                },
                {
                    "time_s": 1.0,
                    "x_m": 1.0,
                    "y_m": 0.0,
                    "target_speed_mps": 0.2,
                    "measured_speed_mps": 0.15,
                    "command_sent": 1.0,
                },
            )
        )

        result = _metrics(vehicles)

        vehicle = result["vehicles"]["car1"]
        self.assertAlmostEqual(vehicle["max_speed_error_mps"], 0.2)
        self.assertNotIn("speed_rmse_mps", vehicle)


if __name__ == "__main__":
    unittest.main()
