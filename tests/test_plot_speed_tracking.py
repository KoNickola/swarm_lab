"""Tests for separate per-vehicle speed-tracking plots."""

from __future__ import annotations

import csv
from pathlib import Path
import tempfile
import unittest

from tools.plot_speed_tracking import plot_speed_tracking


class SpeedTrackingPlotTest(unittest.TestCase):
    def test_generates_one_png_for_each_vehicle(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run"
            run_directory.mkdir()
            with (run_directory / "trajectory.csv").open(
                "w",
                newline="",
                encoding="utf-8",
            ) as output:
                writer = csv.DictWriter(
                    output,
                    fieldnames=(
                        "time_s",
                        "vehicle_id",
                        "target_speed_mps",
                        "measured_speed_mps",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    (
                        {
                            "time_s": 1.0,
                            "vehicle_id": "car2",
                            "target_speed_mps": 0.2,
                            "measured_speed_mps": 0.18,
                        },
                        {
                            "time_s": 0.0,
                            "vehicle_id": "car1",
                            "target_speed_mps": 0.1,
                            "measured_speed_mps": 0.08,
                        },
                        {
                            "time_s": 0.0,
                            "vehicle_id": "car2",
                            "target_speed_mps": 0.15,
                            "measured_speed_mps": 0.12,
                        },
                    )
                )

            generated = plot_speed_tracking(run_directory, dpi=80)

            self.assertEqual(
                [path.name for path in generated],
                ["car1.png", "car2.png"],
            )
            for path in generated:
                self.assertTrue(path.is_file())
                self.assertGreater(path.stat().st_size, 0)
                self.assertEqual(path.parent, run_directory / "speed_tracking")

    def test_rejects_missing_speed_column(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            run_directory = Path(temporary_directory) / "run"
            run_directory.mkdir()
            with (run_directory / "trajectory.csv").open(
                "w",
                newline="",
                encoding="utf-8",
            ) as output:
                writer = csv.DictWriter(
                    output,
                    fieldnames=(
                        "time_s",
                        "vehicle_id",
                        "target_speed_mps",
                    ),
                )
                writer.writeheader()

            with self.assertRaisesRegex(
                ValueError,
                "measured_speed_mps",
            ):
                plot_speed_tracking(run_directory)


if __name__ == "__main__":
    unittest.main()
