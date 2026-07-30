"""Tests for separate per-vehicle heading-tracking plots."""

from __future__ import annotations

import csv
from math import pi
from pathlib import Path
import tempfile
import unittest

from tools.plot_speed_tracking import (
    _radians_to_degrees,
    plot_angle_tracking,
)


class AngleTrackingPlotTest(unittest.TestCase):
    def test_generates_one_heading_png_for_each_vehicle(self) -> None:
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
                        "target_heading_rad",
                        "yaw_rad",
                    ),
                )
                writer.writeheader()
                writer.writerows(
                    (
                        {
                            "time_s": 0.0,
                            "vehicle_id": "car1",
                            "target_heading_rad": 3.0,
                            "yaw_rad": 2.9,
                        },
                        {
                            "time_s": 1.0,
                            "vehicle_id": "car1",
                            "target_heading_rad": -3.0,
                            "yaw_rad": -3.1,
                        },
                        {
                            "time_s": 0.0,
                            "vehicle_id": "car2",
                            "target_heading_rad": 0.5,
                            "yaw_rad": 0.4,
                        },
                    )
                )

            generated = plot_angle_tracking(run_directory, dpi=80)

            self.assertEqual(
                [path.name for path in generated],
                ["car1.png", "car2.png"],
            )
            for path in generated:
                self.assertTrue(path.is_file())
                self.assertGreater(path.stat().st_size, 0)
                self.assertEqual(path.parent, run_directory / "angle_tracking")

    def test_converts_radians_to_degrees_without_unwrapping(self) -> None:
        converted = _radians_to_degrees([pi, -pi, pi / 2.0])

        self.assertEqual(converted, [180.0, -180.0, 90.0])


if __name__ == "__main__":
    unittest.main()
