#!/usr/bin/env python3
"""Save separate speed and heading tracking plots for each recorded vehicle."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from math import degrees
from pathlib import Path
from typing import DefaultDict, List, Optional, Tuple


SpeedSample = Tuple[float, float, float]
VehicleSpeedSamples = DefaultDict[str, List[SpeedSample]]
AngleSample = Tuple[float, float, float]
VehicleAngleSamples = DefaultDict[str, List[AngleSample]]
SPEED_REQUIRED_COLUMNS = {
    "time_s",
    "vehicle_id",
    "target_speed_mps",
    "measured_speed_mps",
}
ANGLE_REQUIRED_COLUMNS = {
    "time_s",
    "vehicle_id",
    "target_heading_rad",
    "yaw_rad",
}


def _positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return number


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Save separate speed and heading plots for every vehicle"
    )
    parser.add_argument(
        "run_directory",
        type=Path,
        help="experiment directory containing trajectory.csv",
    )
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=None,
        help="speed output directory, default: <run_directory>/speed_tracking",
    )
    parser.add_argument(
        "--angle-output-directory",
        type=Path,
        default=None,
        help="heading output directory, default: <run_directory>/angle_tracking",
    )
    parser.add_argument(
        "--dpi",
        type=_positive_int,
        default=160,
        help="PNG resolution, default: 160",
    )
    return parser.parse_args()


def _read_speed_samples(path: Path) -> VehicleSpeedSamples:
    vehicles: VehicleSpeedSamples = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        columns = set(reader.fieldnames or ())
        missing = sorted(SPEED_REQUIRED_COLUMNS - columns)
        if missing:
            raise ValueError(
                "trajectory.csv is missing columns: " + ", ".join(missing)
            )
        for row_number, row in enumerate(reader, start=2):
            vehicle_id = row["vehicle_id"].strip()
            if not vehicle_id:
                raise ValueError(
                    "trajectory.csv row %d has an empty vehicle_id" % row_number
                )
            try:
                sample = (
                    float(row["time_s"]),
                    float(row["target_speed_mps"]),
                    float(row["measured_speed_mps"]),
                )
            except ValueError as error:
                raise ValueError(
                    "trajectory.csv row %d contains a non-numeric speed value"
                    % row_number
                ) from error
            vehicles[vehicle_id].append(sample)
    for samples in vehicles.values():
        samples.sort(key=lambda sample: sample[0])
    return vehicles


def _read_angle_samples(path: Path) -> VehicleAngleSamples:
    vehicles: VehicleAngleSamples = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        columns = set(reader.fieldnames or ())
        missing = sorted(ANGLE_REQUIRED_COLUMNS - columns)
        if missing:
            raise ValueError(
                "trajectory.csv is missing columns: " + ", ".join(missing)
            )
        for row_number, row in enumerate(reader, start=2):
            vehicle_id = row["vehicle_id"].strip()
            if not vehicle_id:
                raise ValueError(
                    "trajectory.csv row %d has an empty vehicle_id" % row_number
                )
            try:
                sample = (
                    float(row["time_s"]),
                    float(row["target_heading_rad"]),
                    float(row["yaw_rad"]),
                )
            except ValueError as error:
                raise ValueError(
                    "trajectory.csv row %d contains a non-numeric angle value"
                    % row_number
                ) from error
            vehicles[vehicle_id].append(sample)
    for samples in vehicles.values():
        samples.sort(key=lambda sample: sample[0])
    return vehicles


def _safe_filename(vehicle_id: str) -> str:
    normalized = "".join(
        character
        if character.isalnum() or character in "._-"
        else "_"
        for character in vehicle_id
    ).strip("._")
    return normalized or "vehicle"


def plot_speed_tracking(
    run_directory: Path,
    output_directory: Optional[Path] = None,
    dpi: int = 160,
) -> List[Path]:
    """Create one PNG per vehicle and return the generated paths."""

    if dpi <= 0:
        raise ValueError("dpi must be positive")
    run_directory = run_directory.resolve()
    trajectory_path = run_directory / "trajectory.csv"
    if not trajectory_path.is_file():
        raise FileNotFoundError(trajectory_path)
    vehicles = _read_speed_samples(trajectory_path)
    if not vehicles:
        raise ValueError("trajectory.csv contains no vehicle samples")

    destination = (
        output_directory.resolve()
        if output_directory is not None
        else run_directory / "speed_tracking"
    )
    destination.mkdir(parents=True, exist_ok=True)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    generated: List[Path] = []
    used_filenames = set()
    for vehicle_id, samples in sorted(vehicles.items()):
        filename = _safe_filename(vehicle_id) + ".png"
        if filename in used_filenames:
            raise ValueError(
                "vehicle IDs produce the same output filename: " + filename
            )
        used_filenames.add(filename)
        output_path = destination / filename
        times = [sample[0] for sample in samples]
        target_speeds = [sample[1] for sample in samples]
        measured_speeds = [sample[2] for sample in samples]

        figure, axes = plt.subplots(figsize=(8, 4.5))
        axes.plot(
            times,
            target_speeds,
            linestyle="--",
            linewidth=1.8,
            label="target speed",
        )
        axes.plot(
            times,
            measured_speeds,
            linewidth=1.5,
            label="measured speed",
        )
        axes.set_title(vehicle_id + " speed tracking")
        axes.set_xlabel("time (s)")
        axes.set_ylabel("speed (m/s)")
        axes.set_ylim(bottom=0.0)
        axes.grid(True, linestyle="--", alpha=0.4)
        axes.legend(loc="best")
        figure.tight_layout()
        figure.savefig(str(output_path), dpi=dpi)
        plt.close(figure)
        generated.append(output_path)
    return generated


def _radians_to_degrees(values: List[float]) -> List[float]:
    """Convert recorded radians directly to degrees without angle unwrapping."""

    return [degrees(value) for value in values]


def plot_angle_tracking(
    run_directory: Path,
    output_directory: Optional[Path] = None,
    dpi: int = 160,
) -> List[Path]:
    """Create one target-heading-versus-yaw PNG per vehicle."""

    if dpi <= 0:
        raise ValueError("dpi must be positive")
    run_directory = run_directory.resolve()
    trajectory_path = run_directory / "trajectory.csv"
    if not trajectory_path.is_file():
        raise FileNotFoundError(trajectory_path)
    vehicles = _read_angle_samples(trajectory_path)
    if not vehicles:
        raise ValueError("trajectory.csv contains no vehicle samples")

    destination = (
        output_directory.resolve()
        if output_directory is not None
        else run_directory / "angle_tracking"
    )
    destination.mkdir(parents=True, exist_ok=True)

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    generated: List[Path] = []
    used_filenames = set()
    for vehicle_id, samples in sorted(vehicles.items()):
        filename = _safe_filename(vehicle_id) + ".png"
        if filename in used_filenames:
            raise ValueError(
                "vehicle IDs produce the same output filename: " + filename
            )
        used_filenames.add(filename)
        output_path = destination / filename
        times = [sample[0] for sample in samples]
        target_headings = _radians_to_degrees(
            [sample[1] for sample in samples]
        )
        measured_headings = _radians_to_degrees(
            [sample[2] for sample in samples]
        )

        figure, axes = plt.subplots(figsize=(8, 4.5))
        axes.plot(
            times,
            target_headings,
            linestyle="--",
            linewidth=1.8,
            label="target heading",
        )
        axes.plot(
            times,
            measured_headings,
            linewidth=1.5,
            label="measured yaw",
        )
        axes.set_title(vehicle_id + " heading tracking")
        axes.set_xlabel("time (s)")
        axes.set_ylabel("heading (deg)")
        axes.grid(True, linestyle="--", alpha=0.4)
        axes.legend(loc="best")
        figure.tight_layout()
        figure.savefig(str(output_path), dpi=dpi)
        plt.close(figure)
        generated.append(output_path)
    return generated


def main() -> None:
    args = _parse_args()
    speed_plots = plot_speed_tracking(
        args.run_directory,
        output_directory=args.output_directory,
        dpi=args.dpi,
    )
    angle_plots = plot_angle_tracking(
        args.run_directory,
        output_directory=args.angle_output_directory,
        dpi=args.dpi,
    )
    for output_path in speed_plots:
        print("speed plot | %s" % output_path)
    for output_path in angle_plots:
        print("angle plot | %s" % output_path)


if __name__ == "__main__":
    main()
