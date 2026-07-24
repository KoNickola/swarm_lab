#!/usr/bin/env python3
"""Generate compact metrics and plots from one recorded experiment run."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from math import hypot, sqrt
from pathlib import Path
from typing import DefaultDict, Dict, List


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Post-process a swarm experiment run")
    parser.add_argument("run_directory", type=Path, help="directory containing trajectory.csv")
    return parser.parse_args()


def _read_rows(path: Path) -> DefaultDict[str, List[Dict[str, float]]]:
    vehicles: DefaultDict[str, List[Dict[str, float]]] = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            vehicle_id = raw.pop("vehicle_id")
            vehicles[vehicle_id].append({key: float(value) for key, value in raw.items()})
    return vehicles


def _metrics(vehicles: DefaultDict[str, List[Dict[str, float]]]) -> Dict[str, object]:
    result: Dict[str, object] = {"vehicles": {}}
    vehicle_metrics: Dict[str, object] = result["vehicles"]  # type: ignore
    for vehicle_id, rows in vehicles.items():
        path_length = sum(
            hypot(current["x_m"] - previous["x_m"], current["y_m"] - previous["y_m"])
            for previous, current in zip(rows, rows[1:])
        )
        speed_error_sq = [
            (row["target_speed_mps"] - row["measured_speed_mps"]) ** 2 for row in rows
        ]
        vehicle_metrics[vehicle_id] = {
            "samples": len(rows),
            "duration_s": rows[-1]["time_s"] - rows[0]["time_s"] if len(rows) > 1 else 0.0,
            "path_length_m": path_length,
            "speed_rmse_mps": sqrt(sum(speed_error_sq) / len(speed_error_sq)) if speed_error_sq else 0.0,
            "mean_bearing_error": (
                sum(row["bearing_error"] for row in rows) / len(rows) if rows else 0.0
            ),
            "max_bearing_error": max((row["bearing_error"] for row in rows), default=0.0),
            "send_success_rate": (
                sum(row["command_sent"] for row in rows) / len(rows) if rows else 0.0
            ),
        }
    return result


def _plot(run_directory: Path, vehicles: DefaultDict[str, List[Dict[str, float]]]) -> None:
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(1, 3, figsize=(16, 5))
    trajectory_axes, error_axes, speed_axes = axes
    for vehicle_id, rows in vehicles.items():
        times = [row["time_s"] for row in rows]
        trajectory_axes.plot(
            [row["x_m"] for row in rows],
            [row["y_m"] for row in rows],
            label=vehicle_id,
        )
        error_axes.plot(times, [row["bearing_error"] for row in rows], label=vehicle_id)
        speed_axes.plot(times, [row["measured_speed_mps"] for row in rows], label=vehicle_id + " actual")
        speed_axes.plot(
            times,
            [row["target_speed_mps"] for row in rows],
            linestyle="--",
            label=vehicle_id + " target",
        )
    trajectory_axes.set_title("2D trajectory")
    trajectory_axes.set_xlabel("x (m)")
    trajectory_axes.set_ylabel("y (m)")
    trajectory_axes.set_aspect("equal", adjustable="datalim")
    error_axes.set_title("Bearing error")
    error_axes.set_xlabel("time (s)")
    speed_axes.set_title("Speed tracking")
    speed_axes.set_xlabel("time (s)")
    speed_axes.set_ylabel("speed (m/s)")
    for axes_item in axes:
        axes_item.grid(True, linestyle="--", alpha=0.4)
        axes_item.legend(loc="best")
    figure.tight_layout()
    figure.savefig(str(run_directory / "analysis.png"), dpi=160)
    plt.close(figure)


def main() -> None:
    run_directory = _parse_args().run_directory.resolve()
    csv_path = run_directory / "trajectory.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    vehicles = _read_rows(csv_path)
    if not vehicles:
        raise ValueError("trajectory.csv contains no vehicle samples")
    metrics = _metrics(vehicles)
    with (run_directory / "analysis.json").open("w", encoding="utf-8") as output:
        json.dump(metrics, output, indent=2, ensure_ascii=False)
    _plot(run_directory, vehicles)
    print("analysis | %s" % run_directory)


if __name__ == "__main__":
    main()
