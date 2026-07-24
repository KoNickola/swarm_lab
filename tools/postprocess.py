#!/usr/bin/env python3
"""Generate compact metrics and plots from one recorded experiment run."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from math import degrees, hypot, isfinite, sqrt
from pathlib import Path
from typing import DefaultDict, Dict, List, Optional, Tuple


EdgeKey = Tuple[str, str]
VehicleRows = DefaultDict[str, List[Dict[str, float]]]
BearingEdgeRows = DefaultDict[EdgeKey, List[Dict[str, float]]]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Post-process vehicle trajectories and directed-edge bearing errors"
    )
    parser.add_argument(
        "run_directory",
        type=Path,
        help="directory containing trajectory.csv and bearing_edges.csv",
    )
    return parser.parse_args()


def _read_rows(path: Path) -> VehicleRows:
    vehicles: VehicleRows = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            vehicle_id = raw.pop("vehicle_id")
            vehicles[vehicle_id].append({key: float(value) for key, value in raw.items()})
    return vehicles


def _read_bearing_rows(path: Path) -> BearingEdgeRows:
    edges: BearingEdgeRows = defaultdict(list)
    if not path.is_file():
        return edges
    with path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            source_id = raw.pop("source_id")
            target_id = raw.pop("target_id")
            edges[(source_id, target_id)].append(
                {
                    key: float(value) if value else float("nan")
                    for key, value in raw.items()
                }
            )
    return edges


def _angle_statistics(rows: List[Dict[str, float]]) -> Dict[str, object]:
    angles_rad = [
        row["bearing_error_rad"]
        for row in rows
        if row.get("bearing_valid", 0.0) == 1.0
        and isfinite(row["bearing_error_rad"])
    ]
    if angles_rad:
        mean_rad: Optional[float] = sum(angles_rad) / len(angles_rad)
        rms_rad: Optional[float] = sqrt(
            sum(angle * angle for angle in angles_rad) / len(angles_rad)
        )
        max_rad: Optional[float] = max(angles_rad)
    else:
        mean_rad = None
        rms_rad = None
        max_rad = None
    return {
        "samples": len(rows),
        "valid_samples": len(angles_rad),
        "invalid_samples": len(rows) - len(angles_rad),
        "mean_error_rad": mean_rad,
        "rms_error_rad": rms_rad,
        "max_error_rad": max_rad,
        "mean_error_deg": degrees(mean_rad) if mean_rad is not None else None,
        "rms_error_deg": degrees(rms_rad) if rms_rad is not None else None,
        "max_error_deg": degrees(max_rad) if max_rad is not None else None,
    }


def _metrics(
    vehicles: VehicleRows,
    bearing_edges: Optional[BearingEdgeRows] = None,
) -> Dict[str, object]:
    result: Dict[str, object] = {
        "bearing_error_metric": "directed_edge_angular_error_3d",
        "vehicles": {},
        "bearing_edges": {},
    }
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
            "send_success_rate": (
                sum(row["command_sent"] for row in rows) / len(rows) if rows else 0.0
            ),
        }
    edge_rows = bearing_edges or defaultdict(list)
    edge_metrics: Dict[str, object] = result["bearing_edges"]  # type: ignore
    all_rows: List[Dict[str, float]] = []
    for (source_id, target_id), rows in sorted(edge_rows.items()):
        edge_metrics[source_id + "->" + target_id] = _angle_statistics(rows)
        all_rows.extend(rows)
    result["bearing_global"] = _angle_statistics(all_rows)
    return result


def _plot(
    run_directory: Path,
    vehicles: VehicleRows,
    bearing_edges: Optional[BearingEdgeRows] = None,
) -> None:
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
        speed_axes.plot(times, [row["measured_speed_mps"] for row in rows], label=vehicle_id + " actual")
        speed_axes.plot(
            times,
            [row["target_speed_mps"] for row in rows],
            linestyle="--",
            label=vehicle_id + " target",
        )
    edge_rows = bearing_edges or defaultdict(list)
    for (source_id, target_id), rows in sorted(edge_rows.items()):
        error_axes.plot(
            [row["time_s"] for row in rows],
            [row["bearing_error_deg"] for row in rows],
            label=source_id + "->" + target_id,
        )
    trajectory_axes.set_title("2D trajectory")
    trajectory_axes.set_xlabel("x (m)")
    trajectory_axes.set_ylabel("y (m)")
    trajectory_axes.set_aspect("equal", adjustable="datalim")
    error_axes.set_title("Directed-edge bearing angle error")
    error_axes.set_xlabel("time (s)")
    error_axes.set_ylabel("error (deg)")
    speed_axes.set_title("Speed tracking")
    speed_axes.set_xlabel("time (s)")
    speed_axes.set_ylabel("speed (m/s)")
    for axes_item in axes:
        axes_item.grid(True, linestyle="--", alpha=0.4)
        handles, labels = axes_item.get_legend_handles_labels()
        if handles:
            axes_item.legend(handles, labels, loc="best")
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
    bearing_edges = _read_bearing_rows(run_directory / "bearing_edges.csv")
    metrics = _metrics(vehicles, bearing_edges)
    with (run_directory / "analysis.json").open("w", encoding="utf-8") as output:
        json.dump(metrics, output, indent=2, ensure_ascii=False)
    _plot(run_directory, vehicles, bearing_edges)
    print("analysis | %s" % run_directory)


if __name__ == "__main__":
    main()
