#!/usr/bin/env python3
"""Generate maximum-error and terminal steady-state experiment metrics."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from math import ceil, degrees, hypot, isfinite, sqrt
from pathlib import Path
from typing import DefaultDict, Dict, List, Optional, Tuple


EdgeKey = Tuple[str, str]
VehicleRows = DefaultDict[str, List[Dict[str, float]]]
BearingEdgeRows = DefaultDict[EdgeKey, List[Dict[str, float]]]
SeriesPoint = Tuple[float, Optional[float]]
SubspaceSummaryRows = List[Dict[str, float]]


def _percentage(value: str) -> float:
    number = float(value)
    if number <= 0.0 or number > 100.0:
        raise argparse.ArgumentTypeError("percentage must be in (0, 100]")
    return number


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Post-process vehicle trajectories and directed-edge bearing errors"
        )
    )
    parser.add_argument(
        "run_directory",
        type=Path,
        help="directory containing trajectory.csv and bearing_edges.csv",
    )
    parser.add_argument(
        "--steady-tail-percent",
        type=_percentage,
        default=20.0,
        help="data percentage used to estimate steady error, default: 20",
    )
    parser.add_argument(
        "--steady-tolerance-percent",
        type=_percentage,
        default=5.0,
        help="relative steady-error band, default: 5",
    )
    parser.add_argument(
        "--max-time-s",
        type=float,
        default=None,
        help="plot only samples at or before this time; default: all samples",
    )
    return parser.parse_args()


def _read_rows(path: Path) -> VehicleRows:
    vehicles: VehicleRows = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            vehicle_id = raw.pop("vehicle_id")
            vehicles[vehicle_id].append(
                {key: float(value) for key, value in raw.items()}
            )
    for rows in vehicles.values():
        rows.sort(key=lambda row: row["time_s"])
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
    for rows in edges.values():
        rows.sort(key=lambda row: row["time_s"])
    return edges


def _read_subspace_summary(path: Path) -> SubspaceSummaryRows:
    """Read the recorded subspace edge RMS series when present."""

    if not path.is_file():
        return []
    rows: SubspaceSummaryRows = []
    with path.open("r", newline="", encoding="utf-8") as source:
        for raw in csv.DictReader(source):
            rows.append({
                key: float(value) if value else float("nan")
                for key, value in raw.items()
            })
    rows.sort(key=lambda row: row["time_s"])
    return rows


def _valid_angle_rad(row: Dict[str, float]) -> Optional[float]:
    angle_rad = row.get("bearing_error_rad", float("nan"))
    if row.get("bearing_valid", 0.0) != 1.0 or not isfinite(angle_rad):
        return None
    return angle_rad


def _bearing_frames(
    bearing_edges: BearingEdgeRows,
) -> List[Dict[str, object]]:
    """Align all observed edges and calculate the complete-frame L2 error."""

    expected_edges = tuple(sorted(bearing_edges))
    rows_by_time: DefaultDict[
        float,
        Dict[EdgeKey, Dict[str, float]],
    ] = defaultdict(dict)
    for edge, rows in bearing_edges.items():
        for row in rows:
            rows_by_time[row["time_s"]][edge] = row

    frames: List[Dict[str, object]] = []
    for time_s, edge_rows in sorted(rows_by_time.items()):
        edge_errors: Dict[EdgeKey, Optional[float]] = {}
        for edge in expected_edges:
            row = edge_rows.get(edge)
            edge_errors[edge] = (
                _valid_angle_rad(row) if row is not None else None
            )
        complete = bool(expected_edges) and all(
            edge_errors[edge] is not None for edge in expected_edges
        )
        error_l2_rad = (
            sqrt(
                sum(
                    float(edge_errors[edge]) ** 2
                    for edge in expected_edges
                )
            )
            if complete
            else None
        )
        frames.append(
            {
                "time_s": time_s,
                "edge_errors_rad": edge_errors,
                "complete": complete,
                "error_l2_rad": error_l2_rad,
            }
        )
    return frames


def _edge_series(
    frames: List[Dict[str, object]],
    edge: EdgeKey,
) -> List[SeriesPoint]:
    points: List[SeriesPoint] = []
    for frame in frames:
        edge_errors = frame["edge_errors_rad"]
        if not isinstance(edge_errors, dict):
            continue
        value = edge_errors.get(edge)
        points.append(
            (
                float(frame["time_s"]),
                float(value) if value is not None else None,
            )
        )
    return points


def _global_series(frames: List[Dict[str, object]]) -> List[SeriesPoint]:
    return [
        (
            float(frame["time_s"]),
            (
                float(frame["error_l2_rad"])
                if frame["error_l2_rad"] is not None
                else None
            ),
        )
        for frame in frames
    ]


def _empty_steady_state(reason: str) -> Dict[str, object]:
    return {
        "available": False,
        "reason": reason,
        "tail_samples": 0,
        "tail_valid_samples": 0,
        "estimated_steady_error_deg": None,
        "convergence_time_s": None,
        "end_time_s": None,
        "duration_s": 0.0,
        "steady_samples": 0,
        "steady_mean_error_deg": None,
        "steady_rms_error_deg": None,
    }


def _steady_state(
    points: List[SeriesPoint],
    tail_fraction: float,
    tolerance_fraction: float,
) -> Dict[str, object]:
    """Estimate a tail error and analyze the final contiguous in-band region."""

    if not points:
        return _empty_steady_state("no_samples")

    tail_count = max(1, int(ceil(len(points) * tail_fraction)))
    tail_points = points[-tail_count:]
    tail_values = [
        value for _, value in tail_points if value is not None
    ]
    result = _empty_steady_state("no_valid_tail_samples")
    result["tail_samples"] = len(tail_points)
    result["tail_valid_samples"] = len(tail_values)
    result["end_time_s"] = points[-1][0]
    if not tail_values:
        return result

    estimate_rad = sum(tail_values) / len(tail_values)
    tolerance_rad = abs(estimate_rad) * tolerance_fraction
    lower_bound_rad = max(0.0, estimate_rad - tolerance_rad)
    upper_bound_rad = estimate_rad + tolerance_rad
    result.update(
        {
            "estimated_steady_error_deg": degrees(estimate_rad),
        }
    )

    final_value = points[-1][1]
    if final_value is None:
        result["reason"] = "final_sample_invalid"
        return result
    if not lower_bound_rad <= final_value <= upper_bound_rad:
        result["reason"] = "final_sample_outside_band"
        return result

    steady_start_index = len(points) - 1
    while steady_start_index > 0:
        previous_value = points[steady_start_index - 1][1]
        if (
            previous_value is None
            or previous_value < lower_bound_rad
            or previous_value > upper_bound_rad
        ):
            break
        steady_start_index -= 1

    steady_points = points[steady_start_index:]
    steady_values = [
        float(value)
        for _, value in steady_points
        if value is not None
    ]
    steady_mean_rad = sum(steady_values) / len(steady_values)
    steady_rms_rad = sqrt(
        sum(value * value for value in steady_values)
        / len(steady_values)
    )
    convergence_time_s = steady_points[0][0]
    end_time_s = steady_points[-1][0]
    result.update(
        {
            "available": True,
            "reason": None,
            "convergence_time_s": convergence_time_s,
            "end_time_s": end_time_s,
            "duration_s": max(0.0, end_time_s - convergence_time_s),
            "steady_samples": len(steady_values),
            "steady_mean_error_deg": degrees(steady_mean_rad),
            "steady_rms_error_deg": degrees(steady_rms_rad),
        }
    )
    return result


def _error_metrics(
    points: List[SeriesPoint],
    tail_fraction: float,
    tolerance_fraction: float,
) -> Dict[str, object]:
    valid_values = [
        value for _, value in points if value is not None
    ]
    max_error_rad = max(valid_values) if valid_values else None
    return {
        "observed_frames": len(points),
        "valid_samples": len(valid_values),
        "invalid_or_missing_samples": len(points) - len(valid_values),
        "max_error_deg": (
            degrees(max_error_rad) if max_error_rad is not None else None
        ),
        "steady_state": _steady_state(
            points,
            tail_fraction,
            tolerance_fraction,
        ),
    }


def _frame_quality(
    frames: List[Dict[str, object]],
    expected_edge_count: int,
) -> Dict[str, object]:
    complete_frames = sum(bool(frame["complete"]) for frame in frames)
    gaps = [
        float(current["time_s"]) - float(previous["time_s"])
        for previous, current in zip(frames, frames[1:])
        if float(current["time_s"]) > float(previous["time_s"])
    ]
    return {
        "expected_edges": expected_edge_count,
        "observed_frames": len(frames),
        "complete_frames": complete_frames,
        "complete_frame_rate": (
            complete_frames / len(frames) if frames else None
        ),
        "maximum_observed_gap_s": max(gaps) if gaps else None,
    }


def _metrics(
    vehicles: VehicleRows,
    bearing_edges: Optional[BearingEdgeRows] = None,
    steady_tail_fraction: float = 0.20,
    steady_tolerance_fraction: float = 0.05,
) -> Dict[str, object]:
    if not 0.0 < steady_tail_fraction <= 1.0:
        raise ValueError("steady_tail_fraction must be in (0, 1]")
    if not 0.0 < steady_tolerance_fraction <= 1.0:
        raise ValueError("steady_tolerance_fraction must be in (0, 1]")

    result: Dict[str, object] = {
        "bearing_error_metric": "directed_edge_angular_error_3d",
        "global_error_metric": "l2_norm_of_complete_directed_edge_errors",
        "steady_state_method": {
            "tail_percent": steady_tail_fraction * 100.0,
            "estimator": "arithmetic_mean_of_valid_tail_samples",
            "tolerance_percent": steady_tolerance_fraction * 100.0,
            "interval": (
                "final_contiguous_recorded_samples_within_relative_band"
            ),
        },
        "vehicles": {},
        "bearing_global": {},
        "bearing_edges": {},
    }

    vehicle_metrics: Dict[str, object] = result["vehicles"]  # type: ignore
    for vehicle_id, rows in sorted(vehicles.items()):
        path_length = sum(
            hypot(
                current["x_m"] - previous["x_m"],
                current["y_m"] - previous["y_m"],
            )
            for previous, current in zip(rows, rows[1:])
        )
        speed_errors = [
            abs(row["target_speed_mps"] - row["measured_speed_mps"])
            for row in rows
        ]
        vehicle_metrics[vehicle_id] = {
            "samples": len(rows),
            "duration_s": (
                rows[-1]["time_s"] - rows[0]["time_s"]
                if len(rows) > 1
                else 0.0
            ),
            "path_length_m": path_length,
            "max_speed_error_mps": (
                max(speed_errors) if speed_errors else None
            ),
            "send_success_rate": (
                sum(row["command_sent"] for row in rows) / len(rows)
                if rows
                else None
            ),
        }

    edge_rows = bearing_edges or defaultdict(list)
    frames = _bearing_frames(edge_rows)
    result["bearing_global"] = _error_metrics(
        _global_series(frames),
        steady_tail_fraction,
        steady_tolerance_fraction,
    )

    edge_metrics: Dict[str, object] = result["bearing_edges"]  # type: ignore
    for edge in sorted(edge_rows):
        source_id, target_id = edge
        edge_metrics[source_id + "->" + target_id] = _error_metrics(
            _edge_series(frames, edge),
            steady_tail_fraction,
            steady_tolerance_fraction,
        )

    result["frame_quality"] = _frame_quality(frames, len(edge_rows))
    return result


def _edges_by_source(
    bearing_edges: BearingEdgeRows,
) -> Dict[str, List[Tuple[str, List[Dict[str, float]]]]]:
    """Group directed edges by their source vehicle for plotting."""

    grouped: DefaultDict[
        str,
        List[Tuple[str, List[Dict[str, float]]]],
    ] = defaultdict(list)
    for (source_id, target_id), rows in sorted(bearing_edges.items()):
        grouped[source_id].append((target_id, rows))
    return dict(grouped)


def _plot_subspace_edges(
    run_directory: Path,
    max_time_s: Optional[float] = None,
) -> None:
    """Plot recorded normalized errors for all directed subspace edges."""

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    path = run_directory / "subspace_edges.csv"
    if not path.is_file():
        return
    edges: DefaultDict[EdgeKey, List[Tuple[float, float]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            time_s = float(row["time_s"])
            if max_time_s is not None and time_s > max_time_s:
                continue
            # Missing or invalid samples create gaps in the plotted curves.
            error = (
                float(row["normalized_error"])
                if row["valid"] == "1" and row["normalized_error"]
                else float("nan")
            )
            edges[(row["source_id"], row["target_id"])].append((time_s, error))
    if not edges:
        return
    with plt.rc_context({"font.size": 30}):
        figure, axis = plt.subplots(figsize=(12, 9))
        for index, (edge, points) in enumerate(sorted(edges.items())):
            points.sort()
            axis.plot(
                [point[0] for point in points],
                [point[1] for point in points],
                linewidth=2.0,
                linestyle=("-", "--", "-.")[index % 3],
                label="%s → %s" % edge,
            )
        axis.set_xlabel("time (s)")
        axis.set_ylabel("Normalized constraint error")
        axis.set_ylim(0.0, 1.0)
        axis.grid(True, linestyle="--", alpha=0.4)
        axis.legend(loc="upper right", fontsize=18, ncol=3)
        figure.tight_layout()
        figure.savefig(str(run_directory / "edge_error.png"), dpi=160)
        plt.close(figure)


def _plot(
    run_directory: Path,
    vehicles: VehicleRows,
    bearing_edges: Optional[BearingEdgeRows] = None,
    analysis: Optional[Dict[str, object]] = None,
    max_time_s: Optional[float] = None,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if max_time_s is not None and max_time_s < 0.0:
        raise ValueError("max_time_s must be non-negative")
    subspace_summary = _read_subspace_summary(
        run_directory / "subspace_summary.csv"
    )
    if max_time_s is not None:
        subspace_summary = [
            row for row in subspace_summary if row["time_s"] <= max_time_s
        ]
    if subspace_summary:
        # Allow vertical space for the enlarged constraint-error label.
        figure, axes = plt.subplots(1, 1, figsize=(10, 9), squeeze=False)
        axis = axes[0][0]
        valid_rows = [
            row for row in subspace_summary
            if isfinite(row.get("subspace_edge_rms", float("nan")))
        ]
        axis.plot(
            [row["time_s"] for row in valid_rows],
            [row["subspace_edge_rms"] for row in valid_rows],
            color="black",
            linewidth=2.0,
        )
        axis.set_xlabel("time (s)")
        axis.set_ylabel("Normalized constraint error (RMS)")
        axis.set_ylim(0.0, 1.0)
        axis.grid(True, linestyle="--", alpha=0.4)
        # Scale axis labels and tick labels from their configured font sizes.
        for text in (
            [axis.xaxis.label, axis.yaxis.label]
            + axis.get_xticklabels()
            + axis.get_yticklabels()
        ):
            text.set_fontsize(text.get_fontsize() * 3.0)
        figure.tight_layout()
        figure.savefig(str(run_directory / "analysis.png"), dpi=160)
        plt.close(figure)
        _plot_subspace_edges(run_directory, max_time_s)
        return

    if max_time_s is not None:
        vehicles = {
            vehicle_id: [row for row in rows if row["time_s"] <= max_time_s]
            for vehicle_id, rows in vehicles.items()
        }
        bearing_edges = {
            edge: [row for row in rows if row["time_s"] <= max_time_s]
            for edge, rows in (bearing_edges or {}).items()
        }
    edge_rows = bearing_edges or defaultdict(list)
    grouped_edges = _edges_by_source(edge_rows)
    panel_count = 2 + len(grouped_edges)
    column_count = 2
    row_count = int(ceil(panel_count / column_count))
    figure, axes_grid = plt.subplots(
        row_count,
        column_count,
        figsize=(16, 4.5 * row_count),
        squeeze=False,
    )
    axes = [axes_item for row in axes_grid for axes_item in row]
    trajectory_axes = axes[0]
    global_error_axes = axes[1]

    for vehicle_id, rows in sorted(vehicles.items()):
        trajectory_axes.plot(
            [row["x_m"] for row in rows],
            [row["y_m"] for row in rows],
            label=vehicle_id,
        )

    for axes_index, (source_id, outgoing_edges) in enumerate(
        sorted(grouped_edges.items()),
        start=2,
    ):
        edge_axes = axes[axes_index]
        for target_id, rows in outgoing_edges:
            edge_axes.plot(
                [row["time_s"] for row in rows],
                [
                    (
                        degrees(angle_rad)
                        if angle_rad is not None
                        else float("nan")
                    )
                    for row in rows
                    for angle_rad in (_valid_angle_rad(row),)
                ],
                label=source_id + "->" + target_id,
            )
        edge_axes.set_title(source_id + " outgoing bearing errors")
        edge_axes.set_xlabel("time (s)")
        edge_axes.set_ylabel("error (deg)")
        edge_axes.set_ylim(bottom=0.0)

    frames = _bearing_frames(edge_rows)
    global_points = _global_series(frames)
    valid_global_points = [
        (time_s, error_rad)
        for time_s, error_rad in global_points
        if error_rad is not None
    ]
    if valid_global_points:
        global_error_axes.plot(
            [time_s for time_s, _ in valid_global_points],
            [degrees(float(value)) for _, value in valid_global_points],
            color="black",
            linewidth=2.0,
            label="global edge L2",
        )

    if analysis is not None:
        global_metrics = analysis.get("bearing_global", {})
        if isinstance(global_metrics, dict):
            steady = global_metrics.get("steady_state", {})
            if isinstance(steady, dict):
                estimate_deg = steady.get("estimated_steady_error_deg")
                method = analysis.get("steady_state_method", {})
                tolerance_percent = (
                    method.get("tolerance_percent")
                    if isinstance(method, dict)
                    else None
                )
                if isinstance(estimate_deg, (int, float)):
                    global_error_axes.axhline(
                        estimate_deg,
                        color="black",
                        linestyle=":",
                        linewidth=1.2,
                        label="global steady estimate",
                    )
                if isinstance(estimate_deg, (int, float)) and isinstance(
                    tolerance_percent,
                    (int, float),
                ):
                    tolerance_fraction = tolerance_percent / 100.0
                    lower_deg = max(
                        0.0,
                        estimate_deg * (1.0 - tolerance_fraction),
                    )
                    upper_deg = estimate_deg * (
                        1.0 + tolerance_fraction
                    )
                    global_error_axes.axhspan(
                        lower_deg,
                        upper_deg,
                        color="green",
                        alpha=0.08,
                        label="global steady tolerance band",
                    )
                if steady.get("available"):
                    start_time_s = steady.get("convergence_time_s")
                    end_time_s = steady.get("end_time_s")
                    if isinstance(start_time_s, (int, float)) and isinstance(
                        end_time_s,
                        (int, float),
                    ):
                        global_error_axes.axvspan(
                            start_time_s,
                            end_time_s,
                            color="blue",
                            alpha=0.06,
                            label="global steady interval",
                        )

    trajectory_axes.set_title("2D trajectory")
    trajectory_axes.set_xlabel("x (m)")
    trajectory_axes.set_ylabel("y (m)")
    trajectory_axes.set_aspect("equal", adjustable="datalim")
    global_error_axes.set_title("Global L2 bearing error")
    global_error_axes.set_xlabel("time (s)")
    global_error_axes.set_ylabel("error (deg)")
    global_error_axes.set_ylim(bottom=0.0)

    for axes_item in axes[:panel_count]:
        axes_item.grid(True, linestyle="--", alpha=0.4)
        handles, labels = axes_item.get_legend_handles_labels()
        if handles:
            axes_item.legend(handles, labels, loc="best")
    for axes_item in axes[panel_count:]:
        axes_item.set_visible(False)
    figure.tight_layout()
    figure.savefig(str(run_directory / "analysis.png"), dpi=160)
    plt.close(figure)


def main() -> None:
    args = _parse_args()
    run_directory = args.run_directory.resolve()
    csv_path = run_directory / "trajectory.csv"
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    vehicles = _read_rows(csv_path)
    if not vehicles:
        raise ValueError("trajectory.csv contains no vehicle samples")
    bearing_edges = _read_bearing_rows(
        run_directory / "bearing_edges.csv"
    )
    metrics = _metrics(
        vehicles,
        bearing_edges,
        steady_tail_fraction=args.steady_tail_percent / 100.0,
        steady_tolerance_fraction=(
            args.steady_tolerance_percent / 100.0
        ),
    )
    with (run_directory / "analysis.json").open(
        "w",
        encoding="utf-8",
    ) as output:
        json.dump(metrics, output, indent=2, ensure_ascii=False)
    _plot(
        run_directory,
        vehicles,
        bearing_edges,
        metrics,
        max_time_s=args.max_time_s,
    )
    print("analysis | %s" % run_directory)


if __name__ == "__main__":
    main()
