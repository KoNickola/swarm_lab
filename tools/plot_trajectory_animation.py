#!/usr/bin/env python3
"""Animate vehicle motion from a recorded trajectory.csv file.

The animation shows only the current position of each vehicle at each time step,
without drawing cumulative history, and uses the recorded time stamps so the
playback duration remains consistent with the experiment.
"""

from __future__ import annotations

import argparse
import csv
from bisect import bisect_right
from collections import defaultdict
from pathlib import Path
from typing import DefaultDict, Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon

if __package__:
    from .animation_frames import animation_timing, save_timed_gif
else:
    from animation_frames import animation_timing, save_timed_gif

Sample = Tuple[float, float, float]
VehicleSamples = DefaultDict[str, List[Sample]]
OBSTACLE_VEHICLE_IDS = ("car1", "car3", "car5", "car6")


def _positive_float(value: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return number


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Animate vehicle motion from trajectory.csv"
    )
    parser.add_argument(
        "run_directory",
        type=Path,
        help="experiment directory containing trajectory.csv",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="output animation path, default: <run_directory>/trajectory_animation.mp4",
    )
    parser.add_argument(
        "--fps",
        type=_positive_float,
        default=20.0,
        help="interpolated frame rate in (0, 50]; default: 20; preserves playback speed",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=180,
        help="output resolution, default: 180",
    )
    parser.add_argument(
        "--obstacle-rectangle",
        nargs=4,
        type=float,
        metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
        help="fixed obstacle rectangle in world coordinates",
    )
    return parser.parse_args()


def _read_vehicle_samples(path: Path) -> VehicleSamples:
    vehicles: VehicleSamples = defaultdict(list)
    with path.open("r", newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        required_columns = {"time_s", "vehicle_id", "x_m", "y_m"}
        missing = sorted(required_columns - set(reader.fieldnames or ()))
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
                    float(row["x_m"]),
                    float(row["y_m"]),
                )
            except ValueError as error:
                raise ValueError(
                    "trajectory.csv row %d contains non-numeric position data"
                    % row_number
                ) from error
            vehicles[vehicle_id].append(sample)
    for samples in vehicles.values():
        samples.sort(key=lambda sample: sample[0])
    return vehicles


def _build_frames(vehicles: VehicleSamples, frame_times=None) -> List[Tuple[float, Dict[str, Tuple[float, float]]]]:
    """Interpolate recorded XY positions onto an optional animation time grid."""

    unique_times = sorted(
        {time for samples in vehicles.values() for time, _, _ in samples}
    )
    if not unique_times:
        raise ValueError("trajectory.csv contains no samples")

    frames: List[Tuple[float, Dict[str, Tuple[float, float]]]] = []
    sample_times_by_vehicle = {
        vehicle_id: [sample[0] for sample in samples]
        for vehicle_id, samples in vehicles.items()
    }
    for time in (unique_times if frame_times is None else frame_times):
        positions: Dict[str, Tuple[float, float]] = {}
        for vehicle_id, samples in vehicles.items():
            sample_times = sample_times_by_vehicle[vehicle_id]
            index = bisect_right(sample_times, time) - 1
            if index < 0:
                continue
            _, x_m, y_m = samples[index]
            if index + 1 < len(samples):
                next_time, next_x, next_y = samples[index + 1]
                fraction = (time - sample_times[index]) / (next_time - sample_times[index])
                x_m += fraction * (next_x - x_m)
                y_m += fraction * (next_y - y_m)
            positions[vehicle_id] = (x_m, y_m)
        frames.append((time, positions))
    return frames


def _default_output_path(run_directory: Path) -> Path:
    return run_directory / "trajectory_animation.mp4"


def _ordered_obstacle_vertices(
    positions: Dict[str, Tuple[float, float]],
) -> List[Tuple[float, float]]:
    """Return the four obstacle points in perimeter order."""

    points = [positions[vehicle_id] for vehicle_id in OBSTACLE_VEHICLE_IDS]
    center_x = sum(point[0] for point in points) / len(points)
    center_y = sum(point[1] for point in points) / len(points)
    from math import atan2

    return sorted(
        points,
        key=lambda point: atan2(point[1] - center_y, point[0] - center_x),
    )


def plot_trajectory_animation(
    run_directory: Path,
    output_path: Path | None = None,
    fps: float | None = None,
    dpi: int = 180,
    obstacle_rectangle: Tuple[float, float, float, float] | None = None,
) -> Path:
    run_directory = run_directory.resolve()
    trajectory_path = run_directory / "trajectory.csv"
    if not trajectory_path.is_file():
        raise FileNotFoundError(trajectory_path)

    vehicles = _read_vehicle_samples(trajectory_path)
    if not vehicles:
        raise ValueError("trajectory.csv contains no vehicle samples")

    playback_fps = 20.0 if fps is None else fps
    sample_times = sorted({sample[0] for samples in vehicles.values() for sample in samples})
    frame_times, durations_ms = animation_timing(sample_times, playback_fps)
    frames = _build_frames(vehicles, frame_times)
    if len(frames) < 2:
        raise ValueError("trajectory.csv must contain at least two time steps")

    output_path = (
        output_path.resolve()
        if output_path is not None
        else _default_output_path(run_directory)
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    interval_ms = max(20, int(1000.0 / playback_fps))

    all_positions = [position for _, position in frames]
    x_values = [x for positions in all_positions for x, _ in positions.values()]
    y_values = [y for positions in all_positions for _, y in positions.values()]
    if obstacle_rectangle is not None:
        x_min_obstacle, y_min_obstacle, x_max_obstacle, y_max_obstacle = obstacle_rectangle
        if x_min_obstacle >= x_max_obstacle or y_min_obstacle >= y_max_obstacle:
            raise ValueError("obstacle rectangle must have increasing bounds")
        x_values.extend((x_min_obstacle, x_max_obstacle))
        y_values.extend((y_min_obstacle, y_max_obstacle))
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    span_x = max(x_max - x_min, 1e-6)
    span_y = max(y_max - y_min, 1e-6)
    margin_x = 0.08 * span_x
    margin_y = 0.08 * span_y

    fig, ax = plt.subplots(figsize=(8.2, 8.2), dpi=dpi)
    fig.patch.set_facecolor("#f7f7f7")
    ax.set_facecolor("#f7f7f7")
    ax.grid(True, linestyle="--", linewidth=0.7, alpha=0.5, color="#b4b4b4")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(x_min - margin_x, x_max + margin_x)
    ax.set_ylim(y_min - margin_y, y_max + margin_y)
    ax.set_xlabel("x (m)", fontsize=11)
    ax.set_ylabel("y (m)", fontsize=11)
    ax.set_title(
        run_directory.name + " trajectory animation",
        fontsize=13,
        pad=8,
        fontweight="semibold",
    )
    ax.tick_params(labelsize=9)

    colors = plt.cm.tab10(range(len(vehicles)))
    vehicle_ids = sorted(vehicles)
    scatter_handles = {}
    text_handles = {}
    for index, vehicle_id in enumerate(vehicle_ids):
        color = colors[index % len(colors)]
        scatter = ax.scatter([], [], s=110, color=color, edgecolor="white", linewidth=0.7)
        text = ax.text(
            0.0,
            0.0,
            vehicle_id,
            color=color,
            fontsize=9,
            fontweight="bold",
            ha="left",
            va="bottom",
        )
        scatter_handles[vehicle_id] = scatter
        text_handles[vehicle_id] = text

    obstacle_patch = Polygon(
        [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)],
        closed=True,
        facecolor="#b8cbe8",
        edgecolor="#7894bd",
        linewidth=1.2,
        alpha=0.28,
        visible=False,
        zorder=1,
    )
    ax.add_patch(obstacle_patch)
    fixed_obstacle_vertices = None
    if obstacle_rectangle is not None:
        x_min_obstacle, y_min_obstacle, x_max_obstacle, y_max_obstacle = obstacle_rectangle
        fixed_obstacle_vertices = [
            (x_min_obstacle, y_min_obstacle),
            (x_max_obstacle, y_min_obstacle),
            (x_max_obstacle, y_max_obstacle),
            (x_min_obstacle, y_max_obstacle),
        ]

    time_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        fontsize=10,
        fontweight="bold",
        va="top",
        ha="left",
        color="#333333",
    )

    def _update(frame_index: int) -> List[object]:
        time_s, positions = frames[frame_index]
        for vehicle_id in vehicle_ids:
            if vehicle_id in positions:
                x_m, y_m = positions[vehicle_id]
                scatter_handles[vehicle_id].set_visible(True)
                scatter_handles[vehicle_id].set_offsets([[x_m, y_m]])
                text_handles[vehicle_id].set_position((x_m + 0.02 * span_x, y_m + 0.02 * span_y))
                text_handles[vehicle_id].set_visible(True)
            else:
                scatter_handles[vehicle_id].set_visible(False)
                text_handles[vehicle_id].set_visible(False)
        if fixed_obstacle_vertices is not None:
            obstacle_patch.set_xy(fixed_obstacle_vertices)
            obstacle_patch.set_visible(True)
        elif all(vehicle_id in positions for vehicle_id in OBSTACLE_VEHICLE_IDS):
            obstacle_patch.set_xy(_ordered_obstacle_vertices(positions))
            obstacle_patch.set_visible(True)
        else:
            obstacle_patch.set_visible(False)
        time_text.set_text(f"t = {time_s:.2f} s")
        return (
            list(scatter_handles.values())
            + list(text_handles.values())
            + [obstacle_patch, time_text]
        )

    if output_path.suffix.lower() == ".gif":
        try:
            save_timed_gif(fig, _update, durations_ms, output_path)
        finally:
            plt.close(fig)
    else:
        animation = FuncAnimation(
            fig, _update, frames=len(frames), interval=interval_ms,
            blit=False, repeat=False,
        )
        try:
            animation.save(
                str(output_path),
                writer="ffmpeg",
                fps=playback_fps,
                dpi=dpi,
                extra_args=["-pix_fmt", "yuv420p"],
            )
        except Exception:
            fallback_path = output_path.with_suffix(".gif")
            save_timed_gif(fig, _update, durations_ms, fallback_path)
            output_path = fallback_path

    plt.close(fig)
    return output_path


def main() -> None:
    args = _parse_args()
    output_path = plot_trajectory_animation(
        run_directory=args.run_directory,
        output_path=args.output,
        fps=args.fps,
        dpi=args.dpi,
        obstacle_rectangle=(
            tuple(args.obstacle_rectangle)
            if args.obstacle_rectangle is not None
            else None
        ),
    )
    print("Saved animation to", output_path)


if __name__ == "__main__":
    main()
