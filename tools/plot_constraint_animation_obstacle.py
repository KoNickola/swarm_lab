#!/usr/bin/env python3
"""Render recorded planar motion with constraint edges and enclosed regions."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import re

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, PathPatch, Polygon
from matplotlib.path import Path as PlotPath
import numpy as np

if __package__:
    from .animation_frames import animation_timing, save_timed_gif
    from .plot_trajectory_animation import _build_frames, _read_vehicle_samples
else:
    from animation_frames import animation_timing, save_timed_gif
    from plot_trajectory_animation import _build_frames, _read_vehicle_samples


TOLERANCE = 1e-9
DEFAULT_OBSTACLE_RECTANGLES = (
    (0.30, 0.20, 0.70, 0.60),
    (1.25, 1.20, 1.65, 1.60),
)
AXIS_UPPER_BOUND = 2.5


@dataclass(frozen=True)
class ConstraintEdge:
    source: str
    target: str
    affine: bool = False
    direction: tuple[float, float] | None = None
    normal: tuple[float, float] | None = None
    target_offset: float = 0.0


def _read_edges(path: Path, vehicle_ids: set[str]) -> list[ConstraintEdge]:
    """Read recorded topology and identify translated one-dimensional spaces."""

    experiment = json.loads(path.read_text(encoding="utf-8"))["experiment"]
    if experiment.get("control_space", "planar_xy") != "planar_xy":
        raise ValueError("constraint animation requires planar_xy records")
    topology = experiment["topology"]
    edges = []
    if experiment["algorithm"] == "subspace":
        for raw in topology["edges"]:
            if raw.get("matrix") is not None:
                matrix = np.asarray(raw["matrix"], dtype=float)
            else:
                if raw.get("free_direction") is not None:
                    direction = np.asarray(raw["free_direction"], dtype=float)
                else:
                    points = np.asarray(raw["expected_points"], dtype=float)
                    if points.shape != (2, 2):
                        raise ValueError("expected_points must contain two XY points")
                    direction = points[1] - points[0]
                if (direction.shape != (2,) or not np.isfinite(direction).all()
                        or np.linalg.norm(direction) <= TOLERANCE):
                    raise ValueError("free direction must be a finite nonzero XY vector")
                direction = direction / np.linalg.norm(direction)
                matrix = np.eye(2) - np.outer(direction, direction)
            bias = np.asarray(
                raw["bias"] if raw.get("bias") is not None else (0.0, 0.0),
                dtype=float,
            )
            if (matrix.shape != (2, 2) or bias.shape != (2,)
                    or not np.isfinite(matrix).all() or not np.isfinite(bias).all()):
                raise ValueError("constraint matrix and bias must be finite planar arrays")
            # A(r-b)=0 is translated only when the constrained bias A b is nonzero.
            affine = bool(np.linalg.norm(matrix @ bias) > TOLERANCE)
            _, values, vectors = np.linalg.svd(matrix)
            free_direction = None
            normal_direction = None
            target_offset = 0.0
            if np.count_nonzero(values > TOLERANCE) == 1:
                free_vector = vectors[-1]
                free_direction = tuple(float(value) for value in free_vector)
            if affine:
                if free_direction is None:
                    raise ValueError("parallel guides require a one-dimensional affine space")
                constrained_bias = bias - np.dot(bias, free_vector) * free_vector
                target_offset = float(np.linalg.norm(constrained_bias))
                normal_direction = tuple(float(value) for value in constrained_bias / target_offset)
            edges.append(ConstraintEdge(
                raw["source_id"], raw["target_id"], affine, free_direction,
                normal_direction, target_offset,
            ))
    elif experiment["algorithm"] in ("bearing", "bearing_projection"):
        ids = [vehicle["vehicle_id"] for vehicle in experiment["vehicles"]]
        adjacency = np.asarray(topology["adjacency_matrix"])
        if adjacency.shape != (len(ids), len(ids)):
            raise ValueError("adjacency dimensions must match the recorded vehicles")
        for i, source in enumerate(ids):
            for j in range(i + 1, len(ids)):
                if adjacency[i, j] or adjacency[j, i]:
                    edges.append(ConstraintEdge(source, ids[j]))
    else:
        raise ValueError("unsupported recorded constraint topology")
    seen = set()
    for edge in edges:
        pair = tuple(sorted((edge.source, edge.target)))
        if edge.source == edge.target or pair in seen:
            raise ValueError("constraint topology contains a self edge or duplicate edge")
        if not {edge.source, edge.target} <= vehicle_ids:
            raise ValueError("constraint endpoint is missing from trajectory.csv")
        seen.add(pair)
    return edges


def _constraint_cycles(edges: list[ConstraintEdge]) -> list[tuple[str, ...]]:
    """Find undirected chordless cycles without adding geometric boundary edges."""

    adjacency: dict[str, set[str]] = {}
    for edge in edges:
        adjacency.setdefault(edge.source, set()).add(edge.target)
        adjacency.setdefault(edge.target, set()).add(edge.source)
    cycles = []

    def visit(start: str, path: list[str]) -> None:
        if len(path) >= 3 and start in adjacency[path[-1]]:
            if path[1] < path[-1]:
                cycles.append(tuple(path))
            return
        for node in sorted(adjacency[path[-1]]):
            if node <= start or node in path:
                continue
            if any(node in adjacency[inner] for inner in path[1:-1]):
                continue
            visit(start, path + [node])

    for start in sorted(adjacency):
        visit(start, [start])
    return cycles


def _region_path(cycles, positions) -> PlotPath:
    """Combine equally oriented loops so overlapping fills have uniform opacity."""

    vertices, codes = [], []
    for cycle in cycles:
        if not all(node in positions for node in cycle):
            continue
        points = np.asarray([positions[node] for node in cycle])
        following = np.roll(points, -1, axis=0)
        area = np.sum(points[:, 0] * following[:, 1] - following[:, 0] * points[:, 1])
        if abs(area) <= TOLERANCE:
            continue
        if area < 0:
            points = points[::-1]
        vertices.extend([*points, points[0]])
        codes.extend([PlotPath.MOVETO] + [PlotPath.LINETO] * (len(points) - 1)
                     + [PlotPath.CLOSEPOLY])
    return PlotPath(np.asarray(vertices, dtype=float).reshape(-1, 2), codes)


def _perpendicular_segment(endpoints, direction):
    """Join the two measured guide lines at their common longitudinal midpoint."""

    direction = np.asarray(direction)
    along = np.dot(endpoints[1] - endpoints[0], direction) * direction / 2
    return np.asarray([endpoints[0] + along, endpoints[1] - along])


def _axis_name(direction) -> str:
    """Return a concise axis name for cardinal directions."""

    direction = np.abs(np.asarray(direction))
    if direction[0] >= 0.98:
        return "x"
    if direction[1] >= 0.98:
        return "y"
    return "normal"


def _robot_label(vehicle_id: str) -> str:
    """Convert recorded car IDs to labels matching the filmed robots."""

    match = re.search(r"(\d+)$", vehicle_id)
    return "Robot " + match.group(1) if match else vehicle_id


def _presentation_title(metadata_path: Path, edges: list[ConstraintEdge]) -> str:
    """Describe the demonstrated behavior without exposing a run identifier."""

    experiment = json.loads(metadata_path.read_text(encoding="utf-8"))["experiment"]
    name = experiment.get("name", "")
    titles = {
        "subspace_four_square": "Four-Robot Square Formation",
        "subspace_four_rectangle": "Four-Robot Rectangle Formation",
        "subspace_six_triangles": "Six-Robot Triangle Formation",
        "subspace_seven": "Seven-Robot Relative Motion",
    }
    if name in titles:
        return titles[name]
    if len(experiment.get("vehicles", ())) == 2 and len(edges) == 1:
        edge = edges[0]
        if edge.affine:
            axis = _axis_name(edge.normal)
            return (f"Relative Motion with a {axis}-Offset"
                    if axis != "normal" else "Relative Motion with a Fixed Offset")
        if edge.direction is not None:
            axis = _axis_name(edge.direction)
            return (f"Relative Motion along {axis}"
                    if axis != "normal" else "Relative Motion in a Free Direction")
    return "Measured Formation Motion"


def plot_constraint_animation(
    run_directory: Path,
    output_path: Path | None = None,
    fps: float | None = None,
    dpi: int = 180,
    obstacle_rectangles: tuple[tuple[float, float, float, float], ...] = DEFAULT_OBSTACLE_RECTANGLES,
) -> Path:
    """Write interpolated positions and constraints on the experiment time scale."""

    if dpi <= 0 or (fps is not None and (not np.isfinite(fps) or fps <= 0)):
        raise ValueError("dpi and fps must be positive; fps must be finite")
    run_directory = run_directory.resolve()
    vehicles = _read_vehicle_samples(run_directory / "trajectory.csv")
    if not vehicles or any(not np.isfinite(samples).all() for samples in vehicles.values()):
        raise ValueError("trajectory.csv must contain finite position and time samples")
    playback_fps = 20.0 if fps is None else fps
    sample_times = sorted({sample[0] for samples in vehicles.values() for sample in samples})
    frame_times, durations_ms = animation_timing(sample_times, playback_fps)
    frames = _build_frames(vehicles, frame_times)
    if len(frames) < 2:
        raise ValueError("trajectory.csv must contain at least two time steps")
    edges = _read_edges(run_directory / "metadata.json", set(vehicles))
    cycles = _constraint_cycles(edges)
    output_path = (output_path or run_directory / "trajectory_constraints.gif").resolve()
    if output_path.suffix.lower() != ".gif":
        raise ValueError("output path must end in .gif")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    points = np.asarray([sample[1:] for samples in vehicles.values() for sample in samples])
    lower, upper = points.min(axis=0), points.max(axis=0)
    for rectangle in obstacle_rectangles:
        x_min, y_min, x_max, y_max = rectangle
        if x_min >= x_max or y_min >= y_max:
            raise ValueError("obstacle rectangles must have increasing bounds")
        lower = np.minimum(lower, (x_min, y_min))
        upper = np.maximum(upper, (x_max, y_max))
    span = np.maximum(upper - lower, 0.1)
    margin = np.maximum(0.08 * span, 0.04)
    fig, ax = plt.subplots(figsize=(10.0, 5.8), dpi=dpi)
    fig.patch.set_facecolor("#f7f7f7")
    ax.set_facecolor("#f7f7f7")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(lower[0] - margin[0], AXIS_UPPER_BOUND)
    ax.set_ylim(lower[1] - margin[1], AXIS_UPPER_BOUND)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.suptitle(_presentation_title(run_directory / "metadata.json", edges),
                 fontsize=15, fontweight="semibold", y=0.965)
    fig.text(0.5, 0.915, "Measured robot positions", ha="center",
             color="#555555", fontsize=10)
    ax.grid(True, linestyle="--", linewidth=0.7, alpha=0.35)
    ax.set_axisbelow(True)
    region = PathPatch(_region_path([], {}), facecolor="#93b8e0",
                       edgecolor="none", alpha=0.24, zorder=1)
    ax.add_patch(region)
    obstacle_patches = []
    for x_min, y_min, x_max, y_max in obstacle_rectangles:
        obstacle = Polygon(
            [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)],
            closed=True,
            facecolor="#b8cbe8",
            edgecolor="#7894bd",
            linewidth=1.0,
            alpha=0.30,
            zorder=1,
        )
        ax.add_patch(obstacle)
        obstacle_patches.append(obstacle)
    edge_artists = []
    # Guide segments exceed the fixed view diagonal and are clipped by the axes.
    guide_length = float(np.linalg.norm(upper - lower + 2 * margin))
    for index, edge in enumerate(edges):
        color = "#65758b"
        # Affine edges are represented by parallel guides and normal separation.
        connection = None
        if not edge.affine:
            connection, = ax.plot([], [], "-", color=color, linewidth=1.6, zorder=3)
        guides = []
        dimension_artists = []
        if edge.affine:
            for _ in range(2):
                line, = ax.plot([], [], "-", color="#4c78a8", linewidth=1.0,
                                alpha=0.55, zorder=2)
                guides.append(line)
            measured_arrow = FancyArrowPatch(
                (0, 0), (0, 0), arrowstyle="-|>", mutation_scale=12,
                linestyle="--", linewidth=1.6, color="#75548c", zorder=3,
            )
            free_arrow = FancyArrowPatch(
                (0, 0), (0, 0), arrowstyle="<->", mutation_scale=11,
                linewidth=1.5, color="#2f6f9f", zorder=4,
            )
            ax.add_patch(measured_arrow)
            ax.add_patch(free_arrow)
            dimension_artists = [measured_arrow, free_arrow]
        edge_artists.append((edge, connection, guides, dimension_artists))
    node_artists = {}
    for index, vehicle_id in enumerate(sorted(vehicles)):
        color = plt.cm.tab10(index % 10)
        dot, = ax.plot([], [], "o", markersize=9, color=color,
                       markeredgecolor="white", zorder=4)
        label = ax.text(0, 0, _robot_label(vehicle_id), color=color, fontsize=9,
                        fontweight="bold", zorder=5, visible=False)
        node_artists[vehicle_id] = (dot, label)
    time_label = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                         fontsize=10, fontweight="bold", zorder=6)
    offset_label = ax.text(
        0.98, 0.98, "", transform=ax.transAxes, ha="right", va="top",
        fontsize=9.5, color="#75548c", zorder=7,
        bbox={"facecolor": "#f7f7f7", "edgecolor": "#d6ccdc", "alpha": 0.9,
              "boxstyle": "round,pad=0.35"},
        visible=False,
    )
    fig.tight_layout(rect=(0.025, 0.035, 0.985, 0.88))

    def update(frame_index):
        time_s, positions = frames[frame_index]
        region.set_path(_region_path(cycles, positions))
        offset_lines = []
        for edge, connection, guides, dimension_artists in edge_artists:
            visible = edge.source in positions and edge.target in positions
            for artist in [connection, *guides, *dimension_artists]:
                if artist is not None:
                    artist.set_visible(visible)
            if not visible:
                continue
            endpoints = np.asarray([positions[edge.source], positions[edge.target]])
            if connection is not None:
                connection.set_data(endpoints[:, 0], endpoints[:, 1])
            for point, line in zip(endpoints, guides):
                offset = guide_length * np.asarray(edge.direction)
                segment = np.asarray([point - offset, point + offset])
                line.set_data(segment[:, 0], segment[:, 1])
            if dimension_artists:
                measured_arrow, free_arrow = dimension_artists
                direction = np.asarray(edge.direction)
                segment = _perpendicular_segment(endpoints, direction)
                measured_arrow.set_positions(segment[0], segment[1])
                arrow_half_length = 0.11 * guide_length
                free_center = endpoints[0]
                free_arrow.set_positions(
                    free_center - arrow_half_length * direction,
                    free_center + arrow_half_length * direction,
                )
                normal = np.asarray(edge.normal)
                measured = float(np.dot(endpoints[1] - endpoints[0], normal))
                offset_axis = _axis_name(normal)
                prefix = f"{offset_axis}-offset" if offset_axis != "normal" else "normal offset"
                offset_lines.extend([
                    f"Target {prefix}: {edge.target_offset:.2f} m",
                    f"Measured {prefix}: {measured:+.3f} m",
                ])
        for vehicle_id, (dot, label) in node_artists.items():
            visible = vehicle_id in positions
            dot.set_visible(visible)
            label.set_visible(visible)
            if visible:
                point = np.asarray(positions[vehicle_id])
                dot.set_data([point[0]], [point[1]])
                label.set_position(point + 0.015 * span)
        offset_label.set_visible(bool(offset_lines))
        offset_label.set_text("\n".join(offset_lines))
        time_label.set_text(f"t = {time_s:.1f} s")

    try:
        save_timed_gif(fig, update, durations_ms, output_path)
    finally:
        plt.close(fig)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path,
                        help="directory containing trajectory.csv and metadata.json")
    parser.add_argument("--output", type=Path,
                        help="GIF path; default: <run_directory>/trajectory_constraints.gif")
    parser.add_argument("--fps", type=float, default=20.0,
                        help="interpolated frame rate in (0, 50]; default: 20; preserves playback speed")
    parser.add_argument("--dpi", type=int, default=180, help="resolution; default: 180")
    parser.add_argument(
        "--obstacle-rectangle",
        nargs=4,
        action="append",
        type=float,
        metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
        help="fixed obstacle rectangle; may be repeated",
    )
    args = parser.parse_args()
    obstacle_rectangles = (
        tuple(tuple(rectangle) for rectangle in args.obstacle_rectangle)
        if args.obstacle_rectangle is not None else DEFAULT_OBSTACLE_RECTANGLES
    )
    print("Saved animation to", plot_constraint_animation(
        args.run_directory, args.output, args.fps, args.dpi, obstacle_rectangles,
    ))


if __name__ == "__main__":
    main()
