#!/usr/bin/env python3
"""Animate the two subspace error plots using recorded sample intervals."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
from math import isfinite
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

if __package__:
    from .animation_frames import animation_timing, save_timed_gif, visible_curve
    from .postprocess import _read_subspace_summary
else:
    from animation_frames import animation_timing, save_timed_gif, visible_curve
    from postprocess import _read_subspace_summary


def _read_errors(run_directory: Path, max_time_s: float | None):
    """Read the same recorded quantities and validity rules as postprocess.py."""

    rows = _read_subspace_summary(run_directory / "subspace_summary.csv")
    rms, times = [], set()
    for row in rows:
        time_s = row["time_s"]
        if not isfinite(time_s) or time_s < 0:
            raise ValueError("sample times must be finite and non-negative")
        if max_time_s is not None and time_s > max_time_s:
            continue
        times.add(time_s)
        value = row.get("subspace_edge_rms", float("nan"))
        # The static RMS plot omits nonfinite samples before drawing the line.
        if isfinite(value):
            rms.append((time_s, value))
    if not times:
        raise ValueError("subspace_summary.csv contains no samples in the selected range")
    edges = defaultdict(list)
    with (run_directory / "subspace_edges.csv").open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            time_s = float(row["time_s"])
            if not isfinite(time_s) or time_s < 0:
                raise ValueError("sample times must be finite and non-negative")
            if max_time_s is not None and time_s > max_time_s:
                continue
            value = (float(row["normalized_error"])
                     if row["valid"] == "1" and row["normalized_error"]
                     else float("nan"))
            edges[(row["source_id"], row["target_id"])].append((time_s, value))
            times.add(time_s)
    if not edges:
        raise ValueError("subspace_edges.csv contains no samples in the selected range")
    for points in edges.values():
        points.sort(key=lambda point: point[0])
    return rms, dict(sorted(edges.items())), sorted(times)


def _frame_timing(times: list[float], fps: float = 20.0) -> tuple[list[float], list[int]]:
    """Use the same experiment-time frame grid as the trajectory animations."""

    return animation_timing(times, fps)


def _write_gif(figure, axis, series, frame_times, durations_ms, output_path):
    """Reveal samples cumulatively and encode each frame with its own delay."""

    cursor = axis.axvline(0.0, color="#888888", linestyle=":", linewidth=1.2)
    time_label = axis.text(
        0.02, 0.98, "", transform=axis.transAxes, va="top", fontsize=18,
        bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.8},
    )
    def update(index):
        time_s = frame_times[index]
        for line, times, values in series:
            line.set_data(*visible_curve(times, values, time_s))
        cursor.set_xdata([time_s, time_s])
        time_label.set_text(f"t = {time_s:.2f} s")

    try:
        save_timed_gif(figure, update, durations_ms, output_path)
    finally:
        plt.close(figure)


def plot_error_animations(
    run_directory: Path,
    output_directory: Path | None = None,
    dpi: int = 160,
    max_time_s: float | None = None,
    fps: float = 20.0,
) -> tuple[Path, Path]:
    """Write synchronized RMS and per-edge GIFs in the experiment time scale."""

    if dpi <= 0:
        raise ValueError("dpi must be positive")
    if max_time_s is not None and (not isfinite(max_time_s) or max_time_s < 0):
        raise ValueError("max_time_s must be finite and non-negative")
    run_directory = run_directory.resolve()
    rms, edges, times = _read_errors(run_directory, max_time_s)
    frame_times, durations_ms = _frame_timing(times, fps)
    output_directory = (output_directory or run_directory).resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    rms_path = output_directory / "analysis.gif"
    edge_path = output_directory / "edge_error.gif"

    figure, axis = plt.subplots(figsize=(10, 9), dpi=dpi)
    rms_line, = axis.plot([], [], color="black", linewidth=2.0)
    axis.set_xlabel("time (s)")
    axis.set_ylabel("Normalized constraint error (RMS)")
    axis.set_xlim(0, times[-1])
    axis.set_ylim(0, 1)
    axis.grid(True, linestyle="--", alpha=0.4)
    for label in ([axis.xaxis.label, axis.yaxis.label]
                  + axis.get_xticklabels() + axis.get_yticklabels()):
        label.set_fontsize(label.get_fontsize() * 3)
    figure.tight_layout()
    series = [(rms_line, [point[0] for point in rms], [point[1] for point in rms])]
    _write_gif(figure, axis, series, frame_times, durations_ms, rms_path)

    with plt.rc_context({"font.size": 30}):
        figure, axis = plt.subplots(figsize=(12, 9), dpi=dpi)
        series = []
        for index, (edge, points) in enumerate(edges.items()):
            line, = axis.plot([], [], linewidth=2.0,
                              linestyle=("-", "--", "-.")[index % 3],
                              label="%s → %s" % edge)
            series.append((line, [point[0] for point in points],
                           [point[1] for point in points]))
        axis.set_xlabel("time (s)")
        axis.set_ylabel("Normalized constraint error")
        axis.set_xlim(0, times[-1])
        axis.set_ylim(0, 1)
        axis.grid(True, linestyle="--", alpha=0.4)
        axis.legend(loc="upper right", fontsize=18, ncol=3)
        figure.tight_layout()
        _write_gif(figure, axis, series, frame_times, durations_ms, edge_path)
    print(f"Timing: data end {times[-1]:.6f} s; GIF loop {sum(durations_ms) / 1000:.3f} s",
          flush=True)
    return rms_path, edge_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory", type=Path,
                        help="directory containing subspace_summary.csv and subspace_edges.csv")
    parser.add_argument("--output-directory", type=Path, help="default: the run directory")
    parser.add_argument("--dpi", type=int, default=160, help="resolution; default: 160")
    parser.add_argument("--max-time-s", type=float, help="include samples at or before this time")
    parser.add_argument("--fps", type=float, default=20.0,
                        help="interpolated frame rate in (0, 50]; default: 20; preserves playback speed")
    args = parser.parse_args()
    for path in plot_error_animations(
        args.run_directory, args.output_directory, args.dpi, args.max_time_s, args.fps,
    ):
        print("Saved animation to", path)


if __name__ == "__main__":
    main()
