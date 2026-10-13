"""Shared experiment-time sampling and streaming GIF encoding."""

from math import ceil, isfinite
from pathlib import Path
from tempfile import NamedTemporaryFile

from PIL import GifImagePlugin, Image


def animation_timing(times, fps=20.0):
    """Resample from experiment time zero, preserving duration at GIF precision."""

    if not times or any(not isfinite(t) or t < 0 for t in times):
        raise ValueError("sample times must be finite and non-negative")
    if not isfinite(fps) or not 0 < fps <= 50:
        raise ValueError("fps must be in (0, 50]")
    end = max(times)
    if end < 0.02:
        raise ValueError("animation requires at least 20 ms of recorded time")
    samples = [index / fps for index in range(ceil(end * fps))] + [end]
    by_tick = {int(round(time_s * 100)): time_s for time_s in samples}
    ticks = sorted(by_tick)
    # Avoid sub-20-ms frame delays that some GIF players lengthen automatically.
    if len(ticks) > 2 and ticks[-1] - ticks[-2] < 2:
        del ticks[-2]
    durations = [(b - a) * 10 for a, b in zip(ticks, ticks[1:])] + [20]
    return [by_tick[tick] for tick in ticks], durations


def visible_curve(times, values, time_s):
    """Reveal original samples and interpolate only the current finite segment."""

    from bisect import bisect_right

    count = bisect_right(times, time_s)
    x, y = list(times[:count]), list(values[:count])
    if (0 < count < len(times) and times[count - 1] < time_s
            and isfinite(values[count - 1]) and isfinite(values[count])):
        fraction = (time_s - times[count - 1]) / (times[count] - times[count - 1])
        x.append(time_s)
        y.append(values[count - 1] + fraction * (values[count] - values[count - 1]))
    return x, y


def save_timed_gif(figure, update, durations_ms, output_path):
    """Encode one indexed frame at a time and replace the output after success."""

    output_path = Path(output_path)
    temporary_path = None
    try:
        with NamedTemporaryFile(dir=output_path.parent, suffix=".gif", delete=False) as output:
            temporary_path = Path(output.name)
            for index, duration in enumerate(durations_ms):
                update(index)
                figure.canvas.draw()
                size = figure.canvas.get_width_height()
                with Image.frombytes("RGBA", size, figure.canvas.buffer_rgba().tobytes()) as frame:
                    # GIF color tables contain RGB triples, including on older Pillow.
                    with frame.convert("RGB") as rgb:
                        indexed = rgb.quantize(colors=256, method=Image.FASTOCTREE)
                try:
                    if index == 0:
                        header, _ = GifImagePlugin.getheader(indexed, info={"duration": duration})
                        for block in header:
                            output.write(block)
                        # A Netscape loop extension repeats the animation indefinitely.
                        output.write(b"!\xff\x0bNETSCAPE2.0\x03\x01\x00\x00\x00")
                    for block in GifImagePlugin.getdata(
                        indexed, duration=duration, disposal=2, include_color_table=True,
                    ):
                        output.write(block)
                finally:
                    indexed.close()
            output.write(b";")
        temporary_path.replace(output_path)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()
