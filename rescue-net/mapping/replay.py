"""Replay recorded LiDAR scans and export a map with NumPy and the stdlib."""
import argparse
from collections import Counter
from dataclasses import dataclass
import gzip
import json
import math
import os
from pathlib import Path
import struct
import sys
import tempfile
import time
import zlib

import numpy as np


class ReplayError(ValueError):
    """Invalid recording or SLAM output, with actionable input context."""


@dataclass
class ReplayStats:
    scans: int
    accepted: int
    points: int
    elapsed_s: float
    recorded_s: float
    pose: tuple
    rejection_reasons: dict
    output: Path

    @property
    def rejected(self):
        return self.scans - self.accepted


def _finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ReplayError(f"{label} must be a finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ReplayError(f"{label} must be a finite number") from None
    if not math.isfinite(result):
        raise ReplayError(f"{label} must be a finite number")
    return result


def _scan(record, line_number):
    label = f"line {line_number}"
    if not isinstance(record, dict):
        raise ReplayError(f"{label}: scan must be a JSON object")
    timestamp = _finite(record.get("t"), f"{label}: t")
    angles, distances = record.get("ang"), record.get("mm")
    if not isinstance(angles, list) or not isinstance(distances, list):
        raise ReplayError(f"{label}: ang and mm must be integer arrays")
    if len(angles) != len(distances) or not angles:
        raise ReplayError(f"{label}: ang and mm must have the same nonzero length")
    for name, values in (("ang", angles), ("mm", distances)):
        if any(type(value) is not int for value in values):
            raise ReplayError(f"{label}: {name} must contain integers")
        for value in values:
            _finite(value, f"{label}: {name}")
    if any(value < 0 for value in distances):
        raise ReplayError(f"{label}: mm cannot contain negative distances")
    acceleration = record.get("a")
    if acceleration is not None:
        if not isinstance(acceleration, list) or len(acceleration) != 3:
            raise ReplayError(f"{label}: a must be null or [ax, ay, az]")
        acceleration = [_finite(value, f"{label}: a") for value in acceleration]
    return (np.asarray(angles, dtype=np.float64) / 100.0,
            np.asarray(distances, dtype=np.float64), timestamp, acceleration)


def _read_json(line, line_number):
    try:
        return json.loads(line)
    except (json.JSONDecodeError, ValueError) as exc:
        raise ReplayError(f"line {line_number}: invalid JSON ({exc})") from None


def _dot(image, x, y, color, radius=1):
    height, width = image.shape[:2]
    for iy in range(max(0, y - radius), min(height, y + radius + 1)):
        for ix in range(max(0, x - radius), min(width, x + radius + 1)):
            if (ix - x) ** 2 + (iy - y) ** 2 <= radius ** 2:
                image[iy, ix] = color


def _line(image, start, end, color, radius=0):
    # Clip before rasterizing, including paths that extend beyond the map.
    height, width = image.shape[:2]
    x, y = start
    dx, dy = end[0] - x, end[1] - y
    low, high = 0.0, 1.0
    for p, q in ((-dx, x), (dx, width - 1 - x), (-dy, y), (dy, height - 1 - y)):
        if p == 0:
            if q < 0:
                return
        elif p < 0:
            low = max(low, q / p)
        else:
            high = min(high, q / p)
        if low > high:
            return
    ax, ay = round(x + low * dx), round(y + low * dy)
    bx, by = round(x + high * dx), round(y + high * dy)
    steps = max(abs(bx - ax), abs(by - ay), 1)
    for step in range(steps + 1):
        _dot(image, round(ax + (bx - ax) * step / steps),
             round(ay + (by - ay) * step / steps), color, radius)


def render_map(grid, origin_xy, res, path, pose):
    """Return RGB pixels, with +y upward and clockwise heading from +y."""
    grid = np.asarray(grid)
    if (grid.ndim != 2 or not grid.size or grid.dtype != np.uint8
            or not np.isin(grid, (0, 127, 255)).all()):
        raise ReplayError("SLAM grid must be nonempty uint8 with values 0/127/255")
    try:
        if len(origin_xy) != 2 or len(pose) != 3:
            raise ValueError
        origin = tuple(float(v) for v in origin_xy)
        pose = tuple(float(v) for v in pose)
        res = float(res)
        if not all(math.isfinite(v) for v in (*origin, *pose, res)) or res <= 0:
            raise ValueError
    except (TypeError, ValueError, OverflowError):
        raise ReplayError("SLAM origin, resolution, and pose must be finite; res > 0") from None
    gray = np.full(grid.shape, 150, dtype=np.uint8)
    gray[grid == 0] = 245
    gray[grid == 255] = 25
    image = np.repeat(np.flipud(gray)[:, :, None], 3, axis=2)
    height = grid.shape[0]

    def pixel(x, y):
        return ((x - origin[0]) / res - 0.5,
                height - 0.5 - (y - origin[1]) / res)

    previous = None
    for index, row in enumerate(path):
        try:
            if len(row) != 4 or not all(math.isfinite(float(v)) for v in row):
                raise ValueError
            point = pixel(float(row[1]), float(row[2]))
        except (TypeError, ValueError, OverflowError):
            raise ReplayError(f"SLAM path[{index}] must be finite (t, x, y, th)") from None
        if previous is not None:
            _line(image, previous, point, (30, 120, 230), radius=1)
        previous = point
    x, y, heading = pose
    current = pixel(x, y)
    arrow_m = max(0.5, 6 * res)
    tip = pixel(x + arrow_m * math.sin(heading), y + arrow_m * math.cos(heading))
    _line(image, current, tip, (25, 180, 80), radius=1)
    if -4 <= current[0] < image.shape[1] + 4 and -4 <= current[1] < height + 4:
        _dot(image, round(current[0]), round(current[1]), (240, 100, 35), radius=3)
    return image


def write_png(path, image):
    """Atomically write an RGB PNG; no graphics library is required."""
    image = np.asarray(image)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or not image.size:
        raise ReplayError("PNG image must be a nonempty HxWx3 uint8 array")
    height, width = image.shape[:2]

    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))

    compressor = zlib.compressobj()
    pieces = [compressor.compress(b"\0" + row.tobytes()) for row in image]
    pieces.append(compressor.flush())
    content = (b"\x89PNG\r\n\x1a\n"
               + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
               + chunk(b"IDAT", b"".join(pieces)) + chunk(b"IEND", b""))
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=f".{path.name}.",
                                         suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(content)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def replay_file(input_path, output_path, *, slam=None):
    """Stream gzip JSONL through Slam2D; injected ``slam`` supports offline tests."""
    input_path, output_path = Path(input_path), Path(output_path)
    if input_path.resolve() == output_path.resolve():
        raise ReplayError("input recording and output image must be different files")
    if slam is None:
        from .slam import Slam2D
        slam = Slam2D()
    started = time.perf_counter()
    scans = accepted = points = 0
    first_t = last_t = None
    reasons = Counter()
    try:
        with gzip.open(input_path, "rt", encoding="utf-8") as stream:
            first = stream.readline()
            if not first:
                raise ReplayError("recording is empty")
            metadata = _read_json(first, 1)
            if not isinstance(metadata, dict) or not isinstance(metadata.get("meta"), dict):
                raise ReplayError('line 1: expected {"meta": {...}} recording metadata')
            for line_number, line in enumerate(stream, 2):
                if not line.strip():
                    continue
                angles, distances, timestamp, acceleration = _scan(_read_json(line, line_number), line_number)
                result = slam.update(angles, distances, timestamp, acc=acceleration)
                if not isinstance(result, dict) or not isinstance(result.get("ok"), (bool, np.bool_)):
                    raise ReplayError("Slam2D.update must return a dict containing boolean ok")
                scans += 1
                points += len(angles)
                accepted += int(result["ok"])
                if not result["ok"]:
                    reasons[str(result.get("reason", "unspecified"))] += 1
                if first_t is None:
                    first_t = timestamp
                last_t = timestamp
    except (gzip.BadGzipFile, EOFError, UnicodeError, zlib.error) as exc:
        raise ReplayError(f"cannot decode gzip JSONL recording: {exc}") from None
    if scans == 0:
        raise ReplayError("recording contains metadata but no scans")
    try:
        grid, origin, res = slam.grid()
        pose = tuple(float(v) for v in slam.pose)
        image = render_map(grid, origin, res, slam.path, pose)
    except (TypeError, AttributeError) as exc:
        raise ReplayError(f"invalid Slam2D map/pose interface: {exc}") from None
    write_png(output_path, image)
    return ReplayStats(scans, accepted, points, time.perf_counter() - started,
                       max(0.0, last_t - first_t), pose, dict(reasons), output_path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", help="gzip JSONL recording, metadata first")
    parser.add_argument("output", help="output PNG path (parent directory must exist)")
    args = parser.parse_args(argv)
    try:
        stats = replay_file(args.recording, args.output)
    except (ReplayError, OSError, ValueError) as exc:
        print(f"Replay failed: {exc}", file=sys.stderr)
        return 2
    x, y, heading = stats.pose
    print(f"Scans: {stats.scans}; accepted: {stats.accepted}; rejected: {stats.rejected}; "
          f"input points: {stats.points}")
    print(f"Recording: {stats.recorded_s:.2f} s; replay + PNG: {stats.elapsed_s:.3f} s; "
          f"{stats.scans / max(stats.elapsed_s, 1e-9):.1f} scans/s")
    print(f"Final pose: x={x:.3f} m, y={y:.3f} m, "
          f"heading={math.degrees(heading):.2f} deg clockwise from +y")
    if stats.rejection_reasons:
        print("Rejected scans: " + ", ".join(f"{reason}={count}" for reason, count in sorted(stats.rejection_reasons.items())))
    print(f"PNG: {stats.output}")
    if stats.accepted == 0:
        print("No scans were accepted; image is not a validated map.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
