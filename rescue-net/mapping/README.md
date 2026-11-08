# Offline mapping and relay planning

Run from `rescue-net` with Python 3.13 on Windows, or a compatible Python on Linux:

```text
python -m mapping.replay mapping/data/walk1.jsonl.gz out.png
python -m pytest -q tests/test_mapping*.py
```

Runtime dependencies are NumPy and the Python standard library. Replay does not
open hardware, contact a server, or require a display or graphics package. The
optional walk recording is supplied separately; the CLI does not download it.

## Coordinates and SLAM interface

All map positions and resolutions are in metres. The initial position is
`(0, 0)`. Heading `th` is in radians, clockwise from initial/map **+y**; forward is
`(sin(th), cos(th))`. The map does not claim an absolute compass orientation.

The integration contract is:

```python
from mapping.slam import Slam2D

slam = Slam2D()
result = slam.update(angles_deg, ranges_mm, timestamp_s, acc=acc_counts_or_none)
# result: dict with x, y, th, ok (boolean), score (normalized 0..1)
# Optional reason explains a rejected scan.
grid, origin_xy, res = slam.grid()
pose = slam.pose                  # (x, y, th)
path = slam.path                  # [(t, x, y, th), ...]
person_x, person_y = slam.to_map(bearing_deg, range_m)
```

`grid` is a two-dimensional **uint8** array: `0` free, `127` unknown, and `255`
occupied. `grid[iy, ix]` has row zero at minimum y, and `origin_xy` is the
lower-left outside corner of cell `[0, 0]`. A cell centre is
`origin_xy + ((ix + 0.5) * res, (iy + 0.5) * res)`. These are discrete classes;
relay planning and rendering do not interpret them as probabilities or log odds.

## SLAM behavior and limits

The first sufficiently populated, valid scan seeds the map at zero pose and
reports score `1`; that seed score does not independently establish accuracy.
Only accepted scans update the pose, map, and path. When accelerometer data is
present, the mapper rejects scans tilted more than 10 degrees relative to the
reference counts `[1.45, 1.33, 21.26]`. It also rejects acceleration magnitudes
outside 0.7–1.3 times the reference magnitude. Missing accelerometer readings
are allowed, so those scans have no accelerometer-based tilt check.

The fixed 6-degree mounting tilt is not geometrically corrected because its
direction is unknown. Floor or ceiling returns can therefore become apparent
obstacles in the 2D map even when localization and the tilt gate succeed.

The default map is a fixed 40 m square centred on the seed. Matching uses an
accumulated field of mostly static structure; there is no global loop closure
or automatic map resizing. Featureless corridors, repeated geometry, moving
objects, and handheld motion within a LiDAR revolution can produce ambiguous or
distorted scans. Quality gates reduce bad updates but do not guarantee that a
map never becomes corrupted. Longer outages expand the local search, which can
exceed 30 ms per update and still fail to recover tracking.

Synthetic benchmarks used Linux under WSL2, Python 3.12.3, NumPy 2.4.3, and an
Intel Xeon Gold 6230R. For simulated 6.5 m walks at 1 m/s with 4 Hz and 10 Hz
scan streams, update latency was approximately 16–17 ms at p50 and 19–20 ms at
p95; position RMSE was 1.9–3.2 cm. These figures describe those synthetic cases,
not general accuracy or timing guarantees. Windows Python 3.13 and a real
handheld walk have not been benchmarked.

## Recording and replay

Input is gzip-compressed UTF-8 JSONL. The first line must be `{"meta": {...}}`.
Following nonblank lines may be scans:

```json
{"t": 1720000000.25, "a": [1, 2, 21], "ang": [0, 9000, 18000], "mm": [1200, 2300, 1800]}
```

`ang` contains integer hundredths of a degree; replay divides by 100 before
calling SLAM. `mm` contains nonnegative integer millimetres; zero readings are
passed to SLAM for filtering. `a` is either three finite MMA7660 accelerometer
counts or `null`, not radians or g. The recorder metadata may include
`acc_ref_level`, `acc_counts_per_g`, and `lidar_mount_tilt_deg`; replay validates
the metadata envelope but uses the SLAM implementation's calibration defaults.
It does not silently change calibration from arbitrary recording fields.
Timestamps are passed in file order so SLAM can apply its own timing checks.

Recordings from `tools/tap_walk.py` may also include a finite `rx` receive
timestamp on each scan and interleave these auxiliary records:

```json
{"rx": 1720000000.26, "m": {"lvl": 0, "persons": []}}
{"rx": 1720000000.27, "msg": {"t": "MARK", "id": "relay-1"}}
```

Replay validates that each auxiliary envelope contains exactly `rx` and one of
`m` or `msg`, with a finite receive timestamp and an object payload, then counts
and skips it. Payload contents are not interpreted. MARK, PERSON, and other
relay-chain messages do not update pose or create map coordinates. Scan order
and sensor timestamps `t` are retained; `rx` never replaces `t` or changes the
reported scan duration. Unknown record envelopes, ambiguous mixed scan/event
rows, malformed scans, and recordings with no scans remain errors.

The CLI prints total/accepted/rejected scans, input-point count, rejection reasons
when available, counts of skipped spotter-meta and node-message records when
present, recording duration, replay-plus-PNG timing, and final pose. Exit
status is `0` when at least one scan is accepted, `1` when none are accepted
(an image is still written), and `2` for a recording/interface/file error. Scan
format errors identify the offending line. The output directory must exist;
the PNG replaces the specified output atomically after successful replay and
cannot overwrite the input recording.

The PNG has one pixel per grid cell, with +y upward: free space is off-white,
unknown space gray, occupied cells dark, trajectory blue, current position orange,
and forward heading green. Path/pose overlays may cover occupancy pixels. The
PNG is encoded with `struct`/`zlib`, without Pillow, OpenCV, or Matplotlib.

## Relay suggestions

```python
from mapping.relay import suggest_relays

recommendations = suggest_relays(
    grid, origin_xy, res,
    nodes={"base": (0.0, 0.0), "relay-1": (4.0, 2.0)},
    path=slam.path,
)
# [{"x": map_x_metres, "y": map_y_metres, "reason": explanation}, ...]
```

Paths may contain either `(t, x, y, th)` SLAM rows or `(x, y)` dashboard rows.
An empty node mapping returns an empty list until the operator places a node.
Planning starts with the known node nearest
the first path point and switches to a node when the walk reaches its vicinity.
It samples the recorded segments at no more than one cell or 0.25 m apart. Before
predicted margin falls below the threshold, it suggests the last eligible free
point, then assumes that suggestion becomes the next anchor. Suggestions are
sequential: later suggestions assume earlier ones have been installed.

All radio and placement parameters are centralized in `RelayModel` / `MODEL` in
`relay.py`. The default planning heuristic is:

```text
margin_dB = 30 - 20 * log10(max(distance_m, 1)) - 8 * wall_segments
suggest before margin_dB < 8
```

The raycast counts contiguous occupied **segments**, so a wall eight cells thick
counts once. A free/unknown gap splits segments; this is not physical wall
recognition. An exact grid-corner crossing advances both ray axes. Unknown and
outside-map cells do not invent walls; any such radio path is marked as crossing
unmapped space in the reason. Every placement itself must be in a known-free
cell, at least 1 m from known/planned nodes, with no occupied cell in its small
clearance neighborhood. Unknown neighboring cells add a caution to the reason.

This is an **unmeasured heuristic**, not a calibrated propagation model or a
coverage guarantee. It omits antenna orientation, material differences,
interference, and multipath. Empty recommendations can mean there was no safe
placement before a predicted loss, not that connectivity is assured. Verify
placements and calibrate the constants with actual radio measurements before
using them for operational decisions.
