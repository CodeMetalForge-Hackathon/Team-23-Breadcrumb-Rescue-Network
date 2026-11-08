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

slam = Slam2D(
    res=0.05, size_m=40, min_range=0.6, max_range=12,
    sensor_height_m=1.0, mount_tilt_deg=6.0,
)
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

The default minimum range is 0.6 m. The previous 0.25 m cutoff already removed
reported rig returns near 0.1 m, but walk2 also contains persistent rear returns
at roughly 0.3–0.6 m during motion. Operator/body occlusion is a plausible source,
not a confirmed object classification. The new cutoff also discards real nearby
obstacles; callers can override `min_range` for another installation.

With valid accelerometer data, matching and map integration also discard ranges
beyond a conservative floor-intersection bound:

```text
total_tilt = min(90 degrees, relative_tilt + mount_tilt_deg + 1 degree)
effective_max_range = min(max_range, 0.95 * sensor_height_m / sin(total_tilt))
```

`sensor_height_m` is the minimum sensor height above a level floor, defaulting to
1.0 m; `mount_tilt_deg` bounds the fixed LiDAR-plane inclination, defaulting to
6 degrees. The 5% height reduction and extra degree are modeling margins. The
mount tilt axis is unknown, so this is a range restriction, not per-ray floor
classification or geometric tilt correction. It also discards real distant
walls and can weaken along-corridor localization. The integer accelerometer
resolution is about 2.7 degrees per horizontal count near level. Floor/ceiling
returns and scan distortion can still corrupt the map. Setting
`sensor_height_m=None`, or supplying no accelerometer reading, disables this
additional cap; the configured `max_range` still applies. The 10-degree
whole-scan tilt rejection remains in force when acceleration is supplied.

Pause mapping explicitly **before placing the carried rig on the floor**:

```python
slam.set_mapping_enabled(False)
enabled = slam.mapping_enabled  # Read-only boolean; False after the call.
# After restoring the same known position, orientation, and sensor setup:
slam.set_mapping_enabled(True)
```

Mapping starts enabled. The caller must invoke this control; the mapper does
not infer sensor height or detect floor placement. The default 1.0 m height
still requires the corresponding physical setup. Disabling the floor-range
cap with `sensor_height_m=None` does not make floor placement safe for mapping.
While paused, each `update` returns `ok=False`, `reason="mapping_paused"`, and
increments `rejected`, while preserving the accepted pose, path, and grid.
`to_map` continues to use that last pose, which becomes stale if the sensor moves.

Resume clears velocity prediction and uses the nominal local search until the
first resumed scan is accepted; rejected scans do not consume this one-shot
resume state. The last accepted timestamp is retained, so resumed explicit
timestamps must still be newer than that timestamp. Enabling mapping does not
verify the sensor's location or relocalize it automatically. Resume the existing
map only at its last known position and orientation with the correct sensor
configuration. If the sensor moved and that pose cannot be restored, start a
fresh mapper instead of continuing the old map at an assumed pose.

The default map is a fixed 40 m square centred on the seed. Matching uses an
accumulated field of mostly static structure; there is no global loop closure
or automatic map resizing. Featureless corridors, repeated geometry, moving
objects, and handheld motion within a LiDAR revolution can produce ambiguous or
distorted scans. Quality gates reduce bad updates but do not guarantee that a
map never becomes corrupted. Longer outages expand the local search, which can
exceed 30 ms per update and still fail to recover tracking.

Initial synthetic benchmarks used Linux under WSL2, Python 3.12.3, NumPy 2.4.3, and an
Intel Xeon Gold 6230R. For simulated 6.5 m walks at 1 m/s with 4 Hz and 10 Hz
scan streams, update latency was approximately 16–17 ms at p50 and 19–20 ms at
p95; position RMSE was 1.9–3.2 cm. These figures describe those synthetic cases,
not general accuracy or timing guarantees. Windows Python 3.13 has not been
benchmarked. Offline real-recording checks must distinguish return-to-start
agreement from intermediate trajectory accuracy: good final closure alone can
hide compressed or distorted corridor geometry. No intermediate positions in
walk2 have been surveyed, and the mapper does not enforce a known endpoint.

An independent offline replay of the 3,411-scan, 419.81 s `walk2.jsonl.gz` on
the same Linux environment compared the original mapper with the combined
0.6 m minimum and attitude-dependent maximum range:

| Implementation | Accepted / rejected scans | BASE MARK to final position | Update p95 |
| --- | ---: | ---: | ---: |
| Original 0.25–12 m range | 3,177 / 234 | 8.2165 m | 12.50 ms |
| Current combined filtering | 3,183 / 228 | 0.0662 m | 16.02 ms |

The current run's rejects were 227 excessive-tilt scans and one implausible
motion. Its update p50 was 11.82 ms, p99 18.86 ms, and maximum 70.32 ms; the
complete instrumented replay took 39.58 s. The human reported returning within
0.5 m of BASE. The MARK occurs about 118.4 s after recording begins, and the
table measures from that MARK's estimated pose. This is one recorded walk with
an approximate return constraint, not surveyed centimetre-level ground truth
or a general accuracy benchmark. The first and final locations were not
supplied to the mapper as constraints. Independently masking only near or far
returns also improved final closure, so these results do not establish floor
contamination as the sole cause of the original drift.

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
