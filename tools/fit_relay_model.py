#!/usr/bin/env python3
"""Fit an offline Wi-Fi relay model from a known, fixed AP and a recorded walk.

From the repository root::

    python tools/fit_relay_model.py WALK.jsonl.gz --node X,Y \
        [--t0 SECONDS --t1 SECONDS] [--node-uncertainty METRES] [--output PATH]

--node ASSERTS the AP stayed at that position in the replay's map frame for
the selected interval. The tool cannot establish AP identity, position, or
stationarity. For walk3 the coordinator supplied an approximate fixed B2 at
(0,0), with +/-0.5 m uncertainty; this is not derived from the earlier BASE
MARK, which precedes the first accepted SLAM pose. Supply that uncertainty
explicitly with --node-uncertainty 0.5; it is recorded, not silently treated as
surveyed geometry or propagated into the coefficient standard errors.
The inclusive window is elapsed RECEIVE time (rx) relative to the first scan's
rx, not the first MARK or the scan's source clock. All scans are replayed in
input order to construct the final map, regardless of the RSSI fitting window.
Time-window, refresh-spacing, and pose-age comparisons allow one microsecond
for float subtraction rounding; stored timestamps are never rounded or changed.

Only m.rssi (the spotter's Wi-Fi association RSSI) is used, never message RSSI
from another radio hop. Consecutive equal RSSI broadcasts collapse to one run.
The initial run is skipped because its measurement age is unknown; subsequent
changed-value runs supply at most one sample, spaced at least two source-clock
seconds apart. Unchanged values at distinct two-second refreshes cannot be
distinguished and are deliberately undersampled. Broadcast time is still only
an approximation of measurement time; no independent radio timestamp exists.
At most eight samples per 0.1-decade distance bin are retained, evenly spread
in receive order without inspecting RSSI, so a stationary near-field tail
does not dominate just because it lasts longer.

Each candidate uses the latest preceding scan on the shared m.t / scan.t clock
when m.t exists, otherwise on receive rx (reported per sample). That latest
scan must be accepted, score >= 0.6, and at most 0.5 seconds old. Rejected scans
block reuse of older accepted poses. Source/receive timestamps and pose age
are retained in the output. Replay quality and final wall extraction are not
surveyed geometric truth. Missing scan rx or decreasing clocks are errors.

The least-squares model is RSSI = P0 - 10*n*log10(max(distance_m,1)) - W*walls.
Wall counts use mapping.relay.count_wall_crossings on final floorplan segments.
A usable fit needs >= 6 selected samples, rank 3, condition <= 10,000,
log10-distance span >= 0.15, varying wall counts, RMS <= 10 dB, and plausible
P0 [-120,0] dBm, n [0.5,6], W [0,40] dB. These are explicit quality heuristics,
not statistical assurance. n/W identifiability is also reported separately.

JSON keeps raw fit diagnostics separate from the effective model. Rejected
fits use safe defaults with calibrated=false; an existing output is preserved
on rejection. Output defaults to relay_model.json beside the input recording;
--output overrides it. Exit codes:
0 usable fit, 1 rejected fit, 2 invalid input or file error. NumPy + stdlib only.
"""

import argparse
from bisect import bisect_right
from collections import Counter
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
import zlib

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "rescue-net"))
from mapping.replay import ReplayError, _finite, _read_json, _record_kind, _scan


DEFAULT_MODEL = dict(p0_dbm=-50.0, path_loss_exponent=2.0, wall_loss_db=8.0,
                     threshold_dbm=-72.0, calibrated=False)
MIN_SAMPLES = 6
MIN_LOG_DISTANCE_SPAN = 0.15
MAX_CONDITION = 10000.0
MAX_RMS_DB = 10.0
MAX_POSE_AGE_S = 0.5
MIN_POSE_SCORE = 0.6
RSSI_REFRESH_S = 2.0
DISTANCE_BIN_LOG10 = 0.1
MAX_SAMPLES_PER_DISTANCE_BIN = 8
TIME_EPS_S = 1e-6
WINDOW_EPS_S = TIME_EPS_S
COEFFICIENT_BOUNDS = ((-120.0, 0.0), (0.5, 6.0), (0.0, 40.0))
COEFFICIENT_NAMES = ("p0_dbm", "path_loss_exponent", "wall_loss_db")


def _vector(values, name):
    try:
        raw = np.asarray(values)
        if raw.dtype.kind in "bc" or raw.ndim != 1:
            raise ValueError
        array = raw.astype(float)
        if not np.isfinite(array).all():
            raise ValueError
    except (TypeError, ValueError, OverflowError):
        raise ReplayError(f"{name} must be a one-dimensional finite numeric array") from None
    return array


def fit_samples(distances_m, wall_counts, rssi_dbm):
    """Fit known geometry; return effective model plus separate raw diagnostics."""
    distance = _vector(distances_m, "distances_m")
    walls = _vector(wall_counts, "wall_counts")
    rssi = _vector(rssi_dbm, "rssi_dbm")
    if not (len(distance) == len(walls) == len(rssi)):
        raise ReplayError("distance, wall count, and RSSI arrays must have equal lengths")
    if np.any(distance < 0) or np.any(walls < 0) or np.any(walls != np.floor(walls)):
        raise ReplayError("distances must be nonnegative and wall counts nonnegative integers")
    if np.any((rssi < -150) | (rssi > 0)):
        raise ReplayError("RSSI must be in [-150, 0] dBm")
    n_samples = len(rssi)
    log_distance = np.log10(np.maximum(distance, 1.0))
    design = np.column_stack((np.ones(n_samples), -10 * log_distance, -walls))
    rank = int(np.linalg.matrix_rank(design)) if n_samples else 0
    identifiable = {
        name: bool(rank > np.linalg.matrix_rank(np.delete(design, i, axis=1)))
        if n_samples else False for i, name in enumerate(COEFFICIENT_NAMES)
    }
    coefficients = None
    rms = condition = None
    standard_errors = None
    if n_samples:
        beta, _, _, singular = np.linalg.lstsq(design, rssi, rcond=None)
        coefficients = dict(zip(COEFFICIENT_NAMES, map(float, beta)))
        rms = float(np.sqrt(np.mean((design @ beta - rssi) ** 2)))
        if rank == 3:
            condition = float(singular[0] / singular[-1])
            if n_samples > 3:
                variance = float(np.sum((design @ beta - rssi) ** 2) / (n_samples - 3))
                covariance = np.linalg.pinv(design.T @ design) * variance
                standard_errors = dict(zip(COEFFICIENT_NAMES,
                                           map(float, np.sqrt(np.maximum(0, np.diag(covariance))))))
    log_span = float(np.ptp(log_distance)) if n_samples else 0.0
    wall_range = [int(walls.min()), int(walls.max())] if n_samples else None
    reasons = []
    if n_samples < MIN_SAMPLES:
        reasons.append(f"need at least {MIN_SAMPLES} independent candidate samples")
    if rank != 3:
        reasons.append("rank-deficient fit: P0, distance loss, and wall loss are not jointly identifiable")
    if condition is not None and condition > MAX_CONDITION:
        reasons.append("ill-conditioned fit")
    if log_span < MIN_LOG_DISTANCE_SPAN:
        reasons.append("insufficient distance variation")
    if wall_range is None or wall_range[0] == wall_range[1]:
        reasons.append("insufficient wall-count variation")
        if wall_range is not None:
            reasons.append("wall loss W is unobserved at zero crossings" if wall_range[0] == 0 else
                           "constant wall count confounds wall loss W with intercept P0")
    if coefficients is not None:
        for name, (low, high) in zip(COEFFICIENT_NAMES, COEFFICIENT_BOUNDS):
            value = coefficients[name]
            if not math.isfinite(value) or not low - 1e-9 <= value <= high + 1e-9:
                reasons.append(f"nonphysical or implausible {name}: expected [{low:g}, {high:g}]")
    if rms is not None and rms > MAX_RMS_DB:
        reasons.append(f"RMS residual exceeds {MAX_RMS_DB:g} dB")
    model = dict(DEFAULT_MODEL)
    if not reasons:
        for name, (low, high) in zip(COEFFICIENT_NAMES, COEFFICIENT_BOUNDS):
            model[name] = min(high, max(low, coefficients[name]))
        model["calibrated"] = True
    model["fit"] = dict(sample_count=n_samples, coefficients=coefficients, rms_db=rms,
                        rank=rank, condition_number=condition, identifiable=identifiable,
                        distance_range_m=[float(distance.min()), float(distance.max())]
                        if n_samples else None, log10_distance_span=log_span,
                        wall_count_range=wall_range, coefficient_standard_errors=standard_errors,
                        rejection_reasons=reasons)
    model["quality_limits"] = dict(min_samples=MIN_SAMPLES,
                                   min_log10_distance_span=MIN_LOG_DISTANCE_SPAN,
                                   max_condition=MAX_CONDITION, max_rms_db=MAX_RMS_DB,
                                   coefficient_bounds=dict(zip(COEFFICIENT_NAMES, COEFFICIENT_BOUNDS)))
    return model


def _node_xy(value):
    try:
        values = value.split(",") if isinstance(value, str) else value
        if len(values) != 2:
            raise ValueError
        node = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in node):
            raise ValueError
        return node
    except (TypeError, ValueError, OverflowError):
        raise ReplayError("node must be two finite map coordinates X,Y") from None


def fit_recording(input_path, node_xy, t0=0.0, t1=None, *, node_uncertainty_m=None,
                  slam=None, wall_extractor=None):
    """Replay all scans; fit only eligible RSSI events in the elapsed-rx window.

    ``slam`` and ``wall_extractor`` injection supports offline integration tests.
    The recording is never modified, and no model is written by this function.
    """
    node = _node_xy(node_xy)
    t0 = _finite(t0, "t0")
    t1 = None if t1 is None else _finite(t1, "t1")
    if t0 < 0 or (t1 is not None and t1 < t0):
        raise ReplayError("require 0 <= t0 <= t1 (elapsed receive seconds)")
    if node_uncertainty_m is not None:
        node_uncertainty_m = _finite(node_uncertainty_m, "node uncertainty")
        if node_uncertainty_m < 0:
            raise ReplayError("node uncertainty must be nonnegative")
    if slam is None:
        from mapping.slam import Slam2D
        slam = Slam2D()
    if wall_extractor is None:
        from mapping.floorplan import extract
        wall_extractor = extract
    from mapping.relay import count_wall_crossings

    input_path = Path(input_path)
    started = time.perf_counter()
    scans, events = [], []
    counts = Counter()
    previous_rx = previous_scan_t = previous_meta_t = None
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
                record = _read_json(line, line_number)
                kind = _record_kind(record, line_number)
                rx = _finite(record.get("rx"), f"line {line_number}: rx (required for fitting)")
                if previous_rx is not None and rx < previous_rx:
                    raise ReplayError(f"line {line_number}: receive timestamps decrease")
                previous_rx = rx
                if kind == "scan":
                    angles, distances, stamp, acceleration = _scan(record, line_number)
                    if previous_scan_t is not None and stamp < previous_scan_t:
                        raise ReplayError(f"line {line_number}: scan source timestamps decrease")
                    previous_scan_t = stamp
                    result = slam.update(angles, distances, stamp, acc=acceleration)
                    if not isinstance(result, dict) or not isinstance(result.get("ok"), (bool, np.bool_)):
                        raise ReplayError("Slam2D.update must return boolean ok")
                    pose = [_finite(result.get(key), f"SLAM {key}") for key in ("x", "y", "th")]
                    score = _finite(result.get("score"), "SLAM score")
                    if not 0 <= score <= 1:
                        raise ReplayError("SLAM score must be in [0, 1]")
                    scans.append(dict(t=stamp, rx=rx, pose=pose, ok=bool(result["ok"]),
                                      score=score, reason=str(result.get("reason", "")), line=line_number))
                    counts["scans"] += 1
                    counts["accepted_scans" if result["ok"] else "rejected_scans"] += 1
                elif kind == "m":
                    counts["meta_events"] += 1
                    payload = record["m"]
                    rssi = payload.get("rssi")
                    if rssi is None:
                        counts["meta_without_rssi"] += 1
                        continue
                    rssi = _finite(rssi, f"line {line_number}: m.rssi")
                    if not -150 <= rssi <= 0:
                        raise ReplayError(f"line {line_number}: m.rssi must be in [-150, 0] dBm")
                    stamp = None if "t" not in payload else _finite(payload["t"], f"line {line_number}: m.t")
                    if stamp is not None:
                        if previous_meta_t is not None and stamp < previous_meta_t:
                            raise ReplayError(f"line {line_number}: metadata source timestamps decrease")
                        previous_meta_t = stamp
                    events.append(dict(rx=rx, t=stamp, rssi_dbm=rssi, line=line_number))
                else:
                    counts["message_events"] += 1
    except (gzip.BadGzipFile, EOFError, UnicodeError, zlib.error) as exc:
        raise ReplayError(f"cannot decode gzip JSONL recording: {exc}") from None
    if not scans:
        raise ReplayError("recording contains metadata but no scans")
    grid, origin, res = slam.grid()
    floorplan = wall_extractor(grid, origin, res)
    walls = floorplan["walls"]
    source_times = [row["t"] for row in scans]
    receive_times = [row["rx"] for row in scans]
    first_rx = receive_times[0]
    selected = []
    previous_rssi = None
    last_sample_clock = {}
    last_sample_rx = -math.inf
    for event in events:
        if previous_rssi is not None and event["rssi_dbm"] == previous_rssi:
            counts["duplicate_rssi_broadcasts"] += 1
            continue
        initial = previous_rssi is None
        previous_rssi = event["rssi_dbm"]
        counts["rssi_value_runs"] += 1
        if initial:
            counts["initial_unknown_age_run"] += 1
            continue
        elapsed = event["rx"] - first_rx
        if elapsed < t0 - WINDOW_EPS_S or (t1 is not None and elapsed > t1 + WINDOW_EPS_S):
            counts["outside_window"] += 1
            continue
        clock = "source_t" if event["t"] is not None else "receive_rx"
        stamp = event["t"] if clock == "source_t" else event["rx"]
        if (stamp - last_sample_clock.get(clock, -math.inf) < RSSI_REFRESH_S - TIME_EPS_S
                or event["rx"] - last_sample_rx < RSSI_REFRESH_S - TIME_EPS_S):
            counts["refresh_spacing"] += 1
            continue
        # Reserve the radio time even if its pose fails, so a later broadcast
        # cannot create another sample from the same approximate refresh.
        last_sample_clock[clock] = stamp
        last_sample_rx = event["rx"]
        timeline = source_times if clock == "source_t" else receive_times
        index = bisect_right(timeline, stamp) - 1
        if index < 0:
            counts["no_preceding_scan"] += 1
            continue
        scan = scans[index]
        age = stamp - timeline[index]
        if not scan["ok"] or scan["score"] < MIN_POSE_SCORE:
            counts["poor_tracking"] += 1
            continue
        if age > MAX_POSE_AGE_S + TIME_EPS_S:
            counts["stale_pose"] += 1
            continue
        point = scan["pose"][:2]
        distance = math.dist(node, point)
        crossing_count = count_wall_crossings(node, point, walls)
        selected.append(dict(meta_line=event["line"], meta_rx=event["rx"], meta_t=event["t"],
                             elapsed_rx_s=elapsed, rssi_dbm=event["rssi_dbm"],
                             pose_line=scan["line"], pose_t=scan["t"], pose_rx=scan["rx"],
                             pose=scan["pose"], pose_score=scan["score"], pose_age_s=age,
                             alignment_clock=clock, distance_m=distance, wall_count=crossing_count))
    bins = {}
    for index, sample in enumerate(selected):
        key = math.floor(math.log10(max(sample["distance_m"], 1.0)) / DISTANCE_BIN_LOG10)
        bins.setdefault(key, []).append(index)
    retained = []
    for indices in bins.values():
        if len(indices) > MAX_SAMPLES_PER_DISTANCE_BIN:
            offsets = np.linspace(0, len(indices) - 1, MAX_SAMPLES_PER_DISTANCE_BIN, dtype=int)
            retained.extend(indices[int(i)] for i in offsets)
        else:
            retained.extend(indices)
    counts["distance_bin_cap"] = len(selected) - len(retained)
    selected = [selected[i] for i in sorted(retained)]
    report = fit_samples([q["distance_m"] for q in selected],
                         [q["wall_count"] for q in selected], [q["rssi_dbm"] for q in selected])
    report["source"] = dict(recording=str(input_path.resolve()),
                            recording_sha256=hashlib.sha256(input_path.read_bytes()).hexdigest(),
                            node_xy=list(node), node_position_basis="caller assertion; fixed AP in replay map frame",
                            node_uncertainty_m=node_uncertainty_m,
                            window_clock="elapsed receive rx since first scan; inclusive endpoints",
                            first_scan_t=scans[0]["t"], first_scan_rx=first_rx, t0_s=t0, t1_s=t1,
                            counts=dict(counts), floorplan_wall_segments=len(walls),
                            elapsed_processing_s=time.perf_counter() - started)
    report["sampling"] = dict(method="first changed-value broadcast per run; initial run skipped",
                              min_refresh_spacing_s=RSSI_REFRESH_S, max_pose_age_s=MAX_POSE_AGE_S,
                              min_pose_score=MIN_POSE_SCORE,
                              distance_bin_log10_width=DISTANCE_BIN_LOG10,
                              max_samples_per_distance_bin=MAX_SAMPLES_PER_DISTANCE_BIN,
                              caveats="Repeated equal refresh values are undersampled; RSSI measurement time is not recorded. "
                                      "Receiver timestamps are a fallback when metadata t is absent; no interpolation or dead reckoning.")
    report["samples"] = selected
    return report


def write_report(output_path, report):
    """Atomically write a report; rejected fits never replace an existing file."""
    output_path = Path(output_path)
    if output_path.exists() and not report["calibrated"]:
        raise FileExistsError("rejected fit: existing output preserved")
    data = json.dumps(report, indent=2, allow_nan=False) + "\n"
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=output_path.parent,
                                         prefix=f".{output_path.name}.", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(data)
        os.replace(temporary, output_path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", help="gzip JSONL walk with metadata, scans containing rx, and spotter m.rssi")
    parser.add_argument("--node", required=True, metavar="X,Y", help="asserted fixed AP position in replay map metres")
    parser.add_argument("--t0", type=float, default=0.0, help="inclusive start: seconds since first scan rx (default 0)")
    parser.add_argument("--t1", type=float, help="inclusive end: seconds since first scan rx (default through end)")
    parser.add_argument("--node-uncertainty", type=float, metavar="METRES",
                        help="reported AP position uncertainty; not propagated into fit standard errors")
    parser.add_argument("--output", type=Path,
                        help="JSON model + diagnostics (default: relay_model.json beside recording)")
    args = parser.parse_args(argv)
    try:
        output = args.output or Path(args.recording).parent / "relay_model.json"
        if output.resolve() == Path(args.recording).resolve():
            raise ReplayError("output must not overwrite the input recording")
        report = fit_recording(args.recording, args.node, args.t0, args.t1,
                               node_uncertainty_m=args.node_uncertainty)
        fit = report["fit"]
        print(f"Samples: {fit['sample_count']}; wall-count range: {fit['wall_count_range']}; "
              f"distance range (m): {fit['distance_range_m']}")
        print(f"RMS residual (dB): {fit['rms_db']}; rank: {fit['rank']}/3; "
              f"condition: {fit['condition_number']}")
        print(f"Identifiable n: {fit['identifiable']['path_loss_exponent']}; "
              f"W: {fit['identifiable']['wall_loss_db']}; raw coefficients: {fit['coefficients']}")
        print("Calibration accepted" if report["calibrated"] else
              "Calibration rejected; defaults remain effective: " + "; ".join(fit["rejection_reasons"]))
        print("Sampling/association counts: " + json.dumps(report["source"]["counts"], sort_keys=True))
        try:
            write_report(output, report)
            print(f"Wrote {output}")
        except FileExistsError:
            if report["calibrated"]:
                raise
            print("Rejected fit: existing output preserved", file=sys.stderr)
        return 0 if report["calibrated"] else 1
    except (ReplayError, OSError, ValueError, KeyError, TypeError) as exc:
        print(f"Relay fit failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
