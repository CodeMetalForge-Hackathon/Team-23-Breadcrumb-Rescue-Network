import gzip
import json
import struct
import zlib

import numpy as np
import pytest

from mapping import replay


class FakeSlam:
    def __init__(self):
        self.calls = []
        self.pose = (0.0, 0.0, 0.0)
        self.path = []

    def update(self, angles, distances, timestamp, acc=None):
        self.calls.append((angles.copy(), distances.copy(), timestamp, acc))
        ok = len(self.calls) != 2
        if ok:
            self.pose = (0.0, len(self.calls) * 0.25, 0.0)
            self.path.append((timestamp, *self.pose))
        return dict(zip(("x", "y", "th"), self.pose), ok=ok, score=0.8 if ok else 0.0,
                    reason="tilt" if not ok else "accepted")

    def grid(self):
        grid = np.zeros((24, 32), dtype=np.uint8)
        grid[0, 0], grid[-1, 0] = 255, 127
        return grid, (-4.0, -3.0), 0.25


def recording(path, rows):
    with gzip.open(path, "wt", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row) + "\n")
    return path


def decode_png(path):
    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    offset, compressed, size = 8, bytearray(), None
    while offset < len(data):
        length = struct.unpack_from(">I", data, offset)[0]
        kind = data[offset + 4:offset + 8]
        body = data[offset + 8:offset + 8 + length]
        crc = struct.unpack_from(">I", data, offset + 8 + length)[0]
        assert crc == zlib.crc32(kind + body) & 0xffffffff
        if kind == b"IHDR":
            width, height, depth, color, compression, filtering, interlace = struct.unpack(">IIBBBBB", body)
            assert (depth, color, compression, filtering, interlace) == (8, 2, 0, 0, 0)
            size = height, width
        elif kind == b"IDAT":
            compressed.extend(body)
        elif kind == b"IEND":
            assert not body
            assert offset + 12 == len(data)
        offset += length + 12
    height, width = size
    scanlines = np.frombuffer(zlib.decompress(compressed), dtype=np.uint8).reshape(height, 1 + width * 3)
    assert not scanlines[:, 0].any()
    return scanlines[:, 1:].reshape(height, width, 3)


def test_streaming_replay_units_acceleration_counts_and_valid_png(tmp_path):
    source = recording(tmp_path / "walk.jsonl.gz", [
        {"meta": {"rig": "synthetic"}},
        {"t": 10.0, "ang": [0, 9000, 35999], "mm": [500, 2000, 0], "a": [1, 2, 21]},
        {"t": 10.2, "ang": [100, 200], "mm": [300, 400], "a": [10, 2, 10]},
        {"t": 10.4, "ang": [0, 100], "mm": [500, 600], "a": None},
    ])
    output, slam = tmp_path / "map.png", FakeSlam()
    result = replay.replay_file(source, output, slam=slam)
    assert (result.scans, result.accepted, result.rejected, result.points) == (3, 2, 1, 7)
    assert result.rejection_reasons == {"tilt": 1}
    assert result.recorded_s == pytest.approx(0.4)
    np.testing.assert_allclose(slam.calls[0][0], [0, 90, 359.99])
    np.testing.assert_array_equal(slam.calls[0][1], [500, 2000, 0])
    assert slam.calls[0][3] == [1.0, 2.0, 21.0]
    assert slam.calls[-1][3] is None
    image = decode_png(output)
    assert image.shape == (24, 32, 3)
    assert tuple(image[0, 0]) == (150, 150, 150)  # highest map y is top
    assert tuple(image[-1, 0]) == (25, 25, 25)
    assert np.any(np.all(image == (240, 100, 35), axis=2))
    assert np.any(np.all(image == (25, 180, 80), axis=2))


def test_mixed_walk2_events_do_not_change_scan_order_pose_or_pixels(tmp_path):
    scans = [
        {"t": 10.0, "rx": 999.0, "ang": [0, 9000], "mm": [500, 2000], "a": [1, 2, 21]},
        {"t": 10.2, "rx": 998.0, "ang": [100, 200], "mm": [300, 400], "a": None},
        {"t": 10.4, "rx": 1001.0, "ang": [0, 100], "mm": [500, 600], "a": None},
    ]
    header = {"meta": {"source": "synthetic tap_walk"}}
    mixed = [header,
             {"rx": 0.0, "m": {"lvl": 0, "persons": []}},
             scans[0],
             {"rx": 500.0, "msg": {"t": "MARK", "id": "relay-1", "x": 9000, "y": 9000}},
             {"rx": 500.1, "msg": {"t": "PERSON", "b": 90, "r": 3}},
             scans[1],
             {"rx": 20.0, "m": {"rssi": -75}},
             scans[2],
             {"rx": 99999.0, "msg": {"t": "CUSTOM", "extra": "opaque payload"}}]
    plain_slam, mixed_slam = FakeSlam(), FakeSlam()
    plain_png, mixed_png = tmp_path / "plain.png", tmp_path / "mixed.png"
    plain_stats = replay.replay_file(recording(tmp_path / "plain.gz", [header, *scans]),
                                     plain_png, slam=plain_slam)
    stats = replay.replay_file(recording(tmp_path / "mixed.gz", mixed), mixed_png, slam=mixed_slam)
    assert (stats.meta_events, stats.message_events, stats.skipped_events) == (2, 3, 5)
    assert plain_stats.skipped_events == 0
    assert (stats.scans, stats.accepted, stats.rejected) == (3, 2, 1)
    assert stats.recorded_s == pytest.approx(0.4)
    assert [call[2] for call in mixed_slam.calls] == [10.0, 10.2, 10.4]
    assert mixed_slam.pose == plain_slam.pose
    assert mixed_slam.path == plain_slam.path
    assert mixed_png.read_bytes() == plain_png.read_bytes()
    for actual, expected in zip(mixed_slam.calls, plain_slam.calls):
        np.testing.assert_array_equal(actual[0], expected[0])
        np.testing.assert_array_equal(actual[1], expected[1])
        assert actual[2:] == expected[2:]


@pytest.mark.parametrize("row,match", [
    ({"rx": 1, "event": {}}, "unknown recording record"),
    ({"rx": 1}, "unknown recording record"),
    ({"meta": {}}, "unknown recording record"),
    ([], "JSON object"),
    ({"m": {}}, "exactly rx"),
    ({"rx": 1, "m": {}, "msg": {}}, "exactly rx"),
    ({"rx": 1, "m": {}, "extra": 2}, "exactly rx"),
    ({"rx": 1, "m": {}, "t": 1, "ang": [0], "mm": [10]}, "exactly rx"),
    ({"rx": 1, "msg": [], }, "payload must be a JSON object"),
    ({"rx": 1, "m": None}, "payload must be a JSON object"),
    ({"rx": True, "msg": {}}, "rx must be a finite number"),
    ({"rx": float("inf"), "m": {}}, "rx must be a finite number"),
    ({"t": 1, "rx": None, "ang": [0], "mm": [10]}, "rx must be a finite number"),
    ({"t": 1, "ang": [0], "mm": [10], "event": {}}, "unknown scan field"),
])
def test_unknown_or_malformed_event_envelopes_are_not_silently_skipped(tmp_path, row, match):
    source = recording(tmp_path / "bad.gz", [{"meta": {}}, row])
    slam = FakeSlam()
    output = tmp_path / "map.png"
    with pytest.raises(replay.ReplayError, match=f"line 2.*{match}"):
        replay.replay_file(source, output, slam=slam)
    assert slam.calls == []
    assert not output.exists()


def test_metadata_and_auxiliary_events_without_scans_remain_an_error(tmp_path):
    source = recording(tmp_path / "events.gz", [
        {"meta": {}}, {"rx": 1, "m": {}}, {"rx": 2, "msg": {"t": "MARK"}},
    ])
    slam = FakeSlam()
    with pytest.raises(replay.ReplayError, match="no scans"):
        replay.replay_file(source, tmp_path / "out.png", slam=slam)
    assert slam.calls == []


@pytest.mark.parametrize("row,match", [
    ({"t": 0, "ang": [0], "mm": []}, "same nonzero length"),
    ({"t": 0, "ang": [0.5], "mm": [10]}, "integers"),
    ({"t": 0, "ang": [False], "mm": [10]}, "integers"),
    ({"t": 0, "ang": [0], "mm": [-10]}, "negative"),
    ({"t": float("nan"), "ang": [0], "mm": [10]}, "finite"),
    ({"t": 0, "ang": [0], "mm": [10], "a": [1, 2]}, "null or"),
])
def test_bad_scan_is_line_specific_and_does_not_write_png(tmp_path, row, match):
    source = recording(tmp_path / "bad.gz", [{"meta": {}}, row])
    output = tmp_path / "map.png"
    with pytest.raises(replay.ReplayError, match=f"line 2.*{match}"):
        replay.replay_file(source, output, slam=FakeSlam())
    assert not output.exists()


def test_bad_metadata_empty_scans_and_truncated_gzip(tmp_path):
    source, output = tmp_path / "bad.gz", tmp_path / "map.png"
    recording(source, [{"t": 0}])
    with pytest.raises(replay.ReplayError, match="line 1.*metadata"):
        replay.replay_file(source, output, slam=FakeSlam())
    recording(source, [{"meta": {}}])
    with pytest.raises(replay.ReplayError, match="no scans"):
        replay.replay_file(source, output, slam=FakeSlam())
    source.write_bytes(b"not a gzip file")
    with pytest.raises(replay.ReplayError, match="gzip JSONL"):
        replay.replay_file(source, output, slam=FakeSlam())


def test_heading_convention_points_right_at_pi_over_two():
    grid = np.zeros((30, 30), dtype=np.uint8)
    image = replay.render_map(grid, (0, 0), 0.1, [], (1.05, 1.05, np.pi / 2))
    green_y, green_x = np.nonzero(np.all(image == (25, 180, 80), axis=2))
    assert green_x.min() > 10
    assert np.all(np.abs(green_y - 19) <= 1)


def test_do_not_overwrite_input_recording(tmp_path):
    source = recording(tmp_path / "walk.gz", [{"meta": {}}])
    original = source.read_bytes()
    with pytest.raises(replay.ReplayError, match="different files"):
        replay.replay_file(source, source, slam=FakeSlam())
    assert source.read_bytes() == original


def test_invalid_recording_preserves_an_existing_output(tmp_path):
    source = recording(tmp_path / "walk.gz", [{"meta": {}}, {"t": 0}])
    output = tmp_path / "existing.png"
    output.write_bytes(b"existing image must survive")
    with pytest.raises(replay.ReplayError, match="line 2"):
        replay.replay_file(source, output, slam=FakeSlam())
    assert output.read_bytes() == b"existing image must survive"


def test_cli_reports_acceptance_and_pose(tmp_path, monkeypatch, capsys):
    stats = replay.ReplayStats(3, 2, 700, 0.5, 1.0, (1.0, 2.0, 0.0), {"tilt": 1}, tmp_path / "out.png")
    assert stats.skipped_events == 0  # Existing positional construction is compatible.
    stats.meta_events, stats.message_events = 2, 3
    monkeypatch.setattr(replay, "replay_file", lambda *args: stats)
    assert replay.main(["walk.gz", str(stats.output)]) == 0
    output = capsys.readouterr().out
    assert "accepted: 2; rejected: 1" in output
    assert "x=1.000 m, y=2.000 m" in output
    assert "Skipped auxiliary records: 5 (spotter meta: 2; node messages: 3)" in output
    stats.accepted = 0
    assert replay.main(["walk.gz", str(stats.output)]) == 1
    assert "No scans were accepted" in capsys.readouterr().err
