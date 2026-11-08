import base64
import json
import struct

import numpy as np

from dashboard.remote import Remote
from spotter import bulklink


class FakeSock:
    def __init__(self):
        self.pkts = []

    def sendto(self, data, addr):
        self.pkts.append(data)


def lines(pkts):
    for p in pkts:
        _, _, _, kind, seq, i, n = struct.unpack("<BBBBHHH", p[:10])
        yield f"$B {kind} {seq} {i} {n} {base64.b64encode(p[10:]).decode()}"


def test_lidar_v2_and_meta_roundtrip():
    got = []
    r = Remote(on_object=lambda name, blob, parsed: got.append((name, parsed)))
    bl = bulklink.BulkLink(None, None, None, gap_s=0, meta_fn=lambda: {"persons": [{"id": 1, "b": 12, "r": 2.5}]})
    s = FakeSock()
    scan = (np.array([0.2, 90.0, 180.4]), np.array([1000.0, 2000.0, 3000.0]))
    bl._send(s, bulklink.KIND_LIDAR, bulklink.lidar_blob(scan, 123.5, (1.4, -2.0, 21.2)))
    bl._send(s, bulklink.KIND_META, bl._meta())
    for ln in lines(s.pkts):
        r.add(ln)
    (n1, lid), (n2, meta) = got
    assert n1 == "lidar" and lid[0][0] == 1000 and lid[0][90] == 2000 and lid[0][180] == 3000
    assert lid[1] == 123.5 and lid[2] == (1, -2, 21)
    assert n2 == "meta" and meta["persons"][0]["r"] == 2.5 and meta["lvl"] == 0
    snap = r.snapshot()
    assert snap["lidar"]["t"] == 123.5 and snap["meta"]["persons"][0]["id"] == 1
    json.dumps(snap)


def test_old_lidar_blob_still_parses():
    r = Remote()
    bl = bulklink.BulkLink(None, None, None, gap_s=0)
    s = FakeSock()
    bl._send(s, bulklink.KIND_LIDAR, bulklink.lidar_bins(None))
    for ln in lines(s.pkts):
        r.add(ln)
    snap = r.snapshot()
    assert snap["lidar"]["t"] is None and len(snap["lidar"]["mm"]) == 360


def test_level_hysteresis():
    assert bulklink.pick_level(0, -60) == 0
    assert bulklink.pick_level(0, -70) == 1
    assert bulklink.pick_level(1, -66) == 1          # not clearly better yet
    assert bulklink.pick_level(1, -63) == 0
    assert bulklink.pick_level(1, -80) == 2
    assert bulklink.pick_level(0, None) == 2
