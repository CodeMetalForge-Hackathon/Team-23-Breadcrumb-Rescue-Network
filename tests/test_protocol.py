import json
import re

import pytest

from common.protocol import MAX_BYTES, encode_line, extract_message, make_msg, new_id, parse_line


def test_new_id_is_4_hex():
    for _ in range(100):
        assert re.fullmatch(r"[0-9a-f]{4}", new_id())


def test_make_msg_wire_order_and_drops_none():
    m = make_msg("PERSON", "SP1", 4, c=88, th=1, z="B3", v="C", dt=None)
    assert list(m) == ["id", "src", "ttl", "rt", "t", "c", "th", "z", "v"]
    assert m["rt"] == [] and m["ttl"] == 4 and m["src"] == "SP1"


def test_encode_line_is_compact_and_newline_terminated():
    line = encode_line({"id": "a3f1", "src": "SP1", "t": "HB"})
    assert line == b'{"id":"a3f1","src":"SP1","t":"HB"}\n'


def test_worst_case_spotter_message_after_3_hops_fits():
    m = make_msg("PERSON", "SP1", 4, c=100, th=1, z="B3", v="C", dt=-99)
    m["rt"] = ["B3", "B2", "B1"]
    assert len(encode_line(m)) < MAX_BYTES


def test_worst_case_sos_fits():
    m = make_msg("SOS", "B2", 4, z="B2", msg="x" * 100)
    m["rt"] = ["B1"]
    assert len(encode_line(m)) <= MAX_BYTES


def test_oversized_message_is_refused():
    with pytest.raises(ValueError):
        encode_line(make_msg("SOS", "B2", msg="x" * 300))


@pytest.mark.parametrize("bad", [None, "", "   \r\n", "garbage", "[1,2,3]", '"str"', "42", "{bad json",
                                 "RX rssi=-80 {\"id\":1}", b"\xff\xfe\xfd", b"{\"id\":\"a\xff"])
def test_bad_lines_parse_to_none(bad):
    assert parse_line(bad) is None


def test_parse_line_roundtrip_bytes_and_str():
    m = make_msg("HEAT", "SP1", 4, c=40, z="B3", v="R", dt=7)
    line = encode_line(m)
    assert parse_line(line) == m
    assert parse_line(line.decode() + "\r") == m
    assert parse_line(json.dumps(m)) == m


def test_parse_keeps_unknown_fields():
    assert parse_line('{"t":"PHONE","n":3,"new_field":true}')["new_field"] is True


# ---- firmware output seen on the Feather RP2040 RFM ----

def test_bare_message_is_a_message():
    m = make_msg("HB", "SP1", 4, z="B3")
    assert extract_message(m) == m


def test_lora_rx_wrapper_is_unwrapped_with_rssi():
    line = '{"lora_rx":{"id":"bb6d","src":"SP1","ttl":2,"rt":["B3","B2"],"t":"HB","z":"B3"},"rssi":-87}'
    msg = extract_message(parse_line(line))
    assert msg["id"] == "bb6d" and msg["rt"] == ["B3", "B2"] and msg["_rssi"] == -87


def test_board_status_and_foreign_packets_are_not_messages():
    assert extract_message(parse_line('{"sent":"bb6d","len":59}')) is None
    assert extract_message(parse_line('{"lora_rx":SN1,BRAVO,31898,2,BRAVO,20.95,42.95,S,"rssi":-104}')) is None
    assert extract_message(parse_line('{"lora_rx":"some other team","rssi":-90}')) is None
    assert extract_message(None) is None
