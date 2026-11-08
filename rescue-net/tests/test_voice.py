import json
import socket
import time

from spotter.voice import Voice, audio_device


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


def test_say_rate_limited_per_key_and_mode():
    c, played = Clock(), []
    v = Voice(device="plughw:9", play_fn=lambda p, d: played.append(p), clock=c)
    say = lambda k: json.dumps({"t": "SAY", "k": k, "m": "x"}).encode()
    assert v.handle(say("place1")) == "place1"
    c.t = 5
    assert v.handle(say("place1")) is None          # B2 repeats; once per 8 s
    assert v.handle(say("lost1")) == "lost1"        # other keys are independent
    c.t = 9
    assert v.handle(say("place1")) == "place1"
    assert v.handle(b"not json") is None and v.handle(b"[1]") is None
    v.handle(b'{"t":"MODE","lo":1}')
    assert v.lora_only
    v.handle(b'{"t":"MODE","lo":0}')
    assert not v.lora_only
    v.say("place1")
    assert played and played[0].endswith("place1.wav")


def test_beep_fallback_and_log_only():
    class B:
        n = 0

        def play(self, pattern):
            B.n += len(pattern)
    v = Voice(device=None, beeper=B())
    v.say("place2")
    assert B.n == 2 and v.out == "beep"
    assert Voice(device=None).out == "log only"


def test_audio_device_skips_hdmi(tmp_path):
    p = tmp_path / "cards"
    p.write_text(" 0 [vc4hdmi0       ]: vc4-hdmi - vc4-hdmi-0\n                      vc4-hdmi-0\n")
    assert audio_device(str(p)) is None
    p.write_text(p.read_text() + " 2 [Device         ]: USB-Audio - USB Audio Device\n                      x\n")
    assert audio_device(str(p)) == "plughw:2"


def test_udp_listener():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    v = Voice(port=port, device=None).start()
    socket.socket(socket.AF_INET, socket.SOCK_DGRAM).sendto(b'{"t":"SAY","k":"place1","m":"test"}', ("127.0.0.1", port))
    for _ in range(50):
        if v.said:
            break
        time.sleep(0.02)
    assert v.said == [("place1", "test")]
