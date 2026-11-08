"""Spoken prompts from B2: relay placement, link lost, LoRa-only mode.

B2 sends one JSON object per UDP datagram to port 5006:
  {"t":"SAY","k":"place1","m":"Place relay B1 here, then press D2"}   k: place1 place2 lost1 lost2 lora bulk
  {"t":"MODE","lo":1}                                                  LoRa-only mode on (1) / off (0)
The Pi has no TTS engine and no internet, so each k is a wav rendered ahead of time (tools: espeak-ng on a
laptop, see spotter/sounds/README). Playback runs on its own thread; the camera loop never waits on it.
B2 repeats a prompt every 15 s until D2, so one k plays at most once per rate_s.
Output: `aplay` on the first ALSA card that is not HDMI; with none, a beep pattern on beep_gpio (if set);
with neither, the prompt is only logged.
"""
import json
import logging
import os
import queue
import socket
import subprocess
import threading
import time

log = logging.getLogger("voice")

SOUNDS = os.path.join(os.path.dirname(__file__), "sounds")
# beep patterns when there is no audio device: (on_s, off_s) per beep
BEEPS = {"place1": [(0.6, 0.2)], "place2": [(0.6, 0.2), (0.6, 0.2)],
         "lost1": [(0.12, 0.08)] * 3, "lost2": [(0.12, 0.08)] * 4,
         "lora": [(0.3, 0.1), (0.1, 0.1)], "bulk": [(0.1, 0.1), (0.3, 0.1)]}


def audio_device(cards_path="/proc/asound/cards"):
    """ALSA 'plughw:N' of the first non-HDMI card (USB speaker, PWM audio), else None."""
    try:
        lines = open(cards_path).read().splitlines()
    except OSError:
        return None
    for line in lines:
        parts = line.split()
        if len(parts) > 1 and parts[0].isdigit() and parts[1].startswith("[") and "hdmi" not in line.lower():
            return f"plughw:{parts[0]}"
    return None


def aplay(path, device):
    subprocess.run(["aplay", "-q", "-D", device, path], timeout=15,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class GpioBeeper:
    """Square-wave tone on one GPIO (passive speaker or buzzer); idle low so a floating pin does not hum."""

    def __init__(self, pin, hz=2000):
        from gpiozero import PWMOutputDevice
        self.dev = PWMOutputDevice(pin, frequency=hz, initial_value=0)

    def play(self, pattern):
        for on, off in pattern:
            self.dev.value = 0.5
            time.sleep(on)
            self.dev.value = 0
            time.sleep(off)


class Voice:
    def __init__(self, port=5006, rate_s=8.0, sounds=SOUNDS, device="auto", beep_gpio=None,
                 play_fn=aplay, beeper=None, clock=time.monotonic):
        self.port, self.rate_s, self.sounds = port, rate_s, sounds
        self.device = audio_device() if device == "auto" else device
        self.play_fn, self.clock = play_fn, clock
        self.beeper = beeper
        if self.beeper is None and self.device is None and beep_gpio is not None:
            try:
                self.beeper = GpioBeeper(int(beep_gpio))
            except Exception as e:                    # gpiozero missing, pin busy
                log.warning("voice: no beeper on GPIO%s: %s", beep_gpio, e)
        self.last = {}                                # k -> time last played
        self.lora_only = False
        self.said = []                                # (k, m) played, newest last (tests, debug)
        self.q = queue.Queue(maxsize=4)
        self.sock = None

    @property
    def out(self):
        return self.device or ("beep" if self.beeper else "log only")

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port))
        threading.Thread(target=self._rx, name="voice-rx", daemon=True).start()
        threading.Thread(target=self._play, name="voice-play", daemon=True).start()
        log.info("voice: listening on udp %d, output %s", self.port, self.out)
        return self

    def handle(self, data):
        """One datagram. Returns the k queued for playback, or None."""
        try:
            msg = json.loads(data.decode("utf-8", "replace") if isinstance(data, bytes) else data)
        except ValueError:
            return None
        if not isinstance(msg, dict):
            return None
        if msg.get("t") == "MODE":
            lo = bool(msg.get("lo"))
            if lo != self.lora_only:
                log.info("voice: LoRa-only mode %s", "on" if lo else "off")
            self.lora_only = lo
            return None
        if msg.get("t") != "SAY":
            return None
        k, m = str(msg.get("k") or ""), str(msg.get("m") or "")
        now = self.clock()
        if not k or now - self.last.get(k, -1e9) < self.rate_s:
            return None
        self.last[k] = now
        log.info("voice: SAY %s: %s", k, m)
        try:
            self.q.put_nowait((k, m))
        except queue.Full:                            # speaker busy for a while; newest prompt wins next
            return None
        return k

    def say(self, k, m=""):
        """Play one prompt now (blocking): wav for k, else beep pattern, else nothing."""
        wav = os.path.join(self.sounds, f"{k}.wav")
        if self.device and os.path.exists(wav):
            self.play_fn(wav, self.device)
        elif self.beeper:
            self.beeper.play(BEEPS.get(k, [(0.2, 0.1)]))
        self.said.append((k, m))

    def _rx(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                self.handle(data)
            except Exception:
                log.exception("voice rx")

    def _play(self):
        while True:
            k, m = self.q.get()
            try:
                self.say(k, m)
            except Exception:
                log.exception("voice play")
