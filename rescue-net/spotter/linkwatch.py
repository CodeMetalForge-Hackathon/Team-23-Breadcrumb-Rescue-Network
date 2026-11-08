"""Wi-Fi watchdog for the bulk link: re-joins B2's RESCUE-LINK AP when B2 stops answering.

When B2 reboots (power bank swap, brownout) its softAP comes back without deauthing the Pi, so NetworkManager
keeps a stale association (good signal, address still set) and every UDP packet is lost silently. Every
every_s seconds we ping B2; after `fails` misses in a row we bounce the connection (sudo nmcli --wait up_wait_s
con up), and while B2 stays silent we retry right after each failed attempt (cooldown_s after a success), so
the Pi is back within about up_wait_s + every_s of B2's AP returning. status: "ok", "no reply xN", "rejoined",
"rejoin failed: ...".
"""
import logging
import subprocess
import threading
import time

log = logging.getLogger("linkwatch")


def ping(host, timeout_s=1):
    return subprocess.run(["ping", "-c", "1", "-W", str(timeout_s), host], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def nmcli_up(con, wait_s=8):
    r = subprocess.run(["sudo", "-n", "nmcli", "--wait", str(int(wait_s)), "con", "up", con], capture_output=True,
                       text=True, timeout=wait_s + 4)
    return r.returncode == 0, (r.stdout + r.stderr).strip().splitlines()[-1:] or [""]


class LinkWatch:
    def __init__(self, host="192.168.4.1", con="RESCUE-LINK", every_s=2.0, fails=3, cooldown_s=4.0,
                 ping_fn=ping, up_fn=nmcli_up, clock=time.monotonic, sleep=time.sleep):
        self.host, self.con, self.every_s, self.fails, self.cooldown_s = host, con, every_s, fails, cooldown_s
        self.ping_fn, self.up_fn, self.clock, self.sleep = ping_fn, up_fn, clock, sleep
        self.misses, self.rejoins, self.status, self.hold_until = 0, 0, "starting", 0.0
        self._stop = False

    def start(self):
        threading.Thread(target=self._run, name="linkwatch", daemon=True).start()
        return self

    def stop(self):
        self._stop = True

    def step(self):
        """One check. Returns True if it bounced the connection."""
        if self.clock() < self.hold_until:
            return False
        if self.ping_fn(self.host):
            if self.misses >= self.fails or self.status != "ok":
                log.info("linkwatch: %s answers again", self.host)
            self.misses, self.status = 0, "ok"
            return False
        self.misses += 1
        self.status = f"no reply x{self.misses}"
        if self.misses < self.fails:
            return False
        log.warning("linkwatch: %s silent %d times, re-joining %s", self.host, self.misses, self.con)
        try:
            ok, msg = self.up_fn(self.con)
        except Exception as e:                       # nmcli missing, timeout, ...
            ok, msg = False, [str(e)]
        self.rejoins += 1
        self.status = "rejoined" if ok else f"rejoin failed: {msg[0][:80]}"
        (log.info if ok else log.warning)("linkwatch: %s", self.status)
        if ok:                                       # give DHCP / the first pings a moment
            self.misses, self.hold_until = 0, self.clock() + self.cooldown_s
        else:                                        # B2 still gone: try again on the next check
            self.misses = self.fails - 1
        return True

    def _run(self):
        while not self._stop:
            try:
                self.step()
            except Exception:
                log.exception("linkwatch")
            self.sleep(self.every_s)
