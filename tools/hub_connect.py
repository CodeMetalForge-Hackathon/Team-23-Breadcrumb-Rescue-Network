#!/usr/bin/env python3
"""ssh ProxyCommand through the board hub's CONNECT tunnel to a device on the laptop's LAN (the Pi).
   ssh -o ProxyCommand='python3 tools/hub_connect.py %h %p' pi@192.168.137.x
Hub URL from $BOARD_HUB (default http://100.115.103.120:8765), token from ~/.config/board_hub_token."""
import os, socket, sys, threading
from urllib.parse import urlparse

host, port = sys.argv[1], sys.argv[2]
hub = urlparse(os.environ.get("BOARD_HUB", "http://100.115.103.120:8765"))
token = open(os.path.expanduser("~/.config/board_hub_token")).read().strip()
s = socket.create_connection((hub.hostname, hub.port or 80), timeout=15)
s.sendall(f"CONNECT {host}:{port} HTTP/1.1\r\nHost: {host}:{port}\r\nX-Token: {token}\r\n\r\n".encode())
buf = b""
while b"\r\n\r\n" not in buf:
    d = s.recv(4096)
    if not d:
        sys.exit("hub closed the connection")
    buf += d
head, rest = buf.split(b"\r\n\r\n", 1)
status = head.split(b"\r\n")[0].decode(errors="replace")
if " 200 " not in status + " ":
    sys.exit(f"hub refused: {status}")
s.settimeout(None)
out = sys.stdout.buffer
if rest:
    out.write(rest); out.flush()

def down():
    while True:
        d = s.recv(65536)
        if not d:
            break
        out.write(d); out.flush()
    os._exit(0)

threading.Thread(target=down, daemon=True).start()
inp = sys.stdin.buffer
while True:
    d = os.read(inp.fileno(), 65536)
    if not d:
        break
    s.sendall(d)
s.shutdown(socket.SHUT_WR)
threading.Event().wait(5)
