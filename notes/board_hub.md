# Board Hub — shared access to the laptop's USB boards (for every agent on jump)

The boards are plugged into the user's laptop (Tailscale 100.115.103.120). `tools/board_hub.py` runs there,
owns all serial ports, and serves HTTP on :8765 (tailnet only + token in ~/.config/board_hub_token on jump).
On jump, any agent (Claude, Codex, Kiro) uses the `hub` command (~/.local/bin/hub -> tools/hub):

    hub boards              # which boards are plugged in, COM port, last serial line
    hub log CP 50           # last 50 lines from the command post (every message that reached CP)
    hub tail CP             # follow live
    hub send SP1 test       # write a line to a board's serial (RP2040 LoRa bridge: JSON line or "test")
    hub deploy B3           # compile firmware/breadcrumb_espnow for B3 on jump and flash it
    hub deploy SP1          # compile + flash the RP2040 LoRa bridge
    hub reset B2

Rules for agents
- One flash at a time (the hub serializes); tell the others in tmux before reflashing a node.
- Never flash SP1 / B3 LoRa transmit tests unless the antenna is soldered (user confirms).
- Nodes (chain SP1 -LoRa-> B2 -> B1 -> CP): CP (COM4, MAC 98:c3:77:62:67:24), B1 (COM9, 98:c3:77:63:3A:48), B2 = gateway with LoRa+BLE (COM11, 98:c3:77:62:62:8c),
  SP1 = Feather RP2040 RFM95 (not yet plugged in). Firmware sources: firmware/*/.
- Physical things (buttons, screens, wiring, soldering) still need the user: ask for a photo.
