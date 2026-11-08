# Plan after Shivansh's FIRMWARE_SPEC.md (received 2026-11-07 15:11 EST)

Spec: data/team_email/Shivansh_FIRMWARE_SPEC.md. Team is effectively 2 in person (Shivansh: Pi spotter + laptop
dashboard; Xiaofan: all ESP32 firmware). Dani and Ziwen remote. Full chain by Sat 20:00 EST, freeze Sun 10:00,
submission Sun 12:00.

## Spec vs. our hardware
Spec assumes every node is an ESP32 LoRa board. We have 3x ESP32-S3 (no LoRa) + Feather RP2040 RFM95 + 1 RFM95W.
So: LoRa only for SP1 -> B2 (gateway); ESP-NOW for B2 -> B1 -> BASE. Same JSON contract on both links.

Proposed mapping (to confirm with Shivansh):
  SP1 = RP2040 RFM95 (Pi bridge) | B2 = ESP32 + RFM95W, LoRa RX + BLE phone scan (spec's B3 role)
  B1 = ESP32 relay (+ captive portal Sunday, spec's B2 role) | BASE = ESP32 at laptop (was "CP")

## Firmware changes to match the contract
- CP -> BASE; BASE prints own HB line every 15 s; debug lines on BASE start with '#'
- messages < 200 bytes; forward unknown fields unchanged (already)
- PHONE: n (unique addr, RSSI threshold), rssi (strongest), ap (Apple count), every 10 s
- SP1 TX power to minimum; RadioHead default = BW125/SF7/CR4-5, sync 0x12 (matches spec)

## Open question
Dashboard reads BASE over USB serial. BASE is on Xiaofan's laptop owned by the board hub; either move BASE to
the dashboard laptop at integration time, or give the dashboard a feed from the hub.
