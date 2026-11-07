"""Find and open the LoRa board's USB serial port on Linux (Pi), macOS or Windows.
Boards seen: ESP32 (USB-UART bridge) and Adafruit Feather RP2040 RFM (native USB)."""
import logging
import time

import serial
from serial.tools import list_ports

log = logging.getLogger("serial")

# ESP32 USB-UART bridges (CP210x, CH340/CH9102, FTDI) and Espressif native USB: DTR/RTS can reset the board.
BRIDGE_VIDS = {0x10C4, 0x1A86, 0x303A, 0x0403}
# Native-USB boards (Adafruit, Raspberry Pi RP2040): DTR does not reset them, and their firmware
# only talks while DTR is on. (Never open these at 1200 baud: that reboots them into the bootloader.)
NATIVE_VIDS = {0x239A, 0x2E8A}
BOARD_VIDS = BRIDGE_VIDS | NATIVE_VIDS
NAME_HINTS = ("ttyUSB", "ttyACM", "usbserial", "usbmodem", "SLAB_USBtoUART", "wchusbserial")


def find_port():
    """Best guess at the LoRa board's port, or None. Ignores Bluetooth/debug ports (no USB VID)."""
    ports = [p for p in list_ports.comports() if p.vid is not None or any(h in p.device for h in NAME_HINTS)]
    if not ports:
        return None
    ports.sort(key=lambda p: (p.vid not in BOARD_VIDS, not any(h in p.device for h in NAME_HINTS), p.device))
    if len(ports) > 1:
        log.info("several serial ports found, using %s (others: %s)",
                 ports[0].device, ", ".join(p.device for p in ports[1:]))
    return ports[0].device


def _vid(device):
    alt = device.replace("/dev/tty.", "/dev/cu.")     # macOS lists cu.*, users often type tty.*
    return next((p.vid for p in list_ports.comports() if p.device in (device, alt)), None)


def open_port(device, baud=115200, timeout=1.0):
    """Open without resetting the board (DTR off for ESP32 bridges, on for RP2040), then let it settle."""
    s = serial.Serial()
    s.port = device
    s.baudrate = baud
    s.timeout = timeout
    s.dtr = _vid(device) in NATIVE_VIDS
    s.rts = False
    s.open()
    time.sleep(2.0)            # in case the board rebooted anyway
    s.reset_input_buffer()     # drop boot noise
    return s
