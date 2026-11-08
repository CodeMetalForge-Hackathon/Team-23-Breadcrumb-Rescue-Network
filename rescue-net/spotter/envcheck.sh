#!/usr/bin/env bash
# M0 hardware sanity check for the spotter Pi. Run: bash spotter/envcheck.sh 2>&1 | tee envcheck.txt
# Never stops on a failure: every section prints, so the whole output can be pasted back in one go.

section() { printf '\n===== %s =====\n' "$1"; }
run() { echo "\$ $*"; "$@" 2>&1 || echo "(exit $?)"; }

section "system"
run uname -a
run grep PRETTY_NAME /etc/os-release
run python3 --version
echo "user: $(whoami)  groups: $(id -nG)"

section "Hailo"
run bash -c "lspci | grep -i hailo || echo 'Hailo not on PCIe: check the HAT ribbon cable'"
run bash -c "ls -l /dev/hailo* 2>/dev/null || echo 'no /dev/hailo0: driver/firmware not loaded (install hailo-all, reboot)'"
run hailortcli fw-control identify
run ls -la /usr/share/hailo-models/
run bash -c "dpkg -l | grep -i -E 'hailo|picamera2|rpicam' | awk '{print \$2, \$3}'"
echo "\$ find picamera2 Hailo examples"
find / -path '*hailo*' -name 'detect*.py' 2>/dev/null | head -10
find / -path '*picamera2*' -name '*hailo*' 2>/dev/null | grep -v __pycache__ | head -10

section "camera"
run rpicam-hello --list-cameras

section "I2C / thermal (MLX90640 expected at 0x33)"
run bash -c "ls /dev/i2c-*"
run /usr/sbin/i2cdetect -y 1
run grep -i -E 'i2c_arm|dtparam=i2c' /boot/firmware/config.txt

section "ESP32 serial port"
run bash -c "ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo 'no /dev/ttyUSB* or /dev/ttyACM*'"
run bash -c "ls -l /dev/serial/by-id/ 2>/dev/null || echo 'no /dev/serial/by-id'"
run bash -c "lsusb"

section "python imports (system python3)"
run python3 -c "import picamera2; print('picamera2 ok')"
run python3 -c "from picamera2.devices import Hailo; print('picamera2 Hailo helper ok')"
run python3 -c "import hailo_platform; print('hailo_platform ok')"
run python3 -c "import cv2; print('cv2', cv2.__version__)"
run python3 -c "import numpy; print('numpy', numpy.__version__)"

if [ -x .venv/bin/python ]; then
  section "python imports (.venv)"
  run .venv/bin/python -c "import picamera2, serial, numpy, cv2; from picamera2.devices import Hailo; print('venv core ok')"
  run .venv/bin/python -c "import board, adafruit_mlx90640; print('mlx90640 lib ok')"
fi

section "network (for the debug view on :8080)"
run hostname -I
