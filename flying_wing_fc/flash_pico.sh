#!/bin/bash
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
ARDUINO_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
PICOTOOL="$HOME/Library/Arduino15/packages/rp2040/tools/pqt-picotool/5.0.0-9576866/picotool"
BUILD_DIR="/tmp/rp2040_build"

echo "============================================================"
echo "  RP2040-Zero (Waveshare) - COMPILE & FLASH"
echo "============================================================"

echo "[1/2] Compiling sketch for RP2040-Zero..."
"$ARDUINO_CLI" compile \
  --fqbn rp2040:rp2040:waveshare_rp2040_zero \
  --output-dir "$BUILD_DIR" \
  "$DIR"

echo ""
echo "[2/2] Flashing via picotool..."
if ! "$PICOTOOL" info >/dev/null 2>&1; then
  echo "      Memicu reboot ke BOOTSEL mode via 1200 baud..."
  killall serial-monitor 2>/dev/null || true
  sleep 0.5
  SERIAL_PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true)
  if [ -n "$SERIAL_PORT" ]; then
    python3 -c "import serial, time; s = serial.Serial('$SERIAL_PORT', 1200); time.sleep(0.1); s.close()" 2>/dev/null || true
    sleep 1.5
  fi
fi

"$PICOTOOL" load -v -x "$BUILD_DIR/flying_wing_fc.ino.elf"

echo ""
echo "============================================================"
echo "  SUKSES! Firmware RP2040-Zero ter-flash & running!"
echo "============================================================"
