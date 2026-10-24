#!/bin/bash
set -e

DIR="$(cd "$(dirname "$0")" && pwd)"
ARDUINO_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
DFU_UTIL="$HOME/Library/Arduino15/packages/STMicroelectronics/tools/STM32Tools/2.5.0/macosx/dfu-util"
BUILD_DIR="/tmp/fc_build"

echo "============================================================"
echo "  OMNIBUS F4 V5 (STM32F405) - COMPILE & AUTO-FLASH via DFU"
echo "============================================================"

echo "[1/3] Compiling firmware..."
"$ARDUINO_CLI" compile \
  --fqbn STMicroelectronics:stm32:GenF4:pnum=GENERIC_F405RGTX,usb=CDCgen,upload_method=dfuMethod,xserial=generic \
  --output-dir "$BUILD_DIR" \
  "$DIR"

echo ""
echo "[2/3] Checking DFU / Auto-Reboot to DFU Bootloader..."
if ! "$DFU_UTIL" -l | grep -q "0483:df11"; then
  SERIAL_PORT=$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true)
  if [ -n "$SERIAL_PORT" ]; then
    echo "      Melepas sementara kunci Serial Monitor Arduino IDE..."
    killall serial-monitor 2>/dev/null || true
    sleep 0.5
    echo "      Mengirim perintah Auto-DFU ('b') ke $SERIAL_PORT (Tanpa tekan tombol BOOT)..."
    stty -f "$SERIAL_PORT" 115200 raw -echo || true
    printf "b\n" > "$SERIAL_PORT" || true
    sleep 2.5
  fi
fi

echo ""
echo "[3/3] Flashing to internal flash @ 0x08000000..."
"$DFU_UTIL" -a 0 -d 0483:df11 -s 0x08000000:leave -D "$BUILD_DIR/flying_wing_fc.ino.bin"

echo ""
echo "============================================================"
echo "  SUKSES! Firmware berhasil di-flash & otomatis Reboot!"
echo "  Buka Serial Monitor di Arduino IDE (115200 baud)."
echo "============================================================"
