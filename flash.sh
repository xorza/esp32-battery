#!/bin/bash
set -euo pipefail

DIR="$(dirname "$0")"
CHIP="${MCU:?Set MCU=esp32c3 or MCU=esp32c6}"

case "$CHIP" in
    esp32c3) PARTITIONS="$DIR/partitions-4mb.csv" ; TARGET="riscv32imc-esp-espidf" ; ALIAS="c3" ;;
    esp32c6) PARTITIONS="$DIR/partitions-8mb.csv" ; TARGET="riscv32imac-esp-espidf" ; ALIAS="c6" ;;
    *)       echo "Unknown MCU: $CHIP"; exit 1 ;;
esac

ELF="$DIR/target/$TARGET/release/esp32-battery"

cargo "$ALIAS"

espflash flash \
    --erase-data-parts ota \
    --monitor \
    --partition-table "$PARTITIONS" \
    --port /dev/ttyACM0 \
    "$ELF"
