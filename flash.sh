#!/bin/bash
set -euo pipefail

DIR="$(dirname "$0")"
CHIP="${MCU:-esp32c6}"

case "$CHIP" in
    esp32c3) PARTITIONS="$DIR/partitions-4mb.csv" ; TARGET="riscv32imc-esp-espidf" ; ALIAS="c3" ;;
    esp32c6) PARTITIONS="$DIR/partitions-8mb.csv" ; TARGET="riscv32imac-esp-espidf" ; ALIAS="c6" ;;
    *)       echo "Unknown MCU: $CHIP"; exit 1 ;;
esac

ELF="$DIR/target/$TARGET/release/esp32-battery"

EXTRA_FEATURES=()
[[ "${INA_FAKE:-0}" == "1" ]] && EXTRA_FEATURES+=("ina-fake")
[[ "${XY_FAKE:-0}"  == "1" ]] && EXTRA_FEATURES+=("xy-fake")

if (( ${#EXTRA_FEATURES[@]} > 0 )); then
    JOINED="$(IFS=, ; echo "${EXTRA_FEATURES[*]}")"
    echo "Building with extra features: $JOINED"
    cargo "$ALIAS" --features "$JOINED"
else
    cargo "$ALIAS"
fi

if [[ -n "${PORT:-}" ]]; then
    : # honor caller override
elif [[ "$(uname -s)" == "Darwin" ]]; then
    PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -n1 || true)"
    [[ -z "$PORT" ]] && { echo "No /dev/cu.usbmodem* device found"; exit 1; }
else
    PORT="/dev/ttyACM0"
fi

espflash flash \
    --erase-data-parts ota \
    --monitor \
    --non-interactive \
    --partition-table "$PARTITIONS" \
    --port "$PORT" \
    "$ELF"
