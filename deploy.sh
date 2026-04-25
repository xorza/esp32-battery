#!/bin/bash
set -euo pipefail

DIR="$(dirname "$0")"
CHIP="${MCU:-esp32c6}"

case "$CHIP" in
    esp32c3) TARGET="riscv32imc-esp-espidf" ; ALIAS="c3" ;;
    esp32c6) TARGET="riscv32imac-esp-espidf" ; ALIAS="c6" ;;
    *)       echo "Unknown MCU: $CHIP"; exit 1 ;;
esac

ENV_FILE="$DIR/.env"
BIN="$DIR/target/$TARGET/release/firmware.bin"
SIGNED="$DIR/target/$TARGET/release/firmware_signed.bin"
ELF="$DIR/target/$TARGET/release/esp32-battery"

if [ -z "${OTA_KEY:-}" ] && [ -f "$ENV_FILE" ]; then
    OTA_KEY=$(grep -E '^OTA_KEY=' "$ENV_FILE" | head -n1 | cut -d= -f2- | tr -d '"' | tr -d "'")
fi

if [ -z "${OTA_KEY:-}" ]; then
    echo "Error: OTA_KEY not set (env var or $ENV_FILE)"
    exit 1
fi

if [ "${#OTA_KEY}" -ne 64 ]; then
    echo "Error: OTA_KEY must be 64 hex chars (32 bytes), got ${#OTA_KEY}"
    exit 1
fi

EXTRA_FEATURES=()
[[ "${INA_FAKE:-0}" == "1" ]] && EXTRA_FEATURES+=("ina-fake")
[[ "${XY_FAKE:-0}"  == "1" ]] && EXTRA_FEATURES+=("xy-fake")

echo "Building release for $CHIP..."
if (( ${#EXTRA_FEATURES[@]} > 0 )); then
    JOINED="$(IFS=, ; echo "${EXTRA_FEATURES[*]}")"
    echo "Building with extra features: $JOINED"
    cargo "$ALIAS" --features "$JOINED"
else
    cargo "$ALIAS"
fi

echo "Creating binary image..."
espflash save-image --chip "$CHIP" "$ELF" "$BIN"

echo "Signing..."
HMAC=$(openssl dgst -sha256 -mac HMAC -macopt "hexkey:$OTA_KEY" -binary "$BIN" | xxd -p -c 64)
printf '%s' "$HMAC" | xxd -r -p | cat - "$BIN" > "$SIGNED"

SIZE=$(wc -c < "$SIGNED")
echo "Done: $SIGNED ($SIZE bytes)"

if [ $# -ge 1 ]; then
    IP="$1"
    echo "Uploading to https://$IP/ota/upload ..."
    curl -k -o /dev/null -X POST -H "Content-Type: application/octet-stream" --data-binary "@$SIGNED" --progress-bar "https://$IP/ota/upload" 2>&1 || true
fi
