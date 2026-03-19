#!/bin/bash
set -euo pipefail

DIR="$(dirname "$0")"
KEY="$DIR/ota_key.bin"
BIN="$DIR/target/riscv32imac-esp-espidf/release/firmware.bin"
SIGNED="$DIR/target/riscv32imac-esp-espidf/release/firmware_signed.bin"
ELF="$DIR/target/riscv32imac-esp-espidf/release/esp32-battery"

if [ ! -f "$KEY" ]; then
    echo "Error: key file not found: $KEY"
    exit 1
fi

echo "Building release..."
cargo build --release

echo "Creating binary image..."
espflash save-image --chip esp32c6 "$ELF" "$BIN"

echo "Signing..."
HMAC=$(openssl dgst -sha256 -mac HMAC -macopt "hexkey:$(xxd -p -c 256 "$KEY")" -binary "$BIN" | xxd -p -c 64)
printf '%s' "$HMAC" | xxd -r -p | cat - "$BIN" > "$SIGNED"

SIZE=$(wc -c < "$SIGNED")
echo "Done: $SIGNED ($SIZE bytes)"

if [ $# -ge 1 ]; then
    IP="$1"
    echo "Uploading to https://$IP/ota/upload ..."
    curl -k -o /dev/null -X POST -H "Content-Type: application/octet-stream" --data-binary "@$SIGNED" --progress-bar "https://$IP/ota/upload" 2>&1 || true
fi
