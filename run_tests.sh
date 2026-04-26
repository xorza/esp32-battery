#!/bin/bash
# Run all checks: host-side logic tests, firmware build/clippy on both
# esp32c6 and esp32c3, with and without fake hardware features. Fails
# fast on the first error.

set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

HOST_TARGET="x86_64-unknown-linux-gnu"
C6_TARGET="riscv32imac-esp-espidf"
C3_TARGET="riscv32imc-esp-espidf"

run() {
    echo
    echo "=== $* ==="
    "$@"
}

# Host-side: pure-logic unit tests + clippy (no esp-idf).
run cargo +stable nextest run -p esp32-battery-logic --target "$HOST_TARGET"
run cargo +stable clippy -p esp32-battery-logic --target "$HOST_TARGET" --all-targets -- -D warnings
run cargo +stable fmt

# Firmware: clippy on both chips, real and fake configurations. Release-mode
# to match what flash.sh ships.
run cargo clippy --target "$C6_TARGET" --release -- -D warnings
run cargo clippy --target "$C6_TARGET" --release --features ina-fake,xy-fake -- -D warnings
run env MCU=esp32c3 cargo clippy --target "$C3_TARGET" --release --no-default-features --features esp32c3 -- -D warnings
run env MCU=esp32c3 cargo clippy --target "$C3_TARGET" --release --no-default-features --features esp32c3,ina-fake,xy-fake -- -D warnings

echo
echo "=== ALL CHECKS PASSED ==="
