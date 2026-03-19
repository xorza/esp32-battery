#!/bin/bash
set -euo pipefail

DIR="$(dirname "$0")"

cargo build --release

espflash flash \
    --erase-data-parts ota \
    --monitor \
    --partition-table "$DIR/partitions.csv" \
    --port /dev/ttyACM0 \
    "$DIR/target/riscv32imac-esp-espidf/release/esp32-battery"
