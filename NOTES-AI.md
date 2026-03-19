# ESP32 Battery Monitor - AI Notes

## Overview
ESP32-C3 based power monitor using two INA228 sensors over I2C, serving real-time measurements and historical charts via a web dashboard.

## Hardware
- MCU: ESP32-C3 (RISC-V)
- Sensors: 2× TI INA228 (I2C addresses 0x40 battery, 0x41 power supply)
- I2C pins: SDA=GPIO8, SCL=GPIO9
- Shunt resistor: 2mΩ, max current: 15A
- Battery: 4S LiFePO4, 12V nominal, 100Ah

## Project Structure
- `src/main.rs` - Entry point, wires modules together
- `src/wifi.rs` - `Wifi` struct: init, mDNS (`esp32-battery.local`), infinite reconnect
- `src/ina.rs` - I2C/INA228 init, measurement thread, watchdog, reads both sensors
- `src/http.rs` - HTTP server: `/`, `/style.css`, `/api` (JSON with both sensors + history)
- `src/ota.rs` - OTA firmware update endpoint
- `src/index.html` - Dashboard: SOC hero, battery metrics, power supply metrics, history chart
- `src/style.css` - Dark theme styling
- `logic/` - Standalone pure-Rust crate for testable logic (no ESP-IDF deps)
  - `logic/src/lib.rs` - Exports: `battery`, `data`, `ring_buffer`
  - `logic/src/battery.rs` - OCV-based SOC estimation (4S LiFePO4 lookup table)
  - `logic/src/data.rs` - `Ina228Reading` (voltage, current, power, charge), `SensorData`, `Sample`, `Stats`, two-tier history
  - `logic/src/ring_buffer.rs` - Generic fixed-size ring buffer
- `Cargo.toml` - Dependencies + mDNS managed component
- `.cargo/config.toml` - Build target: riscv32imc-esp-espidf, ESP-IDF v5.4.3
- `rust-toolchain.toml` - Toolchain: nightly channel

## Two-Tier History
- **Recent**: 3600 samples (1/sec) = 1 hour, full resolution
- **Long-term**: 1440 samples (1/min, averaged from 60 recent) = 24 hours
- Total memory: ~118 KB (24 bytes/sample)
- HTTP API merges long-term prefix (older than recent) + recent for continuous timeline
- Downsamples to max 100 points for JSON

## Functionality
1. WiFi init (deferred connect), mDNS as `esp32-battery.local`
2. Spawns measurement thread:
   - Reads both INA228 sensors 10× at 100ms intervals, averages voltage/current/power, takes latest charge
   - Updates `SensorData` once per second
   - Watchdog timer (10s, panic on trigger)
3. HTTP server serves dashboard + JSON API
4. JSON API: `{s1: {soc, voltage, current, power, charge, max_charge, stats?, history}, s2: {...}}`
5. SOC computed at serialization time via `battery::ocv_soc(voltage)`
6. Stats (min/max/avg) computed over both tiers in single pass

## Testing
Pure-logic modules are in the `logic/` crate and testable on host:
```sh
cd logic && cargo nextest run    # runs 57 tests on x86_64
```
The logic crate has its own `.cargo/config.toml` targeting x86_64 to override the parent's ESP-IDF target.

Tests cover:
- **ring_buffer**: empty, push, fill, wrap-around, iteration order, capacity 1, zero capacity panic
- **battery**: below/above range, exact table entries, interpolation, monotonicity, output range, NaN/infinity panics
- **data**: default state, update, voltage averaging, last readings, charge storage, downsample timing/values/reset (including power averaging), stats (single, min/max/avg, power fields, including longterm, negative currents, empty), history (empty, below/at/over max_points, downsampled value correctness for all fields, longterm prefix with wrap-around, multiple prefix entries, prefix downsampling, chronological order)

## Key Dependencies
- `esp-idf-svc` - WiFi, HTTP server, mDNS
- `esp-idf-hal` - I2C, peripherals, watchdog
- `ina228` - INA228 driver (embedded-hal 1.0)
- `esp32-battery-logic` - Local crate with pure testable logic

## Build & Flash
```sh
cargo build          # build for ESP32
cargo run            # flash + monitor via espflash
cd logic && cargo nextest run  # run unit tests on host
```
