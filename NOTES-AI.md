# ESP32 Battery Monitor — AI Notes

Implementation notes for AI assistants. Tracks current state only — see git history for evolution.

## Hardware

- MCU: ESP32-C6 (default) or ESP32-C3, selected via Cargo features (`board.rs` enforces exactly one).
- Battery sensor: 1× INA228 over I2C @ 0x40, 2 mΩ shunt, 15 A. Supply-side V/I/P comes from the XY7025 over Modbus (no second INA).
- Charger: XY7025 buck on UART1, Modbus-RTU @ 115200 8N1, slave 0x01.
- Battery: 4S LiFePO4 (12 V nominal). Other chemistries defined in `logic/src/charging`.
- Optional ST7789 LCD via SPI (`lcd` feature).

## Workspace Layout

Two Rust crates:

- **Root crate `esp32-battery`** — firmware, ESP-IDF dependencies. Excludes `logic/` from its workspace so `logic/` builds for the host.
- **`logic/`** — pure-Rust library, no ESP deps, host-testable with `cargo nextest run`.

Per-MCU lockfiles: `components_esp32c6.lock`, `components_esp32c3.lock` (managed components for `espressif/mdns`).

## Firmware Modules (`src/`)

- `main.rs` — entry point. Owns the `NetState` supervisor loop (1 Hz ticks). Each tick: `tick_and_persist`, drain reset signal, `step(state)`, publish LCD status.
- `net.rs` — `NetState` enum, `LinkState`, `CaptiveBundle`, `NetStatusHandle`, `ResetSignal`, `SubmissionStatus`. Spec in `src/net_fsm.md`.
- `wifi.rs` — `WifiDriver` → `StaWifi` / `MixedWifi`. Scan cache, mDNS setup (`battery-esp32.local`), AP creds (`Battery-Setup` / `01010101`).
- `dns.rs` — captive DNS responder (spoofs every A query to 192.168.71.1).
- `http/` — `mod.rs` shared helpers (`create_server`, `serve_static`, `mount_*`), `main_server.rs` HTTPS dashboard, `captive.rs` plaintext captive HTTP.
- `api.rs` — `GET /api`. Serializes `ApiResponse { uptime, rssi, voltage, power_online, heap, battery, ps, history }` via serde-json-core into a 16 KiB buffer. History is borrowed during serialization (no clone) under the `SensorData` lock.
- `captive_api.rs` — `/scan`, `/save`, `/status`, `/generate_204`. `/save` writes credentials to a single-slot mailbox; supervisor drains.
- `ina.rs` — INA228 thread, sub-Hz averaging, watchdog. Honors `ina-fake` feature.
- `xy.rs` — XY7025 Modbus thread + `ChargeSupervisor` integration. Honors `xy-fake` feature.
- `ota.rs` — `/ota/upload`. HMAC-SHA256-verified firmware writes to inactive OTA partition.
- `nvs_creds.rs` — WiFi creds in NVS. Single read at boot; supervisor never re-reads per tick.
- `history_store.rs` — serialized history blob in NVS (~4 KiB). Loaded once at boot, saved every `SAVE_INTERVAL_S` (600 s).
- `clock.rs` — `uptime()`, `EspClock` (SNTP-backed wall-clock seconds), `EventRecorder` (clock + event-log shim passed to producers).
- `log_ring.rs` — in-memory log ring; `init()` installs as a slog drain; mounted at `/api/log`.
- `errors.rs` — `/api/errors` reads the structured `EventLog`.
- `wifi_reset.rs` — `/wifi-reset` clears NVS creds and raises `ResetSignal`.
- `reboot.rs` — `/reboot`.
- `lcd.rs` — ST7789 thread, reads `SensorData` + `EventLog` + `NetStatus`. `lcd` feature only.
- `board.rs` — per-MCU peripheral wiring (`Board::take`).

## Logic Crate (`logic/src/`)

- `lib.rs` — re-exports submodules.
- `battery.rs` — `ocv_soc(voltage)` 4S LiFePO4 lookup with linear interpolation.
- `charging/mod.rs` — `Chemistry`, `Profile::for_pack`, `SafetyLimits`, `ChargeSupervisor` two-phase CV FSM (Float ↔ Absorb hysteresis on charging current). `Action`, `BatterySample`, `FaultReason` types. Sign convention: charging current is **negative** (matches INA228 wiring).
- `charging/tests.rs` — supervisor transitions, debounce, hysteresis, fault trips.
- `data/mod.rs` — `SensorData`, `Ina228Reading`, `PsReading`, `Sample`. `LiveReadings` per-producer staleness (5 ticks). Owns the ring + codec.
- `data/history.rs` — adaptive-resolution history ring + serialization. `HISTORY_CAPACITY` derived from `SERIALIZED_MAX_BYTES = 4096`. Compaction halves resolution when full so total time-coverage grows.
- `error_log.rs` — `EventLog` ring + `Event` enum with sensor / Modbus / charging variants.
- `log_ring.rs` — generic in-memory log ring used by both crates.
- `form.rs` — URL-decode + form parsing (used by `/save`).
- `modbus.rs` — Modbus-RTU framing + CRC. `ModbusError`.
- `dns_packet.rs` — DNS packet parse/build for the captive responder.

## Net FSM

See `src/net_fsm.md` (canonical). Five variants: `CaptiveIdle`, `CaptiveTrying`, `CaptiveFallbackRetrying`, `StaConnecting`, `StaServing { link }`. Constants: 20 s captive try window, 2 h STA-side fallback grace, 10 s scan-cache TTL.

Key invariant: each variant carries the full state needed for that arm — no `Option`-as-flag fields, no shared mutex with HTTP for control state.

## API Wire Format

```json
{
  "uptime": 12345,
  "rssi": -45,
  "voltage": 13.256,
  "power_online": 0.95,
  "heap": { "free": 102400, "min_free": 81920 },
  "battery": { "soc": 85.5, "current": -2.5, "power": -33.0 },
  "ps":      { "voltage": 13.6, "current": 1.2, "power": 16.3 },
  "history": [[time_s, voltage, battery_current, ps_current, power_online], ...]
}
```

History rows are 5-tuples (not objects) for compactness. `power_online` is the moving average of a 1.0/0.0 indicator → fractional uptime of the supply over the bin.

## Build

```sh
MCU=esp32c6 ./flash.sh        # flash + monitor (timeout 30 wrapper recommended)
cargo build --release         # default features: esp32c6 + lcd
cd logic && cargo nextest run # host tests
```

For C3: `cargo build --release --no-default-features --features esp32c3,lcd`.
Headless: drop `lcd` from features.
Bench fakes: add `ina-fake` and/or `xy-fake`.

## Patches

Cargo `[patch.crates-io]` pins `esp-idf-sys`, `esp-idf-svc` to upstream `master`, and `esp-idf-hal` to a fork branch carrying an in-flight I2C driver patch (`xorza/esp-idf-hal#feat/i2c-driver`). If you bump these, recheck the I2C init path in `src/ina.rs`.
