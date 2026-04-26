# ESP32 Battery Monitor

Off-grid battery monitor and charger controller built on the ESP32-C6 (ESP32-C3 also supported). Reads two INA228 sensors (battery + power-supply rail) over I2C, drives an XY7025 programmable buck converter over Modbus-RTU for two-phase CV charging, and serves a real-time dashboard over HTTPS.

## Hardware

- **MCU**: ESP32-C6 (default) or ESP32-C3 — selected via the `esp32c6` / `esp32c3` Cargo features
- **Sensors**: 2× INA228 current/voltage/power monitors, I2C addresses 0x40 (battery) and 0x41 (power supply)
- **Shunt**: 2 mΩ, 15 A max
- **I2C**: SDA/SCL pins per `src/board.rs`, 400 kHz
- **Charger**: XY7025 programmable buck converter on UART1, Modbus-RTU @ 115200 8N1, slave 0x01
- **LCD** (optional, `lcd` feature): ST7789 over SPI via `mipidsi`
- **Battery**: 4S LiFePO4 (12 V nominal); chemistry profiles for LiFePO4 (top-balance) and Li-ion are also defined in `logic/src/charging`

## Features

- **HTTPS dashboard** — voltage, current, power, SOC, heap, history chart
- **Two-phase CV charging** — Float ↔ Absorb hysteresis driven by charging current; per-chemistry profiles with hard safety trips
- **Captive portal** — AP mode with DNS spoofing for first-boot WiFi setup
- **Signed OTA** — HMAC-SHA256-verified firmware uploads via `/ota`
- **Adaptive history** — single-tier ring with on-the-fly compaction; sized so the serialized blob fits in 4 KiB of NVS (~58 hours of coverage)
- **Persistent history** — periodically snapshotted to NVS, restored on boot
- **Event log** — ring buffer of structured events (sensor errors, Modbus faults, charging-state transitions) exposed at `/api/errors`
- **mDNS** — discoverable as `battery-esp32.local`
- **Fakes** — `ina-fake` and `xy-fake` features substitute in-memory devices for bench testing without hardware

## Net FSM

WiFi state machine and credential ownership are documented in [`src/net_fsm.md`](src/net_fsm.md). The supervisor is a single flat enum: each variant carries the radio mode, live servers, and credentials valid in that state — illegal combinations are not representable.

## Building & Flashing

Requires the [esp-idf-sys prerequisites](https://github.com/esp-rs/esp-idf-sys#prerequisites) and `espflash`.

```bash
# Flash + monitor (wraps `espflash flash --monitor --non-interactive`)
MCU=esp32c6 ./flash.sh

# Bench-test on hardware without sensors / charger attached
MCU=esp32c6 INA_FAKE=1 XY_FAKE=1 ./flash.sh

# Build only
cargo build --release

# Build for ESP32-C3 instead
cargo build --release --no-default-features --features esp32c3,lcd

# Headless (no LCD)
cargo build --release --no-default-features --features esp32c6
```

Both `flash.sh` and `deploy.sh` honor the same env vars:

| Var          | Values                  | Effect                                                              |
|--------------|-------------------------|---------------------------------------------------------------------|
| `MCU`        | `esp32c6` (default), `esp32c3` | Picks Cargo target, partition table, and `cargo c6` / `cargo c3` alias |
| `INA_FAKE`   | `1`                     | Adds the `ina-fake` Cargo feature — substitutes a canned in-memory device for the INA228 I2C driver |
| `XY_FAKE`    | `1`                     | Adds the `xy-fake` Cargo feature — substitutes a canned in-memory device for the XY7025 Modbus client |

Exactly one MCU feature must be enabled (`board.rs` enforces this with a `compile_error!`). `flash.sh` picks the right partition table per `MCU`.

## OTA Deployment

```bash
# Build, sign, and upload firmware
./deploy.sh <ESP32_IP>

# Build and sign only
./deploy.sh
```

Requires `OTA_KEY` (64 hex chars = 32-byte HMAC key) in a `.env` file at the repo root, or as an environment variable. The signed binary is uploaded to `https://<IP>/ota/upload`. Same `MCU` / `INA_FAKE` / `XY_FAKE` env vars as `flash.sh` apply.

## WiFi Setup

On first boot (or after `/wifi-reset`), the device starts an access point:

- **SSID**: `Battery-Setup`
- **Password**: `01010101`
- **Portal**: redirects to `192.168.71.1` via captive-portal probes (or browse there directly)

Pick a network, submit the password, and the device promotes to STA mode and starts the dashboard. See `src/net_fsm.md` for transition timing (20 s try window, 2 h fallback grace).

## API

`GET /api` returns JSON:

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

Sign convention: battery current is **negative when charging**.

Other endpoints mounted on the dashboard server:

| Method | Path           | Purpose                                            |
|--------|----------------|----------------------------------------------------|
| GET    | `/`            | Dashboard HTML (gzipped at build)                  |
| GET    | `/ota`         | OTA upload page                                    |
| GET    | `/api`         | Sensor + history snapshot (above)                  |
| GET    | `/api/errors`  | Structured event log                               |
| GET    | `/api/log`     | In-memory log ring                                 |
| POST   | `/ota/upload`  | Signed firmware upload (HMAC-SHA256)               |
| POST   | `/wifi-reset`  | Drop creds, reboot to captive portal               |

The captive server (mounted only when in AP/Mixed mode) exposes `/scan`, `/save`, `/status`, and the usual captive-portal probe URLs (`/generate_204` etc.) — see `src/captive_api.rs`.

## Monitoring

A Telegraf + InfluxDB + Grafana stack is included in `monitoring/`. See [`monitoring/README.md`](monitoring/README.md).

```bash
cd monitoring
cp .env.example .env  # fill in ESP32_HOST, INFLUXDB_TOKEN, etc.
docker compose up -d
```

Grafana at `http://localhost:3000` (admin/admin) with a pre-provisioned dashboard.

## Testing

The `logic/` crate has no ESP dependencies and runs natively:

```bash
cd logic && cargo nextest run
```

Tests cover battery SOC interpolation, history compaction and codec round-trip, charging FSM transitions and safety trips, Modbus framing, DNS packet parsing, and form decoding.

## HTTPS

The server uses a self-signed certificate. Place your cert and key at:

- `certs/selfsigned.crt`
- `certs/selfsigned.key`

These are not committed (see `.gitignore`).
