# ESP32 Battery Monitor

Battery monitoring system built on the ESP32-C6. Measures voltage, current, and power from two INA228 sensors (battery + power supply) over I2C, and serves real-time data through a web dashboard.

## Hardware

- **MCU**: ESP32-C6
- **Sensors**: 2× INA228 current/voltage/power monitors (I2C addresses 0x40, 0x41)
- **Shunt**: 2mΩ, 15A max
- **I2C**: GPIO8 (SDA), GPIO9 (SCL), 400kHz
- **LCD** (optional): ST7789 via SPI (feature-gated with `lcd`)

## Features

- **Web dashboard** — real-time voltage, current, power, SOC, and history chart over HTTPS
- **Captive portal** — AP mode with DNS spoofing for WiFi setup when no credentials are stored
- **OTA updates** — signed firmware uploads (HMAC-SHA256) via web UI
- **Adaptive history** — 144-sample circular buffer with automatic compaction, covers ~41 hours in 4KB
- **LiFePO4 4S SOC** — voltage-based state-of-charge estimation with lookup table interpolation
- **Monitoring stack** — Telegraf + InfluxDB + Grafana integration via JSON API
- **mDNS** — discoverable as `battery-esp32.local`

## Building & Flashing

Requires the [esp-idf-sys prerequisites](https://github.com/esp-rs/esp-idf-sys#prerequisites) and `espflash`.

```bash
# Initial flash (erases OTA partitions, starts serial monitor)
./flash.sh

# Build only
cargo build --release

# Without LCD support
cargo build --release --no-default-features
```

### Partition Tables

- `partitions.csv` — 8MB flash (default)
- `partitions-4mb.csv` — 4MB flash

## OTA Deployment

```bash
# Build, sign, and upload firmware
./deploy.sh <ESP32_IP>

# Build and sign only
./deploy.sh
```

Requires `ota_key.bin` (32-byte HMAC key, not in repo). The signed binary is uploaded to `https://<IP>/ota/upload`.

## WiFi Setup

On first boot (or after credential reset), the device starts an access point:

- **SSID**: `Battery-Setup`
- **Password**: `01010101`
- **Portal**: connects automatically via captive portal, or navigate to `192.168.71.1`

Select your WiFi network, enter the password, and the device reboots to connect.

## API

`GET /api` returns JSON:

```json
{
  "uptime": 12345,
  "rssi": -45,
  "voltage": 13.256,
  "interval": 1,
  "read_err": [2, 1000],
  "charge": 45.12,
  "max_charge": 100.5,
  "power_online": 0.95,
  "s1": { "soc": 85.5, "current": -2.5, "power": -33.0 },
  "s2": { "current": 1.2, "power": 15.8 },
  "history": [[timestamp, voltage, current1, current2, power_online], ...]
}
```

Supports incremental updates via `?since=<timestamp>` query parameter.

## Monitoring

A Telegraf + InfluxDB + Grafana stack is included in `monitoring/`.

```bash
cd monitoring

# Copy and fill in .env
cp .env.example .env
# Edit .env with your values

docker compose up -d
```

Grafana is available at `http://localhost:3000` (admin/admin) with a pre-provisioned dashboard.

## Project Structure

```
├── src/
│   ├── main.rs             # Entry point, WiFi state machine
│   ├── wifi.rs             # STA/AP management, scanning, mDNS
│   ├── dns.rs              # Captive portal DNS responder
│   ├── http.rs             # HTTPS server, API, web UI
│   ├── ina.rs              # INA228 measurement thread
│   ├── ota.rs              # OTA with HMAC verification
│   ├── nvs_creds.rs        # WiFi credential storage (NVS)
│   ├── platform.rs         # History persistence (NVS)
│   ├── lcd.rs              # ST7789 display (optional)
│   └── *.html, *.css       # Web assets (gzipped at build time)
├── logic/                  # Pure Rust library (no ESP deps, testable on host)
│   └── src/
│       ├── battery.rs      # LiFePO4 OCV→SOC lookup
│       ├── data.rs         # SensorData, history, compaction
│       └── form.rs         # URL decoding, form parsing
├── monitoring/             # TIG stack (docker compose)
├── deploy.sh               # OTA build + upload
├── flash.sh                # Initial flash via USB
└── build.rs                # Gzips web assets at compile time
```

## HTTPS

The server uses a self-signed certificate. Place your cert and key at:

- `certs/selfsigned.crt`
- `certs/selfsigned.key`

These are not included in the repo.
