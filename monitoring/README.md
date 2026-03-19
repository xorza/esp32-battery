# Battery Monitoring Stack

Telegraf + InfluxDB + Grafana (TIG) stack that polls the ESP32 battery monitor API.

## Setup

1. Edit `.env`:
   - `ESP32_HOST` — your ESP32's IP address
   - `INFLUXDB_TOKEN` — random secret token
   - `INFLUXDB_ADMIN_PASSWORD` — InfluxDB admin password

2. Start:
   ```
   docker compose up -d
   ```

3. Open Grafana at http://localhost:3000 (admin/admin).
   The "ESP32 Battery Monitor" dashboard is pre-provisioned.

## Architecture

- **Telegraf** polls `http://<ESP32_HOST>/api` every 1s, parses the JSON response, and writes metrics to InfluxDB.
- **InfluxDB v2** stores time-series data with 365-day retention.
- **Grafana** visualizes data from InfluxDB using Flux queries.

## Metrics collected

| Field | Unit | Description |
|-------|------|-------------|
| voltage | V | Battery voltage (avg of both sensors) |
| soc | % | State of charge |
| battery_current | A | Battery current (sensor 1) |
| battery_power | W | Battery power (sensor 1) |
| supply_current | A | Power supply current (sensor 2) |
| supply_power | W | Power supply power (sensor 2) |
| charge | Ah | Accumulated charge |
| max_charge | Ah | Max charge seen |
| rssi | dBm | WiFi signal strength |
| uptime | s | ESP32 uptime |
| read_failures | count | I2C read failures |
| read_total | count | Total I2C reads |
