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
| voltage | V | Battery voltage |
| soc | % | State of charge (computed from OCV) |
| battery_current | A | Battery current (negative when charging) |
| battery_power | W | Battery power |
| supply_voltage | V | Power-supply (XY7025) output voltage |
| supply_current | A | Power-supply output current |
| supply_power | W | Power-supply output power |
| power_online | 0/1 | Supply-online indicator (averaged over the window for uptime %) |
| heap_free | bytes | Free heap right now |
| heap_min_free | bytes | Low-water mark of free heap since boot |
| rssi | dBm | WiFi signal strength |
| uptime | s | ESP32 uptime |
