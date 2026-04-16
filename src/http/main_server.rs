//! Main dashboard HTTPS server (dashboard, /api, /wifi-reset, /ota).

use std::sync::{Arc, Mutex};
use std::time::Duration;

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;
use log::{debug, warn};

use esp32_battery_logic::battery;

use crate::AppState;
use crate::api::{ApiResponse, BatteryReading, HistoryRow, PsReading, RESPONSE_BUF_SIZE};

use super::{create_server, get_rssi, serve_common_assets, serve_static, text_response};

pub fn start(state: Arc<AppState>, nvs: Arc<EspNvs<NvsDefault>>) -> EspHttpServer<'static> {
    let mut server = create_server(10240, false, 4, Some(Duration::from_secs(0)), true);

    serve_common_assets(&mut server);
    serve_static(
        &mut server,
        "/",
        "text/html",
        "max-age=3600",
        include_bytes!(concat!(env!("OUT_DIR"), "/index.html")),
        true,
    );

    let json_buf = Mutex::new(Box::new([0u8; RESPONSE_BUF_SIZE]));

    server
        .fn_handler("/api", esp_idf_svc::http::Method::Get, move |req| {
            // Snapshot sensor state, release the lock, then serialize.
            // Keeps the measurement thread unblocked during JSON serialization.
            let response = {
                let store = state.sensor_data.lock().unwrap();
                let bat = store.battery_reading;
                let ps = store.ps_reading;
                let history = store.history();
                // When PS is offline its sensor may be absent or reading zero; averaging
                // would halve the reported voltage. Use battery voltage alone in that case.
                let voltage = if store.power_online >= 0.5 {
                    (bat.voltage + ps.voltage) / 2.0
                } else {
                    bat.voltage
                };
                let max_charge = history
                    .iter()
                    .map(|s| s.max_charge)
                    .fold(0.0_f64, f64::max);
                let history_rows: Vec<HistoryRow> = history.iter().map(HistoryRow::from).collect();

                ApiResponse {
                    uptime: crate::uptime_s(),
                    rssi: get_rssi(),
                    voltage,
                    read_err: [store.read_failures, store.read_total],
                    charge: bat.charge,
                    max_charge,
                    power_online: store.power_online,
                    battery: BatteryReading {
                        soc: battery::ocv_soc(bat.voltage),
                        current: bat.current,
                        power: bat.power,
                    },
                    ps: PsReading {
                        current: ps.current,
                        power: ps.power,
                    },
                    history: history_rows,
                }
            };

            let mut guard = json_buf.lock().unwrap();
            let buf: &mut [u8] = &mut **guard;
            let len = match serde_json_core::to_slice(&response, buf) {
                Ok(n) => n,
                Err(e) => {
                    warn!("API: JSON serialization failed ({:?}); returning 500", e);
                    return super::text_response(req, 500, b"serialization error");
                }
            };

            debug!(
                "API: history={} json={}/{}",
                response.history.len(),
                len,
                RESPONSE_BUF_SIZE,
            );

            let mut resp = req
                .into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Connection", "close"),
                    ],
                )
                .map_err(|e| e.0)?;
            resp.write_all(&buf[..len]).map_err(|e| e.0)?;
            Ok::<(), EspError>(())
        })
        .unwrap();

    server
        .fn_handler("/wifi-reset", esp_idf_svc::http::Method::Post, move |req| {
            crate::nvs_creds::clear(&nvs);
            text_response(req, 200, b"WiFi credentials cleared. Rebooting...")?;
            crate::reboot_after("Rebooting after WiFi reset");
            Ok::<(), EspError>(())
        })
        .unwrap();

    serve_static(
        &mut server,
        "/ota",
        "text/html",
        "no-cache",
        include_bytes!(concat!(env!("OUT_DIR"), "/ota.html")),
        true,
    );

    crate::ota::register(&mut server);

    server
}
