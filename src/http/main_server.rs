//! Host-mode dashboard server (HTTPS on 443). Composes the per-feature mount
//! fns; this file owns ordering and dependency wiring, nothing else.

use std::sync::{Arc, Mutex};

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use esp32_battery_logic::EventLog;
use esp32_battery_logic::{ChargeStatus, SensorData};

use crate::log_ring;
use crate::net::ResetSignal;
use crate::{api, errors, ota, wifi_reset};

use super::{ServerConfig, create_server, serve_common_assets, serve_static};

pub fn start(
    sensor_data: Arc<Mutex<SensorData>>,
    charge_status: Arc<Mutex<ChargeStatus>>,
    event_log: Arc<Mutex<EventLog>>,
    nvs: Arc<EspNvs<NvsDefault>>,
    reset: ResetSignal,
) -> EspHttpServer<'static> {
    let mut server = create_server(ServerConfig {
        stack_size: 10240,
        max_sockets: 3,
        wildcard: false,
        https: true,
    });

    serve_common_assets(&mut server);
    serve_static(
        &mut server,
        "/",
        "text/html",
        "max-age=3600",
        include_bytes!(concat!(env!("OUT_DIR"), "/index.html")),
        true,
    );
    serve_static(
        &mut server,
        "/ota",
        "text/html",
        "no-cache",
        include_bytes!(concat!(env!("OUT_DIR"), "/ota.html")),
        true,
    );

    api::mount(&mut server, sensor_data, charge_status);
    errors::mount(&mut server, event_log);

    log_ring::mount(&mut server);
    wifi_reset::mount(&mut server, nvs, reset);
    ota::mount(&mut server);

    server
}
