//! Host-mode dashboard server (HTTPS on 443). Composes the per-feature mount
//! fns; this file owns ordering and dependency wiring, nothing else.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use esp32_battery_logic::data::SensorData;
use esp32_battery_logic::error_log::EventLog;

use crate::log_ring;
use crate::net::ResetSignal;
use crate::{api, errors, ota, wifi_reset};

use super::{create_server, serve_common_assets, serve_static};

pub fn start(
    sensor_data: Arc<Mutex<SensorData>>,
    event_log: Arc<Mutex<EventLog>>,
    nvs: Arc<EspNvs<NvsDefault>>,
    reset: ResetSignal,
) -> EspHttpServer<'static> {
    let mut server = create_server(10240, false, 3, Some(Duration::from_secs(0)), true);

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

    api::mount(&mut server, sensor_data);
    errors::mount(&mut server, event_log);

    log_ring::mount(&mut server);
    wifi_reset::mount(&mut server, nvs, reset);
    ota::mount(&mut server);

    server
}
