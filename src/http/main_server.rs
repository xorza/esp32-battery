//! Host-mode dashboard server (HTTPS on 443). Composes the per-feature mount
//! fns; this file owns ordering and dependency wiring, nothing else.

use std::sync::Arc;
use std::time::Duration;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use crate::app_state::{EventLogHandle, SensorDataHandle};
use crate::{api, errors, log_ring, ota, wifi_reset};

use super::{create_server, serve_common_assets, serve_static};

pub fn start(
    sensor_data: SensorDataHandle,
    event_log: EventLogHandle,
    nvs: Arc<EspNvs<NvsDefault>>,
) -> EspHttpServer<'static> {
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
    wifi_reset::mount(&mut server, nvs);
    ota::mount(&mut server);

    server
}
