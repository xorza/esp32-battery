//! Main dashboard HTTPS server. Serves the dashboard + static OTA page; each
//! feature module mounts its own routes via its `register` fn.

use std::sync::Arc;
use std::time::Duration;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use crate::app_state::SensorDataHandle;

use super::{create_server, serve_common_assets, serve_static};

pub fn start(
    sensor_data: SensorDataHandle,
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

    crate::api::register(&mut server, sensor_data);
    crate::log_ring::register(&mut server);
    crate::nvs_creds::register_reset(&mut server, nvs);
    crate::ota::register(&mut server);

    server
}
