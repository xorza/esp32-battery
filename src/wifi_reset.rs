//! POST /wifi-reset: clear stored WiFi credentials and signal the
//! supervisor to drop the live association. The page reload that
//! follows lands on the captive portal.

use std::sync::Arc;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;

use crate::http::{json_ok, mount_post};
use crate::net::ResetSignal;
use crate::nvs_creds;

pub fn mount(
    server: &mut EspHttpServer<'static>,
    nvs: Arc<EspNvs<NvsDefault>>,
    reset: ResetSignal,
) {
    mount_post(server, "/wifi-reset", move |req| {
        nvs_creds::clear(&nvs);
        reset.raise();
        json_ok(req)?;
        Ok::<(), EspError>(())
    });
}
