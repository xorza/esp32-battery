//! POST /wifi-reset: clear stored WiFi credentials and reboot into the
//! captive portal. Mounted on the host server only.

use std::sync::Arc;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;

use crate::http::text_response;
use crate::nvs_creds;
use crate::reboot;

pub fn mount(server: &mut EspHttpServer<'static>, nvs: Arc<EspNvs<NvsDefault>>) {
    server
        .fn_handler("/wifi-reset", esp_idf_svc::http::Method::Post, move |req| {
            nvs_creds::clear(&nvs);
            text_response(req, 200, b"WiFi credentials cleared. Rebooting...")?;
            reboot::reboot_after("Rebooting after WiFi reset");
            Ok::<(), EspError>(())
        })
        .unwrap();
}
