//! POST /wifi-reset: clear stored WiFi credentials and reboot into the
//! captive portal. Mounted on the host server only.

use std::sync::Arc;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;

use crate::http::{json_ok, mount_post};
use crate::nvs_creds;
use crate::reboot;

pub fn mount(server: &mut EspHttpServer<'static>, nvs: Arc<EspNvs<NvsDefault>>) {
    mount_post(server, "/wifi-reset", move |req| {
        nvs_creds::clear(&nvs);
        json_ok(req)?;
        reboot::reboot_after("Rebooting after WiFi reset");
        Ok::<(), EspError>(())
    });
}
