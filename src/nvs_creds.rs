use std::sync::Arc;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;
use log::info;

use crate::http::text_response;

const NAMESPACE: &str = "wifi";

pub struct WifiCredentials {
    pub ssid: String,
    pub password: String,
}

pub fn open(partition: esp_idf_svc::nvs::EspDefaultNvsPartition) -> EspNvs<NvsDefault> {
    EspNvs::new(partition, NAMESPACE, true).unwrap()
}

pub fn load(nvs: &EspNvs<NvsDefault>) -> Option<WifiCredentials> {
    let mut ssid_buf = [0u8; 33];
    let mut pass_buf = [0u8; 65];

    let ssid = nvs.get_str("ssid", &mut ssid_buf).unwrap()?;
    let password = nvs.get_str("pass", &mut pass_buf).unwrap()?;

    if ssid.is_empty() {
        return None;
    }

    Some(WifiCredentials {
        ssid: ssid.to_string(),
        password: password.to_string(),
    })
}

pub fn save(nvs: &EspNvs<NvsDefault>, ssid: &str, password: &str) {
    assert!(!ssid.is_empty(), "SSID must not be empty");
    assert!(ssid.len() <= 32, "SSID too long");
    assert!(
        password.is_empty() || (8..=63).contains(&password.len()),
        "password must be empty or 8-63 chars"
    );

    nvs.set_str("ssid", ssid).unwrap();
    nvs.set_str("pass", password).unwrap();
    info!("WiFi credentials saved for '{}'", ssid);
}

pub fn clear(nvs: &EspNvs<NvsDefault>) {
    let _ = nvs.remove("ssid");
    let _ = nvs.remove("pass");
    info!("WiFi credentials cleared");
}

pub fn register_reset(server: &mut EspHttpServer<'static>, nvs: Arc<EspNvs<NvsDefault>>) {
    server
        .fn_handler("/wifi-reset", esp_idf_svc::http::Method::Post, move |req| {
            clear(&nvs);
            text_response(req, 200, b"WiFi credentials cleared. Rebooting...")?;
            crate::reboot::reboot_after("Rebooting after WiFi reset");
            Ok::<(), EspError>(())
        })
        .unwrap();
}
