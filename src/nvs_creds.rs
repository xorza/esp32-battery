use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use esp32_battery_logic::{PASSWORD_MAX, SSID_MAX, WifiCredentials};
use log::info;

const NAMESPACE: &str = "wifi";

pub fn open(partition: esp_idf_svc::nvs::EspDefaultNvsPartition) -> EspNvs<NvsDefault> {
    EspNvs::new(partition, NAMESPACE, true).unwrap()
}

pub fn load(nvs: &EspNvs<NvsDefault>) -> Option<WifiCredentials> {
    // +1 so an at-limit value fits and a corrupt oversize blob is caught
    // by the length asserts in `WifiCredentials::new` instead of being
    // silently truncated by `get_str`.
    let mut ssid_buf = [0u8; SSID_MAX + 1];
    let mut pass_buf = [0u8; PASSWORD_MAX + 1];

    let ssid = nvs.get_str("ssid", &mut ssid_buf).unwrap()?;
    let password = nvs.get_str("pass", &mut pass_buf).unwrap()?;

    if ssid.is_empty() {
        return None;
    }

    Some(WifiCredentials::new(ssid, password))
}

pub fn save(nvs: &EspNvs<NvsDefault>, creds: &WifiCredentials) {
    nvs.set_str("ssid", &creds.ssid).unwrap();
    nvs.set_str("pass", &creds.password).unwrap();
    info!("WiFi credentials saved for '{}'", creds.ssid);
}

pub fn clear(nvs: &EspNvs<NvsDefault>) {
    let _ = nvs.remove("ssid");
    let _ = nvs.remove("pass");
    info!("WiFi credentials cleared");
}
