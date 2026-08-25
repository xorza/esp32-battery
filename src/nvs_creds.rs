use esp_idf_svc::nvs::{EspNvs, NvsDefault};

use esp32_battery_logic::{PASSWORD_MAX, SSID_MAX, WifiCredentials};
use log::{info, warn};

const NAMESPACE: &str = "wifi";

pub fn open(partition: esp_idf_svc::nvs::EspDefaultNvsPartition) -> EspNvs<NvsDefault> {
    EspNvs::new(partition, NAMESPACE, true).unwrap()
}

/// Stored credentials, or `None` when there are none — or when what is stored
/// is unusable.
///
/// Nothing on this path may panic. `main` starts the captive portal precisely
/// when this returns `None`, so a panic (the hook reboots the device) on a
/// corrupt blob would be a boot loop with no way in to enter new credentials.
/// Every failure therefore degrades to `None` and lets the portal come up.
pub fn load(nvs: &EspNvs<NvsDefault>) -> Option<WifiCredentials> {
    // +1 so an at-limit value fits and an over-long one is rejected by
    // `WifiCredentials::new` rather than silently truncated by `get_str`.
    let mut ssid_buf = [0u8; SSID_MAX + 1];
    let mut pass_buf = [0u8; PASSWORD_MAX + 1];

    let ssid = read_str(nvs, "ssid", &mut ssid_buf)?;
    let password = read_str(nvs, "pass", &mut pass_buf)?;

    match WifiCredentials::new(ssid, password) {
        Ok(creds) => Some(creds),
        Err(e) => {
            warn!("stored WiFi credentials rejected ({e}); starting captive portal");
            None
        }
    }
}

/// One NVS string, or `None` if the key is unset or what is stored will not
/// fit `buf` — `get_str` reports `ESP_ERR_NVS_INVALID_LENGTH` for a blob
/// longer than the buffer, which is exactly the corrupt-value case `load`
/// must survive.
fn read_str<'a>(nvs: &EspNvs<NvsDefault>, key: &str, buf: &'a mut [u8]) -> Option<&'a str> {
    match nvs.get_str(key, buf) {
        Ok(value) => value,
        Err(e) => {
            warn!("NVS read of '{key}' failed ({e}); treating as unset");
            None
        }
    }
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
