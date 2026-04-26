use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use log::info;

const NAMESPACE: &str = "wifi";

#[derive(Clone)]
pub struct WifiCredentials {
    pub ssid: String,
    pub password: String,
}

impl WifiCredentials {
    /// Construct after validating ssid/password lengths against the
    /// 802.11 + WPA2 limits the radio enforces. Centralised here so
    /// every site that produces credentials (form parse, NVS load,
    /// future callers) gets the same checks — without this the
    /// `try_into()` inside `wifi::sta_config` would panic on overlong
    /// inputs with a confusing "TryFromSliceError" message.
    pub fn new(ssid: String, password: String) -> Self {
        assert!(!ssid.is_empty(), "SSID must not be empty");
        assert!(ssid.len() <= 32, "SSID too long ({} > 32)", ssid.len());
        assert!(
            password.is_empty() || (8..=63).contains(&password.len()),
            "password must be empty or 8-63 chars (got {})",
            password.len()
        );
        Self { ssid, password }
    }
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

    Some(WifiCredentials::new(ssid.to_string(), password.to_string()))
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
