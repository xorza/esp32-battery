use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use log::info;

const NAMESPACE: &str = "wifi";

pub const SSID_MAX: usize = 32;
pub const PASSWORD_MAX: usize = 64;

#[derive(Clone)]
pub struct WifiCredentials {
    pub ssid: heapless::String<SSID_MAX>,
    pub password: heapless::String<PASSWORD_MAX>,
}

impl WifiCredentials {
    /// Construct after validating ssid/password lengths against the
    /// 802.11 + WPA2 limits the radio enforces. Centralised here so
    /// every site that produces credentials (form parse, NVS load,
    /// future callers) gets the same checks — without this the
    /// `try_into()` inside `wifi::sta_config` would panic on overlong
    /// inputs with a confusing "TryFromSliceError" message.
    pub fn new(ssid: &str, password: &str) -> Self {
        assert!(!ssid.is_empty(), "SSID must not be empty");
        assert!(
            ssid.len() <= SSID_MAX,
            "SSID too long ({} > {SSID_MAX})",
            ssid.len()
        );
        // WPA2 PSK: 8-63 chars, or empty for open networks. Buffer fits 64
        // for headroom, but the radio rejects anything outside 8..=63.
        assert!(
            password.is_empty() || (8..=63).contains(&password.len()),
            "password must be empty or 8-63 chars (got {})",
            password.len()
        );
        Self {
            ssid: heapless::String::try_from(ssid).expect("ssid fits SSID_MAX"),
            password: heapless::String::try_from(password).expect("password fits PASSWORD_MAX"),
        }
    }
}

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
