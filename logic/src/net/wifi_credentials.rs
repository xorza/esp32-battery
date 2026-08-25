//! WiFi credentials as pure data. NVS storage lives in the firmware;
//! the validation belongs here so every producer — captive form parse,
//! NVS load, tests — gets the same checks.

pub const SSID_MAX: usize = 32;
pub const PASSWORD_MAX: usize = 64;

#[derive(Clone, Debug, PartialEq, Eq)]
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
