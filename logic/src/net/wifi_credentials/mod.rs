//! WiFi credentials as pure data. NVS storage lives in the firmware;
//! the validation belongs here so every producer — captive form parse,
//! NVS load, tests — gets the same checks.

use crate::error::CredentialsError;

pub const SSID_MAX: usize = 32;
pub const PASSWORD_MAX: usize = 64;

/// WPA2 pass-phrase length. 64 characters would be a raw 256-bit PSK in hex,
/// which the radio takes but the captive form does not offer, so the buffer
/// is one byte wider than anything accepted here. `CredentialsError::message`
/// quotes this range back to the user; there is no way to derive that string
/// from the range without formatting at runtime, so the two move together.
const PSK_LEN: std::ops::RangeInclusive<usize> = 8..=63;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WifiCredentials {
    pub ssid: heapless::String<SSID_MAX>,
    pub password: heapless::String<PASSWORD_MAX>,
}

impl WifiCredentials {
    /// Validate a candidate pair against the 802.11 + WPA2 limits the radio
    /// enforces.
    ///
    /// This is the only way to build the type, so every producer is held to
    /// the same rules and no caller needs to pre-screen its input. Both
    /// producers are untrusted — a captive-form body and an NVS blob — which
    /// is why the failure is a `Result` and not an assert; see
    /// [`CredentialsError`].
    pub fn new(ssid: &str, password: &str) -> Result<Self, CredentialsError> {
        if ssid.is_empty() {
            return Err(CredentialsError::SsidEmpty);
        }
        if ssid.len() > SSID_MAX {
            return Err(CredentialsError::SsidTooLong);
        }
        if !password.is_empty() && !PSK_LEN.contains(&password.len()) {
            return Err(CredentialsError::PasswordLength);
        }
        Ok(Self {
            ssid: heapless::String::try_from(ssid).expect("ssid fits SSID_MAX"),
            password: heapless::String::try_from(password).expect("password fits PASSWORD_MAX"),
        })
    }
}

#[cfg(test)]
mod tests;
