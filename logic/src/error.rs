//! Errors this crate hands back to the firmware.

/// Why a candidate SSID/password pair was rejected.
///
/// Credentials arrive from two untrusted sources — the captive form and the
/// NVS blob — so the checks return this rather than asserting. Asserting
/// would reboot the device (the panic hook in `src/main.rs`), and since the
/// captive portal only starts when no usable credentials load, a panic on the
/// NVS path is a boot loop with no way in to correct it.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CredentialsError {
    /// 802.11 permits a zero-length SSID only in a wildcard probe, never as
    /// an association target.
    SsidEmpty,
    /// Longer than [`SSID_MAX`](crate::SSID_MAX), the 802.11 limit the radio
    /// enforces.
    SsidTooLong,
    /// Outside the WPA2 pass-phrase range, and not the empty string that
    /// means "open network".
    PasswordLength,
}

impl CredentialsError {
    /// Human-readable cause, safe to hand to an HTTP client. `&'static str`
    /// so the HTTP error path needs no formatting buffer, and so `Display`
    /// below is one list rather than a second copy of these strings.
    pub fn message(self) -> &'static str {
        match self {
            Self::SsidEmpty => "SSID must not be empty",
            Self::SsidTooLong => "SSID must be at most 32 characters",
            Self::PasswordLength => "password must be empty or 8-63 characters",
        }
    }
}

impl std::fmt::Display for CredentialsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.message())
    }
}
