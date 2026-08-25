use super::*;

#[test]
fn credentials_accept_and_reject_at_the_radio_limits() {
    // Both producers of credentials are untrusted — a captive-form body and
    // an NVS blob — so every rule has to reject rather than panic. A panic on
    // the NVS path is a boot loop in particular: the captive portal starts
    // only when no credentials load, so there would be no way back in.
    let ssid_max = "s".repeat(SSID_MAX); // 32, the 802.11 limit
    let ssid_over = "s".repeat(SSID_MAX + 1);
    let psk_min = "p".repeat(8); // WPA2 pass-phrase floor
    let psk_max = "p".repeat(63); // WPA2 pass-phrase ceiling
    let psk_short = "p".repeat(7);
    let psk_over = "p".repeat(64); // a raw hex PSK, not a pass-phrase

    let cases: [(&str, &str, Option<CredentialsError>); 9] = [
        ("home", "password1", None),
        (&ssid_max, &psk_min, None),
        (&ssid_max, &psk_max, None),
        // The empty password is the one length outside 8..=63 that is legal:
        // it means an open network.
        ("home", "", None),
        ("", "password1", Some(CredentialsError::SsidEmpty)),
        // Both fields bad: the SSID is reported, pinning the check order.
        ("", &psk_short, Some(CredentialsError::SsidEmpty)),
        (&ssid_over, "password1", Some(CredentialsError::SsidTooLong)),
        ("home", &psk_short, Some(CredentialsError::PasswordLength)),
        ("home", &psk_over, Some(CredentialsError::PasswordLength)),
    ];

    for (ssid, password, expected) in cases {
        let label = format!("ssid {} chars / password {} chars", ssid.len(), password.len());
        match expected {
            None => {
                let got = WifiCredentials::new(ssid, password)
                    .unwrap_or_else(|e| panic!("{label} rejected: {e}"));
                assert_eq!(got.ssid.as_str(), ssid, "{label}");
                assert_eq!(got.password.as_str(), password, "{label}");
            }
            Some(want) => assert_eq!(
                WifiCredentials::new(ssid, password).err(),
                Some(want),
                "{label}"
            ),
        }
    }
}
