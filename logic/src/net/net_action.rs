//! What the firmware must do with the resources it owns.

use crate::net::wifi_credentials::WifiCredentials;

/// What the firmware must do with the resources it owns. Every variant
/// is idempotent-safe to skip: the supervisor has already moved, so a
/// dropped action shows up as a stale radio rather than a wedged FSM.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NetAction {
    Nothing,
    /// Safe to refresh the AP scan cache: the captive bundle is up and
    /// the STA half is not mid-association.
    RefreshScan,
    /// Apply these credentials to the live radio without stopping it, and
    /// publish submission status `Trying`.
    ApplyCreds(WifiCredentials),
    /// Association succeeded from a captive phase. Persist the creds to
    /// NVS, publish `Connected`, linger for the captive page's poll, drop
    /// the captive bundle, switch the radio to STA-only, and bring up the
    /// dashboard and mDNS.
    PromoteToSta(WifiCredentials),
    /// First association of an STA-only session: the netif is live, so
    /// mDNS can be taken now.
    StartMdns,
    /// The association budget expired; publish submission status `Failed`
    /// so the captive page shows the error.
    MarkSubmissionFailed,
    /// Drop the dashboard and mDNS, switch the radio to Mixed carrying
    /// these credentials so the STA half keeps retrying, and mount the
    /// captive bundle.
    FallbackToCaptive(WifiCredentials),
    /// `/wifi-reset`: drop the live association and return to a bare
    /// captive AP with no credentials on the radio.
    ForceCaptive,
}
