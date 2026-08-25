//! Where the network supervisor is, and the views derived from it.

use std::time::Duration;

use crate::net::wifi_credentials::WifiCredentials;

/// `StaServing` link status. `Up` means the radio is associated this
/// tick; `Down { since }` means it is not, and the captive-fallback timer
/// counts from `since`. The variant encodes "we only need a timer while
/// disconnected" — no always-present `last_assoc` whose meaning depends
/// on a sibling boolean.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LinkState {
    Up,
    Down { since: Duration },
}

/// Where the supervisor is. See `src/net_fsm.md` for the state table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NetPhase {
    /// Captive AP up, no submission in flight. Covers cold boot (no
    /// stored creds) and post-timeout retry, where the failed attempt's
    /// creds were intentionally dropped so the captive page is the source
    /// of truth. The only phase without creds.
    CaptiveIdle,
    /// Submitted creds are on the radio and association is in flight,
    /// with at most [`CAPTIVE_TRYING_TIMEOUT`] to succeed.
    CaptiveTrying {
        creds: WifiCredentials,
        since: Duration,
    },
    /// STA→captive carry-over: the radio is Mixed with the last known-good
    /// creds and the STA half retries in the background, while the captive
    /// page lets the user enter new ones if they need to.
    CaptiveFallbackRetrying { creds: WifiCredentials },
    /// STA-only, never associated this session. The dashboard is up but
    /// mDNS is not — mDNS needs the netif live, only true once associated.
    StaConnecting {
        creds: WifiCredentials,
        session_start: Duration,
    },
    /// STA-only, dashboard + mDNS up. mDNS stays up across `Down` windows
    /// since it is valid again on re-link without re-init.
    StaServing {
        creds: WifiCredentials,
        link: LinkState,
    },
}

/// LCD-visible status, derived from the phase.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, strum::FromRepr)]
pub enum NetStatus {
    Captive = 0,
    CaptiveTrying = 1,
    Connecting = 2,
    Host = 3,
}

impl NetPhase {
    pub fn lcd_status(&self) -> NetStatus {
        match self {
            Self::CaptiveIdle | Self::CaptiveFallbackRetrying { .. } => NetStatus::Captive,
            Self::CaptiveTrying { .. } => NetStatus::CaptiveTrying,
            Self::StaConnecting { .. } => NetStatus::Connecting,
            Self::StaServing {
                link: LinkState::Up,
                ..
            } => NetStatus::Host,
            Self::StaServing {
                link: LinkState::Down { .. },
                ..
            } => NetStatus::Connecting,
        }
    }

    /// Whether this phase attempts association on a tick. `CaptiveIdle`
    /// does not: it either has no credentials at all (cold boot) or the
    /// last attempt's were deliberately dropped, so a connect attempt
    /// would only produce per-second log noise.
    pub fn polls_association(&self) -> bool {
        !matches!(self, Self::CaptiveIdle)
    }
}
