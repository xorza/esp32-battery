//! Network supervisor: a flat state machine over the WiFi lifecycle.
//!
//! The phase alone determines radio mode (STA-only vs Mixed AP+STA),
//! which servers should be alive (dashboard vs captive bundle), what the
//! LCD shows, and which transitions are legal. Each variant carries only
//! the data meaningful to it — no `Option`s used as state flags, and no
//! illegal combinations to represent.
//!
//! Pure logic: no I/O, no clock, no radio. The firmware gathers a
//! [`NetPoll`], calls [`NetSupervisor::tick`], and performs the returned
//! [`NetAction`] against the resources it owns. Timing policy (the 20 s
//! association budget, the 2 h fallback grace) lives here so it is
//! testable on the host; resource ownership stays in firmware.

use std::time::Duration;

pub mod wifi_credentials;

use wifi_credentials::WifiCredentials;

/// How long `associated == false` may persist before the dashboard comes
/// down and the captive AP takes over. The AP is a fallback for "the
/// saved creds no longer work" (rotated password, SSID gone), so the wait
/// is long enough that a real outage of the user's router — ISP reboot,
/// scheduled maintenance — doesn't flap us into captive mode and break
/// the dashboard for everyone on the LAN.
pub const CAPTIVE_AFTER_DISCONNECT: Duration = Duration::from_secs(2 * 60 * 60);

/// How long the captive page's "Connecting…" spinner may run before the
/// submitted credentials are declared a failure and the user gets to
/// re-enter them. ESP-IDF associates good creds in 3–8 s typically; 20 s
/// is comfortably past that.
pub const CAPTIVE_TRYING_TIMEOUT: Duration = Duration::from_secs(20);

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

/// One tick's view of the world.
#[derive(Clone, Debug, Default)]
pub struct NetPoll {
    /// Monotonic uptime. Compared against the phases' own timestamps, so
    /// only differences matter.
    pub now: Duration,
    /// Result of this tick's association attempt. Meaningless — and not
    /// gathered — where [`NetPhase::polls_association`] is false.
    pub associated: bool,
    /// Credentials drained from the captive `/save` mailbox, if any.
    pub submitted: Option<WifiCredentials>,
    /// `/wifi-reset` was raised since the last tick.
    pub reset_requested: bool,
}

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

/// Result of one pure transition. Named rather than returned as a tuple
/// so every arm of `step` has to say what phase it leaves behind.
#[derive(Debug)]
struct Step {
    phase: NetPhase,
    action: NetAction,
}

#[derive(Debug)]
pub struct NetSupervisor {
    phase: NetPhase,
}

impl NetSupervisor {
    /// Boot into STA if credentials were loaded from NVS, captive
    /// otherwise. The caller reads [`Self::phase`] to build the matching
    /// resources.
    pub fn new(boot_creds: Option<WifiCredentials>, now: Duration) -> Self {
        let phase = match boot_creds {
            Some(creds) => NetPhase::StaConnecting {
                creds,
                session_start: now,
            },
            None => NetPhase::CaptiveIdle,
        };
        Self { phase }
    }

    pub fn phase(&self) -> &NetPhase {
        &self.phase
    }

    pub fn tick(&mut self, p: NetPoll) -> NetAction {
        // Moved out so each arm can take the credentials rather than
        // clone them; `step` is total, so the placeholder never survives.
        let phase = std::mem::replace(&mut self.phase, NetPhase::CaptiveIdle);
        let step = Self::step(phase, p);
        self.phase = step.phase;
        step.action
    }

    fn step(phase: NetPhase, p: NetPoll) -> Step {
        // `/wifi-reset` is mounted only on the dashboard, so it can only
        // be raised from an STA phase. A captive phase has no association
        // to drop, which makes it a no-op there rather than an error.
        if p.reset_requested
            && matches!(
                phase,
                NetPhase::StaConnecting { .. } | NetPhase::StaServing { .. }
            )
        {
            return Step {
                phase: NetPhase::CaptiveIdle,
                action: NetAction::ForceCaptive,
            };
        }

        match phase {
            NetPhase::CaptiveIdle => match p.submitted {
                Some(creds) => Step {
                    phase: NetPhase::CaptiveTrying {
                        creds: creds.clone(),
                        since: p.now,
                    },
                    action: NetAction::ApplyCreds(creds),
                },
                None => Step {
                    phase: NetPhase::CaptiveIdle,
                    action: NetAction::RefreshScan,
                },
            },

            NetPhase::CaptiveTrying { creds, since } => {
                // Order matters: an association that succeeded on the
                // in-flight credentials beats a `/save` that landed in the
                // same tick. The other way round we would disconnect from
                // the network we just joined.
                if p.associated {
                    return Step {
                        phase: NetPhase::StaServing {
                            creds: creds.clone(),
                            link: LinkState::Up,
                        },
                        action: NetAction::PromoteToSta(creds),
                    };
                }
                if let Some(new_creds) = p.submitted {
                    // Latest submission wins, and restarts the budget.
                    return Step {
                        phase: NetPhase::CaptiveTrying {
                            creds: new_creds.clone(),
                            since: p.now,
                        },
                        action: NetAction::ApplyCreds(new_creds),
                    };
                }
                if p.now.saturating_sub(since) >= CAPTIVE_TRYING_TIMEOUT {
                    return Step {
                        phase: NetPhase::CaptiveIdle,
                        action: NetAction::MarkSubmissionFailed,
                    };
                }
                Step {
                    phase: NetPhase::CaptiveTrying { creds, since },
                    action: NetAction::Nothing,
                }
            }

            NetPhase::CaptiveFallbackRetrying { creds } => {
                // Same ordering rule as CaptiveTrying.
                if p.associated {
                    return Step {
                        phase: NetPhase::StaServing {
                            creds: creds.clone(),
                            link: LinkState::Up,
                        },
                        action: NetAction::PromoteToSta(creds),
                    };
                }
                if let Some(new_creds) = p.submitted {
                    return Step {
                        phase: NetPhase::CaptiveTrying {
                            creds: new_creds.clone(),
                            since: p.now,
                        },
                        action: NetAction::ApplyCreds(new_creds),
                    };
                }
                Step {
                    phase: NetPhase::CaptiveFallbackRetrying { creds },
                    action: NetAction::RefreshScan,
                }
            }

            NetPhase::StaConnecting {
                creds,
                session_start,
            } => {
                if p.associated {
                    return Step {
                        phase: NetPhase::StaServing {
                            creds,
                            link: LinkState::Up,
                        },
                        action: NetAction::StartMdns,
                    };
                }
                if p.now.saturating_sub(session_start) >= CAPTIVE_AFTER_DISCONNECT {
                    return Step {
                        phase: NetPhase::CaptiveFallbackRetrying {
                            creds: creds.clone(),
                        },
                        action: NetAction::FallbackToCaptive(creds),
                    };
                }
                Step {
                    phase: NetPhase::StaConnecting {
                        creds,
                        session_start,
                    },
                    action: NetAction::Nothing,
                }
            }

            NetPhase::StaServing { creds, link } => {
                let link = match (p.associated, link) {
                    (true, _) => LinkState::Up,
                    (false, LinkState::Up) => LinkState::Down { since: p.now },
                    // Carry the original `since` — the grace counts from
                    // the moment we went down, not from the latest miss.
                    (false, down) => down,
                };
                if let LinkState::Down { since } = link
                    && p.now.saturating_sub(since) >= CAPTIVE_AFTER_DISCONNECT
                {
                    return Step {
                        phase: NetPhase::CaptiveFallbackRetrying {
                            creds: creds.clone(),
                        },
                        action: NetAction::FallbackToCaptive(creds),
                    };
                }
                Step {
                    phase: NetPhase::StaServing { creds, link },
                    action: NetAction::Nothing,
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
