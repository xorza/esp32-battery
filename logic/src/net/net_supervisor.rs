//! The network supervisor: one pure transition per tick.

use std::time::Duration;

use crate::net::net_action::NetAction;
use crate::net::net_phase::{LinkState, NetPhase};
use crate::net::net_poll::NetPoll;
use crate::net::wifi_credentials::WifiCredentials;
use crate::net::{CAPTIVE_AFTER_DISCONNECT, CAPTIVE_TRYING_TIMEOUT};

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
