//! Network-supervisor state machine. Host-testable; generic over the server
//! handle types so the ESP-side wrapper can plug in `EspHttpServer<'static>`
//! and `(EspHttpServer<'static>, DnsHandle)` while tests use trivial markers.
//!
//! Driven by periodic supervisor ticks (1 Hz on the device). Each tick
//! folds a "WiFi reports associated" or "WiFi reports not associated"
//! event into the phase, building a fresh server (via the caller-supplied
//! closure) or dropping an old one as needed. The caller passes a
//! monotonic `now: Duration` (any epoch — boot is convenient) so the
//! state machine can express its grace windows as `Duration` rather than
//! tick counts. Status changes are reported back via the return value so
//! the caller can mirror them to readers (LCD).

use core::time::Duration;

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum NetStatus {
    Captive = 0,
    Connecting = 1,
    Host = 2,
}

/// Why `build_host` is being called. The Captive→Host path runs through
/// `CaptiveHandoff` and then needs the caller to drop the AP (Mixed →
/// Sta) before mounting the host server. The Bootstrap path doesn't —
/// `main` already started the STA at boot.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum HostTransition {
    FromBootstrap,
    FromCaptive,
}

/// `H` is the dashboard server handle, `C` is the captive bundle (server +
/// DNS responder on the real device). Both are dropped when their phase
/// ends — that's how the ESP-side handles get torn down.
///
/// All in-phase timers are stored as `Duration` since some monotonic
/// epoch (the caller's choice — boot is convenient). `CaptiveHandoff` is
/// the brief window after the STA associates while we're still serving
/// the captive AP, so the user's browser (polling `/status`) sees the
/// `Connected` lifecycle before the AP disappears.
pub enum Phase<H, C> {
    Bootstrap {
        entered_at: Duration,
    },
    Host {
        server: H,
        disconnected_since: Option<Duration>,
    },
    Captive {
        bundle: C,
    },
    CaptiveHandoff {
        bundle: C,
        sta_associated_at: Duration,
    },
}

impl<H, C> Phase<H, C> {
    /// Fresh `Bootstrap` phase entering at `now`. Caller's monotonic
    /// epoch — `Duration::ZERO` is fine if you measure from this point.
    pub fn bootstrap(now: Duration) -> Self {
        Self::Bootstrap { entered_at: now }
    }

    pub fn status(&self) -> NetStatus {
        match self {
            Phase::Bootstrap { .. }
            | Phase::Host {
                disconnected_since: Some(_),
                ..
            } => NetStatus::Connecting,
            Phase::Host {
                disconnected_since: None,
                ..
            } => NetStatus::Host,
            Phase::Captive { .. } | Phase::CaptiveHandoff { .. } => NetStatus::Captive,
        }
    }

    /// Fold a "WiFi associated" tick.
    ///
    /// - `Host`: clear the disconnect timer; reuse the server.
    /// - `Bootstrap`: build host with `FromBootstrap`.
    /// - `Captive`: enter `CaptiveHandoff { sta_associated_at: now }` —
    ///   keeps the AP up so the captive page can observe the lifecycle
    ///   flip to `Connected`. The handoff completes on a subsequent tick.
    /// - `CaptiveHandoff`: once `now - sta_associated_at >= handoff_grace`,
    ///   drop the bundle (server stop, DNS join), call `build_host` with
    ///   `FromCaptive`, and transition to `Host`.
    pub fn tick_connected(
        self,
        now: Duration,
        handoff_grace: Duration,
        build_host: impl FnOnce(HostTransition) -> H,
    ) -> Self {
        match self {
            Phase::Host { server, .. } => Phase::Host {
                server,
                disconnected_since: None,
            },
            Phase::Bootstrap { .. } => Phase::Host {
                server: build_host(HostTransition::FromBootstrap),
                disconnected_since: None,
            },
            Phase::Captive { bundle } => Phase::CaptiveHandoff {
                bundle,
                sta_associated_at: now,
            },
            Phase::CaptiveHandoff {
                bundle,
                sta_associated_at,
            } => {
                if now.saturating_sub(sta_associated_at) >= handoff_grace {
                    drop(bundle);
                    Phase::Host {
                        server: build_host(HostTransition::FromCaptive),
                        disconnected_since: None,
                    }
                } else {
                    Phase::CaptiveHandoff {
                        bundle,
                        sta_associated_at,
                    }
                }
            }
        }
    }

    /// Fold a "WiFi not associated" tick.
    ///
    /// - `!has_creds`: jump straight to captive (or stay in it). The
    ///   grace windows don't apply because there's nothing the STA could
    ///   associate with.
    /// - `Host { disconnected_since: None }` stamps `now`; later ticks
    ///   keep the stamp and check against `captive_grace`.
    /// - `Bootstrap`: stays until `now - entered_at >= captive_grace`.
    /// - `Captive`: stays put — the in-flight portal isn't rebuilt.
    /// - `CaptiveHandoff`: STA dropped mid-handoff; fall back to Captive
    ///   with the bundle preserved (no AP blip).
    pub fn tick_disconnected(
        self,
        now: Duration,
        has_creds: bool,
        captive_grace: Duration,
        build_captive: impl FnOnce() -> C,
    ) -> Self {
        if !has_creds {
            return match self {
                Phase::Captive { bundle } => Phase::Captive { bundle },
                Phase::CaptiveHandoff { bundle, .. } => Phase::Captive { bundle },
                _ => Phase::Captive {
                    bundle: build_captive(),
                },
            };
        }
        match self {
            Phase::Host {
                server,
                disconnected_since,
            } => {
                let since = disconnected_since.unwrap_or(now);
                if now.saturating_sub(since) >= captive_grace {
                    drop(server);
                    Phase::Captive {
                        bundle: build_captive(),
                    }
                } else {
                    Phase::Host {
                        server,
                        disconnected_since: Some(since),
                    }
                }
            }
            Phase::Bootstrap { entered_at } => {
                if now.saturating_sub(entered_at) >= captive_grace {
                    Phase::Captive {
                        bundle: build_captive(),
                    }
                } else {
                    Phase::Bootstrap { entered_at }
                }
            }
            Phase::Captive { bundle } => Phase::Captive { bundle },
            Phase::CaptiveHandoff { bundle, .. } => Phase::Captive { bundle },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Trivial server handle for tests. Carries an id so we can assert
    /// "the same server was reused" vs "a fresh one was built". Drop is
    /// noisy via `Cell<Vec<u32>>` would couple tests; instead we just
    /// compare ids by value.
    type TestPhase = Phase<u32, u32>;

    /// All grace windows are 3 s in tests; ticks happen at integer seconds.
    const GRACE: Duration = Duration::from_secs(3);
    const HANDOFF: Duration = Duration::from_secs(2);

    fn t(s: u64) -> Duration {
        Duration::from_secs(s)
    }

    #[test]
    fn bootstrap_status_is_connecting() {
        let p: TestPhase = Phase::bootstrap(t(0));
        assert_eq!(p.status(), NetStatus::Connecting);
    }

    #[test]
    fn bootstrap_stays_until_grace_elapsed_then_falls_to_captive() {
        // Bootstrap entered at t=0, GRACE=3s. Disconnected ticks at t=1, 2
        // stay; t=3 trips (3 - 0 >= 3).
        let mut p: TestPhase = Phase::bootstrap(t(0));
        p = p.tick_disconnected(t(1), true, GRACE, || 99);
        assert!(matches!(p, Phase::Bootstrap { entered_at } if entered_at == t(0)));
        assert_eq!(p.status(), NetStatus::Connecting);
        p = p.tick_disconnected(t(2), true, GRACE, || 99);
        assert!(matches!(p, Phase::Bootstrap { .. }));
        p = p.tick_disconnected(t(3), true, GRACE, || 99);
        assert!(matches!(p, Phase::Captive { bundle: 99 }));
        assert_eq!(p.status(), NetStatus::Captive);
    }

    #[test]
    fn no_creds_jumps_straight_to_captive_ignoring_grace() {
        let p: TestPhase = Phase::bootstrap(t(0));
        let p = p.tick_disconnected(t(0), false, GRACE, || 7);
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn captive_without_creds_is_idempotent_and_does_not_rebuild() {
        let p: TestPhase = Phase::Captive { bundle: 7 };
        let p = p.tick_disconnected(t(0), false, GRACE, || panic!("must not rebuild captive"));
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn captive_with_creds_stays_captive_does_not_rebuild() {
        let p: TestPhase = Phase::Captive { bundle: 7 };
        let p = p.tick_disconnected(t(0), true, GRACE, || panic!("must not rebuild captive"));
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn bootstrap_to_host_on_connected() {
        let p: TestPhase = Phase::bootstrap(t(0));
        let p = p.tick_connected(t(0), HANDOFF, |reason| {
            assert_eq!(reason, HostTransition::FromBootstrap);
            42
        });
        assert!(matches!(
            p,
            Phase::Host {
                server: 42,
                disconnected_since: None
            }
        ));
        assert_eq!(p.status(), NetStatus::Host);
    }

    #[test]
    fn host_disconnect_stamps_then_falls_to_captive_after_grace() {
        // First disconnect at t=10 stamps disconnected_since=Some(10).
        // GRACE=3s: next ticks at t=11, 12 stay; t=13 trips.
        let mut p: TestPhase = Phase::Host {
            server: 10,
            disconnected_since: None,
        };
        p = p.tick_disconnected(t(10), true, GRACE, || panic!("not yet"));
        assert!(matches!(
            p,
            Phase::Host {
                server: 10,
                disconnected_since: Some(s),
            } if s == t(10)
        ));
        assert_eq!(p.status(), NetStatus::Connecting);
        p = p.tick_disconnected(t(11), true, GRACE, || panic!("not yet"));
        p = p.tick_disconnected(t(12), true, GRACE, || panic!("not yet"));
        assert!(matches!(p, Phase::Host { .. }));
        // t=13 - since=10 = 3s, >= GRACE → captive, drop server.
        p = p.tick_disconnected(t(13), true, GRACE, || 77);
        assert!(matches!(p, Phase::Captive { bundle: 77 }));
    }

    #[test]
    fn reassociate_within_grace_reuses_host_server() {
        let p: TestPhase = Phase::Host {
            server: 10,
            disconnected_since: Some(t(5)),
        };
        let p = p.tick_connected(t(6), HANDOFF, |_| panic!("must reuse existing server"));
        assert!(matches!(
            p,
            Phase::Host {
                server: 10,
                disconnected_since: None
            }
        ));
    }

    #[test]
    fn host_no_grace_is_idempotent_on_connected_tick() {
        let p: TestPhase = Phase::Host {
            server: 10,
            disconnected_since: None,
        };
        let p = p.tick_connected(t(0), HANDOFF, |_| panic!("must not rebuild"));
        assert!(matches!(
            p,
            Phase::Host {
                server: 10,
                disconnected_since: None
            }
        ));
    }

    #[test]
    fn captive_to_handoff_on_connected_does_not_build_host_yet() {
        let p: TestPhase = Phase::Captive { bundle: 7 };
        // Bundle preserved during the handoff window so the captive page
        // can observe `Connected` via `/status`. build_host must NOT be
        // called yet.
        let p = p.tick_connected(t(5), HANDOFF, |_| panic!("must not build host yet"));
        assert!(matches!(
            p,
            Phase::CaptiveHandoff {
                bundle: 7,
                sta_associated_at: s,
            } if s == t(5)
        ));
        assert_eq!(p.status(), NetStatus::Captive);
    }

    #[test]
    fn captive_handoff_holds_then_falls_to_host_after_grace() {
        // HANDOFF=2s: entered at t=5; tick at t=6 still 1s, hold; tick at
        // t=7 elapsed=2s, drop bundle and build host with FromCaptive.
        let p: TestPhase = Phase::Captive { bundle: 7 };
        let p = p.tick_connected(t(5), HANDOFF, |_| panic!("not yet"));
        assert!(matches!(p, Phase::CaptiveHandoff { .. }));
        let p = p.tick_connected(t(6), HANDOFF, |_| panic!("not yet"));
        assert!(matches!(
            p,
            Phase::CaptiveHandoff {
                bundle: 7,
                sta_associated_at: s,
            } if s == t(5)
        ));
        let p = p.tick_connected(t(7), HANDOFF, |reason| {
            assert_eq!(reason, HostTransition::FromCaptive);
            42
        });
        assert!(matches!(
            p,
            Phase::Host {
                server: 42,
                disconnected_since: None
            }
        ));
    }

    #[test]
    fn handoff_disconnected_falls_back_to_captive_preserving_bundle() {
        let p: TestPhase = Phase::CaptiveHandoff {
            bundle: 7,
            sta_associated_at: t(5),
        };
        let p = p.tick_disconnected(t(6), true, GRACE, || panic!("must not rebuild captive"));
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn no_creds_from_host_drops_server_and_mounts_captive() {
        let p: TestPhase = Phase::Host {
            server: 10,
            disconnected_since: None,
        };
        let p = p.tick_disconnected(t(0), false, GRACE, || 77);
        assert!(matches!(p, Phase::Captive { bundle: 77 }));
    }

    #[test]
    fn no_creds_with_zero_grace_still_just_mounts_captive() {
        let p: TestPhase = Phase::Bootstrap {
            entered_at: t(1000),
        };
        let p = p.tick_disconnected(t(1001), false, Duration::ZERO, || 7);
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn host_disconnect_with_zero_grace_falls_to_captive_immediately() {
        // GRACE=0s: any disconnected tick trips because 0 - 0 >= 0.
        let p: TestPhase = Phase::Host {
            server: 10,
            disconnected_since: None,
        };
        let p = p.tick_disconnected(t(42), true, Duration::ZERO, || 77);
        assert!(matches!(p, Phase::Captive { bundle: 77 }));
    }
}
