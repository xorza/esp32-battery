//! Network-supervisor state machine. Host-testable; generic over the server
//! handle types so the ESP-side wrapper can plug in `EspHttpServer<'static>`
//! and `(EspHttpServer<'static>, DnsHandle)` while tests use trivial markers.
//!
//! Driven by 1 Hz supervisor ticks. The two transition entry points fold a
//! "WiFi reports associated" or "WiFi reports not associated" event into the
//! phase, building a fresh server (via the caller-supplied closure) or
//! dropping an old one as needed. Status changes are reported back via the
//! return value so the caller can mirror them to readers (LCD).

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum NetStatus {
    Captive = 0,
    Connecting = 1,
    Host = 2,
}

/// `H` is the dashboard server handle, `C` is the captive bundle (server +
/// DNS responder on the real device). Both are dropped when their phase
/// ends — that's how the ESP-side handles get torn down.
pub enum Phase<H, C> {
    Bootstrap { ticks: u32 },
    Host { server: H, grace: Option<u32> },
    Captive { bundle: C },
}

impl<H, C> Phase<H, C> {
    pub fn bootstrap() -> Self {
        Self::Bootstrap { ticks: 0 }
    }

    pub fn status(&self) -> NetStatus {
        match self {
            Phase::Bootstrap { .. } | Phase::Host { grace: Some(_), .. } => NetStatus::Connecting,
            Phase::Host { grace: None, .. } => NetStatus::Host,
            Phase::Captive { .. } => NetStatus::Captive,
        }
    }

    /// Fold a "WiFi associated" tick. Reuses an in-grace `Host` server if
    /// one is around; otherwise builds a fresh one when transitioning out
    /// of `Bootstrap` or `Captive`.
    pub fn tick_connected(self, build_host: impl FnOnce() -> H) -> Self {
        let server = match self {
            Phase::Host {
                server,
                grace: None,
            } => {
                return Phase::Host {
                    server,
                    grace: None,
                };
            }
            Phase::Host {
                server,
                grace: Some(_),
            } => server,
            Phase::Bootstrap { .. } | Phase::Captive { .. } => build_host(),
        };
        Phase::Host {
            server,
            grace: None,
        }
    }

    /// Fold a "WiFi not associated" tick.
    ///
    /// - `!has_creds`: jump straight to captive (or stay in it). The
    ///   bootstrap / grace counters don't apply because there's nothing
    ///   the STA could associate with.
    /// - `Host { grace: None }` → `Host { grace: Some(1) }`.
    /// - `Host { grace: Some(n) }` / `Bootstrap`: bump; mount captive once
    ///   the counter hits `grace_ticks`, dropping any leftover host server.
    /// - `Captive`: stays put — the in-flight portal isn't rebuilt.
    pub fn tick_disconnected(
        self,
        has_creds: bool,
        grace_ticks: u32,
        build_captive: impl FnOnce() -> C,
    ) -> Self {
        if !has_creds {
            return match self {
                Phase::Captive { bundle } => Phase::Captive { bundle },
                _ => Phase::Captive {
                    bundle: build_captive(),
                },
            };
        }
        match self {
            Phase::Host { server, grace } => {
                let next = grace.unwrap_or(0).saturating_add(1);
                if next >= grace_ticks {
                    drop(server);
                    Phase::Captive {
                        bundle: build_captive(),
                    }
                } else {
                    Phase::Host {
                        server,
                        grace: Some(next),
                    }
                }
            }
            Phase::Bootstrap { ticks } => {
                let next = ticks.saturating_add(1);
                if next >= grace_ticks {
                    Phase::Captive {
                        bundle: build_captive(),
                    }
                } else {
                    Phase::Bootstrap { ticks: next }
                }
            }
            Phase::Captive { bundle } => Phase::Captive { bundle },
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

    const GRACE: u32 = 3;

    #[test]
    fn bootstrap_status_is_connecting() {
        let p: TestPhase = Phase::bootstrap();
        assert_eq!(p.status(), NetStatus::Connecting);
    }

    #[test]
    fn bootstrap_accumulates_ticks_until_captive_when_creds_present() {
        let mut p: TestPhase = Phase::bootstrap();
        // GRACE=3: tick 1 → ticks=1, tick 2 → ticks=2, tick 3 → captive.
        p = p.tick_disconnected(true, GRACE, || 99);
        assert!(matches!(p, Phase::Bootstrap { ticks: 1 }));
        assert_eq!(p.status(), NetStatus::Connecting);
        p = p.tick_disconnected(true, GRACE, || 99);
        assert!(matches!(p, Phase::Bootstrap { ticks: 2 }));
        p = p.tick_disconnected(true, GRACE, || 99);
        assert!(matches!(p, Phase::Captive { bundle: 99 }));
        assert_eq!(p.status(), NetStatus::Captive);
    }

    #[test]
    fn no_creds_jumps_straight_to_captive_ignoring_ticks() {
        // From Bootstrap with ticks accumulated, !has_creds shortcuts.
        let p: TestPhase = Phase::Bootstrap { ticks: 0 };
        let p = p.tick_disconnected(false, GRACE, || 7);
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn captive_without_creds_is_idempotent_and_does_not_rebuild() {
        let p: TestPhase = Phase::Captive { bundle: 7 };
        // build_captive panics — proving it isn't called again.
        let p = p.tick_disconnected(false, GRACE, || panic!("must not rebuild captive"));
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn captive_with_creds_stays_captive_does_not_rebuild() {
        let p: TestPhase = Phase::Captive { bundle: 7 };
        let p = p.tick_disconnected(true, GRACE, || panic!("must not rebuild captive"));
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn bootstrap_to_host_on_connected() {
        let p: TestPhase = Phase::bootstrap();
        let p = p.tick_connected(|| 42);
        assert!(matches!(
            p,
            Phase::Host {
                server: 42,
                grace: None
            }
        ));
        assert_eq!(p.status(), NetStatus::Host);
    }

    #[test]
    fn host_grace_counter_increments_then_falls_to_captive() {
        // Start in Host{grace:None}. Each disconnected tick bumps grace.
        // GRACE=3: None → Some(1) → Some(2) → captive at next=3.
        let mut p: TestPhase = Phase::Host {
            server: 10,
            grace: None,
        };
        p = p.tick_disconnected(true, GRACE, || panic!("not yet"));
        assert!(matches!(
            p,
            Phase::Host {
                server: 10,
                grace: Some(1)
            }
        ));
        assert_eq!(p.status(), NetStatus::Connecting);
        p = p.tick_disconnected(true, GRACE, || panic!("not yet"));
        assert!(matches!(
            p,
            Phase::Host {
                server: 10,
                grace: Some(2)
            }
        ));
        // Third disconnected tick — next=3 >= GRACE=3 → mount captive,
        // drop the host server.
        p = p.tick_disconnected(true, GRACE, || 77);
        assert!(matches!(p, Phase::Captive { bundle: 77 }));
    }

    #[test]
    fn reassociate_within_grace_reuses_host_server() {
        let p: TestPhase = Phase::Host {
            server: 10,
            grace: Some(2),
        };
        // build_host must NOT be called — server is reused.
        let p = p.tick_connected(|| panic!("must reuse existing server"));
        assert!(matches!(
            p,
            Phase::Host {
                server: 10,
                grace: None
            }
        ));
    }

    #[test]
    fn host_no_grace_is_idempotent_on_connected_tick() {
        let p: TestPhase = Phase::Host {
            server: 10,
            grace: None,
        };
        let p = p.tick_connected(|| panic!("must not rebuild"));
        assert!(matches!(
            p,
            Phase::Host {
                server: 10,
                grace: None
            }
        ));
    }

    #[test]
    fn captive_to_host_on_connected_builds_fresh() {
        let p: TestPhase = Phase::Captive { bundle: 7 };
        let p = p.tick_connected(|| 42);
        assert!(matches!(
            p,
            Phase::Host {
                server: 42,
                grace: None
            }
        ));
    }

    #[test]
    fn no_creds_from_host_drops_server_and_mounts_captive() {
        // !has_creds from Host: drops the host server (we don't observe it
        // here, but the type forces the drop), mounts captive.
        let p: TestPhase = Phase::Host {
            server: 10,
            grace: None,
        };
        let p = p.tick_disconnected(false, GRACE, || 77);
        assert!(matches!(p, Phase::Captive { bundle: 77 }));
    }

    #[test]
    fn no_creds_with_zero_grace_still_just_mounts_captive() {
        // grace_ticks is irrelevant when !has_creds.
        let p: TestPhase = Phase::Bootstrap { ticks: 1000 };
        let p = p.tick_disconnected(false, 0, || 7);
        assert!(matches!(p, Phase::Captive { bundle: 7 }));
    }

    #[test]
    fn grace_one_falls_to_captive_on_first_disconnected_tick() {
        // grace_ticks=1 means the first disconnected tick already trips
        // (next = 0+1 = 1, 1 >= 1).
        let p: TestPhase = Phase::Host {
            server: 10,
            grace: None,
        };
        let p = p.tick_disconnected(true, 1, || 77);
        assert!(matches!(p, Phase::Captive { bundle: 77 }));
    }
}
