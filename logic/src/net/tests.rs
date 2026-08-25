use super::*;

fn creds(ssid: &str) -> WifiCredentials {
    WifiCredentials::new(ssid, "password1")
}

/// One supervisor tick, matching the firmware's 1 Hz loop.
const TICK: Duration = Duration::from_secs(1);

fn secs(n: u64) -> Duration {
    Duration::from_secs(n)
}

/// A quiet tick: nothing submitted, no reset, not associated.
fn idle(now: Duration) -> NetPoll {
    NetPoll {
        now,
        ..NetPoll::default()
    }
}

fn assoc(now: Duration) -> NetPoll {
    NetPoll {
        now,
        associated: true,
        ..NetPoll::default()
    }
}

fn save(now: Duration, ssid: &str) -> NetPoll {
    NetPoll {
        now,
        submitted: Some(creds(ssid)),
        ..NetPoll::default()
    }
}

/// Cold-boot supervisor with no stored credentials.
fn captive() -> NetSupervisor {
    let s = NetSupervisor::new(None, Duration::ZERO);
    assert_eq!(*s.phase(), NetPhase::CaptiveIdle);
    s
}

/// Supervisor already serving the dashboard on `ssid`.
fn serving(ssid: &str) -> NetSupervisor {
    let mut s = NetSupervisor::new(Some(creds(ssid)), Duration::ZERO);
    assert_eq!(s.tick(assoc(TICK)), NetAction::StartMdns);
    assert!(matches!(
        s.phase(),
        NetPhase::StaServing {
            link: LinkState::Up,
            ..
        }
    ));
    s
}

#[test]
fn boot_without_creds_is_captive_and_boot_with_creds_connects() {
    assert_eq!(*captive().phase(), NetPhase::CaptiveIdle);

    let s = NetSupervisor::new(Some(creds("home")), secs(7));
    assert_eq!(
        *s.phase(),
        NetPhase::StaConnecting {
            creds: creds("home"),
            session_start: secs(7),
        }
    );
}

#[test]
fn captive_idle_refreshes_scan_and_holds() {
    let mut s = captive();
    for t in 1..5 {
        assert_eq!(s.tick(idle(secs(t))), NetAction::RefreshScan);
        assert_eq!(*s.phase(), NetPhase::CaptiveIdle);
    }
    // CaptiveIdle never attempts association: it has no credentials to
    // attempt one with.
    assert!(!NetPhase::CaptiveIdle.polls_association());
}

#[test]
fn save_applies_creds_and_starts_the_budget() {
    let mut s = captive();
    assert_eq!(s.tick(save(secs(3), "home")), NetAction::ApplyCreds(creds("home")));
    assert_eq!(
        *s.phase(),
        NetPhase::CaptiveTrying {
            creds: creds("home"),
            since: secs(3),
        }
    );
}

#[test]
fn trying_times_out_at_exactly_the_budget() {
    let mut s = captive();
    s.tick(save(Duration::ZERO, "home"));

    // One second short of the budget: still waiting.
    let last_ok = CAPTIVE_TRYING_TIMEOUT - TICK;
    assert_eq!(s.tick(idle(last_ok)), NetAction::Nothing);
    assert!(matches!(s.phase(), NetPhase::CaptiveTrying { .. }));

    // At the budget: failed, and the credentials are dropped so the
    // captive page is the source of truth on retry.
    assert_eq!(
        s.tick(idle(CAPTIVE_TRYING_TIMEOUT)),
        NetAction::MarkSubmissionFailed
    );
    assert_eq!(
        *s.phase(),
        NetPhase::CaptiveIdle,
        "the failed credentials must be dropped, not carried"
    );
}

#[test]
fn a_second_save_overrides_the_one_in_flight_and_restarts_the_budget() {
    let mut s = captive();
    s.tick(save(Duration::ZERO, "first"));
    assert_eq!(
        s.tick(save(secs(5), "second")),
        NetAction::ApplyCreds(creds("second"))
    );
    assert_eq!(
        *s.phase(),
        NetPhase::CaptiveTrying {
            creds: creds("second"),
            since: secs(5),
        }
    );
    // Budget runs from the *second* submission: the original deadline
    // (t=20) must pass without a failure.
    assert_eq!(s.tick(idle(CAPTIVE_TRYING_TIMEOUT)), NetAction::Nothing);
    assert_eq!(
        s.tick(idle(secs(5) + CAPTIVE_TRYING_TIMEOUT)),
        NetAction::MarkSubmissionFailed
    );
}

#[test]
fn association_beats_a_save_arriving_in_the_same_tick() {
    // The ordering rule that keeps us from disconnecting from a network
    // we have just successfully joined.
    let mut s = captive();
    s.tick(save(Duration::ZERO, "home"));
    let both = NetPoll {
        now: secs(4),
        associated: true,
        submitted: Some(creds("late")),
        reset_requested: false,
    };
    assert_eq!(s.tick(both), NetAction::PromoteToSta(creds("home")));
    assert_eq!(
        *s.phase(),
        NetPhase::StaServing {
            creds: creds("home"),
            link: LinkState::Up,
        }
    );
}

#[test]
fn connecting_falls_back_only_after_the_full_grace() {
    let mut s = NetSupervisor::new(Some(creds("home")), Duration::ZERO);
    assert_eq!(s.tick(idle(CAPTIVE_AFTER_DISCONNECT - TICK)), NetAction::Nothing);
    assert!(matches!(s.phase(), NetPhase::StaConnecting { .. }));
    assert_eq!(
        s.tick(idle(CAPTIVE_AFTER_DISCONNECT)),
        NetAction::FallbackToCaptive(creds("home"))
    );
    // Creds are carried over so the STA half keeps retrying behind the
    // captive portal.
    assert_eq!(
        *s.phase(),
        NetPhase::CaptiveFallbackRetrying {
            creds: creds("home")
        }
    );
}

#[test]
fn serving_counts_the_grace_from_the_first_miss_not_the_latest() {
    let mut s = serving("home");
    // Goes down at t=100 and stays down.
    assert_eq!(s.tick(idle(secs(100))), NetAction::Nothing);
    assert_eq!(
        *s.phase(),
        NetPhase::StaServing {
            creds: creds("home"),
            link: LinkState::Down { since: secs(100) },
        }
    );
    // Later misses must not push the deadline out.
    for t in [200, 500, 1000] {
        s.tick(idle(secs(t)));
        assert_eq!(
            *s.phase(),
            NetPhase::StaServing {
                creds: creds("home"),
                link: LinkState::Down { since: secs(100) },
            },
            "grace restarted at t={t}"
        );
    }
    let deadline = secs(100) + CAPTIVE_AFTER_DISCONNECT;
    assert_eq!(s.tick(idle(deadline - TICK)), NetAction::Nothing);
    assert_eq!(
        s.tick(idle(deadline)),
        NetAction::FallbackToCaptive(creds("home"))
    );
}

#[test]
fn a_re_association_clears_the_down_timer() {
    let mut s = serving("home");
    s.tick(idle(secs(100)));
    assert!(matches!(
        s.phase(),
        NetPhase::StaServing {
            link: LinkState::Down { .. },
            ..
        }
    ));
    s.tick(assoc(secs(200)));
    assert!(matches!(
        s.phase(),
        NetPhase::StaServing {
            link: LinkState::Up,
            ..
        }
    ));
    // Dropping again starts a fresh grace from the new moment, so the
    // old deadline passes quietly.
    s.tick(idle(secs(300)));
    let old_deadline = secs(100) + CAPTIVE_AFTER_DISCONNECT;
    assert_eq!(s.tick(idle(old_deadline)), NetAction::Nothing);
}

#[test]
fn fallback_retrying_promotes_on_association() {
    let mut s = serving("home");
    s.tick(idle(secs(10)));
    let deadline = secs(10) + CAPTIVE_AFTER_DISCONNECT;
    assert_eq!(
        s.tick(idle(deadline)),
        NetAction::FallbackToCaptive(creds("home"))
    );
    // The background STA half gets there eventually.
    assert_eq!(
        s.tick(assoc(deadline + TICK)),
        NetAction::PromoteToSta(creds("home"))
    );
    assert!(matches!(
        s.phase(),
        NetPhase::StaServing {
            link: LinkState::Up,
            ..
        }
    ));
}

#[test]
fn fallback_retrying_accepts_new_creds_over_the_carried_ones() {
    let mut s = serving("old");
    s.tick(idle(Duration::ZERO));
    s.tick(idle(CAPTIVE_AFTER_DISCONNECT));
    assert!(matches!(s.phase(), NetPhase::CaptiveFallbackRetrying { .. }));

    let t = CAPTIVE_AFTER_DISCONNECT + TICK;
    assert_eq!(s.tick(save(t, "new")), NetAction::ApplyCreds(creds("new")));
    assert_eq!(
        *s.phase(),
        NetPhase::CaptiveTrying {
            creds: creds("new"),
            since: t,
        }
    );
}

#[test]
fn reset_returns_sta_phases_to_a_bare_captive_ap() {
    for mut s in [serving("home"), NetSupervisor::new(Some(creds("home")), Duration::ZERO)] {
        let p = NetPoll {
            now: secs(50),
            reset_requested: true,
            ..NetPoll::default()
        };
        assert_eq!(s.tick(p), NetAction::ForceCaptive);
        assert_eq!(
            *s.phase(),
            NetPhase::CaptiveIdle,
            "reset must land in the one phase that holds no credentials"
        );
    }
}

#[test]
fn reset_from_a_captive_phase_is_a_no_op() {
    // `/wifi-reset` is only mounted on the dashboard, so this cannot
    // happen — but it must degrade to nothing rather than panic, which
    // is what the old `unreachable!()` did.
    let mut s = captive();
    let p = NetPoll {
        now: secs(3),
        reset_requested: true,
        ..NetPoll::default()
    };
    assert_eq!(s.tick(p), NetAction::RefreshScan);
    assert_eq!(*s.phase(), NetPhase::CaptiveIdle);
}

#[test]
fn lcd_status_and_association_polling_track_the_phase() {
    let cases = [
        (NetPhase::CaptiveIdle, NetStatus::Captive, false),
        (
            NetPhase::CaptiveTrying {
                creds: creds("a"),
                since: Duration::ZERO,
            },
            NetStatus::CaptiveTrying,
            true,
        ),
        (
            NetPhase::CaptiveFallbackRetrying { creds: creds("a") },
            NetStatus::Captive,
            true,
        ),
        (
            NetPhase::StaConnecting {
                creds: creds("a"),
                session_start: Duration::ZERO,
            },
            NetStatus::Connecting,
            true,
        ),
        (
            NetPhase::StaServing {
                creds: creds("a"),
                link: LinkState::Up,
            },
            NetStatus::Host,
            true,
        ),
        (
            NetPhase::StaServing {
                creds: creds("a"),
                link: LinkState::Down {
                    since: Duration::ZERO,
                },
            },
            NetStatus::Connecting,
            true,
        ),
    ];
    for (phase, status, polls) in cases {
        assert_eq!(phase.lcd_status(), status, "{phase:?}");
        assert_eq!(phase.polls_association(), polls, "{phase:?}");
    }
}

#[test]
fn net_status_round_trips_through_its_discriminant() {
    // The LCD reads this back out of an AtomicU8.
    for s in [
        NetStatus::Captive,
        NetStatus::CaptiveTrying,
        NetStatus::Connecting,
        NetStatus::Host,
    ] {
        assert_eq!(NetStatus::from_repr(s as u8), Some(s));
    }
}
