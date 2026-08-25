//! The pack-temperature gate: what it refuses, what it merely waits for,
//! and what a board with no sensor gets instead.

use super::*;

/// A poll carrying `temp_c`, otherwise drift-free and healthy.
fn poll_at(s: &ChargeSupervisor, temp_c: Option<f32>) -> PollResult {
    PollResult {
        pack_temp_c: temp_c,
        ..expected_poll(s, b(OK_V, -0.1))
    }
}

/// Bring a temperature-sensing supervisor up into `Float`.
fn active_with_sensor(profile: Profile) -> ChargeSupervisor {
    bring_up(supervisor_with_temp_sensor(profile), profile.absorb_v)
}

#[test]
fn a_cold_pack_is_never_energised() {
    // Charging below freezing plates lithium, so bring-up refuses. It
    // *waits* rather than latching: the pack warms up on its own, and
    // there is no output to disable while we are refusing to start one.
    let mut s = supervisor_with_temp_sensor(lfp_4s());
    let below = CHARGE_TEMP_MIN_C - 0.1;
    for _ in 0..20 {
        assert!(matches!(s.tick(poll_at(&s, Some(below)), TICK), Action::None));
        assert_eq!(s.inhibit(), Some(InhibitReason::PackTooCold));
        assert_eq!(s.fault(), None, "a cold pack must not burn a reboot");
    }
    // Warms to the boundary — inclusive, so exactly the minimum charges.
    let a = s.tick(poll_at(&s, Some(CHARGE_TEMP_MIN_C)), TICK);
    assert!(matches!(a, Action::EnableOutput(_)));
    assert_eq!(s.inhibit(), None);
}

#[test]
fn the_charge_window_is_a_closed_interval() {
    // All four boundary points of the window, from the sourcing side where
    // there is something to stop. The ends are the spec figures themselves,
    // so a pack sitting exactly on one is inside.
    //
    // Out of range disables rather than parking: parking holds the float
    // target, which still pushes current into a discharged pack, and with a
    // frozen cell it is the charging itself that does the damage.
    let cases: [(f32, Option<FaultReason>); 4] = [
        (CHARGE_TEMP_MIN_C - 0.1, Some(FaultReason::PackTooCold)),
        (CHARGE_TEMP_MIN_C, None),
        (CHARGE_TEMP_MAX_C, None),
        (CHARGE_TEMP_MAX_C + 0.1, Some(FaultReason::PackTooHot)),
    ];
    for (temp, want) in cases {
        let mut s = active_with_sensor(lfp_4s());
        let a = s.tick(poll_at(&s, Some(temp)), TICK);
        match want {
            Some(fault) => {
                assert!(matches_disable(&a, fault), "{temp} °C");
                assert!(!s.parked(), "{temp} °C: only a dark output stops it");
            }
            None => {
                assert!(matches!(a, Action::None), "{temp} °C must be chargeable");
                assert_eq!(s.fault(), None, "{temp} °C");
            }
        }
    }
}

#[test]
fn a_fitted_sensor_that_stops_reading_fails_closed() {
    // Same rule as the INA228: we do not charge on a measurement we do not
    // have. Debounced, so a dropped read is not a fault.
    let mut s = active_with_sensor(lfp_4s());
    for _ in 0..(PACK_TEMP_STALE_TIMEOUT.as_secs() - 1) {
        assert!(matches!(s.tick(poll_at(&s, None), TICK), Action::None));
    }
    assert_eq!(s.fault(), None);
    assert!(matches_disable(
        &s.tick(poll_at(&s, None), TICK),
        FaultReason::PackTempStale
    ));
}

#[test]
fn a_board_without_a_sensor_charges_regardless() {
    // The accepted risk, pinned so it is a decision rather than a
    // side effect: `PackTemp::Absent` skips the gate entirely, and a
    // reading that somehow arrives anyway is ignored rather than obeyed.
    let mut s = active(lfp_4s());
    for temp in [None, Some(CHARGE_TEMP_MIN_C - 40.0), Some(CHARGE_TEMP_MAX_C + 40.0)] {
        for _ in 0..(PACK_TEMP_STALE_TIMEOUT.as_secs() + 5) {
            assert!(
                matches!(s.tick(poll_at(&s, temp), TICK), Action::None),
                "{temp:?}"
            );
        }
        assert_eq!(s.fault(), None, "{temp:?}");
        assert_eq!(s.inhibit(), None, "{temp:?}");
    }
}
