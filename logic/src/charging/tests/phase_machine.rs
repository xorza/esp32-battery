//! Float <-> Absorb: the hysteresis thresholds, the leaky exit window,
//! and that a steady pack stays put.

use super::*;

// All currents below are tuned to the 50 Ah pack: enter > 3 A, exit < 2.5 A.

#[test]
fn starts_in_float_at_float_voltage() {
    let s = ChargeSupervisor::new(lfp_4s());
    assert_eq!(s.phase(), Phase::Float);
    assert_approx(s.target_voltage(), 13.5);
}

#[test]
fn enters_absorb_when_charging_current_exceeds_threshold() {
    let mut s = active(lfp_4s());
    // Charging at 4 A → -4 A on the bus; threshold is 3 A.
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.phase(), Phase::Absorb);
    assert_approx(s.target_voltage(), 14.4);
}

#[test]
fn does_not_enter_absorb_at_exact_threshold() {
    // Strictly greater: 3.0 A must NOT trigger; 3.001 A must.
    let mut s = active(lfp_4s());
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    assert_eq!(s.phase(), Phase::Float);
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -3.001), TICK),
        Action::UpdateVoltage { .. }
    ));
}

#[test]
fn discharge_current_does_not_enter_absorb() {
    // 5 A discharge (positive). |I| > 3 A but it's NOT charging.
    let mut s = active(lfp_4s());
    assert!(matches!(ok_tick(&mut s, b(OK_V, 5.0), TICK), Action::None));
    assert_eq!(s.phase(), Phase::Float);
}

#[test]
fn stays_in_absorb_above_exit_threshold() {
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK); // → Absorb
    // Exit threshold is 2.5 A — anything above keeps us in absorb.
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    assert!(matches!(ok_tick(&mut s, b(OK_V, -2.7), TICK), Action::None));
    // Strictly less-than, so 2.5 stays.
    assert!(matches!(ok_tick(&mut s, b(OK_V, -2.5), TICK), Action::None));
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn exits_absorb_when_taper_drops_below_threshold() {
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK); // → Absorb
    // Sustained taper below 2.5 A for the debounce window. BUDGET-1 ticks
    // hold absorb; one more crosses → drop to float.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -2.4), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.phase(), Phase::Float);
    assert_approx(s.target_voltage(), 13.5);
}

#[test]
fn exits_absorb_when_load_pulls_current() {
    // Battery discharging mid-absorb (charger off / heavy load) for the
    // debounce window. Same path as a real taper — counter accumulates,
    // transition fires once it crosses.
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK);
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, 3.0), TICK), Action::None));
    }
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, 3.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_approx(s.target_voltage(), 13.5);
}

#[test]
fn sustained_recharge_drains_exit_gate() {
    // The gate is leaky: a *sustained* return to real charging must drain it
    // back down and block the exit (the pack is genuinely taking charge, not
    // just pulsing). Fill it to one tick shy of the window, then charge above
    // tail for a full window — symmetric drain empties it — and confirm it
    // then takes a fresh full window below tail to actually exit.
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK); // → Absorb
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    // Sustained recharge above tail for the full window drains the gate to 0.
    for _ in 0..EXIT_DEBOUNCE.as_secs() {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -4.0), TICK), Action::None));
    }
    assert_eq!(s.phase(), Phase::Absorb);
    // A fresh full window below tail is now required to exit.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -2.4), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.phase(), Phase::Float);
}

#[test]
fn pulsing_current_still_exits_absorb() {
    // Regression for the real-hardware bug: at a full pack the XY7025 delivers
    // burst pulses — instantaneous current spikes well above the tail every few
    // seconds but averages near zero. The old hard-reset gate re-armed the full
    // window on every spike and never exited, pinning the supervisor in Absorb
    // (load silently running on a "full" pack). The leaky gate must accept the
    // taper because the *average* charging current is below the 2.5 A tail.
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK); // → Absorb
    // One 6 A spike (each would reset a hard-reset debounce) per three near-zero
    // ticks: average charging current ≈ 1.5 A, comfortably below the 2.5 A tail.
    let pattern = [-6.0, -0.1, 0.2, -0.1];
    let mut exited_at = None;
    'outer: for cycle in 0..60 {
        for &i in &pattern {
            if matches!(
                ok_tick(&mut s, b(OK_V, i), TICK),
                Action::UpdateVoltage { .. }
            ) {
                exited_at = Some(cycle);
                break 'outer;
            }
        }
    }
    // Net fill is +2 ticks per 4-tick cycle, so ~30 cycles to cross the 60 s
    // window. Assert it lands in that ballpark — not "eventually", and not so
    // fast that the spikes were being ignored entirely.
    let cycle = exited_at.expect("pulsing pack never exited Absorb");
    assert!((25..40).contains(&cycle), "exited after {cycle} cycles");
    assert_eq!(s.phase(), Phase::Float);
    assert_approx(s.target_voltage(), 13.5);
}

#[test]
fn exit_debounce_does_not_accumulate_in_float() {
    // Sub-tail current while in Float must not arm the exit debounce.
    // Otherwise a Float→Absorb transition followed by an immediate dip
    // could fire a spurious Absorb→Float on the very next tick.
    let mut s = active(lfp_4s());
    // Sit in Float at sub-tail current well past the debounce window.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() * 2) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -1.0), TICK), Action::None));
    }
    // Enter Absorb, then dip below tail. Counter must start fresh — one
    // tick is not enough to transition out.
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn exit_debounce_honors_elapsed() {
    // One big-elapsed tick crossing the full debounce window must transition.
    // Mirrors the equivalent OV / absorb-timeout tests.
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK);
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -2.4), EXIT_DEBOUNCE),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.phase(), Phase::Float);
    assert_approx(s.target_voltage(), 13.5);
}

#[test]
fn hysteresis_no_flap_between_thresholds() {
    let mut s = active(lfp_4s());
    // 2.7 A sits in the hysteresis band: > exit (2.5) but < enter (3.0).
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.7), TICK), Action::None));
    }
    assert_eq!(s.phase(), Phase::Float);
    ok_tick(&mut s, b(OK_V, -4.0), TICK);
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.7), TICK), Action::None));
    }
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn returns_none_on_steady_state() {
    let mut s = active(lfp_4s());
    for _ in 0..100 {
        assert!(matches!(
            ok_tick(&mut s, b(OK_V, -0.05), TICK),
            Action::None
        ));
    }
}

#[test]
fn transition_only_emits_setpoint_once() {
    let mut s = active(lfp_4s());
    // First crossing → write.
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    // Already absorb → silent.
    assert!(matches!(ok_tick(&mut s, b(OK_V, -4.0), TICK), Action::None));
    assert!(matches!(ok_tick(&mut s, b(OK_V, -5.0), TICK), Action::None));
}

#[test]
fn full_charge_cycle() {
    let mut s = active(lfp_4s());
    // Bulk → absorb on heavy current.
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -8.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_approx(s.target_voltage(), 14.4);
    // Hold absorb across a realistic taper — all values stay above 2.5 A.
    for &i in &[-7.0, -5.0, -3.5, -3.0, -2.7] {
        assert!(matches!(ok_tick(&mut s, b(OK_V, i), TICK), Action::None));
    }
    // Sustained sub-tail current for the debounce window → drop to float.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -2.4), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_approx(s.target_voltage(), 13.5);
    // Sit at float without retriggering absorb (all below 3 A enter).
    for &i in &[-0.05, -0.02, 0.0, -2.0] {
        assert!(matches!(ok_tick(&mut s, b(OK_V, i), TICK), Action::None));
    }
}

#[test]
fn supervisor_passes_setpoint_through_on_phase_transition() {
    let mut s = active(lfp_4s());
    assert!(matches!(
        ok_tick(&mut s, b(13.5, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.phase(), Phase::Absorb);
    assert_approx(s.target_voltage(), 14.4);
    assert_eq!(s.fault(), None);
}

#[test]
fn supervisor_returns_none_on_steady_state() {
    let mut s = active(lfp_4s());
    for _ in 0..50 {
        assert!(matches!(
            ok_tick(&mut s, b(13.5, -0.05), TICK),
            Action::None
        ));
    }
    assert_eq!(s.fault(), None);
}

#[test]
fn steady_healthy_polls_never_leave_float() {
    // A full pack at rest, drift-free, drawing well under enter_absorb_a:
    // the phase machine has no reason to move and must not, however long
    // it runs. Guards against a debouncer drifting into a spurious trip.
    let mut s = active(lfp_4s());
    for tick in 0..(2 * EXIT_DEBOUNCE.as_secs()) {
        let a = ok_tick(&mut s, b(CV_V, -0.1), TICK);
        assert!(matches!(a, Action::None), "tick {tick}: {a:?}");
        assert_eq!(s.phase(), Phase::Float);
        assert_eq!(s.fault(), None);
    }
}
