//! The CV-plateau time cap: what arms it, what resets it, and that only
//! time actually spent at the plateau counts.

use super::*;

#[test]
fn absorb_timeout_in_a_single_large_elapsed_tick() {
    // After entering Absorb, one tick covering the full cap must fault.
    // Equivalent to the iteration-based test but proves elapsed is honored.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    assert!(matches_disable(
        &ok_tick(&mut s, b(CV_V, -3.0), MAX_ABSORB),
        FaultReason::AbsorbTimeout,
    ));
}

#[test]
fn absorb_does_not_time_out_below_budget() {
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    // Hold at the CV plateau just shy of the cap. Current pinned above exit
    // threshold (2.5 A) so we never drop to Float on our own.
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(CV_V, -3.0), TICK), Action::None));
    }
    assert_eq!(s.fault(), None);
}

#[test]
fn float_does_not_accumulate_absorb_ticks() {
    let mut s = active(lfp_4s());
    // Sit in Float for far longer than the absorb cap — must never fault.
    for _ in 0..(MAX_ABSORB.as_secs() + 10) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -0.1), TICK), Action::None));
    }
    assert_eq!(s.phase(), Phase::Float);
}

#[test]
fn absorb_counter_resets_on_taper_back_to_float() {
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    // Spend most of the budget in absorb, leaving room for a full exit
    // debounce window before the absorb timeout would fire.
    for _ in 0..(MAX_ABSORB.as_secs() - EXIT_DEBOUNCE.as_secs() - 10) {
        ok_tick(&mut s, b(CV_V, -3.0), TICK);
    }
    // …then taper to Float. Sub-tail current for the debounce window
    // before the transition fires, then absorb_elapsed resets.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -0.1), TICK), Action::None));
    }
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -0.1), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.phase(), Phase::Float);

    // Re-enter Absorb and burn the original margin's worth of ticks.
    // No fault yet — counter started over.
    enter_absorb(&mut s);
    for _ in 0..20 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    }
    assert_eq!(s.fault(), None);
}

#[test]
fn cc_ramp_below_absorb_v_does_not_arm_timeout() {
    // Core of the empty-pack fix: an empty pack enters Absorb immediately and
    // sits in CC (voltage well below absorb_v=14.4) for hours while it climbs.
    // The timeout clocks the CV plateau only, so the CC ramp must never fault
    // however long it runs.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    for _ in 0..(MAX_ABSORB.as_secs() + 10) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    }
    assert_eq!(s.fault(), None);
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn absorb_timer_resets_on_cc_dip() {
    // A load transient pulling the pack back below absorb_v (CC again) resets
    // the clock — that's genuine charging, not a stuck taper. Arm the timer to
    // one tick shy of the cap, dip once into CC, then a second near-full CV
    // hold must still not fault: proves the dip cleared the accumulated time.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        ok_tick(&mut s, b(CV_V, -3.0), TICK);
    }
    assert_eq!(s.fault(), None);
    // CC dip: voltage below the CV band resets the absorb debouncer.
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(CV_V, -3.0), TICK), Action::None));
    }
    assert_eq!(s.fault(), None);
}

#[test]
fn supervisor_latches_on_absorb_timeout() {
    let mut s = active(lfp_4s());
    // Drive into Absorb.
    let _ = ok_tick(&mut s, b(13.5, -4.0), TICK);
    // Hold Absorb until just before the cap. Current pinned above exit
    // threshold so the controller can't taper out on its own.
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        ok_tick(&mut s, b(CV_V, -3.0), TICK);
    }
    assert_eq!(s.fault(), None);

    let a = ok_tick(&mut s, b(CV_V, -3.0), TICK);
    assert!(matches_disable(&a, FaultReason::AbsorbTimeout));
}
