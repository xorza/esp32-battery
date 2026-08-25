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
    assert_eq!(s.state(), ChargeState::Float);
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
    assert_eq!(s.state(), ChargeState::Float);

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
    assert_eq!(s.state(), ChargeState::Absorb);
}

#[test]
fn cv_dips_shave_the_absorb_clock_instead_of_erasing_it() {
    // A load transient that pulls the buck out of CV for one tick used to
    // zero the whole window, so a load cycling faster than the cap kept it
    // from ever firing and the pack sat at CV indefinitely. Leaky: a dip
    // costs exactly the time it lasted.
    //
    // 7199 s at CV, one 1 s dip (→ 7198), then two more at CV: 7199 holds,
    // 7200 trips.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        ok_tick(&mut s, b(CV_V, -3.0), TICK);
    }
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    assert!(matches!(ok_tick(&mut s, b(CV_V, -3.0), TICK), Action::None));
    assert_eq!(s.fault(), None, "the dip must cost one tick, not the window");
    assert!(matches_disable(
        &ok_tick(&mut s, b(CV_V, -3.0), TICK),
        FaultReason::AbsorbTimeout
    ));
}

#[test]
fn a_sustained_return_to_cc_drains_the_absorb_clock() {
    // The leak has to be symmetric, or the cap would eventually fire on a
    // pack that spends most of its time genuinely charging in CC. Fill the
    // window to one tick shy, spend the same span below the plateau — back
    // to zero — and a fresh full window is then required.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        ok_tick(&mut s, b(CV_V, -3.0), TICK);
    }
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        ok_tick(&mut s, b(OK_V, -3.0), TICK);
    }
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(CV_V, -3.0), TICK), Action::None));
    }
    assert_eq!(s.fault(), None, "the window was not emptied");
    assert!(matches_disable(
        &ok_tick(&mut s, b(CV_V, -3.0), TICK),
        FaultReason::AbsorbTimeout
    ));
}

#[test]
fn charge_timeout_bounds_a_ramp_that_never_reaches_cv() {
    // `MAX_ABSORB` clocks the plateau only, so a pack that never gets there
    // — shorted cell, wiring fault, a load eating the whole charge current
    // — has no other backstop. Hold well below the CV band past the total
    // budget: `AbsorbTimeout` correctly never fires and `ChargeTimeout` must.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -3.0), MAX_CHARGE - TICK),
        Action::None
    ));
    assert_eq!(s.fault(), None);
    assert!(matches_disable(
        &ok_tick(&mut s, b(OK_V, -3.0), TICK),
        FaultReason::ChargeTimeout
    ));
}

#[test]
fn charge_budget_covers_one_cycle_not_the_unit_lifetime() {
    // Two Absorb stretches of half the budget each, with a taper between
    // them. They total more than `MAX_CHARGE`, so a budget that carried
    // across the taper would fire on the second — it must start over.
    let half = MAX_CHARGE / 2;
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), half), Action::None));

    // Sub-tail current for the exit window drops us back to Float.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -0.1), TICK), Action::None));
    }
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -0.1), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.state(), ChargeState::Float);

    enter_absorb(&mut s);
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), half), Action::None));
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
