//! Parking: the fault response that stops charging without dropping the
//! load, and the ways out of it.

use super::*;

/// Drive `s` into `Parked` on an absorb-cap timeout — the cheapest park to
/// reach, and the one whose hazard is unambiguously overcharge.
fn park_on_absorb_timeout(s: &mut ChargeSupervisor) {
    enter_absorb(s);
    let a = ok_tick(s, b(CV_V, -3.0), MAX_ABSORB);
    accept_park(s, a, FaultReason::AbsorbTimeout);
}

#[test]
fn parking_stops_charging_without_dropping_the_load() {
    // The whole point. A latch would stop the overcharge too — and put the
    // load on the pack to drain for however long it takes someone to
    // notice. Parking holds the float target instead: charging over,
    // output still up, and no `DisableOutput` ever emitted.
    let profile = lfp_4s();
    let mut s = active(profile);
    park_on_absorb_timeout(&mut s);

    assert!(s.state().sourcing(), "the buck must stay up");
    assert_approx(s.target_voltage(), profile.float_v);
    assert_eq!(s.phase(), Some(Phase::Float));
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -0.1), TICK), Action::None));
    }
}

#[test]
fn a_parked_supervisor_does_not_charge_again() {
    // Float and Parked both hold the float target; what separates them is
    // that Parked has no way back into Absorb. Heavy charging current —
    // which from Float is exactly the trigger for a step up — must do
    // nothing at all, or the fault that stopped the charge would be
    // undone by the next load transient.
    let mut s = active(lfp_4s());
    park_on_absorb_timeout(&mut s);
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -8.0), TICK), Action::None));
    }
    assert_eq!(s.state(), ChargeState::Parked);
}

#[test]
fn a_second_park_class_fault_escalates_to_a_disable() {
    // Parking is the gentler answer and it is offered once. A park-class
    // fault raised again from inside a park is proof that holding float did
    // not fix it, so there is nothing left to try but the output.
    let profile = lfp_4s();
    let mut s = active(profile);
    park_on_absorb_timeout(&mut s);

    let over = profile.regulation_a * OVERCURRENT_TOL + 0.1;
    for _ in 0..(OVERCURRENT_DURATION.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -over), TICK), Action::None));
    }
    assert!(matches_disable(
        &ok_tick(&mut s, b(OK_V, -over), TICK),
        FaultReason::ChargeOvercurrent
    ));
}

#[test]
fn a_control_loss_fault_while_parked_still_disables() {
    // Parked is a sourcing state, so the gauntlet keeps judging it. Losing
    // the link means we can no longer see or command a buck that is still
    // putting out — which is the one thing parking cannot cover.
    let mut s = active(lfp_4s());
    park_on_absorb_timeout(&mut s);
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        assert!(matches!(fail_tick(&mut s, b(OK_V, -0.1), TICK), Action::None));
    }
    assert!(matches_disable(
        &fail_tick(&mut s, b(OK_V, -0.1), TICK),
        FaultReason::ModbusUnhealthy
    ));
}

#[test]
fn a_parked_buck_waits_out_a_protection_and_comes_back_parked() {
    // The rail drops while parked. Waiting it out is right — the load wants
    // feeding again the moment the supply returns — but the hold has to
    // remember it was parked. Resuming into `Float` would put the unit back
    // to charging on a fault nobody has looked at.
    let mut s = active(lfp_4s());
    park_on_absorb_timeout(&mut s);
    drain_transitions(&mut s);

    assert!(matches!(sag(&mut s), Action::None), "a park must not latch here");
    assert_eq!(s.state(), ChargeState::HoldParked);
    assert!(!s.parked(), "the output is down, so the load is not being fed");
    // Both reported at once, and they say different things: the fault is
    // why charging is over, the inhibit is what the output is down on.
    assert_eq!(s.fault(), Some(FaultReason::AbsorbTimeout));
    assert_eq!(
        s.inhibit(),
        Some(InhibitReason::BuckProtection(ProtectionStatus::Lvp))
    );

    assert!(matches!(recover(&mut s), Action::None));
    assert_eq!(s.state(), ChargeState::Parked);
    assert!(s.parked());
    assert_eq!(
        s.fault(),
        Some(FaultReason::AbsorbTimeout),
        "the park's fault must survive the hold"
    );
    // And it is still parked, not charging: heavy current moves nothing.
    for _ in 0..5 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -8.0), TICK), Action::None));
    }
    assert_eq!(s.state(), ChargeState::Parked);
    assert_eq!(
        drain_transitions(&mut s),
        [ChargeTransition::ProtectHold, ChargeTransition::ProtectCleared],
        "a hold out of a park reads as a protection story, not a fresh park"
    );
}

#[test]
fn a_park_mid_step_down_latches_rather_than_holding() {
    // `ToParked` is the one sourcing state with nowhere to hold: the
    // step-down write is still outstanding, so the output state a hold
    // would claim is not settled. It latches instead — the table says so by
    // having no `SelfDisabled` cell, and `reconcile` reads that back.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    // Drive the cap without letting `ok_tick` commit the step-down.
    let p = expected_poll(&s, b(CV_V, -3.0));
    assert!(matches!(s.tick(p, MAX_ABSORB), Action::UpdateVoltage(_)));
    assert_eq!(s.state(), ChargeState::ToParked);
    assert!(matches_disable(
        &sag(&mut s),
        FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Lvp)
    ));
}

#[test]
fn a_park_is_one_entry_in_the_log() {
    // The move into the park records; the write that completes it does not.
    // A dashboard should see "stopped charging on absorb_timeout", once.
    let mut s = active(lfp_4s());
    drain_transitions(&mut s);
    park_on_absorb_timeout(&mut s);
    assert_eq!(drain_transitions(&mut s), [ChargeTransition::Parked]);
}
