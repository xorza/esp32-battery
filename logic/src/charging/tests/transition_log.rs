//! The latch-transition ring the firmware drains into the event log.

use super::*;

#[test]
fn transitions_record_the_route_not_just_the_destination() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    assert_eq!(s.pop_transition(), None, "nothing before the first tick");

    // Bring-up at the CV plateau: full pack, parks in Float.
    let a = ok_tick(&mut s, b(CV_V, -0.1), TICK);
    accept_enable(&mut s, a);
    assert_eq!(s.pop_transition(), Some(ChargeTransition::Energised));
    assert_eq!(s.pop_transition(), None, "drained");

    // Input rail sags: the buck self-disables on LVP and we step back to
    // bring-up rather than latching.
    let p_lvp = poll_with_output(&s, BuckOutput::Off {
        cause: ProtectionStatus::Lvp,
    });
    assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    assert_eq!(s.pop_transition(), Some(ChargeTransition::ProtectHold));

    // Rail returns and the buck re-enables itself.
    let p_on = poll_with_output(&s, BuckOutput::On);
    assert!(matches!(s.tick(p_on, TICK), Action::None));
    assert_eq!(s.pop_transition(), Some(ChargeTransition::ProtectCleared));

    // Second hold, recovered the other way round: the cause clears but the
    // buck stays off, so the supervisor energises it itself. That commit is
    // the same code path as a cold-boot bring-up and must still log
    // ProtectCleared — a hold ended, not a fresh boot.
    assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    assert_eq!(s.pop_transition(), Some(ChargeTransition::ProtectHold));
    let a = ok_tick(&mut s, b(CV_V, -0.1), TICK);
    assert!(!accept_enable(&mut s, a), "pack at the plateau stays in Float");
    assert_eq!(s.pop_transition(), Some(ChargeTransition::ProtectCleared));
    assert_eq!(s.pop_transition(), None, "the enable itself is not a transition");

    // A non-recoverable self-disable latches.
    let p_ovp = poll_with_output(&s, BuckOutput::Off {
        cause: ProtectionStatus::Ovp,
    });
    assert!(matches_disable(
        &s.tick(p_ovp, TICK),
        FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Ovp)
    ));
    assert_eq!(s.pop_transition(), Some(ChargeTransition::Latched));
    assert_eq!(s.pop_transition(), None);
}

#[test]
fn phase_changes_are_not_latch_transitions() {
    // Arming and committing `pending_voltage` both go through `set_latch`
    // (Active → Active), but neither is a latch-state change and neither
    // may reach the log — otherwise every Float↔Absorb swing would drown
    // the transitions that matter.
    let mut s = active(lfp_4s());
    while s.pop_transition().is_some() {}
    enter_absorb(&mut s);
    assert_eq!(s.state(), ChargeState::Absorb);
    assert_eq!(s.pop_transition(), None);
}

#[test]
fn transition_ring_drops_oldest_when_undrained() {
    let mut s = active(lfp_4s());
    while s.pop_transition().is_some() {}
    let p_lvp = poll_with_output(&s, BuckOutput::Off {
        cause: ProtectionStatus::Lvp,
    });
    let p_on = poll_with_output(&s, BuckOutput::On);

    // 5 hold/clear cycles = 10 transitions into an 8-slot ring.
    for _ in 0..5 {
        s.tick(p_lvp, TICK);
        s.tick(p_on, TICK);
    }
    let mut drained = Vec::new();
    while let Some(t) = s.pop_transition() {
        drained.push(t);
    }
    assert_eq!(drained.len(), TRANSITION_BUFFER);
    // The first hold/clear pair fell out, so what remains starts on a
    // hold and still alternates.
    assert_eq!(drained[0], ChargeTransition::ProtectHold);
    assert_eq!(drained[1], ChargeTransition::ProtectCleared);
    assert_eq!(drained[TRANSITION_BUFFER - 1], ChargeTransition::ProtectCleared);
}
