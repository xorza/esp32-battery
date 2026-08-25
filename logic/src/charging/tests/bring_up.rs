//! `Pending`: what blocks energising, what merely inhibits, and what the
//! first Active tick inherits.

use super::*;

#[test]
fn pending_emits_enable_on_first_healthy_tick() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(matches!(a, Action::EnableOutput { .. }));
}

#[test]
fn pending_re_emits_enable_until_committed() {
    // Until the caller commits an EnableTicket, every tick re-emits EnableOutput
    // — mirrors the DisableOutput retry behavior on failed disable writes.
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..3 {
        let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
        assert!(matches!(a, Action::EnableOutput { .. }));
    }
}

#[test]
fn low_pack_resumes_absorb_after_bringup() {
    // Regression: a partially-charged pack power-cycled mid-charge rests
    // below the CV plateau but near float_v, so it can't draw
    // enter_absorb_a (3 A here). The old current-only gate left it stuck in
    // Float forever. Now the resting voltage at bring-up (< absorb_v -
    // band) routes it back into Absorb regardless of current.
    let mut s = ChargeSupervisor::new(lfp_4s());
    // OK_V = 13.5 is below absorb_v - band (14.3); current -0.1 A is far
    // under enter_absorb_a — exactly the stuck scenario.
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    let resume_absorb = accept_enable(&mut s, a);
    assert!(resume_absorb, "pack below CV plateau ⇒ must request Absorb");
    assert_eq!(s.phase(), Phase::Float); // not committed until V_SET write
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(
        matches!(a, Action::UpdateVoltage(ref t) if approx(t.target_v, lfp_4s().absorb_v)),
        "expected UpdateVoltage to absorb_v, got {a:?}",
    );
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn full_pack_stays_float_after_bringup() {
    // A pack resting at the CV plateau is full — bring-up must NOT resume
    // Absorb. With low current it sits in Float (maintenance), no voltage
    // bump.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = ok_tick(&mut s, b(CV_V, -0.1), TICK);
    assert!(!accept_enable(&mut s, a), "full pack must not resume Absorb");
    let a = ok_tick(&mut s, b(CV_V, -0.1), TICK);
    assert!(matches!(a, Action::None), "expected None, got {a:?}");
    assert_eq!(s.phase(), Phase::Float);
}

#[test]
fn pending_overvolt_inhibits_from_first_sample_and_clears() {
    // Pack already over the OV threshold at boot must never see
    // EnableOutput — from the very first sample, with no 3 s debounce.
    // But the output is already off, so there is nothing to disable:
    // this inhibits rather than latching, and lifts on its own once the
    // pack comes back under the line.
    let absorb = lfp_4s().absorb_v; // 14.4
    let trip = absorb + OV_MARGIN_V; // 14.6
    let mut s = ChargeSupervisor::new(lfp_4s());

    let a = ok_tick(&mut s, b(trip + 0.5, -0.1), TICK);
    assert!(matches!(a, Action::None));
    assert_eq!(s.inhibit(), Some(InhibitReason::Overvoltage));
    assert_eq!(s.fault(), None);

    // Hold it over the line well past OV_DURATION: still no latch. The
    // debounced trip belongs to the regulating path only.
    for _ in 0..(OV_DURATION.as_secs() * 3) {
        let a = ok_tick(&mut s, b(trip + 0.5, -0.1), TICK);
        assert!(matches!(a, Action::None));
        assert_eq!(s.fault(), None);
    }

    // 14.55 is under the 14.6 trip and at/above the CV plateau
    // (14.4 - 0.1), so the pack reads as full: bring-up parks in Float.
    let a = ok_tick(&mut s, b(trip - 0.05, -0.1), TICK);
    assert!(matches!(
        a,
        Action::EnableOutput(ref t) if t.resume_absorb() == false
    ));
    assert_eq!(s.inhibit(), None);
}

#[test]
fn pending_drift_inhibits_without_enabling() {
    // Drift while the output is off blocks bring-up but latches nothing
    // — and a return to the commanded setpoints releases it.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p = PollResult {
        setpoints: Some(Setpoints {
            v_set: 12.0,
            i_set: 10.0,
        }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    assert!(matches!(s.tick(p, TICK), Action::None));
    assert_eq!(s.inhibit(), Some(InhibitReason::SettingsDrift));
    assert_eq!(s.fault(), None);

    // OK_V (13.5) is below the CV plateau, so the pack is not full and
    // bring-up resumes Absorb.
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(matches!(
        a,
        Action::EnableOutput(ref t) if t.resume_absorb() == true
    ));
    assert_eq!(s.inhibit(), None);
}

#[test]
fn pending_no_battery_inhibits_indefinitely() {
    // No battery sample means no enable, ever — but with the output
    // already off there is nothing to disable, so a dead sensor at boot
    // no longer strands the unit behind a reboot. The inhibit reason
    // sharpens from "none yet" to "stale" once the debounce fires.
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        let a = ok_tick(&mut s, None, TICK);
        assert!(matches!(a, Action::None));
        assert_eq!(s.inhibit(), Some(InhibitReason::NoBatterySample));
    }
    // 10th tick: elapsed reaches BATTERY_MISSING_TIMEOUT.
    let a = ok_tick(&mut s, None, TICK);
    assert!(matches!(a, Action::None));
    assert_eq!(s.inhibit(), Some(InhibitReason::BatterySensorStale));
    assert_eq!(s.fault(), None);

    // Well past the timeout it still only waits.
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() * 2) {
        assert!(matches!(ok_tick(&mut s, None, TICK), Action::None));
        assert_eq!(s.fault(), None);
    }

    // Sensor comes back: bring-up proceeds on the next tick.
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(matches!(
        a,
        Action::EnableOutput(ref t) if t.resume_absorb() == true
    ));
    assert_eq!(s.inhibit(), None);
}

#[test]
#[should_panic]
fn commit_enable_from_active_panics() {
    let mut s = active(lfp_4s());
    s.commit_enable(EnableTicket {
        resume_absorb: false,
    });
}

#[test]
fn pending_does_not_enable_without_setpoint_readback() {
    // boot_sequence verified setpoints, but the supervisor still requires
    // a fresh successful readback before energizing — otherwise we'd ask
    // for output-on with no closed-loop confirmation the buck is even
    // reachable. Modbus-down ticks emit None until the modbus_err
    // debounce eventually latches ModbusUnhealthy.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p = PollResult {
        setpoints: None,
        output: None,
        battery: b(OK_V, -0.1),
    };
    // Below the modbus_err debounce window: no fault, no enable.
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        assert!(matches!(s.tick(p, TICK), Action::None));
    }
    // Recovery via a successful readback emits EnableOutput on that tick.
    let p_ok = expected_poll(&s, b(OK_V, -0.1));
    assert!(matches!(s.tick(p_ok, TICK), Action::EnableOutput { .. }));
}

#[test]
fn buck_output_off_in_pending_does_not_fault() {
    // In Pending the buck IS supposed to be off — output_on=Some(false)
    // is normal, must not latch. expected_poll for Pending returns Off.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = s.tick(expected_poll(&s, b(OK_V, -0.1)), TICK);
    assert!(matches!(a, Action::EnableOutput { .. }));
    assert_eq!(s.fault(), None);
}

#[test]
#[should_panic]
fn commit_enable_from_tripped_panics() {
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, b(OK_V, -0.1), TICK);
    }
    s.commit_enable(EnableTicket {
        resume_absorb: false,
    });
}

#[test]
fn pending_does_not_enter_absorb_even_at_high_charge_current() {
    // Defensive: in Pending the buck is OFF, so any "charging current"
    // sample is meaningless (no current is actually flowing). The phase
    // machine MUST NOT advance on it — only `EnableOutput` may be emitted,
    // and after the caller acks we transition to Active still in Float.
    let profile = lfp_4s();
    let mut s = ChargeSupervisor::new(profile);
    // Sample reports current well above enter_absorb_a (3 A). If the phase
    // machine ran in Pending it would emit UpdateVoltage(absorb_v).
    let high_current = -10.0;
    let battery = b(profile.float_v, high_current);
    let a = s.tick(expected_poll(&s, battery), TICK);
    assert_eq!(s.phase(), Phase::Float);
    // After the commit, supervisor goes Active still in Float — the first
    // real tick then runs the phase machine. Verify the very next tick
    // (now Active) is the one that emits the transition.
    accept_enable(&mut s, a);
    let a = s.tick(expected_poll(&s, battery), TICK);
    assert!(matches!(a, Action::UpdateVoltage(ref t) if approx(t.target_v, profile.absorb_v)));
}
