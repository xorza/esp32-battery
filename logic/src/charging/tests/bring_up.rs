//! Bring-up: what blocks energising, what merely inhibits, and what the
//! first sourcing tick inherits.

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
    // near float_v, so it can't draw enter_absorb_a (3 A here). The old
    // current-only gate left it stuck in Float forever. Now the resting
    // SoC at bring-up routes it back into Absorb regardless of current.
    let mut s = ChargeSupervisor::new(lfp_4s());
    // LOW_V = 13.0 is 40 % on the OCV curve; current -0.1 A is far under
    // enter_absorb_a — exactly the stuck scenario.
    let a = ok_tick(&mut s, b(LOW_V, -0.1), TICK);
    let resume_absorb = accept_enable(&mut s, a);
    assert!(resume_absorb, "part-charged pack ⇒ must request Absorb");
    // The step up is armed, not taken: `ToAbsorb` still holds float_v, so a
    // failed write leaves the drift check matching the live V_SET.
    assert_eq!(s.state(), ChargeState::ToAbsorb);
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(
        matches!(a, Action::UpdateVoltage(ref t) if approx(t.target_v, lfp_4s().absorb_v)),
        "expected UpdateVoltage to absorb_v, got {a:?}",
    );
    assert_eq!(s.state(), ChargeState::Absorb);
}

#[test]
fn full_pack_stays_float_after_bringup() {
    // Bring-up must not resume Absorb on a pack that is already full; with
    // low current it sits in Float (maintenance), no voltage bump.
    //
    // The second row is the regression. Once the output has been off a
    // full LFP pack relaxes to ~3.375 V/cell — nowhere near the plateau —
    // so a plateau-proximity test called it empty and forced an Absorb
    // cycle, plus the output-cycling step-down that ends it, on every
    // reboot the unit ever performed.
    let profile = lfp_4s();
    for rest_v in [CV_V, profile.float_v] {
        let mut s = ChargeSupervisor::new(profile);
        let a = ok_tick(&mut s, b(rest_v, -0.1), TICK);
        assert!(
            !accept_enable(&mut s, a),
            "{rest_v} V: full pack must not resume Absorb"
        );
        let a = ok_tick(&mut s, b(rest_v, -0.1), TICK);
        assert!(matches!(a, Action::None), "{rest_v} V: got {a:?}");
        assert_eq!(s.state(), ChargeState::Float, "{rest_v} V");
    }
}

#[test]
fn resume_gate_reads_resting_soc_and_only_when_rested() {
    // The gate's two inputs, one row each.
    //
    // SoC axis: the 4S LFP float target is 3.375 V/cell, which the curve
    // puts at 90 + (3.375 - 3.350) / 0.030 × 9 = 97.5 %, so the bar is
    // 97.5 − FULL_SOC_MARGIN = 92.5 %, back on the curve at
    // 3.350 + 2.5 / 9 × 0.030 = 3.3583 V/cell ⇒ 13.433 V for 4S. Hand-
    // computed from the curve rather than read out of `soc`, which is the
    // thing under test.
    //
    // Rest axis: OCV → SoC is a *rested* measurement, and a pack still
    // moving current reads high while charging and low while discharging.
    // Past `exit_absorb_a` (2.5 A, strictly less) the reading is not
    // trusted and the answer falls back to "not full" — erring toward
    // charging. The last three rows sit at float_v, which alone rests at
    // 97.5 % and is comfortably full, so any resume there is the rest
    // gate talking and not the SoC one.
    let profile = lfp_4s();
    let cases: [(f32, f32, bool); 5] = [
        (13.42, -0.1, true),   // 3.3550 ⇒ 91.5 %, under the bar
        (13.45, -0.1, false),  // 3.3625 ⇒ 93.75 %, over it
        (profile.float_v, -profile.exit_absorb_a, true),
        (profile.float_v, -(profile.exit_absorb_a - 0.1), false),
        (profile.float_v, profile.exit_absorb_a, true),
    ];
    for (rest_v, current, want_resume) in cases {
        let mut s = ChargeSupervisor::new(profile);
        let a = ok_tick(&mut s, b(rest_v, current), TICK);
        assert_eq!(
            accept_enable(&mut s, a),
            want_resume,
            "{rest_v} V @ {current} A"
        );
    }
}

#[test]
fn full_is_relative_to_the_profile_not_the_chemistry_hundred_percent_point() {
    // Li-ion charged to the longevity-tuned 4.10 V/cell rests well below
    // the curve's 4.20 V/100 % point, and its float target rests at ~82 %
    // where LFP's rests at 97.5 %. An absolute bar chosen for one calls
    // the other empty and forces it straight back into absorb — this gate's
    // own bug, reintroduced one chemistry over. Each profile's bar is its
    // own float target, so a pack rested there is full for both.
    for chemistry in [Chemistry::LiFePo4, Chemistry::LiIon] {
        let profile = Profile::for_pack(chemistry, 3, 50.0);
        let mut s = ChargeSupervisor::new(profile);
        let a = ok_tick(&mut s, b(profile.float_v, -0.1), TICK);
        assert!(
            !accept_enable(&mut s, a),
            "{chemistry}: a pack rested at its own float target is full"
        );
    }
    // And the two really do rest at different readings, so the row above
    // is not passing because both happen to clear one shared bar.
    let lfp = Profile::for_pack(Chemistry::LiFePo4, 3, 50.0);
    let liion = Profile::for_pack(Chemistry::LiIon, 3, 50.0);
    let (lfp_rest, liion_rest) = (lfp.soc(lfp.float_v), liion.soc(liion.float_v));
    assert!(
        lfp_rest - liion_rest > 10.0,
        "float targets rest at {lfp_rest} % and {liion_rest} %"
    );
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

    // LOW_V rests at 40 %, so the pack is not full and bring-up resumes
    // Absorb.
    let a = ok_tick(&mut s, b(LOW_V, -0.1), TICK);
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
    let a = ok_tick(&mut s, b(LOW_V, -0.1), TICK);
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
    // At boot the buck IS supposed to be off — output_on=Some(false) is
    // normal, must not latch. expected_poll returns Off for a bring-up
    // state.
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
    // Defensive: in a bring-up state the buck is OFF, so any "charging
    // current" sample is meaningless (no current is actually flowing). The
    // phase machine MUST NOT advance on it — only `EnableOutput` may be
    // emitted, and after the caller acks we land in Float.
    let profile = lfp_4s();
    let mut s = ChargeSupervisor::new(profile);
    // Sample reports current well above enter_absorb_a (3 A). If the phase
    // machine ran during bring-up it would emit UpdateVoltage(absorb_v).
    let high_current = -10.0;
    let battery = b(profile.float_v, high_current);
    let a = s.tick(expected_poll(&s, battery), TICK);
    assert_eq!(s.state(), ChargeState::Boot);
    // After the commit, the supervisor lands in Float — the first real
    // tick then runs the phase machine. Verify that tick is the one which
    // emits the transition.
    accept_enable(&mut s, a);
    let a = s.tick(expected_poll(&s, battery), TICK);
    assert!(matches!(a, Action::UpdateVoltage(ref t) if approx(t.target_v, profile.absorb_v)));
}
