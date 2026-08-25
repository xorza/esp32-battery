//! LVP and OTP: self-clearing protections the buck handles itself, which
//! the supervisor must wait out rather than latch on.

use super::*;

/// LVP and OTP are both sensor-side conditions the buck clears on its own,
/// so the supervisor treats them identically.
const SELF_CLEARING: [ProtectionStatus; 2] = [ProtectionStatus::Lvp, ProtectionStatus::Otp];

#[test]
fn self_clearing_protection_drops_to_pending_without_latching() {
    // The buck self-disabled on a benign sensor-side condition, not a
    // pack fault. The supervisor steps back to bring-up and waits, so no
    // DisableOutput is emitted and no restart budget is burned however
    // long the condition lasts.
    for cause in SELF_CLEARING {
        let mut s = active(lfp_4s());
        let a = s.tick(poll_with_output(&s, BuckOutput::Off { cause }), TICK);
        assert!(matches!(a, Action::None), "{cause}");
        assert_eq!(s.fault(), None, "{cause}");
        assert!(matches!(s.latch, LatchState::Pending { .. }), "{cause}");
    }
}

#[test]
fn self_clearing_protection_accepts_buck_auto_re_enable() {
    // The XY7025 typically re-enables OUTPUT_EN itself once the cause
    // clears — input voltage returns, or the case cools. The supervisor
    // must read that as recovery rather than latching OutputOnInPending:
    // setpoints are still the ones it programmed before the self-disable,
    // so regulation resumes at known targets.
    for cause in SELF_CLEARING {
        let mut s = active(lfp_4s());
        s.tick(poll_with_output(&s, BuckOutput::Off { cause }), TICK);
        assert!(matches!(s.latch, LatchState::Pending { .. }), "{cause}");

        let a = s.tick(poll_with_output(&s, BuckOutput::On), TICK);
        assert!(matches!(a, Action::None), "{cause}");
        assert_eq!(s.fault(), None, "{cause}");
        assert!(matches!(s.latch, LatchState::Active { .. }), "{cause}");
    }
}

#[test]
fn pending_waits_for_lvp_to_clear_before_enable() {
    // While LVP persists, the Pending bring-up must NOT emit EnableOutput
    // — writing set_output(true) into a buck in input UVLO would just
    // flap. Once LVP clears (buck reports Off with no cause), the
    // normal Pending → Active path emits EnableOutput.
    let mut s = active(lfp_4s());
    let p_lvp = poll_with_output(&s, BuckOutput::Off { cause: ProtectionStatus::Lvp });
    // Drop Active → Pending via LVP.
    assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    // Many ticks of sustained LVP: stays Pending, no actions, no fault.
    for _ in 0..120 {
        assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    }
    assert_eq!(s.fault(), None);
    // LVP clears: buck back to Off with no protection cause. Pending
    // bring-up energises on the next tick.
    let p_clear = expected_poll(&s, b(OK_V, -0.1));
    let a = s.tick(p_clear, TICK);
    accept_enable(&mut s, a);
    assert!(matches!(s.latch, LatchState::Active { .. }));
}

#[test]
fn lvp_recovery_resumes_absorb_when_pack_below_plateau() {
    // Pack drains during the input outage to below the CV plateau —
    // when LVP clears, the bring-up's resting-voltage check must resume
    // Absorb (not stall in Float).
    let profile = lfp_4s();
    let mut s = active(profile);
    let p_lvp = poll_with_output(&s, BuckOutput::Off { cause: ProtectionStatus::Lvp });
    s.tick(p_lvp, TICK);
    // Pack rests well below absorb_v - ABSORB_CV_BAND_V (= 14.3).
    let drained = b(13.0, 0.0);
    let p_clear = PollResult {
        output: Some(BuckOutput::Off { cause: ProtectionStatus::Normal }),
        setpoints: Some(s.expected_setpoints()),
        battery: drained,
    };
    // Drained pack ⇒ the ticket carries resume_absorb = true.
    let a = s.tick(p_clear, TICK);
    assert!(accept_enable(&mut s, a));
    // The commit resumed Absorb via pending_voltage, so the next Active
    // tick steps V_SET float_v → absorb_v.
    let p_active = PollResult {
        output: Some(BuckOutput::On),
        setpoints: Some(s.expected_setpoints()),
        battery: drained,
    };
    let a = s.tick(p_active, TICK);
    assert!(matches!(a, Action::UpdateVoltage(ref t) if approx(t.target_v, profile.absorb_v)));
}

#[test]
fn pending_at_boot_with_lvp_waits() {
    // Fresh supervisor + buck reports Off(Lvp) at boot (DC supply not
    // yet present). Must not emit EnableOutput; must not latch.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p_lvp = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Lvp,
        }),
        setpoints: Some(s.expected_setpoints()),
        battery: b(OK_V, -0.1),
    };
    for _ in 0..30 {
        assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    }
    assert_eq!(s.fault(), None);
    assert!(matches!(s.latch, LatchState::Pending { .. }));
}

#[test]
fn protect_recovery_in_absorb_does_not_re_emit_its_own_voltage() {
    // Long input outage while charging: the buck drops on LVP with the
    // supervisor in Absorb, the pack drains below the CV plateau, and the
    // rail returns. Bring-up wants Absorb — which is already the phase, and
    // already the live V_SET — so there is nothing to write.
    let profile = lfp_4s();
    let mut s = active(profile);
    enter_absorb(&mut s);
    assert_eq!(s.phase(), Phase::Absorb);

    let p_lvp = poll_with_output(&s, BuckOutput::Off {
        cause: ProtectionStatus::Lvp,
    });
    assert!(matches!(s.tick(p_lvp, TICK), Action::None));

    // Pack rests at 13.0, below the 14.3 plateau ⇒ resume_absorb is true.
    let drained = b(13.0, 0.0);
    let p_clear = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Normal,
        }),
        setpoints: Some(s.expected_setpoints()),
        battery: drained,
    };
    let a = s.tick(p_clear, TICK);
    assert!(accept_enable(&mut s, a), "drained pack must ask to resume Absorb");

    // Already in Absorb at absorb_v: the next tick regulates, it does not
    // re-write the voltage it is already at.
    let p_on = PollResult {
        output: Some(BuckOutput::On),
        setpoints: Some(s.expected_setpoints()),
        battery: drained,
    };
    let a = s.tick(p_on, TICK);
    assert!(
        !matches!(a, Action::UpdateVoltage(_)),
        "no-op voltage write emitted: {a:?}"
    );
    assert_eq!(s.phase(), Phase::Absorb);
    assert!(approx(s.expected_setpoints().v_set, profile.absorb_v));
}

#[test]
fn protect_hold_clears_the_overvoltage_window() {
    // OV accumulates while regulating, the buck then self-disables on a
    // transient protection. Output is off for the hold, so the pack decays
    // — the partial OV window must not carry across and trip the next
    // regulating stretch early.
    let profile = lfp_4s();
    let over = profile.absorb_v + OV_MARGIN_V + 0.1;
    let mut s = active(profile);

    // Two of the three seconds OV needs.
    for _ in 0..(OV_DURATION.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(over, -0.1), TICK), Action::None));
        assert_eq!(s.fault(), None);
    }

    // Buck drops on input UVLO, then comes back.
    let p_lvp = poll_with_output(&s, BuckOutput::Off {
        cause: ProtectionStatus::Lvp,
    });
    assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    let p_on = poll_with_output(&s, BuckOutput::On);
    assert!(matches!(s.tick(p_on, TICK), Action::None));

    // A single over-volt tick must not latch: the window restarts at zero,
    // so it takes the full OV_DURATION again.
    for _ in 0..(OV_DURATION.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(over, -0.1), TICK), Action::None));
        assert_eq!(s.fault(), None, "OV window carried across the hold");
    }
    assert!(matches_disable(
        &ok_tick(&mut s, b(over, -0.1), TICK),
        FaultReason::Overvoltage
    ));
}
