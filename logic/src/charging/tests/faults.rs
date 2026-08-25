//! Conditions that latch the buck off, and the debounce windows that
//! decide when they do.

use super::*;

/// Commit a `DisableOutput` the way the firmware does.
fn accept_disable(s: &mut ChargeSupervisor, a: Action) {
    let Action::DisableOutput(ticket) = a else {
        panic!("expected DisableOutput, got {a:?}");
    };
    s.commit_disable(ticket);
}

/// Drive `s` from a sourcing state into `Latched`, on
/// `OutputUnexpectedlyOff(cause)`. Cause must be non-recoverable (i.e. not
/// Lvp/Otp, which are waited out in a hold and never latch).
fn latch_self_disable(s: &mut ChargeSupervisor, cause: ProtectionStatus) {
    let p = poll_with_output(s, BuckOutput::Off { cause });
    let a = s.tick(p, TICK);
    assert!(matches_disable(
        &a,
        FaultReason::OutputUnexpectedlyOff(cause)
    ));
    accept_disable(s, a);
}

/// A debounced fault: how to feed it, the window it needs before latching,
/// and what it latches as. The three share one shape, so they share their
/// tests. Nothing says how to *clear* one, because a single healthy tick
/// satisfies all three conditions at once.
///
/// `ChargeOvercurrent` is deliberately not here: feeding it means feeding a
/// charge current far above `enter_absorb_a`, so its every tick also drives
/// the phase machine and cannot answer `Action::None` the way this table
/// requires. It carries its own window and reset coverage instead.
#[derive(Debug)]
struct DebouncedFault {
    name: &'static str,
    window: Duration,
    fault: FaultReason,
    feed: fn(&mut ChargeSupervisor, Duration) -> Action,
}

const DEBOUNCED_FAULTS: [DebouncedFault; 3] = [
    DebouncedFault {
        name: "battery sensor stale",
        window: BATTERY_MISSING_TIMEOUT,
        fault: FaultReason::BatterySensorStale,
        feed: |s, dt| ok_tick(s, None, dt),
    },
    DebouncedFault {
        name: "modbus unhealthy",
        window: MODBUS_UNHEALTHY_TIMEOUT,
        fault: FaultReason::ModbusUnhealthy,
        feed: |s, dt| fail_tick(s, b(OK_V, -0.1), dt),
    },
    DebouncedFault {
        // absorb_v for lfp_4s is 14.4 and the margin 0.2, so 14.7 trips.
        name: "overvoltage",
        window: OV_DURATION,
        fault: FaultReason::Overvoltage,
        feed: |s, dt| ok_tick(s, b(14.7, -0.1), dt),
    },
];

#[test]
fn nan_or_inf_in_sample_treated_as_missing() {
    // Within the missing-battery debounce, a non-finite sample is
    // ignored just like None — no fault yet, no phase change, but no
    // charitable bypass either.
    let mut s = active(lfp_4s());
    for v in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(matches!(ok_tick(&mut s, b(OK_V, v), TICK), Action::None));
        assert!(matches!(ok_tick(&mut s, b(v, -1.0), TICK), Action::None));
    }
    assert_eq!(s.state(), ChargeState::Float);
    assert_eq!(s.fault(), None);
}

#[test]
fn nan_voltage_eventually_latches_battery_stale() {
    // Sustained NaN voltage = sensor stuck. Must NOT silently bypass OV
    // or the phase machine — must drive through the same sensor-stale
    // path as truly missing samples.
    let mut s = active(lfp_4s());
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        assert!(matches!(
            ok_tick(&mut s, b(f32::NAN, -0.1), TICK),
            Action::None
        ));
    }
    assert_eq!(s.fault(), None);
    let a = ok_tick(&mut s, b(f32::NAN, -0.1), TICK);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
}

#[test]
fn nan_current_eventually_latches_battery_stale() {
    // Same as above but for current. A stuck-NaN current sensor would
    // otherwise silently hold whatever phase we were in.
    let mut s = active(lfp_4s());
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        ok_tick(&mut s, b(OK_V, f32::NAN), TICK);
    }
    assert_eq!(s.fault(), None);
    let a = ok_tick(&mut s, b(OK_V, f32::NAN), TICK);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
}

#[test]
fn nan_then_recovery_clears_stale_debounce() {
    // A brief NaN burst followed by recovery must NOT latch — the
    // debounce should reset on the first finite sample.
    let mut s = active(lfp_4s());
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        ok_tick(&mut s, b(f32::NAN, f32::NAN), TICK);
    }
    // One finite tick clears the debounce.
    ok_tick(&mut s, b(OK_V, -0.1), TICK);
    // Now we can NaN-burst again without latching.
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        ok_tick(&mut s, b(f32::NAN, -0.1), TICK);
    }
    assert_eq!(s.fault(), None);
}

#[test]
fn ov_trip_accumulates_elapsed_time_not_tick_count() {
    // Same budget reached three ways. A regression that counts calls
    // instead of `+= elapsed` passes the first row and fails the others.
    let ms = Duration::from_millis;
    let cases: [(&str, &[Duration]); 2] = [
        ("6 x 500 ms = 3.0 s", &[ms(500); 6]),
        ("1500 + 1000 + 600 = 3100 ms", &[ms(1500), ms(1000), ms(600)]),
    ];
    for (label, steps) in cases {
        let mut s = active(lfp_4s());
        let (trip, before) = steps.split_last().expect("each row trips on its last tick");
        for step in before {
            let a = ok_tick(&mut s, b(14.7, -0.1), *step);
            assert!(matches!(a, Action::None), "{label}: tripped early");
        }
        assert!(
            matches_disable(&ok_tick(&mut s, b(14.7, -0.1), *trip), FaultReason::Overvoltage),
            "{label}: did not trip at the budget"
        );
    }
}

#[test]
fn ov_below_threshold_does_not_trip() {
    // absorb_v + OV_MARGIN_V ≈ 14.6. 14.55 is unambiguously below in f32.
    let mut s = active(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() + 5) {
        ok_tick(&mut s, b(14.55, -0.1), TICK);
    }
    assert_eq!(s.fault(), None);
}

#[test]
fn debounced_faults_latch_only_after_their_full_window() {
    for c in DEBOUNCED_FAULTS {
        let mut s = active(lfp_4s());
        for _ in 0..(c.window.as_secs() - 1) {
            let a = (c.feed)(&mut s, TICK);
            assert!(matches!(a, Action::None), "{}: latched early", c.name);
        }
        assert_eq!(s.fault(), None, "{}", c.name);
        let a = (c.feed)(&mut s, TICK);
        assert!(matches_disable(&a, c.fault), "{}: no latch at the window", c.name);
        assert_eq!(s.fault(), Some(c.fault), "{}", c.name);

        // The window is elapsed time, not a tick count: one tick spanning
        // the whole of it latches just the same.
        let mut s = active(lfp_4s());
        let a = (c.feed)(&mut s, c.window);
        assert!(
            matches_disable(&a, c.fault),
            "{}: single big-elapsed tick did not latch",
            c.name
        );
    }
}

#[test]
fn debounced_faults_reset_on_a_single_healthy_tick() {
    // One healthy tick is a fresh sample, a successful read, and a voltage
    // under the line all at once, so the same tick clears whichever window
    // happens to be accumulating.
    for c in DEBOUNCED_FAULTS {
        let mut s = active(lfp_4s());
        for _ in 0..(c.window.as_secs() - 1) {
            (c.feed)(&mut s, TICK);
        }
        ok_tick(&mut s, b(OK_V, -0.1), TICK);
        // Counter is back to zero, so a whole window-minus-one fits again.
        for _ in 0..(c.window.as_secs() - 1) {
            let a = (c.feed)(&mut s, TICK);
            assert!(matches!(a, Action::None), "{}: window was not reset", c.name);
        }
        assert_eq!(s.fault(), None, "{}", c.name);
    }
}

#[test]
fn setpoint_drift_in_either_field_latches_immediately() {
    // Float targets for lfp_4s are 13.5 V and 10 A; the tolerance is
    // 0.02, so either of these is well past it. No debounce — the read
    // itself succeeded, so a mismatch is the device disagreeing with us.
    let drifted = [
        Setpoints { v_set: 12.0, i_set: 10.0 },
        Setpoints { v_set: 13.5, i_set: 5.0 },
    ];
    for sp in drifted {
        let mut s = active(lfp_4s());
        let p = PollResult {
            setpoints: Some(sp),
            ..expected_poll(&s, b(OK_V, -0.1))
        };
        assert!(
            matches_disable(&s.tick(p, TICK), FaultReason::SettingsDrift),
            "{sp:?}"
        );
    }
}

#[test]
fn latch_keeps_emitting_disable_until_acked() {
    let mut s = active(lfp_4s());
    for _ in 0..BATTERY_MISSING_TIMEOUT.as_secs() {
        ok_tick(&mut s, None, TICK);
    }
    assert!(s.fault().is_some());

    // First tick after latch: still wants disable.
    let a = ok_tick(&mut s, b(13.5, -0.1), TICK);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
    // Re-tick with healthy inputs: still disable (caller's set_output failed).
    let a = ok_tick(&mut s, b(13.5, -0.1), TICK);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));

    accept_disable(&mut s, a);
    // Now the supervisor goes quiet — no further commands to the buck.
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(13.5, -4.0), TICK), Action::None));
    }
}

#[test]
fn latched_supervisor_ignores_the_phase_machine() {
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, b(13.5, -0.1), TICK);
    }
    let a = ok_tick(&mut s, b(13.5, -0.1), TICK);
    accept_disable(&mut s, a);
    // Heavy charging current would normally drive Float→Absorb.
    ok_tick(&mut s, b(13.5, -5.0), TICK);
    assert_eq!(s.state(), ChargeState::Latched);
}

#[test]
#[should_panic]
fn commit_disable_without_fault_panics() {
    // Unreachable through the public API — a DisableTicket is only minted
    // by a tick that latched. Constructed here to prove the guard holds
    // for a ticket stashed across ticks.
    let mut s = active(lfp_4s());
    s.commit_disable(DisableTicket {
        reason: FaultReason::Overvoltage,
    });
}

#[test]
fn first_fault_wins_over_simultaneous_conditions() {
    // Both modbus and battery faulting at once. Modbus is checked first.
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, None, TICK);
    }
    assert_eq!(s.fault(), Some(FaultReason::ModbusUnhealthy));
}

#[test]
fn setpoint_within_tolerance_no_drift_fault() {
    // 0.01 V off — one register quantum, under the 0.02 tolerance.
    let mut s = active(lfp_4s());
    let p = PollResult {
        setpoints: Some(Setpoints {
            v_set: 13.51,
            i_set: 10.0,
        }),
        ..expected_poll(&s, b(13.5, -0.1))
    };
    assert!(matches!(s.tick(p, TICK), Action::None));
    assert_eq!(s.fault(), None);
}

#[test]
fn setpoint_drift_does_not_overwrite_existing_latch() {
    // First latch wins — modbus-unhealthy debounce trips before the next
    // good readback can latch SettingsDrift.
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, b(13.5, -0.1), TICK);
    }
    assert_eq!(s.fault(), Some(FaultReason::ModbusUnhealthy));
    let p = PollResult {
        setpoints: Some(Setpoints {
            v_set: 12.0,
            i_set: 10.0,
        }),
        ..expected_poll(&s, b(13.5, -0.1))
    };
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::ModbusUnhealthy
    ));
}

#[test]
fn drift_outranks_overvoltage_while_regulating() {
    // Both conditions true on the same tick. The gauntlet checks
    // setpoint drift (1) before overvoltage (5), and that precedence is
    // load-bearing: drift means we do not know what the buck is
    // regulating to, which is the more urgent of the two.
    let mut s = active(lfp_4s());
    let absorb = lfp_4s().absorb_v;
    let p = PollResult {
        setpoints: Some(Setpoints {
            v_set: 12.0,
            i_set: 10.0,
        }),
        ..expected_poll(&s, b(absorb + OV_MARGIN_V + 0.5, -0.1))
    };
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::SettingsDrift
    ));
}

#[test]
fn overcurrent_is_measured_on_the_pack_not_the_setpoint() {
    // I_SET bounds the buck's *total* output current, load included, so it
    // cannot hold the pack to its own rate: with an idle load the CC loop
    // puts the whole setpoint into the battery. Only the INA228 sees what
    // the pack takes, and `regulation_a` — not `i_set_a` — is the line.
    let profile = lfp_4s();
    let trip = profile.regulation_a * OVERCURRENT_TOL;

    // Just under holds indefinitely, well past the window.
    let mut s = active(profile);
    for _ in 0..(OVERCURRENT_DURATION.as_secs() * 4) {
        ok_tick(&mut s, b(OK_V, -(trip - 0.1)), TICK);
    }
    assert_eq!(s.fault(), None, "{trip} A is the line, not below it");

    // A brief burst is not a fault: the window resets on one healthy tick,
    // so the burst cannot accumulate across a quiet stretch.
    let mut s = active(profile);
    for _ in 0..(OVERCURRENT_DURATION.as_secs() - 1) {
        ok_tick(&mut s, b(OK_V, -(trip + 0.1)), TICK);
    }
    ok_tick(&mut s, b(OK_V, -1.0), TICK);
    for _ in 0..(OVERCURRENT_DURATION.as_secs() - 1) {
        ok_tick(&mut s, b(OK_V, -(trip + 0.1)), TICK);
    }
    assert_eq!(s.fault(), None, "the window did not reset");

    // A board budgeting a load programs a wider I_SET — here twice the
    // charge rate — and the pack's own limit must not move with it. That is
    // the whole point of measuring on the battery instead of the setpoint.
    let wide = ChargeSupervisor::new(profile, profile.regulation_a * 2.0);
    let mut s = bring_up(wide, profile.absorb_v);
    for _ in 0..(OVERCURRENT_DURATION.as_secs() - 1) {
        ok_tick(&mut s, b(OK_V, -(trip + 0.1)), TICK);
    }
    assert_eq!(s.fault(), None);
    let a = ok_tick(&mut s, b(OK_V, -(trip + 0.1)), TICK);
    accept_park(&s, a, FaultReason::ChargeOvercurrent);
}

#[test]
fn discharge_is_never_an_overcurrent() {
    // Sign convention: the check reads charging current, so a pack pushing
    // current *out* under a heavy load must never look like one taking too
    // much in.
    let mut s = active(lfp_4s());
    for _ in 0..(OVERCURRENT_DURATION.as_secs() * 4) {
        ok_tick(&mut s, b(OK_V, 40.0), TICK);
    }
    assert_eq!(s.fault(), None);
}

#[test]
fn output_disagreement_outranks_setpoint_drift() {
    // Both wrong on the same tick. What OUTPUT_EN is doing is the more
    // urgent fact of the two: a buck that is off is not regulating to
    // anything, so "we do not know its setpoints" says strictly less; and a
    // buck that is on while we believe it off is sourcing under setpoints
    // we never confirmed, which used to merely *inhibit* on the drift and
    // go on waiting with the output live.
    let drifted = Some(Setpoints {
        v_set: 12.0,
        i_set: 10.0,
    });

    let mut s = active(lfp_4s());
    let p = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Ocp,
        }),
        setpoints: drifted,
        battery: b(OK_V, -0.1),
    };
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Ocp)
    ));

    let mut s = supervisor(lfp_4s());
    let p = PollResult {
        output: Some(BuckOutput::On),
        setpoints: drifted,
        battery: b(OK_V, -0.1),
    };
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::OutputOnInPending
    ));
}

#[test]
fn buck_self_disable_in_active_latches() {
    // Active supervisor + buck reports output OFF (own OVP/OCP/LVP/over-temp
    // tripped, or panel toggled) → latch OutputUnexpectedlyOff.
    let mut s = active(lfp_4s());
    let p = poll_with_output(&s, BuckOutput::Off { cause: ProtectionStatus::Normal });
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Normal)
    ));
}

#[test]
fn latched_fault_stays_parked_in_none() {
    // Reboot-only recovery: once a fault latches and is acked, tick
    // returns Action::None forever — no Action::RestartSupervisor,
    // regardless of how long the world looks healthy. The caller's
    // reboot is the only way out (LVP/OTP are intercepted before
    // latching and don't reach here).
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, ProtectionStatus::Ocp);
    let p = expected_poll(&s, b(OK_V, -0.1));
    for _ in 0..600 {
        assert!(matches!(s.tick(p, TICK), Action::None));
    }
    assert!(matches!(
        s.fault(),
        Some(FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Ocp))
    ));
}

#[test]
fn latched_supervisor_re_disables_a_resurfaced_output() {
    // A latch is only as good as the output actually being off. Someone
    // presses the front panel, or the buck re-enables itself: the fault
    // that latched it has not gone anywhere, so the supervisor must say so
    // again rather than sit in Action::None watching a pack charge.
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, ProtectionStatus::Ocp);
    let fault = FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Ocp);
    // Drain bring-up and the original latch so what is asserted below is
    // the re-disables alone.
    drain_transitions(&mut s);

    // Two full episodes, to prove the cycle is stable rather than a
    // one-shot that leaves the machine somewhere odd.
    for episode in 0..2 {
        let a = s.tick(poll_with_output(&s, BuckOutput::On), TICK);
        assert!(matches_disable(&a, fault), "episode {episode}");
        // The ticket goes uncommitted, so — exactly as on the first latch
        // — the next tick asks again.
        let a = s.tick(poll_with_output(&s, BuckOutput::On), TICK);
        assert!(matches_disable(&a, fault), "episode {episode}: no retry");
        accept_disable(&mut s, a);
        assert_eq!(s.state(), ChargeState::Latched, "episode {episode}");
        // Output confirmed off again: quiet, and still the same fault.
        let p = poll_with_output(
            &s,
            BuckOutput::Off {
                cause: ProtectionStatus::Normal,
            },
        );
        assert!(matches!(s.tick(p, TICK), Action::None), "episode {episode}");
        assert_eq!(s.fault(), Some(fault), "episode {episode}");
    }

    assert_eq!(
        drain_transitions(&mut s),
        [ChargeTransition::Latched, ChargeTransition::Latched],
        "a buck that keeps resurfacing must keep showing up in the log"
    );
}

#[test]
fn buck_output_on_in_pending_latches() {
    // Boot expects the buck OFF — boot_sequence wrote set_output(false)
    // and S_INI=0. If OUTPUT_EN reads ON anyway, regulation is happening
    // under unknown conditions; latch immediately, no debounce.
    let mut s = supervisor(lfp_4s());
    let p = poll_with_output(&s, BuckOutput::On);
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::OutputOnInPending
    ));
}

#[test]
fn boot_pending_with_buck_on_still_latches() {
    // At cold boot, boot_sequence already wrote set_output(false) and
    // verified OUTPUT_EN=0 — so a poll showing buck=On is a genuine
    // anomaly (firmware bug / panel toggle / EMI). Stays the immediate
    // latch it always was; only ProtectRecovery gets the soft transition.
    let mut s = supervisor(lfp_4s());
    let p_on = poll_with_output(&s, BuckOutput::On);
    assert!(matches_disable(
        &s.tick(p_on, TICK),
        FaultReason::OutputOnInPending
    ));
}

#[test]
fn labels_are_the_snake_case_wire_identifiers() {
    // `/api` publishes these verbatim and dashboards match on them, so the
    // strings are a wire format: pinned here against the literals rather
    // than re-derived from the same `IntoStaticStr` that produces them.
    let faults: [(FaultReason, &str); 10] = [
        (FaultReason::BatterySensorStale, "battery_sensor_stale"),
        (FaultReason::ModbusUnhealthy, "modbus_unhealthy"),
        (FaultReason::Overvoltage, "overvoltage"),
        (FaultReason::AbsorbTimeout, "absorb_timeout"),
        (FaultReason::ChargeTimeout, "charge_timeout"),
        (FaultReason::ChargeOvercurrent, "charge_overcurrent"),
        (FaultReason::ProtectionFlapping, "protection_flapping"),
        (FaultReason::SettingsDrift, "settings_drift"),
        (
            FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Ovp),
            "output_unexpectedly_off",
        ),
        (FaultReason::OutputOnInPending, "output_on_in_pending"),
    ];
    for (reason, want) in faults {
        assert_eq!(reason.label(), want, "{reason:?}");
    }

    let inhibits: [(InhibitReason, &str); 6] = [
        (InhibitReason::SettingsDrift, "settings_drift"),
        (InhibitReason::ModbusUnhealthy, "modbus_unhealthy"),
        (InhibitReason::BatterySensorStale, "battery_sensor_stale"),
        (InhibitReason::NoBatterySample, "no_battery_sample"),
        (InhibitReason::Overvoltage, "overvoltage"),
        (
            InhibitReason::BuckProtection(ProtectionStatus::Lvp),
            "buck_protection",
        ),
    ];
    for (reason, want) in inhibits {
        assert_eq!(reason.label(), want, "{reason:?}");
    }

    // A payload must not leak into the identifier — only the Display form
    // names the cause.
    assert_eq!(
        FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Ovp).label(),
        FaultReason::OutputUnexpectedlyOff(ProtectionStatus::Otp).label()
    );
}
