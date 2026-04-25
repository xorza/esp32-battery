use super::*;

fn lfp_4s() -> Profile {
    // Pack-level defaults: 1 A enter, 0.5 A exit. Real CC-CV chargers
    // terminate absorb at 0.05C (5 A on 100 Ah), but they enter absorb on
    // VOLTAGE — we enter on current, which forces enter > exit so we don't
    // flap. 0.5 A keeps a usable hysteresis band without sitting at CV
    // forever. Absorb voltage (14.4 V) is the bigger longevity win.
    Profile::for_pack(Chemistry::LiFePo4, 4, 10.0, 1.0, 0.5)
}

fn approx(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-4
}

// --- Profile construction ---

#[test]
fn lfp_4s_voltages_match_known_setpoints() {
    let p = lfp_4s();
    // 3.60 V × 4 = 14.4 V CV (daily-cycle); 3.375 V × 4 = 13.5 V float.
    assert!(approx(p.absorb_v, 14.4));
    assert!(approx(p.float_v, 13.5));
}

#[test]
fn lfp_top_balance_uses_manufacturer_max() {
    // 3.65 V/cell — used only when BMS needs the headroom to balance.
    let p = Profile::for_pack(Chemistry::LiFePo4TopBalance, 4, 10.0, 2.0, 0.5);
    assert!(approx(p.absorb_v, 14.6));
    assert!(approx(p.float_v, 13.5));
}

#[test]
fn liion_3s_voltages_match_known_setpoints() {
    let p = Profile::for_pack(Chemistry::LiIon, 3, 10.0, 1.0, 0.1);
    // Longevity-tuned: 4.10 × 3 = 12.3, 4.00 × 3 = 12.0.
    assert!(approx(p.absorb_v, 12.3));
    assert!(approx(p.float_v, 12.0));
}

#[test]
fn voltages_scale_with_cell_count() {
    let p1 = Profile::for_pack(Chemistry::LiFePo4, 1, 10.0, 1.0, 0.1);
    let p4 = Profile::for_pack(Chemistry::LiFePo4, 4, 10.0, 1.0, 0.1);
    let p16 = Profile::for_pack(Chemistry::LiFePo4, 16, 10.0, 1.0, 0.1);
    assert!(approx(p1.absorb_v, 3.60));
    assert!(approx(p4.absorb_v, 3.60 * 4.0));
    assert!(approx(p16.absorb_v, 3.60 * 16.0));
    assert!(approx(p1.float_v, 3.375));
    assert!(approx(p4.float_v, 3.375 * 4.0));
    assert!(approx(p16.float_v, 3.375 * 16.0));
}

#[test]
fn currents_do_not_scale_with_cell_count() {
    // Pack-level current — independent of S.
    let p4 = Profile::for_pack(Chemistry::LiFePo4, 4, 10.0, 2.5, 0.25);
    let p16 = Profile::for_pack(Chemistry::LiFePo4, 16, 10.0, 2.5, 0.25);
    assert_eq!(p4.enter_absorb_a, 2.5);
    assert_eq!(p16.enter_absorb_a, 2.5);
    assert_eq!(p4.exit_absorb_a, 0.25);
    assert_eq!(p16.exit_absorb_a, 0.25);
}

#[test]
#[should_panic]
fn zero_cells_panics() {
    let _ = Profile::for_pack(Chemistry::LiFePo4, 0, 10.0, 1.0, 0.1);
}

#[test]
#[should_panic]
fn enter_must_exceed_exit() {
    let _ = Profile::for_pack(Chemistry::LiFePo4, 4, 10.0, 0.5, 1.0);
}

#[test]
#[should_panic]
fn regulation_must_exceed_enter() {
    // regulation_a == enter_absorb_a would mean we'd enter Absorb at the
    // same current we're regulating to — unstable. Strict inequality.
    let _ = Profile::for_pack(Chemistry::LiFePo4, 4, 1.0, 1.0, 0.5);
}

#[test]
fn safety_limits_match_lfp_4s_known_values() {
    // 4S LFP daily: absorb 14.4 V, float 13.5 V, CC 10 A.
    // OVP = 14.4 + 0.6 = 15.0; OCP = 10 * 1.5 = 15.0.
    // LVP is input UVLO on the XY7025: 24 V nominal − 2 V margin = 22 V.
    let s = lfp_4s().safety_limits();
    assert!(approx(s.ovp_v, 15.0));
    assert!(approx(s.ocp_a, 15.0));
    assert!(approx(s.lvp_v, 22.0));
}

#[test]
fn safety_limits_track_chemistry_change() {
    // Top-balance pushes absorb to 14.6 V — OVP must move up too, not stay
    // at the 4S-daily 15.0 V. Without derived limits this is the footgun.
    let s = Profile::for_pack(Chemistry::LiFePo4TopBalance, 4, 10.0, 2.0, 0.5).safety_limits();
    assert!(approx(s.ovp_v, 15.2));
    assert!(s.ovp_v > 14.6, "OVP must clear absorb_v");
}

#[test]
fn safety_limits_track_cell_count_change() {
    let s4 = Profile::for_pack(Chemistry::LiFePo4, 4, 10.0, 1.0, 0.1).safety_limits();
    let s8 = Profile::for_pack(Chemistry::LiFePo4, 8, 10.0, 1.0, 0.1).safety_limits();
    assert!(s8.ovp_v > s4.ovp_v, "OVP scales with cell count");
    // OCP is current-only and LVP is input-side — both independent of S.
    assert!(approx(s4.ocp_a, s8.ocp_a));
    assert!(approx(s4.lvp_v, s8.lvp_v));
}

#[test]
fn safety_limits_ovp_clears_supervisor_threshold() {
    // The supervisor's OV detection trips at absorb_v + OV_MARGIN_V.
    // The hardware OVP must sit strictly above that — supervisor first,
    // hardware backstop second.
    for (chem, cells, reg) in [
        (Chemistry::LiFePo4, 4, 10.0),
        (Chemistry::LiFePo4TopBalance, 4, 10.0),
        (Chemistry::LiIon, 3, 5.0),
    ] {
        let p = Profile::for_pack(chem, cells, reg, 1.0, 0.1);
        let s = p.safety_limits();
        let supervisor_trip = p.absorb_v + OV_MARGIN_V;
        assert!(
            s.ovp_v > supervisor_trip,
            "ovp {} must exceed supervisor trip {}",
            s.ovp_v,
            supervisor_trip
        );
    }
}

// --- Controller behavior ---

#[test]
fn starts_in_float_at_float_voltage() {
    let c = ChargeController::new(lfp_4s());
    assert!(matches!(c.phase(), Phase::Float));
    assert!(approx(c.target_voltage(), 13.5));
}

/// Sub-OV-threshold voltage used by tests that only care about phase logic.
const OK_V: f32 = 13.5;
/// Wall time elapsed per simulated tick. Tests choose 1 s so iteration
/// counts read as seconds when comparing against duration budgets.
const TICK: Duration = Duration::from_secs(1);

#[test]
fn enters_absorb_when_charging_current_exceeds_threshold() {
    let mut c = ChargeController::new(lfp_4s());
    // charging at 1.5 A → -1.5 A on the bus; threshold is 1.0 A.
    assert!(matches!(
        c.update(OK_V, -1.5, TICK),
        Decision::Setpoint(v) if approx(v, 14.4)
    ));
    assert!(matches!(c.phase(), Phase::Absorb));
}

#[test]
fn does_not_enter_absorb_at_exact_threshold() {
    // Strictly greater: 1.0 A must NOT trigger; 1.001 A must.
    let mut c = ChargeController::new(lfp_4s());
    assert!(matches!(c.update(OK_V, -1.0, TICK), Decision::NoChange));
    assert!(matches!(c.phase(), Phase::Float));
    assert!(matches!(
        c.update(OK_V, -1.001, TICK),
        Decision::Setpoint(_)
    ));
}

#[test]
fn discharge_current_does_not_enter_absorb() {
    // 5 A discharge (positive). |I| > 1 A but it's NOT charging.
    let mut c = ChargeController::new(lfp_4s());
    assert!(matches!(c.update(OK_V, 5.0, TICK), Decision::NoChange));
    assert!(matches!(c.phase(), Phase::Float));
}

#[test]
fn stays_in_absorb_above_exit_threshold() {
    let mut c = ChargeController::new(lfp_4s());
    c.update(OK_V, -2.0, TICK); // → Absorb
    // Exit threshold is 0.5 A — anything above keeps us in absorb.
    assert!(matches!(c.update(OK_V, -1.0, TICK), Decision::NoChange));
    assert!(matches!(c.update(OK_V, -0.6, TICK), Decision::NoChange));
    assert!(matches!(c.update(OK_V, -0.5, TICK), Decision::NoChange)); // strictly less-than, so 0.5 stays
    assert!(matches!(c.phase(), Phase::Absorb));
}

#[test]
fn exits_absorb_when_taper_drops_below_threshold() {
    let mut c = ChargeController::new(lfp_4s());
    c.update(OK_V, -2.0, TICK); // → Absorb
    // 0.4 A charging — below 0.5 A exit.
    assert!(matches!(
        c.update(OK_V, -0.4, TICK),
        Decision::Setpoint(v) if approx(v, 13.5)
    ));
    assert!(matches!(c.phase(), Phase::Float));
}

#[test]
fn exits_absorb_when_load_pulls_current() {
    // Battery starts discharging mid-absorb (charger off / heavy load).
    // charging_a is negative → certainly < 0.1 A → drop to float.
    let mut c = ChargeController::new(lfp_4s());
    c.update(OK_V, -2.0, TICK);
    assert!(matches!(
        c.update(OK_V, 3.0, TICK),
        Decision::Setpoint(v) if approx(v, 13.5)
    ));
}

#[test]
fn hysteresis_no_flap_between_thresholds() {
    let mut c = ChargeController::new(lfp_4s());
    for _ in 0..10 {
        assert!(matches!(c.update(OK_V, -0.5, TICK), Decision::NoChange));
    }
    assert!(matches!(c.phase(), Phase::Float));
    c.update(OK_V, -2.0, TICK);
    for _ in 0..10 {
        assert!(matches!(c.update(OK_V, -0.5, TICK), Decision::NoChange));
    }
    assert!(matches!(c.phase(), Phase::Absorb));
}

#[test]
fn returns_none_on_steady_state() {
    let mut c = ChargeController::new(lfp_4s());
    for _ in 0..100 {
        assert!(matches!(c.update(OK_V, -0.05, TICK), Decision::NoChange));
    }
}

#[test]
fn transition_only_emits_setpoint_once() {
    let mut c = ChargeController::new(lfp_4s());
    assert!(matches!(c.update(OK_V, -2.0, TICK), Decision::Setpoint(_))); // first crossing → write
    assert!(matches!(c.update(OK_V, -2.0, TICK), Decision::NoChange)); // already absorb → silent
    assert!(matches!(c.update(OK_V, -3.0, TICK), Decision::NoChange));
}

#[test]
fn nan_and_inf_current_are_ignored() {
    let mut c = ChargeController::new(lfp_4s());
    assert!(matches!(c.update(OK_V, f32::NAN, TICK), Decision::NoChange));
    assert!(matches!(
        c.update(OK_V, f32::INFINITY, TICK),
        Decision::NoChange
    ));
    assert!(matches!(
        c.update(OK_V, f32::NEG_INFINITY, TICK),
        Decision::NoChange
    ));
    assert!(matches!(c.phase(), Phase::Float));
}

#[test]
fn different_chemistries_yield_different_setpoints() {
    let mut lfp = ChargeController::new(Profile::for_pack(Chemistry::LiFePo4, 4, 10.0, 1.0, 0.1));
    let mut liion = ChargeController::new(Profile::for_pack(Chemistry::LiIon, 3, 10.0, 1.0, 0.1));
    let Decision::Setpoint(v_lfp) = lfp.update(OK_V, -2.0, TICK) else {
        panic!("expected setpoint")
    };
    let Decision::Setpoint(v_liion) = liion.update(12.0, -2.0, TICK) else {
        panic!("expected setpoint")
    };
    assert!(approx(v_lfp, 14.4));
    assert!(approx(v_liion, 12.3));
}

#[test]
fn single_cell_lfp_works() {
    // 1S LFP charger — float 3.375 V, absorb 3.60 V (daily).
    let mut c = ChargeController::new(Profile::for_pack(Chemistry::LiFePo4, 1, 10.0, 1.0, 0.1));
    assert!(approx(c.target_voltage(), 3.375));
    assert!(matches!(
        c.update(3.4, -1.5, TICK),
        Decision::Setpoint(v) if approx(v, 3.60)
    ));
}

#[test]
fn full_charge_cycle() {
    let mut c = ChargeController::new(lfp_4s());
    // Exit threshold is 0.5 A — design taper around that.
    assert!(matches!(
        c.update(OK_V, -8.0, TICK),
        Decision::Setpoint(v) if approx(v, 14.4)
    ));
    for &i in &[-7.0, -5.0, -3.0, -1.0, -0.6] {
        assert!(matches!(c.update(OK_V, i, TICK), Decision::NoChange));
    }
    assert!(matches!(
        c.update(OK_V, -0.4, TICK),
        Decision::Setpoint(v) if approx(v, 13.5)
    ));
    for &i in &[-0.05, -0.02, 0.0, -0.4] {
        assert!(matches!(c.update(OK_V, i, TICK), Decision::NoChange));
    }
}

// --- Controller-level overvoltage detection ---

#[test]
fn ov_emits_fault_after_budget() {
    // absorb_v for lfp_4s = 14.4; margin = 0.2; so > 14.6 trips after BUDGET ticks.
    let mut c = ChargeController::new(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() - 1) {
        assert!(matches!(c.update(14.7, -0.1, TICK), Decision::NoChange));
    }
    assert!(matches!(
        c.update(14.7, -0.1, TICK),
        Decision::Fault(FaultReason::Overvoltage)
    ));
}

#[test]
fn ov_counter_resets_when_voltage_drops() {
    let mut c = ChargeController::new(lfp_4s());
    c.update(14.7, -0.1, TICK);
    c.update(14.7, -0.1, TICK);
    c.update(13.5, -0.1, TICK); // resets
    // Two more above must NOT fault — counter started over.
    assert!(matches!(c.update(14.7, -0.1, TICK), Decision::NoChange));
    assert!(matches!(c.update(14.7, -0.1, TICK), Decision::NoChange));
}

#[test]
fn ov_nan_voltage_does_not_count() {
    let mut c = ChargeController::new(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() + 5) {
        assert!(matches!(c.update(f32::NAN, -0.1, TICK), Decision::NoChange));
    }
}

// --- Elapsed honored, not tick count ---

#[test]
fn ov_fault_honors_sub_second_elapsed() {
    // 500 ms ticks. Five of them = 2.5 s — under the 3 s budget, no fault.
    // Sixth tick brings the accumulated time to exactly OV_DURATION → fault.
    let mut c = ChargeController::new(lfp_4s());
    let step = Duration::from_millis(500);
    for _ in 0..5 {
        assert!(matches!(c.update(14.7, -0.1, step), Decision::NoChange));
    }
    assert!(matches!(
        c.update(14.7, -0.1, step),
        Decision::Fault(FaultReason::Overvoltage)
    ));
}

#[test]
fn ov_fault_in_a_single_large_elapsed_tick() {
    // One call covering the full OV budget — should fault immediately. Catches
    // a regression that accumulates +1 per call instead of `+= elapsed`.
    let mut c = ChargeController::new(lfp_4s());
    assert!(matches!(
        c.update(14.7, -0.1, OV_DURATION),
        Decision::Fault(FaultReason::Overvoltage)
    ));
}

#[test]
fn ov_fault_with_mixed_elapsed_values() {
    // 1500 ms + 1000 ms + 600 ms = 3100 ms ≥ 3000 ms budget. Trips on the
    // third tick, not earlier (cumulative time, not tick count).
    let mut c = ChargeController::new(lfp_4s());
    assert!(matches!(
        c.update(14.7, -0.1, Duration::from_millis(1500)),
        Decision::NoChange
    ));
    assert!(matches!(
        c.update(14.7, -0.1, Duration::from_millis(1000)),
        Decision::NoChange
    ));
    assert!(matches!(
        c.update(14.7, -0.1, Duration::from_millis(600)),
        Decision::Fault(FaultReason::Overvoltage)
    ));
}

#[test]
fn absorb_timeout_in_a_single_large_elapsed_tick() {
    // After entering Absorb, one tick covering the full cap must fault.
    // Equivalent to the iteration-based test but proves elapsed is honored.
    let mut c = ChargeController::new(lfp_4s());
    enter_absorb(&mut c);
    assert!(matches!(
        c.update(OK_V, -1.0, MAX_ABSORB),
        Decision::Fault(FaultReason::AbsorbTimeout)
    ));
}

#[test]
fn battery_stale_honors_elapsed() {
    // One tick with elapsed = BATTERY_MISSING_TIMEOUT must latch.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = s.tick(true, None, BATTERY_MISSING_TIMEOUT);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
}

#[test]
fn modbus_unhealthy_honors_elapsed() {
    // Same: one big-elapsed tick saturates the modbus error counter.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = s.tick(
        false,
        Some(BatterySample {
            voltage: 13.5,
            current: -0.1,
        }),
        MODBUS_UNHEALTHY_TIMEOUT,
    );
    assert!(matches_disable(&a, FaultReason::ModbusUnhealthy));
}

// --- Absorb time cap ---

/// Drive the controller into Absorb. After this, exactly one Absorb tick
/// has elapsed (the transition itself).
fn enter_absorb(c: &mut ChargeController) {
    assert!(matches!(c.update(OK_V, -2.0, TICK), Decision::Setpoint(_)));
    assert!(matches!(c.phase(), Phase::Absorb));
}

#[test]
fn absorb_does_not_time_out_below_budget() {
    let mut c = ChargeController::new(lfp_4s());
    enter_absorb(&mut c);
    // Hold absorb just shy of the cap. Current pinned above exit threshold
    // (0.5 A) so we never drop to Float on our own.
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        assert!(matches!(c.update(OK_V, -1.0, TICK), Decision::NoChange));
    }
}

#[test]
fn absorb_times_out_at_budget() {
    let mut c = ChargeController::new(lfp_4s());
    enter_absorb(&mut c);
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        c.update(OK_V, -1.0, TICK);
    }
    assert!(matches!(
        c.update(OK_V, -1.0, TICK),
        Decision::Fault(FaultReason::AbsorbTimeout)
    ));
}

#[test]
fn float_does_not_accumulate_absorb_ticks() {
    let mut c = ChargeController::new(lfp_4s());
    // Sit in Float for far longer than the absorb cap — must never fault.
    for _ in 0..(MAX_ABSORB.as_secs() + 10) {
        assert!(matches!(c.update(OK_V, -0.1, TICK), Decision::NoChange));
    }
    assert!(matches!(c.phase(), Phase::Float));
}

#[test]
fn absorb_counter_resets_on_taper_back_to_float() {
    let mut c = ChargeController::new(lfp_4s());
    enter_absorb(&mut c);
    // Spend most of the budget in absorb…
    for _ in 0..(MAX_ABSORB.as_secs() - 10) {
        c.update(OK_V, -1.0, TICK);
    }
    // …then taper to Float. Counter must reset.
    assert!(matches!(c.update(OK_V, -0.1, TICK), Decision::Setpoint(_)));
    assert!(matches!(c.phase(), Phase::Float));

    // Re-enter Absorb and burn the original margin's worth of ticks.
    // No fault yet — counter started over.
    enter_absorb(&mut c);
    for _ in 0..20 {
        assert!(matches!(c.update(OK_V, -1.0, TICK), Decision::NoChange));
    }
}

// --- Supervisor ---

fn matches_disable(a: &Action, expected: FaultReason) -> bool {
    matches!(a, Action::DisableOutput(r) if *r == expected)
}

#[test]
fn supervisor_passes_setpoint_through_on_phase_transition() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -2.0,
        }),
        TICK,
    );
    match a {
        Action::SetVoltage(v) => assert!(approx(v, 14.4)),
        _ => panic!("expected SetVoltage"),
    }
    assert!(matches!(s.phase(), Phase::Absorb));
    assert!(s.fault().is_none());
}

#[test]
fn supervisor_returns_none_on_steady_state() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..50 {
        assert!(matches!(
            s.tick(
                true,
                Some(BatterySample {
                    voltage: 13.5,
                    current: -0.05
                }),
                TICK,
            ),
            Action::None
        ));
    }
    assert!(s.fault().is_none());
}

#[test]
fn battery_stale_for_budget_latches() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    // BUDGET-1 ticks of missing battery: still healthy.
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        assert!(matches!(s.tick(true, None, TICK), Action::None));
    }
    assert!(s.fault().is_none());

    let a = s.tick(true, None, TICK);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
    assert!(matches!(s.fault(), Some(FaultReason::BatterySensorStale)));
}

#[test]
fn battery_recovers_within_budget_no_latch() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        s.tick(true, None, TICK);
    }
    // One fresh reading clears the counter.
    s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -0.1,
        }),
        TICK,
    );
    // Now we should be able to miss BUDGET-1 again without latching.
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        assert!(matches!(s.tick(true, None, TICK), Action::None));
    }
    assert!(s.fault().is_none());
}

#[test]
fn modbus_errors_for_budget_latches() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        assert!(matches!(
            s.tick(
                false,
                Some(BatterySample {
                    voltage: 13.5,
                    current: -0.1
                }),
                TICK,
            ),
            Action::None
        ));
    }
    let a = s.tick(
        false,
        Some(BatterySample {
            voltage: 13.5,
            current: -0.1,
        }),
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::ModbusUnhealthy));
}

#[test]
fn modbus_recovers_within_budget_no_latch() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        s.tick(
            false,
            Some(BatterySample {
                voltage: 13.5,
                current: -0.1,
            }),
            TICK,
        );
    }
    s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -0.1,
        }),
        TICK,
    ); // good read clears counter
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        s.tick(
            false,
            Some(BatterySample {
                voltage: 13.5,
                current: -0.1,
            }),
            TICK,
        );
    }
    assert!(s.fault().is_none());
}

#[test]
fn overvoltage_sustained_latches() {
    // absorb_v for lfp_4s = 14.4; margin = 0.2; so > 14.6 trips.
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() - 1) {
        assert!(matches!(
            s.tick(
                true,
                Some(BatterySample {
                    voltage: 14.7,
                    current: -0.1
                }),
                TICK,
            ),
            Action::None
        ));
    }
    let a = s.tick(
        true,
        Some(BatterySample {
            voltage: 14.7,
            current: -0.1,
        }),
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::Overvoltage));
}

#[test]
fn overvoltage_brief_recovers_no_latch() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    // Two ticks above OV; one tick back below. Counter resets.
    s.tick(
        true,
        Some(BatterySample {
            voltage: 14.7,
            current: -0.1,
        }),
        TICK,
    );
    s.tick(
        true,
        Some(BatterySample {
            voltage: 14.7,
            current: -0.1,
        }),
        TICK,
    );
    s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -0.1,
        }),
        TICK,
    );
    // Two more above must NOT latch (< budget after reset).
    s.tick(
        true,
        Some(BatterySample {
            voltage: 14.7,
            current: -0.1,
        }),
        TICK,
    );
    s.tick(
        true,
        Some(BatterySample {
            voltage: 14.7,
            current: -0.1,
        }),
        TICK,
    );
    assert!(s.fault().is_none());
}

#[test]
fn ov_below_threshold_does_not_trip() {
    // absorb_v + OV_MARGIN_V ≈ 14.6. 14.55 is unambiguously below in f32.
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() + 5) {
        s.tick(
            true,
            Some(BatterySample {
                voltage: 14.55,
                current: -0.1,
            }),
            TICK,
        );
    }
    assert!(s.fault().is_none());
}

#[test]
fn nan_voltage_does_not_count_toward_ov() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() + 5) {
        s.tick(
            true,
            Some(BatterySample {
                voltage: f32::NAN,
                current: -0.1,
            }),
            TICK,
        );
    }
    assert!(s.fault().is_none());
}

#[test]
fn latch_keeps_emitting_disable_until_acked() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..BATTERY_MISSING_TIMEOUT.as_secs() {
        s.tick(true, None, TICK);
    }
    assert!(s.fault().is_some());

    // First tick after latch: still wants disable.
    let a = s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -0.1,
        }),
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
    // Re-tick with healthy inputs: still disable (caller's set_output failed).
    let a = s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -0.1,
        }),
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));

    s.ack_disable();
    // Now the supervisor goes quiet — no further commands to the buck.
    for _ in 0..10 {
        assert!(matches!(
            s.tick(
                true,
                Some(BatterySample {
                    voltage: 13.5,
                    current: -2.0
                }),
                TICK,
            ),
            Action::None
        ));
    }
}

#[test]
fn latched_supervisor_does_not_change_phase() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        s.tick(
            false,
            Some(BatterySample {
                voltage: 13.5,
                current: -0.1,
            }),
            TICK,
        );
    }
    s.ack_disable();
    // Heavy charging current would normally drive Float→Absorb.
    s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -5.0,
        }),
        TICK,
    );
    assert!(matches!(s.phase(), Phase::Float));
}

#[test]
#[should_panic]
fn ack_disable_without_fault_panics() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    s.ack_disable();
}

#[test]
fn first_fault_wins_over_simultaneous_conditions() {
    // Both modbus and battery faulting at once. Modbus is checked first.
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        s.tick(false, None, TICK);
    }
    assert!(matches!(s.fault(), Some(FaultReason::ModbusUnhealthy)));
}

#[test]
fn supervisor_latches_on_absorb_timeout() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    // Drive into Absorb.
    let _ = s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -2.0,
        }),
        TICK,
    );
    // Hold Absorb until just before the cap. Current pinned above exit
    // threshold so the controller can't taper out on its own.
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        s.tick(
            true,
            Some(BatterySample {
                voltage: 13.5,
                current: -1.0,
            }),
            TICK,
        );
    }
    assert!(s.fault().is_none());

    let a = s.tick(
        true,
        Some(BatterySample {
            voltage: 13.5,
            current: -1.0,
        }),
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::AbsorbTimeout));
}
