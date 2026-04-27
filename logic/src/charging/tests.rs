use super::*;

/// 4S 50 Ah LFP — the board's actual pack. With the module's C-rate
/// constants this gives reg = 10 A, enter = 3 A, exit = 2.5 A. Tests that
/// exercise threshold edges expect those numbers.
fn lfp_4s() -> Profile {
    Profile::for_pack(Chemistry::LiFePo4, 4, 50.0)
}

fn approx(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-4
}

/// Sub-OV-threshold voltage used by tests that only care about phase logic.
const OK_V: f32 = 13.5;
/// Wall time elapsed per simulated tick. Tests choose 1 s so iteration
/// counts read as seconds when comparing against duration budgets.
const TICK: Duration = Duration::from_secs(1);

fn b(voltage: f32, current: f32) -> Option<BatterySample> {
    Some(BatterySample { voltage, current })
}

fn matches_disable(a: &Action, expected: FaultReason) -> bool {
    matches!(a, Action::DisableOutput(r) if *r == expected)
}

/// Drives the supervisor's recovery path: `tick` in `Tripped { acked: true }`
/// either returns `RestartSupervisor` (caller's cue to tear down) or `None`
/// (still accumulating / non-recoverable / non-healthy).
fn restart_ready(s: &mut ChargeSupervisor, p: PollResult, dt: Duration) -> bool {
    matches!(s.tick(p, dt), Action::RestartSupervisor)
}

/// Tick with a successful, drift-free Modbus readback where the buck
/// reports the output state the supervisor currently expects — the
/// common case. Phase transitions don't fire spurious `SettingsDrift`,
/// and Active ticks don't fire spurious `OutputUnexpectedlyOff`.
fn ok_tick(s: &mut ChargeSupervisor, battery: Option<BatterySample>, elapsed: Duration) -> Action {
    let output = if s.expected_output_on() {
        BuckOutput::On
    } else {
        BuckOutput::Off { cause: None }
    };
    let p = PollResult {
        setpoints: Some(s.expected_setpoints()),
        output: Some(output),
        battery,
    };
    let a = s.tick(p, elapsed);
    // Auto-ack voltage updates: this helper simulates the happy path
    // (every Modbus write succeeds), and a successful set_voltage write
    // is part of that. Tests of the retry-on-failure path use
    // `s.tick(...)` directly so they can skip the ack and verify the
    // re-emit on the next tick.
    if matches!(a, Action::UpdateVoltage { .. }) {
        s.ack_voltage_update();
    }
    a
}

/// Tick where the Modbus read failed — both `setpoints` and
/// `output_on` are None, exercising the modbus-unhealthy debounce path.
fn fail_tick(
    s: &mut ChargeSupervisor,
    battery: Option<BatterySample>,
    elapsed: Duration,
) -> Action {
    s.tick(
        PollResult {
            setpoints: None,
            output: None,
            battery,
        },
        elapsed,
    )
}

/// Build a supervisor and drive it through Pending → Active. Tests that
/// don't care about the bring-up dance use this; tests that exercise
/// Pending behavior call `ChargeSupervisor::new` directly.
fn active(profile: Profile) -> ChargeSupervisor {
    // Use the profile's own float_v for the bring-up sample so this
    // helper works for any chemistry / cell count without crossing OV.
    let bring_up_v = profile.float_v;
    let mut s = ChargeSupervisor::new(profile);
    let a = ok_tick(&mut s, b(bring_up_v, -0.1), TICK);
    assert!(matches!(a, Action::EnableOutput));
    s.ack_enable();
    s
}

// --- Profile construction ---

#[test]
fn lfp_4s_50ah_matches_hand_calculation() {
    // Exhaustive check of the production profile's derived fields against
    // hand math. If any C-rate constant or per-cell voltage is changed
    // accidentally, exactly this test fires.
    //
    // Inputs: chemistry = LiFePo4 (3.60 V/cell absorb, 3.375 V/cell float),
    //         cells = 4, capacity = 50 Ah.
    // C-rates: REGULATION_C = 0.20, ENTER_ABSORB_C = 0.06, EXIT_ABSORB_C = 0.05.
    // Hardware-OVP margin = OV_MARGIN_V × 3 = 0.2 × 3 = 0.6 V.
    // Input UVLO = INPUT_NOMINAL_V − INPUT_LVP_MARGIN_V = 24 − 2 = 22 V.
    //
    //   absorb_v       = 3.60   × 4   = 14.40 V
    //   float_v        = 3.375  × 4   = 13.50 V
    //   regulation_a   = 0.20   × 50  = 10.00 A
    //   enter_absorb_a = 0.06   × 50  =  3.00 A
    //   exit_absorb_a  = 0.05   × 50  =  2.50 A
    //   ovp_v          = 14.40  + 0.6 = 15.00 V
    //   ocp_a          = 10.00  × 1.5 = 15.00 A
    //   lvp_v          = 24     − 2   = 22.00 V
    let p = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
    assert!(approx(p.absorb_v, 14.40));
    assert!(approx(p.float_v, 13.50));
    assert!(approx(p.regulation_a, 10.00));
    assert!(approx(p.enter_absorb_a, 3.00));
    assert!(approx(p.exit_absorb_a, 2.50));
    let s = p.safety_limits();
    assert!(approx(s.ovp_v, 15.00));
    assert!(approx(s.ocp_a, 15.00));
    assert!(approx(s.lvp_v, 22.00));
    // Derived ordering invariants the supervisor relies on.
    assert!(p.absorb_v > p.float_v);
    assert!(p.regulation_a > p.enter_absorb_a);
    assert!(p.enter_absorb_a > p.exit_absorb_a);
    assert!(s.ovp_v > p.absorb_v + OV_MARGIN_V);
}

#[test]
fn lfp_4s_voltages_match_known_setpoints() {
    let p = lfp_4s();
    // 3.60 V × 4 = 14.4 V CV (daily-cycle); 3.375 V × 4 = 13.5 V float.
    assert!(approx(p.absorb_v, 14.4));
    assert!(approx(p.float_v, 13.5));
}

#[test]
fn lfp_4s_currents_derive_from_capacity() {
    // 50 Ah × {0.2C, 0.06C, 0.05C} = {10 A, 3 A, 2.5 A}.
    let p = lfp_4s();
    assert!(approx(p.regulation_a, 10.0));
    assert!(approx(p.enter_absorb_a, 3.0));
    assert!(approx(p.exit_absorb_a, 2.5));
}

#[test]
fn lfp_top_balance_uses_manufacturer_max() {
    // 3.65 V/cell — used only when BMS needs the headroom to balance.
    let p = Profile::for_pack(Chemistry::LiFePo4TopBalance, 4, 50.0);
    assert!(approx(p.absorb_v, 14.6));
    assert!(approx(p.float_v, 13.5));
}

#[test]
fn liion_3s_voltages_match_known_setpoints() {
    let p = Profile::for_pack(Chemistry::LiIon, 3, 50.0);
    // Longevity-tuned: 4.10 × 3 = 12.3, 4.00 × 3 = 12.0.
    assert!(approx(p.absorb_v, 12.3));
    assert!(approx(p.float_v, 12.0));
}

#[test]
fn voltages_scale_with_cell_count() {
    let p1 = Profile::for_pack(Chemistry::LiFePo4, 1, 50.0);
    let p4 = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
    let p16 = Profile::for_pack(Chemistry::LiFePo4, 16, 50.0);
    assert!(approx(p1.absorb_v, 3.60));
    assert!(approx(p4.absorb_v, 3.60 * 4.0));
    assert!(approx(p16.absorb_v, 3.60 * 16.0));
    assert!(approx(p1.float_v, 3.375));
    assert!(approx(p4.float_v, 3.375 * 4.0));
    assert!(approx(p16.float_v, 3.375 * 16.0));
}

#[test]
fn currents_scale_with_capacity_not_cells() {
    // Same capacity, different S → identical currents.
    let p4 = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
    let p16 = Profile::for_pack(Chemistry::LiFePo4, 16, 50.0);
    assert!(approx(p4.regulation_a, p16.regulation_a));
    assert!(approx(p4.enter_absorb_a, p16.enter_absorb_a));
    assert!(approx(p4.exit_absorb_a, p16.exit_absorb_a));
    // Same S, different capacity → currents scale linearly.
    let p100 = Profile::for_pack(Chemistry::LiFePo4, 4, 100.0);
    assert!(approx(p100.regulation_a, 2.0 * p4.regulation_a));
    assert!(approx(p100.enter_absorb_a, 2.0 * p4.enter_absorb_a));
    assert!(approx(p100.exit_absorb_a, 2.0 * p4.exit_absorb_a));
}

#[test]
#[should_panic]
fn zero_cells_panics() {
    let _ = Profile::for_pack(Chemistry::LiFePo4, 0, 50.0);
}

#[test]
#[should_panic]
fn zero_capacity_panics() {
    let _ = Profile::for_pack(Chemistry::LiFePo4, 4, 0.0);
}

#[test]
fn safety_limits_match_lfp_4s_known_values() {
    // 4S LFP daily, 50 Ah: absorb 14.4 V, float 13.5 V, CC 10 A.
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
    let s = Profile::for_pack(Chemistry::LiFePo4TopBalance, 4, 50.0).safety_limits();
    assert!(approx(s.ovp_v, 15.2));
    assert!(s.ovp_v > 14.6, "OVP must clear absorb_v");
}

#[test]
fn safety_limits_track_cell_count_change() {
    let s4 = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0).safety_limits();
    let s8 = Profile::for_pack(Chemistry::LiFePo4, 8, 50.0).safety_limits();
    assert!(s8.ovp_v > s4.ovp_v, "OVP scales with cell count");
    // OCP is current-only (and capacity-derived) and LVP is input-side —
    // both independent of S.
    assert!(approx(s4.ocp_a, s8.ocp_a));
    assert!(approx(s4.lvp_v, s8.lvp_v));
}

#[test]
fn safety_limits_ovp_clears_supervisor_threshold() {
    // The supervisor's OV detection trips at absorb_v + OV_MARGIN_V.
    // The hardware OVP must sit strictly above that — supervisor first,
    // hardware backstop second.
    for (chem, cells) in [
        (Chemistry::LiFePo4, 4),
        (Chemistry::LiFePo4TopBalance, 4),
        (Chemistry::LiIon, 3),
    ] {
        let p = Profile::for_pack(chem, cells, 50.0);
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

// --- Phase machine ---
//
// All currents below are tuned to the 50 Ah pack: enter > 3 A, exit < 2.5 A.

#[test]
fn starts_in_float_at_float_voltage() {
    let s = ChargeSupervisor::new(lfp_4s());
    assert!(matches!(s.phase(), Phase::Float));
    assert!(approx(s.target_voltage(), 13.5));
}

#[test]
fn enters_absorb_when_charging_current_exceeds_threshold() {
    let mut s = active(lfp_4s());
    // Charging at 4 A → -4 A on the bus; threshold is 3 A.
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(matches!(s.phase(), Phase::Absorb));
    assert!(approx(s.target_voltage(), 14.4));
}

#[test]
fn does_not_enter_absorb_at_exact_threshold() {
    // Strictly greater: 3.0 A must NOT trigger; 3.001 A must.
    let mut s = active(lfp_4s());
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    assert!(matches!(s.phase(), Phase::Float));
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
    assert!(matches!(s.phase(), Phase::Float));
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
    assert!(matches!(s.phase(), Phase::Absorb));
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
    assert!(matches!(s.phase(), Phase::Float));
    assert!(approx(s.target_voltage(), 13.5));
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
    assert!(approx(s.target_voltage(), 13.5));
}

#[test]
fn brief_taper_dip_does_not_exit_absorb() {
    // Pack flickers below the tail current for half the debounce window,
    // then comes back. Counter must reset — no transition.
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK);
    for _ in 0..(EXIT_DEBOUNCE.as_secs() / 2) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    // Sag recovers — current back above the tail.
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    // Must now sit through another full debounce window without transitioning.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    assert!(matches!(s.phase(), Phase::Absorb));
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
    assert!(matches!(s.phase(), Phase::Absorb));
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
    assert!(matches!(s.phase(), Phase::Float));
    assert!(approx(s.target_voltage(), 13.5));
}

#[test]
fn hysteresis_no_flap_between_thresholds() {
    let mut s = active(lfp_4s());
    // 2.7 A sits in the hysteresis band: > exit (2.5) but < enter (3.0).
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.7), TICK), Action::None));
    }
    assert!(matches!(s.phase(), Phase::Float));
    ok_tick(&mut s, b(OK_V, -4.0), TICK);
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.7), TICK), Action::None));
    }
    assert!(matches!(s.phase(), Phase::Absorb));
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
fn nan_or_inf_in_sample_treated_as_missing() {
    // Within the missing-battery debounce, a non-finite sample is
    // ignored just like None — no fault yet, no phase change, but no
    // charitable bypass either.
    let mut s = active(lfp_4s());
    for v in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        assert!(matches!(ok_tick(&mut s, b(OK_V, v), TICK), Action::None));
        assert!(matches!(ok_tick(&mut s, b(v, -1.0), TICK), Action::None));
    }
    assert!(matches!(s.phase(), Phase::Float));
    assert!(s.fault().is_none());
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
    assert!(s.fault().is_none());
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
    assert!(s.fault().is_none());
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
    assert!(s.fault().is_none());
}

#[test]
fn different_chemistries_yield_different_setpoints() {
    let mut lfp = active(Profile::for_pack(Chemistry::LiFePo4, 4, 50.0));
    let mut liion = active(Profile::for_pack(Chemistry::LiIon, 3, 50.0));
    assert!(matches!(
        ok_tick(&mut lfp, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(matches!(
        ok_tick(&mut liion, b(12.0, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(approx(lfp.target_voltage(), 14.4));
    assert!(approx(liion.target_voltage(), 12.3));
}

#[test]
fn single_cell_lfp_works() {
    // 1S 50 Ah LFP — float 3.375 V, absorb 3.60 V (daily). Same currents
    // as the 4S pack since they derive from capacity, not cell count.
    let mut s = active(Profile::for_pack(Chemistry::LiFePo4, 1, 50.0));
    assert!(approx(s.target_voltage(), 3.375));
    assert!(matches!(
        ok_tick(&mut s, b(3.4, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(approx(s.target_voltage(), 3.60));
}

#[test]
fn full_charge_cycle() {
    let mut s = active(lfp_4s());
    // Bulk → absorb on heavy current.
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -8.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(approx(s.target_voltage(), 14.4));
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
    assert!(approx(s.target_voltage(), 13.5));
    // Sit at float without retriggering absorb (all below 3 A enter).
    for &i in &[-0.05, -0.02, 0.0, -2.0] {
        assert!(matches!(ok_tick(&mut s, b(OK_V, i), TICK), Action::None));
    }
}

// --- Elapsed honored, not tick count ---

#[test]
fn ov_fault_honors_sub_second_elapsed() {
    // 500 ms ticks. Five of them = 2.5 s — under the 3 s budget, no fault.
    // Sixth tick brings the accumulated time to exactly OV_DURATION → fault.
    let mut s = active(lfp_4s());
    let step = Duration::from_millis(500);
    for _ in 0..5 {
        assert!(matches!(ok_tick(&mut s, b(14.7, -0.1), step), Action::None));
    }
    assert!(matches_disable(
        &ok_tick(&mut s, b(14.7, -0.1), step),
        FaultReason::Overvoltage,
    ));
}

#[test]
fn ov_fault_in_a_single_large_elapsed_tick() {
    // One call covering the full OV budget — should fault immediately. Catches
    // a regression that accumulates +1 per call instead of `+= elapsed`.
    let mut s = active(lfp_4s());
    assert!(matches_disable(
        &ok_tick(&mut s, b(14.7, -0.1), OV_DURATION),
        FaultReason::Overvoltage,
    ));
}

#[test]
fn ov_fault_with_mixed_elapsed_values() {
    // 1500 ms + 1000 ms + 600 ms = 3100 ms ≥ 3000 ms budget. Trips on the
    // third tick, not earlier (cumulative time, not tick count).
    let mut s = active(lfp_4s());
    assert!(matches!(
        ok_tick(&mut s, b(14.7, -0.1), Duration::from_millis(1500)),
        Action::None
    ));
    assert!(matches!(
        ok_tick(&mut s, b(14.7, -0.1), Duration::from_millis(1000)),
        Action::None
    ));
    assert!(matches_disable(
        &ok_tick(&mut s, b(14.7, -0.1), Duration::from_millis(600)),
        FaultReason::Overvoltage,
    ));
}

#[test]
fn absorb_timeout_in_a_single_large_elapsed_tick() {
    // After entering Absorb, one tick covering the full cap must fault.
    // Equivalent to the iteration-based test but proves elapsed is honored.
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    assert!(matches_disable(
        &ok_tick(&mut s, b(OK_V, -3.0), MAX_ABSORB),
        FaultReason::AbsorbTimeout,
    ));
}

#[test]
fn battery_stale_honors_elapsed() {
    // One tick with elapsed = BATTERY_MISSING_TIMEOUT must latch.
    let mut s = active(lfp_4s());
    let a = ok_tick(&mut s, None, BATTERY_MISSING_TIMEOUT);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
}

#[test]
fn modbus_unhealthy_honors_elapsed() {
    // Same: one big-elapsed tick saturates the modbus error counter.
    let mut s = active(lfp_4s());
    let a = fail_tick(&mut s, b(13.5, -0.1), MODBUS_UNHEALTHY_TIMEOUT);
    assert!(matches_disable(&a, FaultReason::ModbusUnhealthy));
}

// --- Absorb time cap ---

/// Drive the supervisor into Absorb. After this, exactly one Absorb tick
/// has elapsed (the transition itself).
fn enter_absorb(s: &mut ChargeSupervisor) {
    assert!(matches!(
        ok_tick(s, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(matches!(s.phase(), Phase::Absorb));
}

#[test]
fn absorb_does_not_time_out_below_budget() {
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    // Hold absorb just shy of the cap. Current pinned above exit threshold
    // (2.5 A) so we never drop to Float on our own.
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    }
    assert!(s.fault().is_none());
}

#[test]
fn float_does_not_accumulate_absorb_ticks() {
    let mut s = active(lfp_4s());
    // Sit in Float for far longer than the absorb cap — must never fault.
    for _ in 0..(MAX_ABSORB.as_secs() + 10) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -0.1), TICK), Action::None));
    }
    assert!(matches!(s.phase(), Phase::Float));
}

#[test]
fn absorb_counter_resets_on_taper_back_to_float() {
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    // Spend most of the budget in absorb, leaving room for a full exit
    // debounce window before the absorb timeout would fire.
    for _ in 0..(MAX_ABSORB.as_secs() - EXIT_DEBOUNCE.as_secs() - 10) {
        ok_tick(&mut s, b(OK_V, -3.0), TICK);
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
    assert!(matches!(s.phase(), Phase::Float));

    // Re-enter Absorb and burn the original margin's worth of ticks.
    // No fault yet — counter started over.
    enter_absorb(&mut s);
    for _ in 0..20 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    }
    assert!(s.fault().is_none());
}

// --- Supervisor faults & latching ---

#[test]
fn supervisor_passes_setpoint_through_on_phase_transition() {
    let mut s = active(lfp_4s());
    assert!(matches!(
        ok_tick(&mut s, b(13.5, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(matches!(s.phase(), Phase::Absorb));
    assert!(approx(s.target_voltage(), 14.4));
    assert!(s.fault().is_none());
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
    assert!(s.fault().is_none());
}

#[test]
fn battery_stale_for_budget_latches() {
    let mut s = active(lfp_4s());
    // BUDGET-1 ticks of missing battery: still healthy.
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, None, TICK), Action::None));
    }
    assert!(s.fault().is_none());

    let a = ok_tick(&mut s, None, TICK);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
    assert!(matches!(s.fault(), Some(FaultReason::BatterySensorStale)));
}

#[test]
fn battery_recovers_within_budget_no_latch() {
    let mut s = active(lfp_4s());
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        ok_tick(&mut s, None, TICK);
    }
    // One fresh reading clears the counter.
    ok_tick(&mut s, b(13.5, -0.1), TICK);
    // Now we should be able to miss BUDGET-1 again without latching.
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, None, TICK), Action::None));
    }
    assert!(s.fault().is_none());
}

#[test]
fn modbus_errors_for_budget_latches() {
    let mut s = active(lfp_4s());
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        assert!(matches!(
            fail_tick(&mut s, b(13.5, -0.1), TICK),
            Action::None
        ));
    }
    let a = fail_tick(&mut s, b(13.5, -0.1), TICK);
    assert!(matches_disable(&a, FaultReason::ModbusUnhealthy));
}

#[test]
fn modbus_recovers_within_budget_no_latch() {
    let mut s = active(lfp_4s());
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        fail_tick(&mut s, b(13.5, -0.1), TICK);
    }
    ok_tick(&mut s, b(13.5, -0.1), TICK); // good read clears counter
    for _ in 0..(MODBUS_UNHEALTHY_TIMEOUT.as_secs() - 1) {
        fail_tick(&mut s, b(13.5, -0.1), TICK);
    }
    assert!(s.fault().is_none());
}

#[test]
fn overvoltage_sustained_latches() {
    // absorb_v for lfp_4s = 14.4; margin = 0.2; so > 14.6 trips.
    let mut s = active(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(14.7, -0.1), TICK), Action::None));
    }
    let a = ok_tick(&mut s, b(14.7, -0.1), TICK);
    assert!(matches_disable(&a, FaultReason::Overvoltage));
}

#[test]
fn overvoltage_brief_recovers_no_latch() {
    let mut s = active(lfp_4s());
    // Two ticks above OV; one tick back below. Counter resets.
    ok_tick(&mut s, b(14.7, -0.1), TICK);
    ok_tick(&mut s, b(14.7, -0.1), TICK);
    ok_tick(&mut s, b(13.5, -0.1), TICK);
    // Two more above must NOT latch (< budget after reset).
    ok_tick(&mut s, b(14.7, -0.1), TICK);
    ok_tick(&mut s, b(14.7, -0.1), TICK);
    assert!(s.fault().is_none());
}

#[test]
fn ov_below_threshold_does_not_trip() {
    // absorb_v + OV_MARGIN_V ≈ 14.6. 14.55 is unambiguously below in f32.
    let mut s = active(lfp_4s());
    for _ in 0..(OV_DURATION.as_secs() + 5) {
        ok_tick(&mut s, b(14.55, -0.1), TICK);
    }
    assert!(s.fault().is_none());
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

    s.ack_disable();
    // Now the supervisor goes quiet — no further commands to the buck.
    for _ in 0..10 {
        assert!(matches!(ok_tick(&mut s, b(13.5, -4.0), TICK), Action::None));
    }
}

#[test]
fn latched_supervisor_does_not_change_phase() {
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, b(13.5, -0.1), TICK);
    }
    s.ack_disable();
    // Heavy charging current would normally drive Float→Absorb.
    ok_tick(&mut s, b(13.5, -5.0), TICK);
    assert!(matches!(s.phase(), Phase::Float));
}

#[test]
#[should_panic]
fn ack_disable_without_fault_panics() {
    let mut s = active(lfp_4s());
    s.ack_disable();
}

#[test]
fn first_fault_wins_over_simultaneous_conditions() {
    // Both modbus and battery faulting at once. Modbus is checked first.
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, None, TICK);
    }
    assert!(matches!(s.fault(), Some(FaultReason::ModbusUnhealthy)));
}

#[test]
fn supervisor_latches_on_absorb_timeout() {
    let mut s = active(lfp_4s());
    // Drive into Absorb.
    let _ = ok_tick(&mut s, b(13.5, -4.0), TICK);
    // Hold Absorb until just before the cap. Current pinned above exit
    // threshold so the controller can't taper out on its own.
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        ok_tick(&mut s, b(13.5, -3.0), TICK);
    }
    assert!(s.fault().is_none());

    let a = ok_tick(&mut s, b(13.5, -3.0), TICK);
    assert!(matches_disable(&a, FaultReason::AbsorbTimeout));
}

#[test]
fn setpoint_drift_v_set_latches_immediately() {
    // Float target is 13.5 V; pretend the buck reports 12.0 V — well past
    // the 0.02 V tolerance.
    let mut s = active(lfp_4s());
    let bad = Some(Setpoints {
        v_set: 12.0,
        i_set: 10.0,
    });
    let a = s.tick(
        PollResult {
            setpoints: bad,
            output: Some(BuckOutput::On),
            battery: b(13.5, -0.1),
        },
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::SettingsDrift));
}

#[test]
fn setpoint_drift_i_set_latches_immediately() {
    // Float target is 13.5 V (matches), but I_SET disagrees with the 10 A regulation.
    let mut s = active(lfp_4s());
    let bad = Some(Setpoints {
        v_set: 13.5,
        i_set: 5.0,
    });
    let a = s.tick(
        PollResult {
            setpoints: bad,
            output: Some(BuckOutput::On),
            battery: b(13.5, -0.1),
        },
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::SettingsDrift));
}

#[test]
fn setpoint_within_tolerance_no_drift_fault() {
    // 0.01 V off — one register quantum, under the 0.02 tolerance.
    let mut s = active(lfp_4s());
    let close = Some(Setpoints {
        v_set: 13.51,
        i_set: 10.0,
    });
    let a = s.tick(
        PollResult {
            setpoints: close,
            output: Some(BuckOutput::On),
            battery: b(13.5, -0.1),
        },
        TICK,
    );
    assert!(matches!(a, Action::None));
    assert!(s.fault().is_none());
}

#[test]
fn setpoint_drift_does_not_overwrite_existing_latch() {
    // First latch wins — modbus-unhealthy debounce trips before the next
    // good readback can latch SettingsDrift.
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, b(13.5, -0.1), TICK);
    }
    assert!(matches!(s.fault(), Some(FaultReason::ModbusUnhealthy)));
    let bad = Some(Setpoints {
        v_set: 12.0,
        i_set: 10.0,
    });
    let a = s.tick(
        PollResult {
            setpoints: bad,
            output: Some(BuckOutput::On),
            battery: b(13.5, -0.1),
        },
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::ModbusUnhealthy));
}

// --- Pending → Active bring-up ---

#[test]
fn pending_emits_enable_on_first_healthy_tick() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(matches!(a, Action::EnableOutput));
}

#[test]
fn pending_re_emits_enable_until_acked() {
    // Until the caller calls ack_enable, every tick re-emits EnableOutput
    // — mirrors the DisableOutput retry behavior on failed disable writes.
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..3 {
        let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
        assert!(matches!(a, Action::EnableOutput));
    }
}

#[test]
fn pending_overvolt_latches_without_debounce() {
    // Pack already over the OV threshold at boot → MUST latch on the
    // first tick, not wait out the 3 s OV debounce. The whole point of
    // Pending is to never enable output in an unsafe state.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let absorb = lfp_4s().absorb_v;
    let a = ok_tick(&mut s, b(absorb + OV_MARGIN_V + 0.5, -0.1), TICK);
    assert!(matches_disable(&a, FaultReason::Overvoltage));
}

#[test]
fn pending_drift_latches_without_enabling() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    let bad = Some(Setpoints {
        v_set: 12.0,
        i_set: 10.0,
    });
    let a = s.tick(
        PollResult {
            setpoints: bad,
            output: Some(BuckOutput::Off { cause: None }),
            battery: b(OK_V, -0.1),
        },
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::SettingsDrift));
}

#[test]
fn pending_no_battery_waits_then_latches() {
    // No battery sample → no enable. After BATTERY_MISSING_TIMEOUT,
    // BatterySensorStale latches.
    let mut s = ChargeSupervisor::new(lfp_4s());
    for _ in 0..(BATTERY_MISSING_TIMEOUT.as_secs() - 1) {
        let a = ok_tick(&mut s, None, TICK);
        assert!(matches!(a, Action::None));
    }
    let a = ok_tick(&mut s, None, TICK);
    assert!(matches_disable(&a, FaultReason::BatterySensorStale));
}

#[test]
#[should_panic]
fn ack_enable_from_active_panics() {
    let mut s = active(lfp_4s());
    s.ack_enable();
}

#[test]
fn buck_self_disable_in_active_latches() {
    // Active supervisor + buck reports output OFF (own OVP/OCP/LVP/over-temp
    // tripped, or panel toggled) → latch OutputUnexpectedlyOff.
    let mut s = active(lfp_4s());
    let p = PollResult {
        setpoints: Some(s.expected_setpoints()),
        output: Some(BuckOutput::Off { cause: None }),
        battery: b(OK_V, -0.1),
    };
    let a = s.tick(p, TICK);
    assert!(matches_disable(&a, FaultReason::OutputUnexpectedlyOff(None)));
}

/// Drive `s` from Active into Tripped(OutputUnexpectedlyOff(cause), acked:true).
/// Shared by every recovery test: latch → ack disable. The cause governs
/// whether `should_restart` will eventually fire — recovery tests pass a
/// recoverable cause (LVP), gate tests pass a non-recoverable one.
fn latch_self_disable(s: &mut ChargeSupervisor, cause: Option<XyProtectionStatus>) {
    let a = s.tick(
        PollResult {
            setpoints: Some(s.expected_setpoints()),
            output: Some(BuckOutput::Off { cause }),
            battery: b(OK_V, -0.1),
        },
        TICK,
    );
    assert!(matches_disable(&a, FaultReason::OutputUnexpectedlyOff(cause)));
    s.ack_disable();
}

/// PollResult that satisfies the should_restart healthy criteria: Modbus
/// up, output reads OFF, finite battery sample below OV.
fn healthy_recovery_poll(s: &ChargeSupervisor) -> PollResult {
    PollResult {
        setpoints: Some(s.expected_setpoints()),
        output: Some(BuckOutput::Off { cause: None }),
        battery: b(OK_V, -0.1),
    }
}

#[test]
fn should_restart_after_healthy_window() {
    // Active → buck self-disables → latch → ack. After
    // OUTPUT_RECOVERY_HEALTHY of continuously-healthy ticks, should_restart
    // returns true (caller's cue to throw this supervisor away and re-run
    // boot_sequence).
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, Some(XyProtectionStatus::Lvp));
    let p = healthy_recovery_poll(&s);
    for _ in 0..(OUTPUT_RECOVERY_HEALTHY.as_secs() - 1) {
        // Recovery accumulates via tick; pre-threshold ticks return None.
        assert!(matches!(s.tick(p, TICK), Action::None));
        assert!(matches!(s.fault(), Some(FaultReason::OutputUnexpectedlyOff(_))));
    }
    // The Nth healthy tick crosses the threshold.
    assert!(restart_ready(&mut s, p, TICK));
}

#[test]
fn should_restart_resets_on_overvoltage() {
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, Some(XyProtectionStatus::Lvp));
    let p = healthy_recovery_poll(&s);
    for _ in 0..(OUTPUT_RECOVERY_HEALTHY.as_secs() - 1) {
        assert!(!restart_ready(&mut s, p, TICK));
    }
    // A single tick with the pack over OV resets the clock.
    let absorb = lfp_4s().absorb_v;
    let p_ov = PollResult {
        battery: b(absorb + OV_MARGIN_V + 0.5, -0.1),
        ..p
    };
    assert!(!restart_ready(&mut s, p_ov, TICK));
    // One more healthy call must NOT cross — clock is back to 1.
    assert!(!restart_ready(&mut s, p, TICK));
}

#[test]
fn should_restart_resets_on_modbus_down() {
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, Some(XyProtectionStatus::Lvp));
    let p = healthy_recovery_poll(&s);
    for _ in 0..(OUTPUT_RECOVERY_HEALTHY.as_secs() - 1) {
        assert!(!restart_ready(&mut s, p, TICK));
    }
    let p_modbus_down = PollResult {
        setpoints: None,
        output: None,
        ..p
    };
    assert!(!restart_ready(&mut s, p_modbus_down, TICK));
    assert!(!restart_ready(&mut s, p, TICK));
}

#[test]
fn should_restart_resets_on_missing_battery() {
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, Some(XyProtectionStatus::Lvp));
    let p = healthy_recovery_poll(&s);
    for _ in 0..(OUTPUT_RECOVERY_HEALTHY.as_secs() - 1) {
        assert!(!restart_ready(&mut s, p, TICK));
    }
    let p_no_batt = PollResult { battery: None, ..p };
    assert!(!restart_ready(&mut s, p_no_batt, TICK));
    assert!(!restart_ready(&mut s, p, TICK));
}

#[test]
fn should_restart_resets_on_unexpected_output_on() {
    // Buck spontaneously came back on (panel toggle, EMC, whatever) —
    // we want a stable OFF state before signalling restart.
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, Some(XyProtectionStatus::Lvp));
    let p = healthy_recovery_poll(&s);
    for _ in 0..(OUTPUT_RECOVERY_HEALTHY.as_secs() - 1) {
        assert!(!restart_ready(&mut s, p, TICK));
    }
    let p_on = PollResult {
        output: Some(BuckOutput::On),
        ..p
    };
    assert!(!restart_ready(&mut s, p_on, TICK));
    assert!(!restart_ready(&mut s, p, TICK));
}

#[test]
fn should_restart_repeats_across_cycles() {
    // The supervisor itself doesn't cap recovery attempts — that lives in
    // the caller, which is what gets thrown away on each restart. A single
    // supervisor will happily fire `should_restart=true` over and over if
    // its latch keeps re-asserting (no flap budget at this layer).
    let mut s = active(lfp_4s());
    latch_self_disable(&mut s, Some(XyProtectionStatus::Lvp));
    let p = healthy_recovery_poll(&s);
    for _ in 0..OUTPUT_RECOVERY_HEALTHY.as_secs() {
        restart_ready(&mut s, p, TICK);
    }
    assert!(restart_ready(&mut s, p, TICK));
    // Even after firing once, calling again with healthy state still
    // returns true — the caller is expected to react by tearing the
    // supervisor down, not to wait for a state transition.
    assert!(restart_ready(&mut s, p, TICK));
}

#[test]
fn should_restart_false_for_non_recoverable_fault() {
    // Confirm only OutputUnexpectedlyOff is recoverable. OV trip in
    // active state should never let should_restart fire.
    let mut s = active(lfp_4s());
    let absorb = lfp_4s().absorb_v;
    for _ in 0..(OV_DURATION.as_secs() + 1) {
        ok_tick(&mut s, b(absorb + OV_MARGIN_V + 0.5, -0.1), TICK);
    }
    assert!(matches!(s.fault(), Some(FaultReason::Overvoltage)));
    s.ack_disable();
    let p = healthy_recovery_poll(&s);
    for _ in 0..(OUTPUT_RECOVERY_HEALTHY.as_secs() * 5) {
        assert!(!restart_ready(&mut s, p, TICK));
    }
    assert!(matches!(s.fault(), Some(FaultReason::Overvoltage)));
}

#[test]
fn should_restart_false_in_pending_and_active() {
    // Outside Tripped, should_restart is always false regardless of input.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p = healthy_recovery_poll(&s);
    assert!(!restart_ready(&mut s, p, TICK)); // Pending
    let mut s = active(lfp_4s());
    assert!(!restart_ready(&mut s, p, TICK)); // Active
}

#[test]
fn should_restart_false_for_non_recoverable_protection_cause() {
    // OCP / OVP / OPP / etc. are causes we don't auto-recover from —
    // re-energizing into a sticky short or pack-side fault is exactly
    // what the gate exists to prevent. Same goes for Unread (we don't
    // know the cause) and Unknown(_) (off-spec read).
    for cause in [
        Some(XyProtectionStatus::Ocp),
        Some(XyProtectionStatus::Ovp),
        Some(XyProtectionStatus::Opp),
        Some(XyProtectionStatus::Icp),
        Some(XyProtectionStatus::Unknown(99)),
        None, // PROTECT read failed
    ] {
        let mut s = active(lfp_4s());
        latch_self_disable(&mut s, cause);
        let p = healthy_recovery_poll(&s);
        for _ in 0..(OUTPUT_RECOVERY_HEALTHY.as_secs() * 2) {
            assert!(!restart_ready(&mut s, p, TICK));
        }
    }
}

#[test]
fn update_voltage_retries_until_acked() {
    // Phase machine wants Float→Absorb (heavy charging current). The
    // first tick emits UpdateVoltage. If the caller doesn't ack (write
    // failed), the next tick must re-emit UpdateVoltage with the same
    // target — and the drift check must NOT latch SettingsDrift, since
    // V_SET on the buck is still the old (Float) value matching the
    // supervisor's still-Float `target_voltage`.
    let profile = lfp_4s();
    let mut s = active(profile);
    let p_in_phase_transition = PollResult {
        // Buck readback is still at the old (Float) voltage — caller's
        // write failed so V_SET on the device didn't change.
        setpoints: Some(Setpoints {
            v_set: profile.float_v,
            i_set: profile.regulation_a,
        }),
        output: Some(BuckOutput::On),
        battery: b(OK_V, -4.0),
    };
    let a = s.tick(p_in_phase_transition, TICK);
    let Action::UpdateVoltage { target_v: t1 } = a else {
        panic!("expected UpdateVoltage, got something else");
    };
    assert!(approx(t1, profile.absorb_v));
    // Phase NOT yet committed.
    assert!(matches!(s.phase(), Phase::Float));
    assert!(s.fault().is_none());

    // No ack — second tick re-emits UpdateVoltage, same target. No
    // SettingsDrift even though setpoints (Float) lag the pending phase
    // (Absorb), because expected_setpoints still uses the old phase.
    let a = s.tick(p_in_phase_transition, TICK);
    let Action::UpdateVoltage { target_v: t2 } = a else {
        panic!("expected UpdateVoltage retry, got something else");
    };
    assert!(approx(t2, profile.absorb_v));
    assert!(matches!(s.phase(), Phase::Float));
    assert!(s.fault().is_none());

    // Now ack — phase commits, debouncers reset, normal operation.
    s.ack_voltage_update();
    assert!(matches!(s.phase(), Phase::Absorb));
}

#[test]
fn buck_output_off_in_pending_does_not_fault() {
    // In Pending the buck IS supposed to be off — output_on=Some(false)
    // is normal, must not latch.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p = PollResult {
        setpoints: Some(s.expected_setpoints()),
        output: Some(BuckOutput::Off { cause: None }),
        battery: b(OK_V, -0.1),
    };
    let a = s.tick(p, TICK);
    assert!(matches!(a, Action::EnableOutput));
    assert!(s.fault().is_none());
}

#[test]
fn buck_output_on_in_pending_latches() {
    // Pending expects the buck OFF — boot_sequence wrote set_output(false)
    // and S_INI=0. If OUTPUT_EN reads ON anyway, regulation is happening
    // under unknown conditions; latch immediately, no debounce.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p = PollResult {
        setpoints: Some(s.expected_setpoints()),
        output: Some(BuckOutput::On),
        battery: b(OK_V, -0.1),
    };
    let a = s.tick(p, TICK);
    assert!(matches_disable(&a, FaultReason::OutputOnInPending));
    // Non-recoverable: caller must reboot, not retry.
    assert_eq!(FaultReason::OutputOnInPending.recovery_healthy_for(), None);
}

#[test]
#[should_panic]
fn ack_enable_from_tripped_panics() {
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, b(OK_V, -0.1), TICK);
    }
    s.ack_enable();
}
