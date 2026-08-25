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
/// Below the LFP fixture's CV plateau (14.4), so it does NOT arm the absorb
/// timeout — represents the CC ramp.
const OK_V: f32 = 13.5;
/// Pack voltage sitting at the LFP fixture's CV plateau (`absorb_v` = 14.4).
/// Arms the `MAX_ABSORB` clock; stays under the OV trip (14.6).
const CV_V: f32 = 14.4;
/// Nominal DC input rail the board feeds the buck — mirrors firmware's
/// `INPUT_NOMINAL_V`. Drives the input-UVLO (LVP) derivation.
const INPUT_NOMINAL_V: f32 = 24.0;
/// Wall time elapsed per simulated tick. Tests choose 1 s so iteration
/// counts read as seconds when comparing against duration budgets.
const TICK: Duration = Duration::from_secs(1);

fn b(voltage: f32, current: f32) -> Option<BatterySample> {
    Some(BatterySample { voltage, current })
}

fn matches_disable(a: &Action, expected: FaultReason) -> bool {
    matches!(a, Action::DisableOutput(r) if *r == expected)
}

/// Drift-free PollResult matching the supervisor's expected state. Active
/// → output ON; Pending or Tripped → output OFF (no protection cause).
/// Tests that need to perturb one field use spread syntax:
/// `PollResult { output: Some(BuckOutput::On), ..expected_poll(&s, ...) }`.
fn expected_poll(s: &ChargeSupervisor, battery: Option<BatterySample>) -> PollResult {
    let output = if matches!(s.latch, LatchState::Active { .. }) {
        BuckOutput::On
    } else {
        BuckOutput::Off { cause: ProtectionStatus::Normal }
    };
    PollResult {
        setpoints: Some(s.expected_setpoints()),
        output: Some(output),
        battery,
    }
}

/// Tick with a successful, drift-free Modbus readback where the buck
/// reports the output state the supervisor currently expects — the
/// common case. Phase transitions don't fire spurious `SettingsDrift`,
/// and Active ticks don't fire spurious `OutputUnexpectedlyOff`.
fn ok_tick(s: &mut ChargeSupervisor, battery: Option<BatterySample>, elapsed: Duration) -> Action {
    let a = s.tick(expected_poll(s, battery), elapsed);
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
    // Bring up at the CV plateau (`absorb_v`, still under the OV trip) so
    // the pack reads as full and the supervisor lands in Float — the
    // precondition these tests assume. A resting voltage below the plateau
    // would (correctly) resume Absorb; that path has its own tests.
    let bring_up_v = profile.absorb_v;
    let mut s = ChargeSupervisor::new(profile);
    let a = ok_tick(&mut s, b(bring_up_v, -0.1), TICK);
    assert!(matches!(a, Action::EnableOutput { .. }));
    s.ack_enable(false);
    assert!(matches!(s.phase(), Phase::Float));
    s
}

/// Drive the supervisor into Absorb. After this, exactly one Absorb tick
/// has elapsed (the transition itself).
fn enter_absorb(s: &mut ChargeSupervisor) {
    assert!(matches!(
        ok_tick(s, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(matches!(s.phase(), Phase::Absorb));
}

/// Drive `s` from Active into `Tripped(OutputUnexpectedlyOff(cause), acked: true)`.
/// Cause must be non-recoverable (i.e. not Lvp/Otp, which are handled
/// in-place and never latch).
fn latch_self_disable(s: &mut ChargeSupervisor, cause: ProtectionStatus) {
    let p = PollResult {
        output: Some(BuckOutput::Off { cause }),
        ..expected_poll(s, b(OK_V, -0.1))
    };
    let a = s.tick(p, TICK);
    assert!(matches_disable(
        &a,
        FaultReason::OutputUnexpectedlyOff(cause)
    ));
    s.ack_disable();
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
    let s = p.safety_limits(INPUT_NOMINAL_V);
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
fn profile_display_is_compact_pack_identity() {
    // chemistry label + series count + rated capacity, no decimals.
    assert_eq!(
        Profile::for_pack(Chemistry::LiFePo4, 4, 50.0).to_string(),
        "LFP 4S 50Ah"
    );
    assert_eq!(
        Profile::for_pack(Chemistry::LiIon, 3, 17.0).to_string(),
        "Li-ion 3S 17Ah"
    );
    assert_eq!(
        Profile::for_pack(Chemistry::LiFePo4TopBalance, 8, 100.0).to_string(),
        "LFP-TB 8S 100Ah"
    );
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
#[should_panic]
fn new_rejects_absorb_not_above_float() {
    // The phase machine assumes absorb_v > float_v (Float→Absorb is "raise
    // V_SET"). Profile-builder construction guarantees that for chemistries,
    // but a hand-rolled Profile must still satisfy it.
    let bogus = Profile {
        chemistry: Chemistry::LiFePo4,
        cells: 4,
        capacity_ah: 50.0,
        absorb_v: 13.5,
        float_v: 13.5,
        regulation_a: 10.0,
        enter_absorb_a: 3.0,
        exit_absorb_a: 2.5,
    };
    let _ = ChargeSupervisor::new(bogus);
}

#[test]
fn safety_limits_track_chemistry_change() {
    // Top-balance pushes absorb to 14.6 V — OVP must move up too, not stay
    // at the 4S-daily 15.0 V. Without derived limits this is the footgun.
    let s = Profile::for_pack(Chemistry::LiFePo4TopBalance, 4, 50.0).safety_limits(INPUT_NOMINAL_V);
    assert!(approx(s.ovp_v, 15.2));
    assert!(s.ovp_v > 14.6, "OVP must clear absorb_v");
}

#[test]
fn safety_limits_track_cell_count_change() {
    let s4 = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0).safety_limits(INPUT_NOMINAL_V);
    let s8 = Profile::for_pack(Chemistry::LiFePo4, 8, 50.0).safety_limits(INPUT_NOMINAL_V);
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
        let s = p.safety_limits(INPUT_NOMINAL_V);
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
fn sustained_recharge_drains_exit_gate() {
    // The gate is leaky: a *sustained* return to real charging must drain it
    // back down and block the exit (the pack is genuinely taking charge, not
    // just pulsing). Fill it to one tick shy of the window, then charge above
    // tail for a full window — symmetric drain empties it — and confirm it
    // then takes a fresh full window below tail to actually exit.
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK); // → Absorb
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    // Sustained recharge above tail for the full window drains the gate to 0.
    for _ in 0..EXIT_DEBOUNCE.as_secs() {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -4.0), TICK), Action::None));
    }
    assert!(matches!(s.phase(), Phase::Absorb));
    // A fresh full window below tail is now required to exit.
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -2.4), TICK), Action::None));
    }
    assert!(matches!(
        ok_tick(&mut s, b(OK_V, -2.4), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert!(matches!(s.phase(), Phase::Float));
}

#[test]
fn pulsing_current_still_exits_absorb() {
    // Regression for the real-hardware bug: at a full pack the XY7025 delivers
    // burst pulses — instantaneous current spikes well above the tail every few
    // seconds but averages near zero. The old hard-reset gate re-armed the full
    // window on every spike and never exited, pinning the supervisor in Absorb
    // (load silently running on a "full" pack). The leaky gate must accept the
    // taper because the *average* charging current is below the 2.5 A tail.
    let mut s = active(lfp_4s());
    ok_tick(&mut s, b(OK_V, -4.0), TICK); // → Absorb
    // One 6 A spike (each would reset a hard-reset debounce) per three near-zero
    // ticks: average charging current ≈ 1.5 A, comfortably below the 2.5 A tail.
    let pattern = [-6.0, -0.1, 0.2, -0.1];
    let mut exited_at = None;
    'outer: for cycle in 0..60 {
        for &i in &pattern {
            if matches!(
                ok_tick(&mut s, b(OK_V, i), TICK),
                Action::UpdateVoltage { .. }
            ) {
                exited_at = Some(cycle);
                break 'outer;
            }
        }
    }
    // Net fill is +2 ticks per 4-tick cycle, so ~30 cycles to cross the 60 s
    // window. Assert it lands in that ballpark — not "eventually", and not so
    // fast that the spikes were being ignored entirely.
    let cycle = exited_at.expect("pulsing pack never exited Absorb");
    assert!((25..40).contains(&cycle), "exited after {cycle} cycles");
    assert!(matches!(s.phase(), Phase::Float));
    assert!(approx(s.target_voltage(), 13.5));
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
        &ok_tick(&mut s, b(CV_V, -3.0), MAX_ABSORB),
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

#[test]
fn absorb_does_not_time_out_below_budget() {
    let mut s = active(lfp_4s());
    enter_absorb(&mut s);
    // Hold at the CV plateau just shy of the cap. Current pinned above exit
    // threshold (2.5 A) so we never drop to Float on our own.
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(CV_V, -3.0), TICK), Action::None));
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
    assert!(matches!(s.phase(), Phase::Float));

    // Re-enter Absorb and burn the original margin's worth of ticks.
    // No fault yet — counter started over.
    enter_absorb(&mut s);
    for _ in 0..20 {
        assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    }
    assert!(s.fault().is_none());
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
    assert!(s.fault().is_none());
    assert!(matches!(s.phase(), Phase::Absorb));
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
    assert!(s.fault().is_none());
    // CC dip: voltage below the CV band resets the absorb debouncer.
    assert!(matches!(ok_tick(&mut s, b(OK_V, -3.0), TICK), Action::None));
    for _ in 0..(MAX_ABSORB.as_secs() - 1) {
        assert!(matches!(ok_tick(&mut s, b(CV_V, -3.0), TICK), Action::None));
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
        ok_tick(&mut s, b(CV_V, -3.0), TICK);
    }
    assert!(s.fault().is_none());

    let a = ok_tick(&mut s, b(CV_V, -3.0), TICK);
    assert!(matches_disable(&a, FaultReason::AbsorbTimeout));
}

#[test]
fn setpoint_drift_v_set_latches_immediately() {
    // Float target is 13.5 V; pretend the buck reports 12.0 V — well past
    // the 0.02 V tolerance.
    let mut s = active(lfp_4s());
    let p = PollResult {
        setpoints: Some(Setpoints {
            v_set: 12.0,
            i_set: 10.0,
        }),
        ..expected_poll(&s, b(13.5, -0.1))
    };
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::SettingsDrift
    ));
}

#[test]
fn setpoint_drift_i_set_latches_immediately() {
    // Float target is 13.5 V (matches), but I_SET disagrees with the 10 A regulation.
    let mut s = active(lfp_4s());
    let p = PollResult {
        setpoints: Some(Setpoints {
            v_set: 13.5,
            i_set: 5.0,
        }),
        ..expected_poll(&s, b(13.5, -0.1))
    };
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::SettingsDrift
    ));
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

// --- Pending → Active bring-up ---

#[test]
fn pending_emits_enable_on_first_healthy_tick() {
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(matches!(a, Action::EnableOutput { .. }));
}

#[test]
fn pending_re_emits_enable_until_acked() {
    // Until the caller calls ack_enable, every tick re-emits EnableOutput
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
    let Action::EnableOutput { resume_absorb } = ok_tick(&mut s, b(OK_V, -0.1), TICK) else {
        panic!("expected EnableOutput")
    };
    assert!(resume_absorb, "pack below CV plateau ⇒ must request Absorb");
    s.ack_enable(resume_absorb);
    assert!(matches!(s.phase(), Phase::Float)); // not committed until V_SET write
    let a = ok_tick(&mut s, b(OK_V, -0.1), TICK);
    assert!(
        matches!(a, Action::UpdateVoltage { target_v, .. } if approx(target_v, lfp_4s().absorb_v)),
        "expected UpdateVoltage to absorb_v, got {a:?}",
    );
    assert!(matches!(s.phase(), Phase::Absorb));
}

#[test]
fn full_pack_stays_float_after_bringup() {
    // A pack resting at the CV plateau is full — bring-up must NOT resume
    // Absorb. With low current it sits in Float (maintenance), no voltage
    // bump.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let a = ok_tick(&mut s, b(CV_V, -0.1), TICK);
    assert!(matches!(a, Action::EnableOutput { .. }));
    s.ack_enable(false);
    let a = ok_tick(&mut s, b(CV_V, -0.1), TICK);
    assert!(matches!(a, Action::None), "expected None, got {a:?}");
    assert!(matches!(s.phase(), Phase::Float));
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
        Action::EnableOutput {
            resume_absorb: false
        }
    ));
    assert_eq!(s.inhibit(), None);
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
        Action::EnableOutput {
            resume_absorb: true
        }
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
        Action::EnableOutput {
            resume_absorb: true
        }
    ));
    assert_eq!(s.inhibit(), None);
}

#[test]
#[should_panic]
fn ack_enable_from_active_panics() {
    let mut s = active(lfp_4s());
    s.ack_enable(false);
}

#[test]
fn buck_self_disable_in_active_latches() {
    // Active supervisor + buck reports output OFF (own OVP/OCP/LVP/over-temp
    // tripped, or panel toggled) → latch OutputUnexpectedlyOff.
    let mut s = active(lfp_4s());
    let p = PollResult {
        output: Some(BuckOutput::Off { cause: ProtectionStatus::Normal }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
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
fn update_voltage_retries_until_acked() {
    // Phase machine wants Float→Absorb (heavy charging current). The
    // first tick emits UpdateVoltage. If the caller doesn't ack (write
    // failed), the next tick must re-emit UpdateVoltage with the same
    // target — and the drift check must NOT latch SettingsDrift, since
    // V_SET on the buck is still the old (Float) value matching the
    // supervisor's still-Float `target_voltage`.
    let profile = lfp_4s();
    let mut s = active(profile);
    // expected_poll uses s.expected_setpoints(), which reflects the
    // *current* phase (Float) — exactly the still-on-the-buck values
    // the failed write would leave behind.
    let p = expected_poll(&s, b(OK_V, -4.0));

    let Action::UpdateVoltage { target_v: t1, .. } = s.tick(p, TICK) else {
        panic!("expected UpdateVoltage");
    };
    assert!(approx(t1, profile.absorb_v));
    assert!(matches!(s.phase(), Phase::Float)); // not yet committed
    assert!(s.fault().is_none());

    // No ack — second tick re-emits UpdateVoltage, same target. No
    // SettingsDrift even though setpoints (Float) lag the pending phase
    // (Absorb), because expected_setpoints still uses the old phase.
    let Action::UpdateVoltage { target_v: t2, .. } = s.tick(p, TICK) else {
        panic!("expected UpdateVoltage retry");
    };
    assert!(approx(t2, profile.absorb_v));
    assert!(matches!(s.phase(), Phase::Float));
    assert!(s.fault().is_none());

    // Now ack — phase commits, debouncers reset, normal operation.
    s.ack_voltage_update();
    assert!(matches!(s.phase(), Phase::Absorb));
}

#[test]
fn float_to_absorb_emits_step_up_no_output_cycle() {
    // V_SET goes up; safe to write live, no output cycling needed.
    // The caller can keep regulating through the transition.
    let profile = lfp_4s();
    let mut s = active(profile);
    let p = expected_poll(&s, b(OK_V, -4.0));
    let Action::UpdateVoltage {
        target_v,
        cycle_output,
    } = s.tick(p, TICK)
    else {
        panic!("expected UpdateVoltage");
    };
    assert!(approx(target_v, profile.absorb_v));
    assert!(!cycle_output, "Float→Absorb is a step UP — must not cycle");
}

#[test]
fn absorb_to_float_emits_step_down_with_output_cycle() {
    // V_SET goes down. The caller MUST disable output around the write
    // — stepping V_SET below V_OUT with output enabled drives reverse
    // current through the buck's synchronous low-side FET (the battery
    // sources back in), which can blow the FET and propagate upstream.
    // The XY7025 has no anti-backup protection on either port.
    let profile = lfp_4s();
    let mut s = active(profile);
    enter_absorb(&mut s); // now in Absorb at absorb_v

    // Hold at CV plateau with tapered current long enough to trip the
    // exit debouncer. Drive the supervisor manually (not via ok_tick)
    // so we can inspect the transition tick before it auto-acks.
    let tapered = b(CV_V, -(profile.exit_absorb_a - 0.1));
    let p = expected_poll(&s, tapered);
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(s.tick(p, TICK), Action::None));
    }
    let Action::UpdateVoltage {
        target_v,
        cycle_output,
    } = s.tick(p, TICK)
    else {
        panic!("expected Absorb→Float UpdateVoltage after EXIT_DEBOUNCE");
    };
    assert!(approx(target_v, profile.float_v));
    assert!(
        cycle_output,
        "Absorb→Float is a step DOWN — caller must cycle output"
    );
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
    assert!(s.fault().is_none());
}

#[test]
fn buck_output_on_in_pending_latches() {
    // Pending expects the buck OFF — boot_sequence wrote set_output(false)
    // and S_INI=0. If OUTPUT_EN reads ON anyway, regulation is happening
    // under unknown conditions; latch immediately, no debounce.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p = PollResult {
        output: Some(BuckOutput::On),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    assert!(matches_disable(
        &s.tick(p, TICK),
        FaultReason::OutputOnInPending
    ));
}

#[test]
#[should_panic]
fn ack_enable_from_tripped_panics() {
    let mut s = active(lfp_4s());
    for _ in 0..MODBUS_UNHEALTHY_TIMEOUT.as_secs() {
        fail_tick(&mut s, b(OK_V, -0.1), TICK);
    }
    s.ack_enable(false);
}

#[test]
#[should_panic]
fn ack_voltage_update_without_pending_phase_panics() {
    // ack_voltage_update only makes sense after an UpdateVoltage was
    // emitted (pending_phase set). Calling it from steady-state Active
    // would commit a None into self.phase — drop it on the floor.
    let mut s = active(lfp_4s());
    s.ack_voltage_update();
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
    assert!(matches!(a, Action::EnableOutput { .. }));
    assert!(matches!(s.phase(), Phase::Float));
    // After ack, supervisor goes Active still in Float — first real tick
    // will then run the phase machine. Verify the very next tick (now
    // Active) is the one that emits the transition.
    s.ack_enable(false);
    let a = s.tick(expected_poll(&s, battery), TICK);
    assert!(matches!(a, Action::UpdateVoltage { target_v, .. } if approx(target_v, profile.absorb_v)));
}

#[test]
fn active_lvp_drops_to_pending_without_latch() {
    // Input UVLO is benign: the buck self-disabled because the DC supply
    // dropped, not because of a pack-side fault. The supervisor must
    // transition back to Pending without latching, so no DisableOutput
    // is emitted and the caller's restart budget is preserved across
    // arbitrarily long outages.
    let mut s = active(lfp_4s());
    let p_lvp = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Lvp,
        }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    let a = s.tick(p_lvp, TICK);
    assert!(matches!(a, Action::None));
    assert!(s.fault().is_none());
    assert!(matches!(s.latch, LatchState::Pending { .. }));
}

#[test]
fn pending_waits_for_lvp_to_clear_before_enable() {
    // While LVP persists, the Pending bring-up must NOT emit EnableOutput
    // — writing set_output(true) into a buck in input UVLO would just
    // flap. Once LVP clears (buck reports Off with no cause), the
    // normal Pending → Active path emits EnableOutput.
    let mut s = active(lfp_4s());
    let p_lvp = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Lvp,
        }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    // Drop Active → Pending via LVP.
    assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    // Many ticks of sustained LVP: stays Pending, no actions, no fault.
    for _ in 0..120 {
        assert!(matches!(s.tick(p_lvp, TICK), Action::None));
    }
    assert!(s.fault().is_none());
    // LVP clears: buck back to Off with no protection cause. Pending
    // bring-up energises on the next tick.
    let p_clear = expected_poll(&s, b(OK_V, -0.1));
    assert!(matches!(s.tick(p_clear, TICK), Action::EnableOutput { .. }));
    s.ack_enable(false);
    assert!(matches!(s.latch, LatchState::Active { .. }));
}

#[test]
fn lvp_recovery_resumes_absorb_when_pack_below_plateau() {
    // Pack drains during the input outage to below the CV plateau —
    // when LVP clears, the bring-up's resting-voltage check must resume
    // Absorb (not stall in Float).
    let profile = lfp_4s();
    let mut s = active(profile);
    let p_lvp = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Lvp,
        }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    s.tick(p_lvp, TICK);
    // Pack rests well below absorb_v - ABSORB_CV_BAND_V (= 14.3).
    let drained = b(13.0, 0.0);
    let p_clear = PollResult {
        output: Some(BuckOutput::Off { cause: ProtectionStatus::Normal }),
        setpoints: Some(s.expected_setpoints()),
        battery: drained,
    };
    // Drained pack ⇒ resume_absorb=true; ack with the same.
    let Action::EnableOutput { resume_absorb } = s.tick(p_clear, TICK) else {
        panic!("expected EnableOutput")
    };
    assert!(resume_absorb);
    s.ack_enable(resume_absorb);
    // ack_enable resumed Absorb via pending_voltage, so the next Active
    // tick steps V_SET float_v → absorb_v.
    let p_active = PollResult {
        output: Some(BuckOutput::On),
        setpoints: Some(s.expected_setpoints()),
        battery: drained,
    };
    let a = s.tick(p_active, TICK);
    assert!(matches!(a, Action::UpdateVoltage { target_v, .. } if approx(target_v, profile.absorb_v)));
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
    assert!(s.fault().is_none());
    assert!(matches!(s.latch, LatchState::Pending { .. }));
}

#[test]
fn lvp_recovery_accepts_buck_auto_re_enable() {
    // After LVP intercept drops the supervisor to Pending, the XY7025
    // typically auto-re-enables OUTPUT_EN once input voltage returns
    // (LVP is a transient input-side protection, not a permanent latch).
    // The supervisor must accept that as recovery — transition back to
    // Active without latching OutputOnInPending — because setpoints are
    // still the values it programmed before LVP, so regulation is at
    // known targets.
    let mut s = active(lfp_4s());
    let p_lvp = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Lvp,
        }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    s.tick(p_lvp, TICK);
    assert!(matches!(s.latch, LatchState::Pending { .. }));
    // Input returns and the buck brings its own output back on.
    let p_recovered = PollResult {
        output: Some(BuckOutput::On),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    let a = s.tick(p_recovered, TICK);
    assert!(matches!(a, Action::None));
    assert!(s.fault().is_none());
    assert!(matches!(
        s.latch,
        LatchState::Active {
            pending_voltage: None
        }
    ));
}

#[test]
fn boot_pending_with_buck_on_still_latches() {
    // At cold boot, boot_sequence already wrote set_output(false) and
    // verified OUTPUT_EN=0 — so a poll showing buck=On is a genuine
    // anomaly (firmware bug / panel toggle / EMI). Stays the immediate
    // latch it always was; only ProtectRecovery gets the soft transition.
    let mut s = ChargeSupervisor::new(lfp_4s());
    let p_on = PollResult {
        output: Some(BuckOutput::On),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    assert!(matches_disable(
        &s.tick(p_on, TICK),
        FaultReason::OutputOnInPending
    ));
}

#[test]
fn active_otp_drops_to_pending_without_latch() {
    // Over-temp self-disable is handled the same way as input UVLO:
    // benign sensor-side condition, supervisor drops to Pending and
    // waits without latching or burning any restart budget.
    let mut s = active(lfp_4s());
    let p_otp = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Otp,
        }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    let a = s.tick(p_otp, TICK);
    assert!(matches!(a, Action::None));
    assert!(s.fault().is_none());
    assert!(matches!(s.latch, LatchState::Pending { .. }));
}

#[test]
fn otp_recovery_accepts_buck_auto_re_enable() {
    // OTP, like LVP, may auto-clear and the buck may auto-re-enable
    // OUTPUT_EN when the case cools. Supervisor follows it back to
    // Active without latching.
    let mut s = active(lfp_4s());
    let p_otp = PollResult {
        output: Some(BuckOutput::Off {
            cause: ProtectionStatus::Otp,
        }),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    s.tick(p_otp, TICK);
    let p_recovered = PollResult {
        output: Some(BuckOutput::On),
        ..expected_poll(&s, b(OK_V, -0.1))
    };
    let a = s.tick(p_recovered, TICK);
    assert!(matches!(a, Action::None));
    assert!(s.fault().is_none());
    assert!(matches!(
        s.latch,
        LatchState::Active {
            pending_voltage: None
        }
    ));
}

// ─── apply_update_voltage (firmware-side sequencing) ────────────────────────

/// Programmable mock for `VoltageWriter`. Records every call in order and
/// can be primed to fail at a specific call index per method, exercising
/// the partial-failure paths in `apply_update_voltage`.
#[derive(Default)]
struct MockWriter {
    set_output_calls: Vec<bool>,
    set_voltage_calls: Vec<f32>,
    fail_set_output_at: Vec<usize>,
    fail_set_voltage_at: Vec<usize>,
}

impl VoltageWriter for MockWriter {
    fn set_voltage(&mut self, volts: f32) -> Result<(), BusError> {
        let idx = self.set_voltage_calls.len();
        self.set_voltage_calls.push(volts);
        if self.fail_set_voltage_at.contains(&idx) {
            Err(BusError::Rtu(RtuError::Timeout))
        } else {
            Ok(())
        }
    }
    fn set_output(&mut self, on: bool) -> Result<(), BusError> {
        let idx = self.set_output_calls.len();
        self.set_output_calls.push(on);
        if self.fail_set_output_at.contains(&idx) {
            Err(BusError::Rtu(RtuError::Timeout))
        } else {
            Ok(())
        }
    }
}

/// Drive `s` from Active+Absorb to Active+pending_voltage=Some(Float) by
/// holding at the CV plateau with tapered current through `EXIT_DEBOUNCE`.
/// Final tick emits the transition; pending_voltage is left set so the
/// test caller owns the apply step.
fn drive_to_absorb_to_float_pending(s: &mut ChargeSupervisor) {
    enter_absorb(s);
    let tapered = b(CV_V, -(lfp_4s().exit_absorb_a - 0.1));
    let p = expected_poll(s, tapered);
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(s.tick(p, TICK), Action::None));
    }
    assert!(matches!(
        s.tick(p, TICK),
        Action::UpdateVoltage {
            cycle_output: true,
            ..
        }
    ));
}

const NO_SETTLE: Duration = Duration::ZERO;

#[test]
fn apply_step_up_happy_path() {
    let mut s = active(lfp_4s());
    let p = expected_poll(&s, b(OK_V, -4.0));
    assert!(matches!(
        s.tick(p, TICK),
        Action::UpdateVoltage {
            cycle_output: false,
            ..
        }
    ));
    let mut errs = Vec::new();
    let mut xy = MockWriter::default();
    apply_update_voltage(&mut xy, &mut s, lfp_4s().absorb_v, false, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().absorb_v]);
    assert!(
        xy.set_output_calls.is_empty(),
        "step-up must not touch output"
    );
    assert!(errs.is_empty());
    assert!(matches!(s.phase(), Phase::Absorb));
}

#[test]
fn apply_step_down_happy_path() {
    let mut s = active(lfp_4s());
    drive_to_absorb_to_float_pending(&mut s);
    let mut errs = Vec::new();
    let mut xy = MockWriter::default();
    apply_update_voltage(&mut xy, &mut s, lfp_4s().float_v, true, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert_eq!(
        xy.set_output_calls,
        vec![false, true],
        "must disable then re-enable around the write"
    );
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().float_v]);
    assert!(errs.is_empty());
    assert!(matches!(s.phase(), Phase::Float));
}

#[test]
fn apply_step_down_step1_failure_does_not_write_voltage() {
    let mut s = active(lfp_4s());
    drive_to_absorb_to_float_pending(&mut s);
    let mut errs = Vec::new();
    let mut xy = MockWriter {
        fail_set_output_at: vec![0],
        ..Default::default()
    };
    apply_update_voltage(&mut xy, &mut s, lfp_4s().float_v, true, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert_eq!(xy.set_output_calls, vec![false]);
    assert!(xy.set_voltage_calls.is_empty());
    assert_eq!(errs, vec![XyError::SetOutput]);
    assert!(matches!(s.phase(), Phase::Absorb));
    // Supervisor re-emits UpdateVoltage on next tick for retry.
    let p = expected_poll(&s, b(CV_V, -(lfp_4s().exit_absorb_a - 0.1)));
    assert!(matches!(
        s.tick(p, TICK),
        Action::UpdateVoltage {
            cycle_output: true,
            ..
        }
    ));
}

#[test]
fn apply_step_down_step2_failure_restores_output() {
    let mut s = active(lfp_4s());
    drive_to_absorb_to_float_pending(&mut s);
    let mut errs = Vec::new();
    let mut xy = MockWriter {
        fail_set_voltage_at: vec![0],
        ..Default::default()
    };
    apply_update_voltage(&mut xy, &mut s, lfp_4s().float_v, true, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert_eq!(
        xy.set_output_calls,
        vec![false, true],
        "must restore output after set_voltage failure"
    );
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().float_v]);
    assert_eq!(errs, vec![XyError::SetVoltage]);
    assert!(matches!(s.phase(), Phase::Absorb));
}

#[test]
fn apply_step_down_step2_then_restore_both_fail_records_both() {
    let mut s = active(lfp_4s());
    drive_to_absorb_to_float_pending(&mut s);
    let mut errs = Vec::new();
    let mut xy = MockWriter {
        fail_set_voltage_at: vec![0],
        // call 0 = initial disable (success), call 1 = restore (fail).
        fail_set_output_at: vec![1],
        ..Default::default()
    };
    apply_update_voltage(&mut xy, &mut s, lfp_4s().float_v, true, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert_eq!(xy.set_output_calls, vec![false, true]);
    assert_eq!(errs, vec![XyError::SetVoltage, XyError::SetOutput]);
    assert!(matches!(s.phase(), Phase::Absorb));
}

#[test]
fn apply_step_down_step3_failure_retries_once_then_records() {
    let mut s = active(lfp_4s());
    drive_to_absorb_to_float_pending(&mut s);
    let mut errs = Vec::new();
    let mut xy = MockWriter {
        // call 0 = initial disable (ok), 1 = re-enable attempt 1 (fail),
        // 2 = re-enable attempt 2 (fail).
        fail_set_output_at: vec![1, 2],
        ..Default::default()
    };
    apply_update_voltage(&mut xy, &mut s, lfp_4s().float_v, true, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert_eq!(xy.set_output_calls, vec![false, true, true]);
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().float_v]);
    assert_eq!(errs, vec![XyError::SetOutput]);
    // Phase IS committed (ack ran between V_SET write and re-enable).
    assert!(matches!(s.phase(), Phase::Float));
}

#[test]
fn apply_step_down_step3_first_attempt_recovers_on_retry() {
    let mut s = active(lfp_4s());
    drive_to_absorb_to_float_pending(&mut s);
    let mut errs = Vec::new();
    let mut xy = MockWriter {
        // Re-enable attempt 1 fails, attempt 2 succeeds — no error recorded.
        fail_set_output_at: vec![1],
        ..Default::default()
    };
    apply_update_voltage(&mut xy, &mut s, lfp_4s().float_v, true, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert_eq!(xy.set_output_calls, vec![false, true, true]);
    assert!(errs.is_empty(), "transient single failure must not record");
    assert!(matches!(s.phase(), Phase::Float));
}

#[test]
fn apply_step_up_failure_does_not_touch_output() {
    let mut s = active(lfp_4s());
    let p = expected_poll(&s, b(OK_V, -4.0));
    assert!(matches!(
        s.tick(p, TICK),
        Action::UpdateVoltage {
            cycle_output: false,
            ..
        }
    ));
    let mut errs = Vec::new();
    let mut xy = MockWriter {
        fail_set_voltage_at: vec![0],
        ..Default::default()
    };
    apply_update_voltage(&mut xy, &mut s, lfp_4s().absorb_v, false, NO_SETTLE, |e| {
        errs.push(e)
    });
    assert!(
        xy.set_output_calls.is_empty(),
        "step-up failure must NOT cycle output"
    );
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().absorb_v]);
    assert_eq!(errs, vec![XyError::SetVoltage]);
    assert!(matches!(s.phase(), Phase::Float));
}
