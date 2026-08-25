//! Pack identity: that a `Profile` derives the setpoints, currents and
//! hardware trip thresholds its chemistry and cell count imply.

use super::*;

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
    //   i_set_a        = 10.00  + 0   = 10.00 A   (no budgeted load)
    //   ocp_a          = 10.00  × 1.5 = 15.00 A
    //   lvp_v          = 24     − 2   = 22.00 V
    let p = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
    assert_approx(p.absorb_v, 14.40);
    assert_approx(p.float_v, 13.50);
    assert_approx(p.regulation_a, 10.00);
    assert_approx(p.enter_absorb_a, 3.00);
    assert_approx(p.exit_absorb_a, 2.50);
    let s = p.buck_setup(TEST_SUPPLY).limits;
    assert_approx(s.ovp_v, 15.00);
    assert_approx(s.ocp_a, 15.00);
    assert_approx(s.lvp_v, 22.00);
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
    assert_approx(p.absorb_v, 14.6);
    assert_approx(p.float_v, 13.5);
}

#[test]
fn liion_3s_voltages_match_known_setpoints() {
    let p = Profile::for_pack(Chemistry::LiIon, 3, 50.0);
    // Longevity-tuned: 4.10 × 3 = 12.3, 4.00 × 3 = 12.0.
    assert_approx(p.absorb_v, 12.3);
    assert_approx(p.float_v, 12.0);
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
    assert_approx(p1.absorb_v, 3.60);
    assert_approx(p4.absorb_v, 3.60 * 4.0);
    assert_approx(p16.absorb_v, 3.60 * 16.0);
    assert_approx(p1.float_v, 3.375);
    assert_approx(p4.float_v, 3.375 * 4.0);
    assert_approx(p16.float_v, 3.375 * 16.0);
}

#[test]
fn currents_scale_with_capacity_not_cells() {
    // Same capacity, different S → identical currents.
    let p4 = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
    let p16 = Profile::for_pack(Chemistry::LiFePo4, 16, 50.0);
    assert_approx(p4.regulation_a, p16.regulation_a);
    assert_approx(p4.enter_absorb_a, p16.enter_absorb_a);
    assert_approx(p4.exit_absorb_a, p16.exit_absorb_a);
    // Same S, different capacity → currents scale linearly.
    let p100 = Profile::for_pack(Chemistry::LiFePo4, 4, 100.0);
    assert_approx(p100.regulation_a, 2.0 * p4.regulation_a);
    assert_approx(p100.enter_absorb_a, 2.0 * p4.enter_absorb_a);
    assert_approx(p100.exit_absorb_a, 2.0 * p4.exit_absorb_a);
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
fn largest_pack_the_buck_can_charge() {
    // OCP is the binding ceiling, not I_SET: 27 A / (0.2 C × 1.5) = 90 Ah
    // exactly, while I_SET's 25 A ceiling would allow 125 Ah. Both hand
    // figures are pinned here so the next test's rejection is known to land
    // one step past the real edge rather than somewhere short of it.
    let p = Profile::for_pack(Chemistry::LiFePo4, 4, 90.0);
    assert_approx(p.regulation_a, 18.0);
    let s = p.buck_setup(TEST_SUPPLY).limits;
    assert_approx(s.ocp_a, 27.0);
}

#[test]
#[should_panic(expected = "derived OCP exceeds")]
fn oversized_pack_rejected_by_derived_ocp() {
    // 100 Ah → 20 A CC → 30 A OCP. The buck rejects the register write, and
    // an unguarded profile turns that into a boot loop with the buck dark.
    let _ = Profile::for_pack(Chemistry::LiFePo4, 4, 100.0).buck_setup(TEST_SUPPLY).limits;
}

#[test]
fn load_budget_widens_i_set_and_ocp() {
    // The buck limits *total* output current and the load sits on that
    // output, so a load that is not in the setpoint is charge current the
    // pack never receives. Both current figures move with it; the voltage
    // ones are pack- and rail-side and must not.
    let p = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
    let unloaded = p.buck_setup(TEST_SUPPLY);
    let loaded = p.buck_setup(SupplyBudget {
        load_a: 5.0,
        ..TEST_SUPPLY
    });
    assert_approx(unloaded.i_set_a, 10.0);
    assert_approx(loaded.i_set_a, 15.0);
    assert_approx(unloaded.limits.ocp_a, 15.0);
    assert_approx(loaded.limits.ocp_a, 22.5);
    assert_approx(unloaded.limits.ovp_v, loaded.limits.ovp_v);
    assert_approx(unloaded.limits.lvp_v, loaded.limits.lvp_v);
}

#[test]
#[should_panic(expected = "derived OCP exceeds")]
fn load_budget_counts_against_the_ceiling_too() {
    // 50 Ah is 10 A of charge, comfortably inside every ceiling on its own.
    // Budget a 9 A load and I_SET becomes 19 A ⇒ OCP 28.5 A, past 27 — the
    // pack is fine, the pack *on this board* is not.
    let supply = SupplyBudget {
        load_a: 9.0,
        ..TEST_SUPPLY
    };
    let _ = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0).buck_setup(supply);
}

#[test]
#[should_panic(expected = "V_SET ceiling")]
fn too_many_cells_rejected_by_v_set_ceiling() {
    // 3.60 × 20 = 72 V, past the 70 V V_SET ceiling. 19S (68.4 V) is the
    // most series cells this buck can take on LFP.
    let _ = Profile::for_pack(Chemistry::LiFePo4, 20, 50.0);
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
    let _ = supervisor(bogus);
}

#[test]
fn safety_limits_track_chemistry_change() {
    // Top-balance pushes absorb to 14.6 V — OVP must move up too, not stay
    // at the 4S-daily 15.0 V. Without derived limits this is the footgun.
    let s = Profile::for_pack(Chemistry::LiFePo4TopBalance, 4, 50.0).buck_setup(TEST_SUPPLY).limits;
    assert_approx(s.ovp_v, 15.2);
    assert!(s.ovp_v > 14.6, "OVP must clear absorb_v");
}

#[test]
fn safety_limits_track_cell_count_change() {
    let s4 = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0).buck_setup(TEST_SUPPLY).limits;
    let s8 = Profile::for_pack(Chemistry::LiFePo4, 8, 50.0).buck_setup(TEST_SUPPLY).limits;
    assert!(s8.ovp_v > s4.ovp_v, "OVP scales with cell count");
    // OCP is current-only (and capacity-derived) and LVP is input-side —
    // both independent of S.
    assert_approx(s4.ocp_a, s8.ocp_a);
    assert_approx(s4.lvp_v, s8.lvp_v);
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
        let s = p.buck_setup(TEST_SUPPLY).limits;
        let supervisor_trip = p.absorb_v + OV_MARGIN_V;
        assert!(
            s.ovp_v > supervisor_trip,
            "ovp {} must exceed supervisor trip {}",
            s.ovp_v,
            supervisor_trip
        );
    }
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
    assert_approx(lfp.target_voltage(), 14.4);
    assert_approx(liion.target_voltage(), 12.3);
}

#[test]
fn single_cell_lfp_works() {
    // 1S 50 Ah LFP — float 3.375 V, absorb 3.60 V (daily). Same currents
    // as the 4S pack since they derive from capacity, not cell count.
    let mut s = active(Profile::for_pack(Chemistry::LiFePo4, 1, 50.0));
    assert_approx(s.target_voltage(), 3.375);
    assert!(matches!(
        ok_tick(&mut s, b(3.4, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_approx(s.target_voltage(), 3.60);
}
