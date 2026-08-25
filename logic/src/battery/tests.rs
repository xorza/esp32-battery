use super::*;

fn approx(a: f32, b: f32) -> bool {
    (a - b).abs() < 0.01
}

/// Assert two floats match, naming both in the failure. `assert!(approx(..))`
/// reports only "assertion failed", which says nothing about how far off a
/// derived value actually was. `#[track_caller]` keeps the reported line at
/// the call site.
#[track_caller]
fn assert_approx(actual: f32, expected: f32) {
    assert!(
        approx(actual, expected),
        "{actual} != {expected}"
    );
}

const LFP: Chemistry = Chemistry::LiFePo4;

#[test]
fn below_minimum_returns_zero() {
    // 4S floor is 4 × 2.5 = 10.0 V.
    assert_eq!(ocv_soc(LFP, 4, 5.0), 0.0);
    assert_eq!(ocv_soc(LFP, 4, 10.0), 0.0);
}

#[test]
fn above_maximum_returns_hundred() {
    // 4S ceiling is 4 × 3.65 = 14.6 V.
    assert_eq!(ocv_soc(LFP, 4, 14.60), 100.0);
    assert_eq!(ocv_soc(LFP, 4, 20.0), 100.0);
}

#[test]
fn exact_curve_entries_4s() {
    // Each per-cell point, scaled to a 4S pack, reads back its SoC.
    for &(cell_v, soc) in LFP_OCV {
        let pack_v = cell_v * 4.0;
        let result = ocv_soc(LFP, 4, pack_v);
        assert!(
            approx(result, soc),
            "ocv_soc(4S, {pack_v}) = {result}, expected {soc}"
        );
    }
}

#[test]
fn matches_legacy_4s_table_points() {
    // Spot-check the old pack table is preserved: (13.04, 50.0), (13.20, 70.0).
    assert_approx(ocv_soc(LFP, 4, 13.04), 50.0);
    assert_approx(ocv_soc(LFP, 4, 13.20), 70.0);
}

#[test]
fn interpolation_midpoint() {
    // Pack 13.02 V = 3.255 V/cell, midway between (3.250, 40.0) and (3.260, 50.0).
    let result = ocv_soc(LFP, 4, 13.02);
    assert!((result - 45.0).abs() < 0.1, "got {result}, expected ~45.0");
}

#[test]
fn cell_count_scales_voltage() {
    // Same per-cell voltage at 4S and 8S yields the same SoC.
    let four = ocv_soc(LFP, 4, 3.26 * 4.0);
    let eight = ocv_soc(LFP, 8, 3.26 * 8.0);
    assert!(
        approx(four, eight),
        "4S at 3.26 V/cell = {four}, 8S = {eight}"
    );
    assert_approx(four, 50.0);
}

#[test]
fn chemistry_changes_result() {
    // Same per-cell voltage, different curve → different SoC.
    // LFP at 3.70 V/cell is past its plateau (100%); Li-ion is at 40%.
    let cell_v = 3.70;
    let lfp = ocv_soc(Chemistry::LiFePo4, 1, cell_v);
    let liion = ocv_soc(Chemistry::LiIon, 1, cell_v);
    assert_approx(lfp, 100.0);
    assert_approx(liion, 40.0);
    assert!(lfp != liion);
}

#[test]
fn top_balance_shares_lfp_curve() {
    let lfp = ocv_soc(Chemistry::LiFePo4, 4, 13.04);
    let top = ocv_soc(Chemistry::LiFePo4TopBalance, 4, 13.04);
    assert_eq!(lfp, top);
}

#[test]
fn monotonically_increasing() {
    for &chem in &[Chemistry::LiFePo4, Chemistry::LiIon] {
        let mut prev = ocv_soc(chem, 1, 2.0);
        let mut v = 2.0;
        while v <= 4.5 {
            let soc = ocv_soc(chem, 1, v);
            assert!(soc >= prev, "{chem:?}: ocv_soc({v}) = {soc} < prev {prev}");
            prev = soc;
            v += 0.01;
        }
    }
}

#[test]
fn output_range() {
    let mut v = 0.0;
    while v <= 20.0 {
        let soc = ocv_soc(LFP, 4, v);
        assert!(
            (0.0..=100.0).contains(&soc),
            "ocv_soc({v}) = {soc} out of range"
        );
        v += 0.1;
    }
}

#[test]
fn non_finite_returns_zero() {
    assert_eq!(ocv_soc(LFP, 4, f32::NAN), 0.0);
    assert_eq!(ocv_soc(LFP, 4, f32::INFINITY), 0.0);
    assert_eq!(ocv_soc(LFP, 4, f32::NEG_INFINITY), 0.0);
}

#[test]
#[should_panic]
fn zero_cells_panics() {
    let _ = ocv_soc(LFP, 0, 13.0);
}
