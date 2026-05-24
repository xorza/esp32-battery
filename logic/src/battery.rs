//! Single source of battery-chemistry knowledge: charge setpoints and the
//! open-circuit-voltage → state-of-charge curve, both expressed **per cell**.
//! Pack-level numbers are derived by scaling with the series cell count, so a
//! chemistry/cell-count change moves charge voltages, safety limits, and the
//! reported SoC in lockstep (see `charging::Profile`).

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Chemistry {
    /// Daily-cycling LFP: 3.60 V/cell absorb, 3.375 V/cell float.
    /// Matches Victron / Battle Born defaults — gentler on cells than 3.65 V,
    /// reaches ~99% SoC either way (Battery University BU-808b, Off-Grid Garage tests).
    LiFePo4,
    /// Top-balance variant for LFP: 3.65 V/cell absorb (manufacturer max).
    /// Use sparingly when the BMS needs the high voltage to balance cells.
    LiFePo4TopBalance,
    /// Longevity-tuned Li-ion (NMC/LCO): 4.10 V/cell absorb, 4.00 V/cell float.
    /// 4.10 V trades ~15% capacity for dramatically more cycles vs. 4.20 V.
    LiIon,
}

/// Per-cell charge setpoints. Scaled by cell count in `charging::Profile::for_pack`.
#[derive(Copy, Clone, Debug)]
pub(crate) struct CellVoltages {
    pub absorb_v: f32,
    pub float_v: f32,
}

/// Per-cell resting OCV → SoC for LFP. Equivalent to the old 4S (12 V) pack
/// table divided by 4 — interpolation is scale-invariant in voltage, so 4S
/// LFP SoC is unchanged. Flat plateau (3.2–3.4 V) is the LFP signature.
const LFP_OCV: &[(f32, f32)] = &[
    (2.500, 0.0),
    (2.540, 0.5),
    (2.800, 5.0),
    (3.000, 9.5),
    (3.050, 15.0),
    (3.200, 20.0),
    (3.230, 30.0),
    (3.250, 40.0),
    (3.260, 50.0),
    (3.280, 60.0),
    (3.300, 70.0),
    (3.330, 80.0),
    (3.350, 90.0),
    (3.380, 99.0),
    (3.450, 99.5),
    (3.650, 100.0),
];

/// Per-cell resting OCV → SoC for Li-ion (NMC/LCO). Sloped curve, unlike LFP's
/// plateau — a cell charged only to the longevity-tuned 4.10 V rests below the
/// 4.20 V/100% point and reads ~90% here, which is honest.
const LIION_OCV: &[(f32, f32)] = &[
    (3.00, 0.0),
    (3.40, 5.0),
    (3.50, 10.0),
    (3.57, 20.0),
    (3.63, 30.0),
    (3.70, 40.0),
    (3.75, 50.0),
    (3.82, 60.0),
    (3.90, 70.0),
    (3.98, 80.0),
    (4.08, 90.0),
    (4.15, 95.0),
    (4.20, 100.0),
];

impl Chemistry {
    /// Per-cell absorb/float setpoints for this chemistry.
    pub(crate) const fn charge_voltages(self) -> CellVoltages {
        match self {
            Chemistry::LiFePo4 => CellVoltages { absorb_v: 3.60, float_v: 3.375 },
            Chemistry::LiFePo4TopBalance => CellVoltages { absorb_v: 3.65, float_v: 3.375 },
            Chemistry::LiIon => CellVoltages { absorb_v: 4.10, float_v: 4.00 },
        }
    }

    /// Per-cell resting OCV → SoC curve. Top-balance shares the LFP curve —
    /// the resting OCV/SoC relationship is set by chemistry, not charge target.
    const fn ocv_curve(self) -> &'static [(f32, f32)] {
        match self {
            Chemistry::LiFePo4 | Chemistry::LiFePo4TopBalance => LFP_OCV,
            Chemistry::LiIon => LIION_OCV,
        }
    }
}

/// Estimated charge percentage (0.0–100.0) from pack bus voltage, for the
/// given chemistry and series cell count. Divides to per-cell voltage, then
/// linearly interpolates the chemistry's OCV curve.
pub(crate) fn ocv_soc(chemistry: Chemistry, cells: u8, pack_voltage_v: f32) -> f32 {
    assert!(cells > 0);
    if !pack_voltage_v.is_finite() {
        return 0.0;
    }
    interpolate(chemistry.ocv_curve(), pack_voltage_v / cells as f32)
}

fn interpolate(curve: &[(f32, f32)], cell_v: f32) -> f32 {
    if cell_v <= curve[0].0 {
        return curve[0].1;
    }
    let last = curve[curve.len() - 1];
    if cell_v >= last.0 {
        return last.1;
    }
    for i in 1..curve.len() {
        let (v_lo, soc_lo) = curve[i - 1];
        let (v_hi, soc_hi) = curve[i];
        if cell_v <= v_hi {
            let t = (cell_v - v_lo) / (v_hi - v_lo);
            return soc_lo + t * (soc_hi - soc_lo);
        }
    }
    unreachable!()
}

#[cfg(test)]
mod tests {
    use super::*;

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
                (result - soc).abs() < 0.01,
                "ocv_soc(4S, {pack_v}) = {result}, expected {soc}"
            );
        }
    }

    #[test]
    fn matches_legacy_4s_table_points() {
        // Spot-check the old pack table is preserved: (13.04, 50.0), (13.20, 70.0).
        assert!((ocv_soc(LFP, 4, 13.04) - 50.0).abs() < 0.01);
        assert!((ocv_soc(LFP, 4, 13.20) - 70.0).abs() < 0.01);
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
        assert!((four - eight).abs() < 0.01);
        assert!((four - 50.0).abs() < 0.01);
    }

    #[test]
    fn chemistry_changes_result() {
        // Same per-cell voltage, different curve → different SoC.
        // LFP at 3.70 V/cell is past its plateau (100%); Li-ion is at 40%.
        let cell_v = 3.70;
        let lfp = ocv_soc(Chemistry::LiFePo4, 1, cell_v);
        let liion = ocv_soc(Chemistry::LiIon, 1, cell_v);
        assert!((lfp - 100.0).abs() < 0.01, "lfp {lfp}");
        assert!((liion - 40.0).abs() < 0.01, "liion {liion}");
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
            assert!((0.0..=100.0).contains(&soc), "ocv_soc({v}) = {soc} out of range");
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
}
