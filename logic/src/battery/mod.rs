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

impl core::fmt::Display for Chemistry {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Chemistry::LiFePo4 => "LFP",
            Chemistry::LiFePo4TopBalance => "LFP-TB",
            Chemistry::LiIon => "Li-ion",
        })
    }
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
            Chemistry::LiFePo4 => CellVoltages {
                absorb_v: 3.60,
                float_v: 3.375,
            },
            Chemistry::LiFePo4TopBalance => CellVoltages {
                absorb_v: 3.65,
                float_v: 3.375,
            },
            Chemistry::LiIon => CellVoltages {
                absorb_v: 4.10,
                float_v: 4.00,
            },
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
mod tests;
