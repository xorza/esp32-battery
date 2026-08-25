//! Pack identity: chemistry, cell count, capacity, and the charge
//! setpoints and safety limits that derive from them.

use crate::battery::{self, Chemistry};
use crate::charging::{
    ENTER_ABSORB_C, EXIT_ABSORB_C, HARDWARE_OVP_MARGIN_V, INPUT_LVP_MARGIN_V, REGULATION_C,
};
use xy_modbus::SafetyLimits;

/// XY7025 register write ceilings, mirrored from `xy_modbus`'s private
/// `input_spec` table.
///
/// The driver rejects an out-of-range write before it reaches the bus. At
/// boot that means `boot_sequence` fails on every retry and
/// `boot_with_retries` reboots straight back into the same failure — the
/// buck never energises and, on a UPS, the load runs the pack flat. The
/// asserts below turn a pack this hardware cannot charge into a build
/// error instead: `for_pack` and `safety_limits` are `const fn`, and the
/// firmware's `PACK_PROFILE` / `SAFETY` are `const`, so the check runs at
/// compile time for the pack actually shipped.
///
/// Mirrored rather than imported because xy-modbus keeps the table
/// private. `ModelCheck::limits_match` is the runtime counterpart — it
/// says whether the connected device's own ceilings are the XY7025's.
const V_SET_CEILING: f32 = 70.0;
const I_SET_CEILING: f32 = 25.0;
const OVP_CEILING: f32 = 72.0;
const OCP_CEILING: f32 = 27.0;
const LVP_FLOOR: f32 = 10.0;
const LVP_CEILING: f32 = 95.0;

/// How far below the float target's *own* resting SoC a pack may sit and
/// still count as full at bring-up, so it parks in Float instead of being
/// owed a step up to absorb.
///
/// Expressed as SoC rather than a voltage because only the chemistry's OCV
/// curve can turn a resting terminal voltage into "how full is it" — the
/// LFP plateau is flat enough that 0.1 V/cell spans most of the pack.
///
/// Expressed as a *margin against `soc(float_v)`* rather than an absolute
/// figure because "full" is a property of the profile, not of the
/// chemistry's 100 % point. Li-ion charged to the longevity-tuned 4.10 V
/// rests at ~91 % of a 4.20 V-referenced full and floats at ~82 %, where
/// LFP floats at 97.5 %. Any absolute bar that suits one calls the other
/// empty and puts it straight back into absorb — which is the bug this
/// gate exists to fix, reintroduced one chemistry over.
///
/// 5 points is wide enough for the droop of a pack that has been floating
/// and then rested — on the flat LFP plateau that is only ~17 mV/cell —
/// and narrow enough that a pack at 90 % still gets topped up, as it
/// should.
const FULL_SOC_MARGIN: f32 = 5.0;

#[derive(Copy, Clone, Debug)]
pub struct Profile {
    pub chemistry: Chemistry,
    pub cells: u8,
    /// Rated pack capacity — the input the `*_a` currents were scaled from.
    /// Kept for display/identity; not used by the supervisor.
    pub capacity_ah: f32,
    pub absorb_v: f32,
    pub float_v: f32,
    /// Constant-current setpoint sent to the buck during normal charging.
    pub regulation_a: f32,
    pub enter_absorb_a: f32,
    pub exit_absorb_a: f32,
}
impl Profile {
    /// Build a pack-level profile from chemistry, series cell count, and
    /// pack capacity. Voltages scale with `cells`; charge/taper currents
    /// scale with `capacity_ah` via the `*_C` constants above. Same C-rates
    /// across chemistries — the LFP literature is the basis, but the
    /// fractions are conservative enough that NMC/LCO are also safe.
    pub const fn for_pack(chemistry: Chemistry, cells: u8, capacity_ah: f32) -> Self {
        assert!(cells > 0);
        assert!(capacity_ah > 0.0);
        let v = chemistry.charge_voltages();
        let s = cells as f32;
        let absorb_v = v.absorb_v * s;
        let regulation_a = capacity_ah * REGULATION_C;
        // absorb_v is the higher of the two targets, so it alone bounds V_SET.
        assert!(
            absorb_v <= V_SET_CEILING,
            "absorb target exceeds the buck's V_SET ceiling — too many cells in series"
        );
        assert!(
            regulation_a <= I_SET_CEILING,
            "charge current exceeds the buck's I_SET ceiling — pack capacity is too large \
             for this hardware at REGULATION_C"
        );
        Self {
            chemistry,
            cells,
            capacity_ah,
            absorb_v,
            float_v: v.float_v * s,
            regulation_a,
            enter_absorb_a: capacity_ah * ENTER_ABSORB_C,
            exit_absorb_a: capacity_ah * EXIT_ABSORB_C,
        }
    }

    /// Estimated state-of-charge (0.0–100.0) from pack bus voltage, using
    /// this pack's chemistry and cell count.
    pub fn soc(&self, pack_voltage_v: f32) -> f32 {
        battery::ocv_soc(self.chemistry, self.cells, pack_voltage_v)
    }

    /// Resting state-of-charge at or above which a pack counts as full, so
    /// bring-up parks in Float instead of owing it a step up to absorb.
    /// Derived from this profile's own float target — see
    /// [`FULL_SOC_MARGIN`] for why it cannot be an absolute figure.
    pub(super) fn full_rest_soc(&self) -> f32 {
        self.soc(self.float_v) - FULL_SOC_MARGIN
    }

    /// Derive hard trip thresholds for the buck's own protection. The buck
    /// fires these only when regulation has already failed — the supervisor's
    /// debounced OV at `absorb_v + OV_MARGIN_V` should catch problems first.
    /// Hardware OVP sits at 3× that margin so the supervisor always wins
    /// (the const-block above enforces this at compile time). OCP is 50%
    /// over the CC setpoint. LVP on the XY7025 is **input** UVLO, not a
    /// pack-side cutoff — it's tied to the supply rail, not the profile.
    pub const fn safety_limits(&self, input_nominal_v: f32) -> SafetyLimits {
        let ovp_v = self.absorb_v + HARDWARE_OVP_MARGIN_V;
        let ocp_a = self.regulation_a * 1.5;
        let lvp_v = input_nominal_v - INPUT_LVP_MARGIN_V;
        // OCP binds before I_SET does: at 1.5× the CC setpoint it reaches its
        // 27 A ceiling while regulation_a is still only 18 A.
        assert!(
            ocp_a <= OCP_CEILING,
            "derived OCP exceeds the buck's ceiling — pack capacity is too large for \
             this hardware at REGULATION_C × 1.5"
        );
        assert!(
            ovp_v <= OVP_CEILING,
            "derived OVP exceeds the buck's ceiling — too many cells in series"
        );
        assert!(
            lvp_v >= LVP_FLOOR && lvp_v <= LVP_CEILING,
            "derived input UVLO is outside the buck's LVP range"
        );
        SafetyLimits {
            ovp_v,
            ocp_a,
            lvp_v,
        }
    }
}

/// Compact pack identity for the LCD / web UI, e.g. `LFP 4S 50Ah`.
impl std::fmt::Display for Profile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} {}S {:.0}Ah",
            self.chemistry, self.cells, self.capacity_ah
        )
    }
}
