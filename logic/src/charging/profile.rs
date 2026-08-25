//! Pack identity: chemistry, cell count, capacity, and the charge
//! setpoints that derive from them — plus the board-side budget they have
//! to be combined with before anything can be programmed into the buck.

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
/// Lowest input the XY7025 will regulate from. Distinct from `LVP_FLOOR`,
/// which is only what the LVP *register* accepts: the derived UVLO has to
/// sit above this so the buck cuts its output before the rail drops out of
/// the converter's operating range, rather than after.
const INPUT_MIN_V: f32 = 12.0;

/// Headroom the buck's own OCP gets over the CC setpoint. The buck fires it
/// only once regulation has already failed, so it wants to sit clear of
/// normal operation without being so wide it never trips.
const OCP_HEADROOM: f32 = 1.5;
/// Which current ceiling binds first. At this headroom OCP always does — an
/// I_SET over 25 A implies an OCP over 37 A, long past 27 — so the I_SET
/// assert in `buck_setup` is unreachable and kept only so that lowering the
/// headroom cannot silently let I_SET outrun its register. If this fires,
/// that has happened and the I_SET check has become the live one.
const _: () = assert!(OCP_CEILING < I_SET_CEILING * OCP_HEADROOM);

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

/// What the board puts around the pack: the rail feeding the buck, and the
/// continuous load hanging off its output.
///
/// Board wiring, not pack identity — two units with identical packs can
/// differ here, and nothing in [`Profile`] can be derived from it or it
/// from [`Profile`]. Kept apart for that reason: what gets programmed into
/// the buck is a function of *both*, which is what [`Profile::buck_setup`]
/// computes.
#[derive(Copy, Clone, Debug)]
pub struct SupplyBudget {
    /// Nominal DC rail feeding the buck's input. Drives the input-UVLO
    /// (LVP) register — a supply property, tied to the rail rather than to
    /// anything about the pack.
    pub input_nominal_v: f32,
    /// Worst-case continuous load on the buck **output**, in amps.
    ///
    /// The buck's CC loop limits *total* output current, and on a UPS the
    /// load sits on that output in parallel with the pack. So the pack only
    /// ever receives `i_set_a - load_a`: sizing I_SET from the charge rate
    /// alone silently derates charging by whatever the load is drawing, and
    /// sizing OCP from it trips the buck's own protection on an ordinary
    /// load surge — which latches the supervisor off and drops the load
    /// onto the pack until someone reboots it.
    ///
    /// Budget the worst case, not the average. The supervisor does not
    /// trust this figure to bound what reaches the pack — that is
    /// `FaultReason::ChargeOvercurrent`, measured on the battery itself.
    pub load_a: f32,
}

/// What gets programmed into the buck, derived from pack × board.
///
/// The named result of [`Profile::buck_setup`]: `i_set_a` is the CC
/// setpoint the supervisor also compares readback against, `limits` are the
/// hard trips written into the device's own protection registers.
#[derive(Copy, Clone, Debug)]
pub struct BuckSetup {
    pub i_set_a: f32,
    pub limits: SafetyLimits,
}

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
        // absorb_v is the higher of the two targets, so it alone bounds V_SET.
        // The current ceilings are `buck_setup`'s to enforce: what reaches
        // I_SET is this rate *plus the board's load*, not this rate alone.
        assert!(
            absorb_v <= V_SET_CEILING,
            "absorb target exceeds the buck's V_SET ceiling — too many cells in series"
        );
        Self {
            chemistry,
            cells,
            capacity_ah,
            absorb_v,
            float_v: v.float_v * s,
            regulation_a: capacity_ah * REGULATION_C,
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

    /// Everything the buck gets programmed with, from this pack on this
    /// board.
    ///
    /// The CC setpoint carries the load as well as the charge rate, because
    /// the buck limits total output current and the load is on that output
    /// — see [`SupplyBudget::load_a`].
    ///
    /// The trip thresholds are the buck's own protection, which fires only
    /// once regulation has already failed: the supervisor's debounced OV at
    /// `absorb_v + OV_MARGIN_V` should catch problems first, so hardware
    /// OVP sits at 3× that margin (the const-block in `mod.rs` enforces it
    /// at compile time). OCP is 50 % over the CC setpoint. LVP on the
    /// XY7025 is **input** UVLO, not a pack-side cutoff — it is tied to the
    /// supply rail, so nothing in the profile can bound how far the pack
    /// discharges.
    pub const fn buck_setup(&self, supply: SupplyBudget) -> BuckSetup {
        let i_set_a = self.regulation_a + supply.load_a;
        let ovp_v = self.absorb_v + HARDWARE_OVP_MARGIN_V;
        let ocp_a = i_set_a * OCP_HEADROOM;
        let lvp_v = supply.input_nominal_v - INPUT_LVP_MARGIN_V;
        // OCP binds before I_SET does: at 1.5× the CC setpoint it reaches
        // its 27 A ceiling while I_SET is still only 18 A.
        assert!(
            ocp_a <= OCP_CEILING,
            "derived OCP exceeds the buck's ceiling — charge rate plus load budget is \
             too large for this hardware at OCP_HEADROOM"
        );
        // Unreachable while OCP binds first — see `OCP_HEADROOM`.
        assert!(
            i_set_a <= I_SET_CEILING,
            "charge rate plus load budget exceeds the buck's I_SET ceiling"
        );
        assert!(
            ovp_v <= OVP_CEILING,
            "derived OVP exceeds the buck's ceiling — too many cells in series"
        );
        assert!(
            lvp_v >= LVP_FLOOR && lvp_v <= LVP_CEILING,
            "derived input UVLO is outside the buck's LVP range"
        );
        assert!(
            lvp_v > INPUT_MIN_V,
            "derived input UVLO is below the buck's minimum operating input — \
             the rail would drop out before the buck cut its output"
        );
        BuckSetup {
            i_set_a,
            limits: SafetyLimits {
                ovp_v,
                ocp_a,
                lvp_v,
            },
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
