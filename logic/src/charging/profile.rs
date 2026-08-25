//! Pack identity: chemistry, cell count, capacity, and the charge
//! setpoints and safety limits that derive from them.

use crate::battery::{self, Chemistry};
use crate::charging::{
    ENTER_ABSORB_C, EXIT_ABSORB_C, HARDWARE_OVP_MARGIN_V, INPUT_LVP_MARGIN_V, REGULATION_C,
};
use xy_modbus::SafetyLimits;

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
        Self {
            chemistry,
            cells,
            capacity_ah,
            absorb_v: v.absorb_v * s,
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

    /// Derive hard trip thresholds for the buck's own protection. The buck
    /// fires these only when regulation has already failed — the supervisor's
    /// debounced OV at `absorb_v + OV_MARGIN_V` should catch problems first.
    /// Hardware OVP sits at 3× that margin so the supervisor always wins
    /// (the const-block above enforces this at compile time). OCP is 50%
    /// over the CC setpoint. LVP on the XY7025 is **input** UVLO, not a
    /// pack-side cutoff — it's tied to the supply rail, not the profile.
    pub const fn safety_limits(&self, input_nominal_v: f32) -> SafetyLimits {
        SafetyLimits {
            ovp_v: self.absorb_v + HARDWARE_OVP_MARGIN_V,
            ocp_a: self.regulation_a * 1.5,
            lvp_v: input_nominal_v - INPUT_LVP_MARGIN_V,
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
