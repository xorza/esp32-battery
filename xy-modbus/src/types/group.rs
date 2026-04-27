//! Memory-group parameters (M0–M9).

/// All 14 registers of a memory group (M0–M9). Field order matches the
/// on-wire register order.
///
/// The two cumulative limits are exposed as raw low/high words. Use the
/// helpers below to compose them — note the scales differ: S-OAH is in
/// mAh (raw / 1000) while S-OWH is in 10 mWh units (raw / 100). This
/// asymmetry matches the firmware and the seller manual; it is *not*
/// the same scale as the cumulative WH counters at 0x0008/0x0009
/// (which are in mWh).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct GroupParams {
    pub v_set: f32,
    pub i_set: f32,
    pub s_lvp_v: f32,
    pub s_ovp_v: f32,
    pub s_ocp_a: f32,
    /// Over-power threshold in W. Resolution depends on model (1 W on
    /// XY6020L/XY7025, 0.1 W on SK family) — encoded by [`super::Model::opp_scale`].
    pub s_opp_w: f32,
    /// Output-on time limit, hours.
    pub s_ohp_h: u16,
    /// Output-on time limit, minutes.
    pub s_ohp_m: u16,
    pub s_oah_low: u16,
    pub s_oah_high: u16,
    pub s_owh_low: u16,
    pub s_owh_high: u16,
    /// Over-temperature threshold (°C/°F per [`super::TempUnit`]).
    pub s_otp: f32,
    /// Power-on output state. `false` = boot with output OFF, `true` = ON.
    pub power_on_output: bool,
}

impl GroupParams {
    /// Compose `s_oah_low`/`s_oah_high` into amp-hours (raw / 1000).
    pub fn max_charge_ah(&self) -> f32 {
        let raw = ((self.s_oah_high as u32) << 16) | self.s_oah_low as u32;
        raw as f32 / 1000.0
    }
    /// Compose `s_owh_low`/`s_owh_high` into watt-hours.
    ///
    /// Scale is **100** (10 mWh units) — *not* 1000 like the cumulative
    /// WH counters. The XY firmware stores the threshold in coarser
    /// units to extend the 32-bit range to ~42.9 GWh.
    pub fn max_energy_wh(&self) -> f32 {
        let raw = ((self.s_owh_high as u32) << 16) | self.s_owh_low as u32;
        raw as f32 / 100.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn group_max_charge_scale_is_1000() {
        // S-OAH is in mAh (raw / 1000). raw = (2<<16) | 500 = 131_572 → 131.572 Ah.
        let g = GroupParams {
            v_set: 0.0,
            i_set: 0.0,
            s_lvp_v: 0.0,
            s_ovp_v: 0.0,
            s_ocp_a: 0.0,
            s_opp_w: 0.0,
            s_ohp_h: 0,
            s_ohp_m: 0,
            s_oah_low: 500,
            s_oah_high: 2,
            s_owh_low: 0,
            s_owh_high: 0,
            s_otp: 0.0,
            power_on_output: false,
        };
        assert_eq!(g.max_charge_ah(), 131.572);
    }

    #[test]
    fn group_max_energy_scale_is_100() {
        // S-OWH is in 10 mWh units (raw / 100), distinct from the cumulative
        // WH counters which are in mWh. raw = 12_345 → 123.45 Wh.
        let g = GroupParams {
            v_set: 0.0,
            i_set: 0.0,
            s_lvp_v: 0.0,
            s_ovp_v: 0.0,
            s_ocp_a: 0.0,
            s_opp_w: 0.0,
            s_ohp_h: 0,
            s_ohp_m: 0,
            s_oah_low: 0,
            s_oah_high: 0,
            s_owh_low: 12_345,
            s_owh_high: 0,
            s_otp: 0.0,
            power_on_output: false,
        };
        assert_eq!(g.max_energy_wh(), 123.45);
    }
}
