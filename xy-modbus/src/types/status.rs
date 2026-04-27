//! Live readings, setpoints, and cumulative counters.

/// Output voltage / current setpoints (registers 0x0000–0x0001).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Setpoints {
    pub v_set: f32,
    pub i_set: f32,
}

/// Live status block (registers 0x0000–0x0005).
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Status {
    pub v_set: f32,
    pub i_set: f32,
    pub v_out: f32,
    pub i_out: f32,
    pub p_out: f32,
    pub v_in: f32,
}

/// Cumulative output counters and on-time (registers 0x0006–0x000C).
///
/// The high-word readings of charge and energy are flagged as untested
/// in community docs; trust the 32-bit composition only after verifying
/// against your hardware. The raw words are exposed alongside the
/// composed values so consumers can reinterpret them.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Totals {
    /// Cumulative output charge in Ah.
    /// `((ah_high as u32) << 16 | ah_low as u32) as f32 / 1000.0`.
    pub charge_ah: f32,
    /// Cumulative output energy in Wh.
    pub energy_wh: f32,
    /// Output-on time, accumulated.
    pub on_time: OnTime,
    pub ah_low_raw: u16,
    pub ah_high_raw: u16,
    pub wh_low_raw: u16,
    pub wh_high_raw: u16,
}

/// Output-on time as reported by the device (h/m/s).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct OnTime {
    pub hours: u16,
    pub minutes: u16,
    pub seconds: u16,
}

impl OnTime {
    pub const fn total_seconds(self) -> u32 {
        self.hours as u32 * 3600 + self.minutes as u32 * 60 + self.seconds as u32
    }
}

/// Hard trip limits programmed into the buck's protection registers.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SafetyLimits {
    pub lvp_v: f32,
    pub ovp_v: f32,
    pub ocp_a: f32,
}
