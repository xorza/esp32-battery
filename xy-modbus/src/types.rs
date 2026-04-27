//! Value types returned and accepted by the device API.

use core::fmt;

// ─── Status & setpoints ──────────────────────────────────────────────────────

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

// ─── Model presets ───────────────────────────────────────────────────────────

/// Hardware variant. Selected at construction (`Xy::new`) and used to
/// scale the registers whose resolution differs across the family —
/// I-SET, IOUT, S-OCP, POWER, S-OPP. See `DATASHEET.md` §3 for the
/// scale table.
///
/// Cross-check by reading `MODEL` (`0x0016`): `0x6100`-class is
/// XY6020L / XY7025; SK-family codes differ. The crate does not probe
/// automatically — pick the variant that matches your hardware.
#[derive(Copy, Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Model {
    Xy6020L,
    Xy7025,
    Sk60,
    Sk120,
    Sk120x,
    /// Escape hatch for hardware not covered by the preset variants.
    /// Supply the per-register scales directly; cross-check against the
    /// vendor docs for your unit.
    Custom {
        current_scale: f32,
        power_scale: f32,
        opp_scale: f32,
    },
}

impl Model {
    /// Scale for I-SET, IOUT, S-OCP. 100 on XY6020L/XY7025 (10 mA),
    /// 1000 on SK family (1 mA).
    pub const fn current_scale(self) -> f32 {
        match self {
            Self::Xy6020L | Self::Xy7025 => 100.0,
            Self::Sk60 | Self::Sk120 | Self::Sk120x => 1000.0,
            Self::Custom { current_scale, .. } => current_scale,
        }
    }

    /// Scale for POWER (`0x0004`). 10 on XY6020L/XY7025 (100 mW),
    /// 100 on SK family (10 mW).
    pub const fn power_scale(self) -> f32 {
        match self {
            Self::Xy6020L | Self::Xy7025 => 10.0,
            Self::Sk60 | Self::Sk120 | Self::Sk120x => 100.0,
            Self::Custom { power_scale, .. } => power_scale,
        }
    }

    /// Scale for S-OPP in memory groups (`0x0055`). 1 W on
    /// XY6020L/XY7025, 0.1 W on SK family.
    pub const fn opp_scale(self) -> f32 {
        match self {
            Self::Xy6020L | Self::Xy7025 => 1.0,
            Self::Sk60 | Self::Sk120 | Self::Sk120x => 10.0,
            Self::Custom { opp_scale, .. } => opp_scale,
        }
    }
}

// ─── Enumerations ────────────────────────────────────────────────────────────

/// Regulation mode reported by `CVCC` (register 0x0011).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RegMode {
    ConstantVoltage,
    ConstantCurrent,
}

/// Temperature unit selected by `F-C` (register 0x0013).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum TempUnit {
    Celsius,
    Fahrenheit,
}

impl TempUnit {
    pub const fn from_reg(v: u16) -> Self {
        match v {
            0 => Self::Celsius,
            _ => Self::Fahrenheit,
        }
    }
    pub const fn to_reg(self) -> u16 {
        match self {
            Self::Celsius => 0,
            Self::Fahrenheit => 1,
        }
    }
}

/// Latched protection cause read from `PROTECT` (register 0x0010).
///
/// `Normal` (0) is the only non-tripped state. The register stays
/// latched until written back to 0.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ProtectionStatus {
    /// Operating normally.
    Normal,
    /// Output overvoltage. Also fires transiently when V-SET is raised
    /// above the current S-OVP threshold — program protection before
    /// raising V-SET.
    Ovp,
    /// Output overcurrent.
    Ocp,
    /// Output overpower.
    Opp,
    /// Input under-voltage (LVP setpoint).
    Lvp,
    /// Cumulative charge limit reached.
    Oah,
    /// Output-on time limit reached.
    Ohp,
    /// Over-temperature.
    Otp,
    /// Cumulative energy (Ah) limit reached.
    Oep,
    /// Cumulative energy (Wh) limit reached.
    Owh,
    /// Input over-current / inrush.
    Icp,
    /// Register read back a value outside the documented 0–10 range.
    Unknown(u16),
}

impl ProtectionStatus {
    pub const fn from_register(raw: u16) -> Self {
        match raw {
            0 => Self::Normal,
            1 => Self::Ovp,
            2 => Self::Ocp,
            3 => Self::Opp,
            4 => Self::Lvp,
            5 => Self::Oah,
            6 => Self::Ohp,
            7 => Self::Otp,
            8 => Self::Oep,
            9 => Self::Owh,
            10 => Self::Icp,
            other => Self::Unknown(other),
        }
    }
}

impl fmt::Display for ProtectionStatus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Normal => f.write_str("normal"),
            Self::Ovp => f.write_str("ovp"),
            Self::Ocp => f.write_str("ocp"),
            Self::Opp => f.write_str("opp"),
            Self::Lvp => f.write_str("lvp"),
            Self::Oah => f.write_str("oah"),
            Self::Ohp => f.write_str("ohp"),
            Self::Otp => f.write_str("otp"),
            Self::Oep => f.write_str("oep"),
            Self::Owh => f.write_str("owh"),
            Self::Icp => f.write_str("icp"),
            Self::Unknown(v) => write!(f, "unknown({v})"),
        }
    }
}

/// Baud-rate codes for `BAUDRATE_L` (register 0x0019).
///
/// Only `B115200` (code 6) is documented in the seller manual; codes
/// 0–5 and 7–8 are community-derived. Verify on your unit before
/// committing a write. Baud changes take effect after device reset.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BaudRate {
    B9600,
    B14400,
    B19200,
    B38400,
    B56000,
    B57600,
    B115200,
    B2400,
    B4800,
    /// Register read back a code outside the documented 0–8 range.
    Unknown(u16),
}

impl BaudRate {
    /// Encoded register value. `Unknown(c)` round-trips its raw code.
    pub const fn code(self) -> u16 {
        match self {
            Self::B9600 => 0,
            Self::B14400 => 1,
            Self::B19200 => 2,
            Self::B38400 => 3,
            Self::B56000 => 4,
            Self::B57600 => 5,
            Self::B115200 => 6,
            Self::B2400 => 7,
            Self::B4800 => 8,
            Self::Unknown(c) => c,
        }
    }
    pub const fn from_code(code: u16) -> Self {
        match code {
            0 => Self::B9600,
            1 => Self::B14400,
            2 => Self::B19200,
            3 => Self::B38400,
            4 => Self::B56000,
            5 => Self::B57600,
            6 => Self::B115200,
            7 => Self::B2400,
            8 => Self::B4800,
            c => Self::Unknown(c),
        }
    }
    /// Bits-per-second, or `None` for `Unknown`.
    pub const fn baud(self) -> Option<u32> {
        Some(match self {
            Self::B2400 => 2400,
            Self::B4800 => 4800,
            Self::B9600 => 9600,
            Self::B14400 => 14400,
            Self::B19200 => 19200,
            Self::B38400 => 38400,
            Self::B56000 => 56000,
            Self::B57600 => 57600,
            Self::B115200 => 115200,
            Self::Unknown(_) => return None,
        })
    }
}

// ─── Memory group parameters ─────────────────────────────────────────────────

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
    /// XY6020L/XY7025, 0.1 W on SK family) — encoded by [`Model::opp_scale`].
    pub s_opp_w: f32,
    /// Output-on time limit, hours.
    pub s_ohp_h: u16,
    /// Output-on time limit, minutes.
    pub s_ohp_m: u16,
    pub s_oah_low: u16,
    pub s_oah_high: u16,
    pub s_owh_low: u16,
    pub s_owh_high: u16,
    /// Over-temperature threshold (°C/°F per [`TempUnit`]).
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
