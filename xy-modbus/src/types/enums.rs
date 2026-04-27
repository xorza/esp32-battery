//! Wire-encoded status enums (regulation mode, temperature unit,
//! protection cause, baud-rate code).

use core::fmt;

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
