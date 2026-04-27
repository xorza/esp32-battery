//! Hardware variant presets and per-model register scales.

/// Hardware variant. Selected at construction (`Xy::new`) and used to
/// scale the registers whose resolution differs across the family —
/// I-SET, IOUT, S-OCP, POWER, S-OPP. See `DATASHEET.md` §3 for the
/// scale table.
///
/// Cross-check by reading `MODEL` (`0x0016`): `0x6100`-class is
/// XY6020L / XY7025; SK-family codes differ. The crate does not probe
/// automatically — pick the variant that matches your hardware.
#[derive(Copy, Clone, Debug, PartialEq)]
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
