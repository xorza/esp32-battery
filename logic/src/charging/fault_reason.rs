//! Why the supervisor latched the buck off.

use strum::IntoStaticStr;
use xy_modbus::ProtectionStatus;

/// Why the supervisor latched the buck off. Once latched, only a reboot
/// clears it — auto-recovery on a battery charger means trying again
/// under the same conditions. `OutputUnexpectedlyOff` carries the
/// device-reported PROTECT cause that was active when the buck
/// self-disabled (or `Normal` if no cause was set).
#[derive(Copy, Clone, Debug, PartialEq, Eq, IntoStaticStr)]
#[strum(serialize_all = "snake_case")]
pub enum FaultReason {
    /// No fresh battery reading for `BATTERY_MISSING_TIMEOUT.as_secs()` consecutive ticks.
    /// Without current/voltage we cannot supervise charging — fail closed.
    BatterySensorStale,
    /// Modbus reads to the XY7025 have been failing for `MODBUS_UNHEALTHY_TIMEOUT`
    /// continuously. We've lost closed-loop control over the buck; disable
    /// while we still can.
    ModbusUnhealthy,
    /// Pack voltage exceeded `absorb_v + OV_MARGIN_V` for `OV_DURATION.as_secs()` ticks.
    /// Catches drift below the XY's hardware OVP trip but above the profile target.
    Overvoltage,
    /// Pack held at the CV plateau (`absorb_v`) for `MAX_ABSORB.as_secs()`
    /// ticks without tapering out. Under a parasitic load pinning current
    /// above `exit_absorb_a` we'd otherwise sit at CV forever. The CC ramp
    /// up to `absorb_v` doesn't count — only time spent actually at CV.
    AbsorbTimeout,
    /// Pack spent `MAX_CHARGE.as_secs()` seconds continuously in Absorb
    /// without the taper ever ending the cycle. `AbsorbTimeout` clocks only
    /// time at the CV plateau, so a pack that never gets there — a shorted
    /// cell, a wiring fault, a load eating the whole charge current — would
    /// otherwise have no cap on it at all.
    ChargeTimeout,
    /// XY7025 setpoint readback (V_SET or I_SET) disagreed with what we
    /// commanded. The buck is sourcing under unknown setpoints — disable
    /// before it can do damage. Triggers immediately, no debounce: the
    /// caller already verified the read itself succeeded, so this isn't
    /// a transport glitch.
    SettingsDrift,
    /// Buck's OUTPUT_EN register read 0 while the supervisor was sourcing.
    /// The buck self-disabled — its own hardware OVP / OCP / over-temp
    /// tripped, or someone toggled the front panel (in which case PROTECT
    /// reads `Normal`). LVP/OTP are intercepted earlier and don't reach
    /// here. Payload is the cause from PROTECT (0x0010).
    OutputUnexpectedlyOff(ProtectionStatus),
    /// Buck's OUTPUT_EN register read 1 at cold boot — output is supposed
    /// to be off until the supervisor itself enables it. A hold reading the
    /// same thing is the recovery it is waiting for, not this.
    /// Means the boot disable / S_INI=0 didn't stick or the front panel
    /// toggled it on. We don't know what setpoints regulation is using;
    /// fail closed and reboot.
    OutputOnInPending,
}

impl std::fmt::Display for FaultReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BatterySensorStale => f.write_str("battery sensor stale"),
            Self::ModbusUnhealthy => f.write_str("modbus link unhealthy"),
            Self::Overvoltage => f.write_str("pack overvoltage"),
            Self::AbsorbTimeout => f.write_str("absorb time cap reached"),
            Self::ChargeTimeout => f.write_str("total charge time cap reached"),
            Self::SettingsDrift => f.write_str("setpoint readback drift"),
            Self::OutputUnexpectedlyOff(s) => write!(f, "buck self-disabled ({s})"),
            Self::OutputOnInPending => f.write_str("buck output on while supervisor pending"),
        }
    }
}

impl FaultReason {
    /// Stable snake_case identifier — what API consumers and dashboards
    /// match on. The `Display` impl is the human-readable form for logs.
    pub fn label(self) -> &'static str {
        self.into()
    }
}
