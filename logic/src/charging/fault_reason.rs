//! Why the supervisor stopped charging, and what it did about it.

use strum::IntoStaticStr;

use crate::charging::inhibit_reason::InhibitReason;
use xy_modbus::ProtectionStatus;

/// Why the supervisor stopped charging. Recovery is a reboot either way —
/// auto-recovery on a battery charger means trying again under the same
/// conditions — but what happens to the *output* differs: see
/// [`FaultReason::response`], which splits these into the ones that take
/// the buck down and the ones that only drop it to the float target.
/// `OutputUnexpectedlyOff` carries the device-reported PROTECT cause that
/// was active when the buck self-disabled (or `Normal` if no cause was
/// set).
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
    /// Pack held at the CV plateau (`absorb_v`) for a net `MAX_ABSORB`
    /// without tapering out. Under a parasitic load pinning current above
    /// `exit_absorb_a` we'd otherwise sit at CV forever. The CC ramp up to
    /// `absorb_v` doesn't count — only time spent actually at CV, and the
    /// window is leaky, so time spent off the plateau subtracts rather
    /// than erasing what came before.
    AbsorbTimeout,
    /// Pack spent `MAX_CHARGE.as_secs()` seconds continuously in Absorb
    /// without the taper ever ending the cycle. `AbsorbTimeout` clocks only
    /// time at the CV plateau, so a pack that never gets there — a shorted
    /// cell, a wiring fault, a load eating the whole charge current — would
    /// otherwise have no cap on it at all.
    ChargeTimeout,
    /// Buck dropped into a self-clearing protection more than `MAX_HOLDS`
    /// times with no `FLAP_WINDOW` of quiet between them. Each hold on its
    /// own is benign and gets waited out; a stream of them is a supply that
    /// cannot carry the charge current, and waiting forever means an output
    /// that blips at the flap rate for as long as the condition lasts.
    ///
    /// Latches even though the output is already off — the point is to stop
    /// bringing it back up.
    ProtectionFlapping,
    /// Pack was below `CHARGE_TEMP_MIN_C` while the buck was sourcing.
    /// Charging a frozen cell plates lithium; the damage is cumulative and
    /// invisible, so this refuses rather than warns.
    PackTooCold,
    /// Pack was above `CHARGE_TEMP_MAX_C` while the buck was sourcing.
    PackTooHot,
    /// A fitted pack-temperature sensor went unread for
    /// `PACK_TEMP_STALE_TIMEOUT`. Same rule as a dead INA228: we do not
    /// charge on a measurement we do not have.
    PackTempStale,
    /// Pack drew more than `OVERCURRENT_TOL ×` the profile's charge rate for
    /// `OVERCURRENT_DURATION`. The buck's own CC loop and OCP bound *total*
    /// output current, which includes the UPS load; this is the only check
    /// that sees what the pack itself is taking.
    ChargeOvercurrent,
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
            Self::ProtectionFlapping => f.write_str("buck protection flapping"),
            Self::PackTooCold => f.write_str("pack too cold to charge"),
            Self::PackTooHot => f.write_str("pack too hot to charge"),
            Self::PackTempStale => f.write_str("pack temperature sensor stale"),
            Self::ChargeOvercurrent => f.write_str("pack charge overcurrent"),
            Self::SettingsDrift => f.write_str("setpoint readback drift"),
            Self::OutputUnexpectedlyOff(s) => write!(f, "buck self-disabled ({s})"),
            Self::OutputOnInPending => f.write_str("buck output on while supervisor pending"),
        }
    }
}

/// What the supervisor does about a fault.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(super) enum FaultResponse {
    /// Output off, and off until a reboot.
    Disable,
    /// Drop to the float target and hold there, load still fed.
    Park,
}

impl FaultReason {
    /// What the supervisor does about this fault.
    ///
    /// Losing control of the buck means the only safe output is no output —
    /// we cannot supervise what we cannot see or command. A pack taking too
    /// much charge is a different problem: control is intact, and the
    /// hazard is the charging itself. Dropping to the float target stops it
    /// while the load stays fed, which on a UPS is the difference between
    /// "charging stopped" and "the pack drains until someone notices".
    ///
    /// `Overvoltage` is the deliberate borderline case, and it disables:
    /// parking means dropping V_SET and trusting the buck to hold it, on a
    /// buck we just caught regulating above the target we gave it.
    pub(super) fn response(self) -> FaultResponse {
        match self {
            Self::BatterySensorStale
            | Self::ModbusUnhealthy
            | Self::Overvoltage
            | Self::SettingsDrift
            | Self::ProtectionFlapping
            | Self::OutputUnexpectedlyOff(_)
            | Self::OutputOnInPending
            // Parking would not help: holding the float target still pushes
            // current into a discharged pack, and it is the charging itself
            // that damages a frozen cell. Only a dark output stops it.
            | Self::PackTooCold
            | Self::PackTooHot
            | Self::PackTempStale => FaultResponse::Disable,
            Self::AbsorbTimeout | Self::ChargeTimeout | Self::ChargeOvercurrent => {
                FaultResponse::Park
            }
        }
    }

    /// The waiting form of this fault: the same condition seen from a state
    /// where the output is already off, so latching would disable nothing
    /// while still costing a reboot to clear.
    ///
    /// `None` for the faults that can only arise while the buck is
    /// sourcing, which therefore never need one. Kept here rather than
    /// named at each check, so the pairing lives in one place and cannot
    /// drift per call site — a mismatched pair would compile and report the
    /// wrong reason.
    pub(super) fn inhibited(self) -> Option<InhibitReason> {
        Some(match self {
            Self::SettingsDrift => InhibitReason::SettingsDrift,
            Self::ModbusUnhealthy => InhibitReason::ModbusUnhealthy,
            Self::BatterySensorStale => InhibitReason::BatterySensorStale,
            Self::Overvoltage => InhibitReason::Overvoltage,
            Self::PackTooCold => InhibitReason::PackTooCold,
            Self::PackTooHot => InhibitReason::PackTooHot,
            Self::PackTempStale => InhibitReason::PackTempStale,
            Self::AbsorbTimeout
            | Self::ChargeTimeout
            | Self::ChargeOvercurrent
            | Self::ProtectionFlapping
            | Self::OutputUnexpectedlyOff(_)
            | Self::OutputOnInPending => return None,
        })
    }

    /// Stable snake_case identifier — what API consumers and dashboards
    /// match on. The `Display` impl is the human-readable form for logs.
    pub fn label(self) -> &'static str {
        self.into()
    }
}
