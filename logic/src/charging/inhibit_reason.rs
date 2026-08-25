//! Why the supervisor is holding the buck off without latching.

use xy_modbus::ProtectionStatus;

/// Why the supervisor is declining to energise the buck this tick,
/// with nothing latched. Every variant is self-clearing: the supervisor
/// re-checks each tick and brings the buck up as soon as the condition
/// lifts. Reported alongside `FaultReason` so a dashboard can tell
/// "waiting for the input rail" from "the INA228 has been dead for
/// eight seconds" — both of which look like a dark output otherwise.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InhibitReason {
    /// Setpoint readback disagrees with what we commanded. Regulating
    /// on unknown setpoints would latch; refusing to *start* on them
    /// only waits. Mirrors `FaultReason::SettingsDrift`.
    SettingsDrift,
    /// Modbus reads have been failing past `MODBUS_UNHEALTHY_TIMEOUT`,
    /// or no setpoint readback has landed yet this tick. Either way we
    /// have no closed-loop confirmation to energise on.
    ModbusUnhealthy,
    /// No fresh battery sample for `BATTERY_MISSING_TIMEOUT`.
    BatterySensorStale,
    /// A sample is simply absent this tick — not yet stale enough to
    /// count against the debounce.
    NoBatterySample,
    /// Pack sits above `absorb_v + OV_MARGIN_V`. Undebounced on purpose:
    /// one sample over the line is enough to refuse bring-up, where the
    /// same single sample is not enough to trip a regulating buck.
    Overvoltage,
    /// Buck is holding itself off on a self-clearing protection (input
    /// UVLO / over-temp). `set_output(true)` would succeed at the
    /// Modbus layer and change nothing, so we wait for the cause.
    BuckProtection(ProtectionStatus),
}

impl InhibitReason {
    /// Stable snake_case identifier, matching `FaultReason::label`.
    pub fn label(self) -> &'static str {
        match self {
            Self::SettingsDrift => "settings_drift",
            Self::ModbusUnhealthy => "modbus_unhealthy",
            Self::BatterySensorStale => "battery_sensor_stale",
            Self::NoBatterySample => "no_battery_sample",
            Self::Overvoltage => "overvoltage",
            Self::BuckProtection(_) => "buck_protection",
        }
    }
}

impl std::fmt::Display for InhibitReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SettingsDrift => f.write_str("waiting: setpoint readback drift"),
            Self::ModbusUnhealthy => f.write_str("waiting: modbus link unhealthy"),
            Self::BatterySensorStale => f.write_str("waiting: battery sensor stale"),
            Self::NoBatterySample => f.write_str("waiting: no battery sample"),
            Self::Overvoltage => f.write_str("waiting: pack overvoltage"),
            Self::BuckProtection(s) => write!(f, "waiting: buck protection ({s})"),
        }
    }
}
