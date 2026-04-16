//! Typed JSON response for GET /api. Serialized via `serde_json_core`.
//!
//! Wire format preserves the prior hand-rolled shape so the frontend is unchanged.
//! History rows use a 5-tuple that serializes as `[t, v, c1, c2, online]`.

use serde::Serialize;

use esp32_battery_logic::data::Sample;

#[derive(Serialize)]
pub struct BatteryReading {
    pub soc: f32,
    pub current: f32,
    pub power: f32,
}

#[derive(Serialize)]
pub struct PsReading {
    pub current: f32,
    pub power: f32,
}

/// Serializes as a JSON array `[time_s, voltage, bat_current, ps_current, power_online]`.
#[derive(Serialize)]
pub struct HistoryRow(pub u32, pub f32, pub f32, pub f32, pub f32);

impl From<&Sample> for HistoryRow {
    fn from(s: &Sample) -> Self {
        Self(
            s.time_s,
            s.voltage,
            s.battery_current,
            s.ps_current,
            s.power_online,
        )
    }
}

#[derive(Serialize)]
pub struct ApiResponse {
    pub uptime: u32,
    pub rssi: i32,
    pub voltage: f32,
    pub read_err: [u32; 2],
    pub charge: f64,
    pub max_charge: f64,
    pub power_online: f32,
    pub battery: BatteryReading,
    pub ps: PsReading,
    pub history: Vec<HistoryRow>,
}

/// Worst-case serialized size. `ryu` produces shortest-round-trip f32 (typically 4–10 chars);
/// row = `[u32,f32,f32,f32,f32]` → ~50 chars. At HISTORY_CAPACITY=144 plus metadata, 8 KiB
/// leaves generous headroom.
pub const RESPONSE_BUF_SIZE: usize = 8192;
