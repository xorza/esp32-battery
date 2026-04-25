//! Sample shapes shared across the live cache, the history pipeline,
//! and the persistence codec.

#[derive(Clone, Copy, Default)]
pub struct Ina228Reading {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
}

/// Power-supply reading sourced from the XY7025 Modbus client (no charge register).
#[derive(Clone, Copy, Default)]
pub struct PsReading {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
}

/// A single timestamped data point for charting (both sensors).
#[derive(Clone, Copy, Default)]
pub struct Sample {
    pub time_s: u32,
    pub voltage: f32,
    pub battery_current: f32,
    pub ps_current: f32,
    /// 1.0 when power supply is online, 0.0 when offline. Averaged during compaction.
    pub power_online: f32,
}
