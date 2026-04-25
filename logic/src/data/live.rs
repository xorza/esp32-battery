//! Live-readings cache with per-producer staleness tracking.
//!
//! Producer threads publish via `update_*`; the supervisor's 1 Hz tick
//! calls `age` to advance the staleness counters. Readers go through
//! `battery` / `ps` which return `None` once a producer goes silent —
//! so a single dead sensor surfaces as missing data downstream rather
//! than a frozen last-known value.

use super::sample::{Ina228Reading, PsReading};

/// Ticks a sensor's reading can go unrefreshed before `tick` treats it as
/// absent. At 1 Hz ticks this is ~5 s — enough to ride out a single missed
/// poll, short enough that a stuck producer flips the dashboard / history
/// to its zero fallback before the user notices.
const STALE_TICKS: u32 = 5;

/// Minimum XY output voltage (V) to consider the PS "online". Uses voltage,
/// not current, so an enabled PSU with no load (fully-charged battery) still
/// registers as online. ~2 V covers noise/leakage while staying well below
/// any real rail.
const POWER_ONLINE_VOLTAGE_THRESHOLD: f32 = 2.0;

pub struct LiveReadings {
    latest_battery: Option<Ina228Reading>,
    latest_ps: Option<PsReading>,
    /// Ticks since the last `update_*`. Initialised to `u32::MAX` so a
    /// fresh `LiveReadings` treats both sensors as absent until the first
    /// live reading lands.
    battery_ticks_stale: u32,
    ps_ticks_stale: u32,
}

impl Default for LiveReadings {
    fn default() -> Self {
        Self::new()
    }
}

impl LiveReadings {
    pub fn new() -> Self {
        Self {
            latest_battery: None,
            latest_ps: None,
            battery_ticks_stale: u32::MAX,
            ps_ticks_stale: u32::MAX,
        }
    }

    /// Called by the INA thread once per cycle. Resets the battery
    /// staleness counter.
    pub fn update_battery(&mut self, bat: Ina228Reading) {
        self.latest_battery = Some(bat);
        self.battery_ticks_stale = 0;
    }

    /// Called by the XY thread once per poll.
    pub fn update_ps(&mut self, ps: PsReading) {
        self.latest_ps = Some(ps);
        self.ps_ticks_stale = 0;
    }

    /// Age both staleness counters by one tick. Called once per supervisor
    /// tick before any reads.
    pub fn age(&mut self) {
        self.battery_ticks_stale = self.battery_ticks_stale.saturating_add(1);
        self.ps_ticks_stale = self.ps_ticks_stale.saturating_add(1);
    }

    /// Latest battery reading, filtered by staleness. `None` before the
    /// first update, or once `STALE_TICKS` have passed without a refresh.
    pub fn battery(&self) -> Option<Ina228Reading> {
        if self.battery_ticks_stale > STALE_TICKS {
            return None;
        }
        self.latest_battery
    }

    pub fn ps(&self) -> Option<PsReading> {
        if self.ps_ticks_stale > STALE_TICKS {
            return None;
        }
        self.latest_ps
    }

    /// `1.0` when a fresh PS reading shows measurable voltage, `0.0` otherwise
    /// (including before the first reading and after PS goes stale).
    pub fn power_online(&self) -> f32 {
        match self.ps() {
            Some(ps) if ps.voltage > POWER_ONLINE_VOLTAGE_THRESHOLD => 1.0,
            _ => 0.0,
        }
    }
}
