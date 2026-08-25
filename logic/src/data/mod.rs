//! Sensor data store: live readings + history pipeline.
//!
//! `SensorData` is a thin orchestrator over two concerns: per-producer
//! staleness tracking (`LiveReadings`) and the adaptive-resolution history
//! ring (`history`).

mod history;

pub use history::HISTORY_CAPACITY;

use crate::charging::{FaultReason, InhibitReason, Phase};
use history::History;

#[derive(Clone, Copy, Default)]
pub struct Ina228Reading {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
}

/// Power-supply reading sourced from the XY7025 Modbus client (no charge register).
/// `v_set`/`i_set` are the programmed CV/CC targets (diagnostic — surfaces what
/// the buck is actually told to do vs. what it outputs).
#[derive(Clone, Copy, Default)]
pub struct PsReading {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
    pub v_set: f32,
    pub i_set: f32,
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

struct LiveReadings {
    latest_battery: Option<Ina228Reading>,
    latest_ps: Option<PsReading>,
    /// Ticks since the last `update_*`. Initialised to `u32::MAX` so a
    /// fresh `LiveReadings` treats both sensors as absent until the first
    /// live reading lands.
    battery_ticks_stale: u32,
    ps_ticks_stale: u32,
}

impl LiveReadings {
    fn new() -> Self {
        Self {
            latest_battery: None,
            latest_ps: None,
            battery_ticks_stale: u32::MAX,
            ps_ticks_stale: u32::MAX,
        }
    }

    fn update_battery(&mut self, bat: Ina228Reading) {
        self.latest_battery = Some(bat);
        self.battery_ticks_stale = 0;
    }

    fn update_ps(&mut self, ps: PsReading) {
        self.latest_ps = Some(ps);
        self.ps_ticks_stale = 0;
    }

    /// Age both staleness counters by one tick. Called once per supervisor
    /// tick before any reads.
    fn age(&mut self) {
        self.battery_ticks_stale = self.battery_ticks_stale.saturating_add(1);
        self.ps_ticks_stale = self.ps_ticks_stale.saturating_add(1);
    }

    fn battery(&self) -> Option<Ina228Reading> {
        if self.battery_ticks_stale > STALE_TICKS {
            return None;
        }
        self.latest_battery
    }

    fn ps(&self) -> Option<PsReading> {
        if self.ps_ticks_stale > STALE_TICKS {
            return None;
        }
        self.latest_ps
    }

    /// `1.0` when a fresh PS reading shows measurable voltage, `0.0` otherwise
    /// (including before the first reading and after PS goes stale).
    fn power_online(&self) -> f32 {
        match self.ps() {
            Some(ps) if ps.voltage > POWER_ONLINE_VOLTAGE_THRESHOLD => 1.0,
            _ => 0.0,
        }
    }
}

/// Central data store with adaptive-resolution history. Producer threads
/// publish via `update_*`; the supervisor's 1 Hz `tick` drives commits.
///
/// Wrapped in `Arc<Mutex<_>>` and shared across the INA/XY producers, the
/// supervisor, HTTP handlers, and the LCD task. All sites use
/// `.lock().unwrap()` deliberately — the panic hook in `src/main.rs`
/// reboots the device on any thread panic, so a poisoned mutex is
/// unreachable in practice. Don't switch these calls to `lock().ok()` or
/// poison-handling: the reboot path is the recovery, and silently
/// continuing past a poisoned lock would mask a real fault.
pub struct SensorData {
    live: LiveReadings,
    history: History,
    /// XY `MODEL` register (`0x0016`) read once at boot. `0` = not yet read.
    /// Diagnostic only — confirms the configured `Model`'s scale family.
    pub model_code: u16,
    /// `true` while the buck reports input UVLO (`ProtectionStatus::Lvp`) —
    /// the DC supply was disconnected or sagged. Set live each XY poll, so
    /// it self-clears when the supply returns. Surfaced to LCD/web as a
    /// benign "PS offline" status rather than a fault, since it recovers on
    /// its own without operator action.
    pub ps_offline: bool,
    /// Current charging phase, or `None` while the supervisor is still in
    /// Pending bring-up / latched off. Written by the XY supervisor each tick.
    pub charge_phase: Option<Phase>,
    /// Latched supervisor fault, if any. `None` during normal operation;
    /// `Some(reason)` once the buck has been latched off, and it stays set
    /// until a reboot. Conditions that recover on their own report through
    /// `charge_inhibit` instead and never reach this field.
    pub charge_fault: Option<FaultReason>,
    /// Why the supervisor is holding the buck off without having latched.
    /// `None` while regulating normally or once a fault has latched. Unlike
    /// `charge_fault` this self-clears, so it distinguishes "waiting for the
    /// input rail" from "the INA228 is dead" — both of which otherwise look
    /// like a dark output with no phase.
    pub charge_inhibit: Option<InhibitReason>,
}

impl Default for SensorData {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorData {
    pub fn new() -> Self {
        Self {
            live: LiveReadings::new(),
            history: History::new(),
            model_code: 0,
            ps_offline: false,
            charge_phase: None,
            charge_fault: None,
            charge_inhibit: None,
        }
    }

    /// Called by the INA thread once per cycle. Stores the latest reading
    /// and resets its staleness counter. Does NOT commit — the main-loop
    /// 1 Hz `tick` drives commits so one dead producer can't halt history.
    pub fn update_battery(&mut self, bat: Ina228Reading) {
        self.live.update_battery(bat);
    }

    /// Called by the XY thread once per poll.
    pub fn update_ps(&mut self, ps: PsReading) {
        self.live.update_ps(ps);
    }

    /// Latest battery reading, filtered by staleness. `None` before the
    /// first update, or once the staleness window has elapsed without a
    /// refresh.
    pub fn battery_reading(&self) -> Option<Ina228Reading> {
        self.live.battery()
    }

    pub fn ps_reading(&self) -> Option<PsReading> {
        self.live.ps()
    }

    /// `1.0` when the PS shows measurable voltage, `0.0` otherwise
    /// (including before the first reading and after PS goes stale).
    pub fn power_online(&self) -> f32 {
        self.live.power_online()
    }

    pub fn history(&self) -> &[Sample] {
        self.history.samples()
    }

    pub fn interval(&self) -> u32 {
        self.history.interval()
    }

    /// Drive the history pipeline forward by one tick. `now_epoch` is the
    /// wall-clock second the caller wants stamped on any committed sample;
    /// `None` (e.g. before NTP sync) gates out commits but still ages the
    /// staleness counters.
    ///
    /// Commits one raw sample per call using whichever readings are
    /// currently fresh — stale producers contribute zeros, so history stays
    /// continuous and the dead side surfaces as flat-line zero on the
    /// dashboard.
    pub fn tick(&mut self, now_epoch: Option<u32>) {
        self.live.age();

        let Some(time_s) = now_epoch else {
            return;
        };

        // Enforce monotonic time_s on the history. An NTP step-back (or a
        // clock correction on first sync) can hand us a time earlier than
        // the last committed sample — pushing it would break every
        // downstream consumer that assumes chronological order.
        if let Some(last) = self.history.last_time()
            && time_s <= last
        {
            log::warn!("skipping out-of-order commit: time_s={time_s} <= last={last}");
            return;
        }

        // Stale / absent producers yield zeroed readings. History stays
        // continuous; the zeros surface the dead producer to the dashboard
        // rather than silently freezing the timeline.
        let bat = self.live.battery().unwrap_or_default();
        let ps = self.live.ps().unwrap_or_default();
        let sample = Sample {
            time_s,
            voltage: bat.voltage,
            battery_current: bat.current,
            ps_current: ps.current,
            power_online: self.live.power_online(),
        };

        self.history.commit(sample);
    }
}

#[cfg(test)]
mod tests;
