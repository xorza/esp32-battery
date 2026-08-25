//! What the device knows about itself: the sensor reading store with its
//! history pipeline, and the supervisor's published status.
//!
//! The two live behind separate mutexes — see [`charge_status::ChargeStatus`]
//! for why.

pub(crate) mod charge_status;
pub(crate) mod history;

use std::time::Duration;

use history::History;

#[derive(Clone, Copy, Default, Debug)]
pub struct Ina228Reading {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
}

impl Ina228Reading {
    /// Every field is a real number. A sensor reporting NaN/Inf is not
    /// reporting a reading, and one that reaches the history accumulator
    /// poisons every average computed from it.
    fn is_finite(&self) -> bool {
        self.voltage.is_finite() && self.current.is_finite() && self.power.is_finite()
    }
}

/// Power-supply reading sourced from the XY7025 Modbus client (no charge register).
/// `v_set`/`i_set` are the programmed CV/CC targets (diagnostic — surfaces what
/// the buck is actually told to do vs. what it outputs).
#[derive(Clone, Copy, Default, Debug)]
pub struct PsReading {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
    pub v_set: f32,
    pub i_set: f32,
}

impl PsReading {
    /// See [`Ina228Reading::is_finite`].
    fn is_finite(&self) -> bool {
        self.voltage.is_finite()
            && self.current.is_finite()
            && self.power.is_finite()
            && self.v_set.is_finite()
            && self.i_set.is_finite()
    }
}

/// A single timestamped data point for charting (both sensors).
#[derive(Clone, Copy, Default, Debug)]
pub struct Sample {
    pub time_s: u32,
    pub voltage: f32,
    pub battery_current: f32,
    pub ps_current: f32,
    /// Fraction of the sample's span the supply was online: exactly `1.0` or
    /// `0.0` on a raw sample, anything between once compaction averages
    /// several together. The dashboard reads it as a duty cycle and reports
    /// the complement as an offline percentage.
    pub power_online: f32,
}

/// How long a sensor's reading may go unrefreshed before `tick` treats it as
/// absent. Long enough to ride out a single missed poll, short enough that a
/// stuck producer flips the dashboard / history to its zero fallback before
/// the user notices. Charged in wall time, not ticks, so a main loop that
/// stalls (a slow association attempt, say) does not silently extend it.
pub(crate) const STALE_WINDOW: Duration = Duration::from_secs(5);

/// Minimum XY output voltage (V) to consider the PS "online". Uses voltage,
/// not current, so an enabled PSU with no load (fully-charged battery) still
/// registers as online. ~2 V covers noise/leakage while staying well below
/// any real rail.
const POWER_ONLINE_VOLTAGE_THRESHOLD: f32 = 2.0;

/// Latest reading per producer plus the adaptive-resolution history built
/// from them. Producer threads publish via `update_*`; the main loop's 1 Hz
/// `tick` charges the staleness clocks and drives history commits.
///
/// Wrapped in `Arc<Mutex<_>>` and shared across the INA/XY producers, the
/// main loop, HTTP handlers, and the LCD task. All sites use
/// `.lock().unwrap()` deliberately — the panic hook in `src/main.rs`
/// reboots the device on any thread panic, so a poisoned mutex is
/// unreachable in practice. Don't switch these calls to `lock().ok()` or
/// poison-handling: the reboot path is the recovery, and silently
/// continuing past a poisoned lock would mask a real fault.
#[derive(Debug)]
pub struct SensorData {
    latest_battery: Option<Ina228Reading>,
    latest_ps: Option<PsReading>,
    /// Time since the last accepted `update_*`. Initialised to
    /// `Duration::MAX` so a fresh store treats both sensors as absent until
    /// the first live reading lands.
    battery_stale: Duration,
    ps_stale: Duration,
    history: History,
}

impl Default for SensorData {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorData {
    pub fn new() -> Self {
        Self {
            latest_battery: None,
            latest_ps: None,
            battery_stale: Duration::MAX,
            ps_stale: Duration::MAX,
            history: History::new(),
        }
    }

    /// Called by the INA thread once per cycle. Stores the latest reading
    /// and resets its staleness counter. Does NOT commit — the main-loop
    /// 1 Hz `tick` drives commits so one dead producer can't halt history.
    pub fn update_battery(&mut self, bat: Ina228Reading) {
        if !bat.is_finite() {
            log::warn!("dropping non-finite battery reading");
            return;
        }
        self.latest_battery = Some(bat);
        self.battery_stale = Duration::ZERO;
    }

    /// Called by the XY thread once per poll.
    pub fn update_ps(&mut self, ps: PsReading) {
        if !ps.is_finite() {
            log::warn!("dropping non-finite PS reading");
            return;
        }
        self.latest_ps = Some(ps);
        self.ps_stale = Duration::ZERO;
    }

    /// Latest battery reading, filtered by staleness. `None` before the
    /// first update, or once [`STALE_WINDOW`] has elapsed without a refresh.
    pub fn battery_reading(&self) -> Option<Ina228Reading> {
        if self.battery_stale > STALE_WINDOW {
            return None;
        }
        self.latest_battery
    }

    pub fn ps_reading(&self) -> Option<PsReading> {
        if self.ps_stale > STALE_WINDOW {
            return None;
        }
        self.latest_ps
    }

    /// `1.0` when a fresh PS reading shows measurable voltage, `0.0`
    /// otherwise (including before the first reading and after PS goes
    /// stale). Only ever stamped onto a raw [`Sample`]; the fractional
    /// values on the wire come from compaction averaging these.
    fn power_online(&self) -> f32 {
        match self.ps_reading() {
            Some(ps) if ps.voltage > POWER_ONLINE_VOLTAGE_THRESHOLD => 1.0,
            _ => 0.0,
        }
    }

    pub fn history(&self) -> &[Sample] {
        self.history.samples()
    }

    /// Drive the history pipeline forward by one tick. `elapsed` is the wall
    /// time since the previous call — the staleness clocks charge that, not a
    /// tick count, so a caller whose loop stalls does not silently widen
    /// [`STALE_WINDOW`]. `now_epoch` is the wall-clock second to stamp on any
    /// committed sample; `None` (e.g. before NTP sync) gates out commits but
    /// still ages the staleness clocks.
    ///
    /// Commits one raw sample per call using whichever readings are
    /// currently fresh — stale producers contribute zeros, so history stays
    /// continuous and the dead side surfaces as flat-line zero on the
    /// dashboard.
    pub fn tick(&mut self, now_epoch: Option<u32>, elapsed: Duration) {
        self.battery_stale = self.battery_stale.saturating_add(elapsed);
        self.ps_stale = self.ps_stale.saturating_add(elapsed);

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
        let bat = self.battery_reading().unwrap_or_default();
        let ps = self.ps_reading().unwrap_or_default();
        let sample = Sample {
            time_s,
            voltage: bat.voltage,
            battery_current: bat.current,
            ps_current: ps.current,
            power_online: self.power_online(),
        };

        self.history.commit(sample);
    }
}

#[cfg(test)]
mod tests;
