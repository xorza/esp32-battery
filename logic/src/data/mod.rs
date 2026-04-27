//! Sensor data store: live readings + history pipeline.
//!
//! `SensorData` is a thin orchestrator over two concerns: per-producer
//! staleness tracking (`LiveReadings`) and the adaptive-resolution history
//! ring + on-flash codec (`history`). Persistence (NVS I/O, save scheduling)
//! lives in the firmware crate's main loop — `data` is pure model.

mod history;

pub use history::{HISTORY_CAPACITY, SERIALIZED_MAX_BYTES};

use history::History;

// --- Sample shapes ----------------------------------------------------------

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

// --- Live readings ----------------------------------------------------------

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

// --- SensorData orchestrator ------------------------------------------------

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

    /// Restore history from a previously-saved blob. Call at startup
    /// before the first `tick`. Returns `false` if the blob is malformed.
    pub fn load_from_bytes(&mut self, bytes: &[u8]) -> bool {
        if history::deserialize(bytes, &mut self.history) {
            log::info!("Loaded {} samples from blob", self.history.samples().len());
            true
        } else {
            log::warn!("Failed to parse history blob ({} bytes)", bytes.len());
            false
        }
    }

    /// Serialize history + metadata into the caller-provided buffer for
    /// NVS storage; returns the number of bytes written. `out` must hold
    /// at least `SERIALIZED_MAX_BYTES` to fit any valid history.
    pub fn serialize_into(&self, out: &mut [u8]) -> usize {
        history::serialize_into(&self.history, out)
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
mod tests {
    use super::*;

    fn bat_reading(voltage: f32, current: f32) -> Ina228Reading {
        Ina228Reading {
            voltage,
            current,
            power: voltage * current,
        }
    }

    fn ps_reading(voltage: f32, current: f32) -> PsReading {
        PsReading {
            voltage,
            current,
            power: voltage * current,
        }
    }

    /// Publish battery + PS readings and run one supervisor tick stamped with `now`.
    fn update(sd: &mut SensorData, bat: Ina228Reading, p: PsReading, now: u32) {
        sd.update_battery(bat);
        sd.update_ps(p);
        sd.tick(Some(now));
    }

    /// Push n uniform samples (v=13, c1=1, c2=2). Returns the next time_s value.
    fn fill(sd: &mut SensorData, n: u32, start_t: u32) -> u32 {
        for i in 0..n {
            update(
                sd,
                bat_reading(13.0, 1.0),
                ps_reading(13.0, 2.0),
                start_t + i,
            );
        }
        start_t + n
    }

    fn sd_with_blob(sd: &SensorData) -> SensorData {
        let mut buf = vec![0u8; super::SERIALIZED_MAX_BYTES];
        let n = sd.serialize_into(&mut buf);
        let mut fresh = SensorData::new();
        assert!(fresh.load_from_bytes(&buf[..n]));
        fresh
    }

    // --- Default / basic update ---

    #[test]
    fn default_is_empty() {
        let sd = SensorData::new();
        assert!(sd.history().is_empty());
        assert_eq!(sd.interval(), 1);
    }

    #[test]
    fn single_update() {
        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 100);

        assert_eq!(sd.history().len(), 1);
        let s = &sd.history()[0];
        assert_eq!(s.time_s, 100);
        assert!((s.voltage - 13.0).abs() < 0.001);
        assert!((s.battery_current - 1.0).abs() < 0.001);
        assert!((s.ps_current - 2.0).abs() < 0.001);
    }

    #[test]
    fn voltage_from_battery_only() {
        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(12.0, 1.0), ps_reading(14.0, 2.0), 1);
        assert!((sd.history()[0].voltage - 12.0).abs() < 0.001);
    }

    #[test]
    fn latest_readings_visible_after_commit() {
        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(13.0, 1.5), ps_reading(13.1, 2.5), 10);
        assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading().unwrap().current - 2.5).abs() < 0.001);
    }

    #[test]
    fn one_commit_per_tick_regardless_of_update_order() {
        let mut sd = SensorData::new();
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick(Some(1));
        assert_eq!(sd.history().len(), 1);

        sd.tick(Some(2));
        assert_eq!(sd.history().len(), 2);
    }

    #[test]
    fn history_returns_all_entries() {
        let mut sd = SensorData::new();
        for i in 0..10u32 {
            update(
                &mut sd,
                bat_reading(13.0, i as f32),
                ps_reading(13.0, 0.0),
                i,
            );
        }
        let h = sd.history();
        assert_eq!(h.len(), 10);
        for (i, s) in h.iter().enumerate() {
            assert_eq!(s.time_s, i as u32);
            assert!((s.battery_current - i as f32).abs() < 0.001);
        }
    }

    #[test]
    fn update_skipped_when_no_time() {
        let mut sd = SensorData::new();
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.5));
        sd.tick(None);
        assert!(sd.history().is_empty());
        assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading().unwrap().current - 2.0).abs() < 0.001);
    }

    #[test]
    fn multiple_updates_accumulate() {
        let mut sd = SensorData::new();
        fill(&mut sd, 10, 0);
        assert_eq!(sd.history().len(), 10);
    }

    // --- Power online tracking ---

    #[test]
    fn power_online_threshold() {
        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 0.0), 1);
        assert!((sd.history()[0].power_online - 1.0).abs() < 0.001);

        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(1.0, 2.0), 1);
        assert!(sd.history()[0].power_online.abs() < 0.001);

        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(0.0, 0.0), 1);
        assert!(sd.history()[0].power_online.abs() < 0.001);
    }

    #[test]
    fn power_online_averaged_during_compaction() {
        let mut sd = SensorData::new();
        for i in 0..(HISTORY_CAPACITY as u32 + 1) {
            let v = if i % 2 == 0 { 13.0 } else { 0.0 };
            update(&mut sd, bat_reading(13.0, 1.0), ps_reading(v, 1.0), i);
        }
        assert_eq!(sd.interval(), 2);
        for s in &sd.history()[..HISTORY_CAPACITY / 2] {
            assert!((s.power_online - 0.5).abs() < 0.01);
        }
    }

    #[test]
    fn power_online_roundtrips_through_persistence() {
        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 0);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(0.0, 0.0), 1);
        let mut buf = vec![0u8; super::SERIALIZED_MAX_BYTES];
        let n = sd.serialize_into(&mut buf);

        let mut sd2 = SensorData::new();
        assert!(sd2.load_from_bytes(&buf[..n]));
        assert!((sd2.history()[0].power_online - 1.0).abs() < 0.001);
        assert!(sd2.history()[1].power_online.abs() < 0.001);
    }

    // --- Ordering / no-NTP guards ---

    #[test]
    fn out_of_order_commits_are_rejected() {
        let mut sd = SensorData::new();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2000);
        assert_eq!(sd.history().len(), 1);

        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 1500);
        assert_eq!(
            sd.history().len(),
            1,
            "backward-jump sample must not be pushed"
        );

        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2000);
        assert_eq!(sd.history().len(), 1);

        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2001);
        assert_eq!(sd.history().len(), 2);
    }

    #[test]
    fn no_commit_before_ntp_sync() {
        let mut sd = SensorData::new();
        for _ in 0..100 {
            sd.update_ps(ps_reading(13.0, 2.0));
            sd.update_battery(bat_reading(13.0, 1.0));
            sd.tick(None);
        }
        assert!(sd.history().is_empty(), "no samples before NTP sync");
    }

    // --- Restore-from-blob ---

    #[test]
    fn loads_from_platform_on_first_update() {
        let mut sd = SensorData::new();
        for i in 0..10u32 {
            update(
                &mut sd,
                bat_reading(13.0, 1.0),
                ps_reading(13.0, 2.0),
                1000 + i,
            );
        }

        let mut sd2 = sd_with_blob(&sd);
        update(
            &mut sd2,
            bat_reading(14.0, 3.0),
            ps_reading(14.0, 4.0),
            1010,
        );

        assert_eq!(sd2.history().len(), 11);
        assert_eq!(sd2.interval(), 1);
        assert_eq!(sd2.history()[0].time_s, 1000);
        assert!((sd2.history()[0].battery_current - 1.0).abs() < 0.001);
        assert_eq!(sd2.history()[10].time_s, 1010);
        assert!((sd2.history()[10].voltage - 14.0).abs() < 0.001);
        assert!((sd2.history()[10].battery_current - 3.0).abs() < 0.001);
    }

    #[test]
    fn load_restores_old_blob_via_update() {
        let mut sd = SensorData::new();
        fill(&mut sd, 100, 1000);

        let mut sd2 = sd_with_blob(&sd);
        update(
            &mut sd2,
            bat_reading(13.0, 1.0),
            ps_reading(13.0, 2.0),
            5000,
        );

        assert_eq!(sd2.history().len(), 101);
        assert_eq!(sd2.history()[0].time_s, 1000);
        assert_eq!(sd2.history()[100].time_s, 5000);
    }

    #[test]
    fn load_rejects_corrupt_blob() {
        let mut sd = SensorData::new();
        assert!(!sd.load_from_bytes(&[0xFF; 10]));
        assert!(sd.history().is_empty());
    }

    // --- Producer-independence: a dead sensor must not halt history ---

    #[test]
    fn battery_only_still_commits_with_ps_zeros() {
        let mut sd = SensorData::new();
        for i in 0..10u32 {
            sd.update_battery(bat_reading(13.0, 1.5));
            sd.tick(Some(100 + i));
        }
        assert_eq!(sd.history().len(), 10);
        for s in sd.history() {
            assert!((s.battery_current - 1.5).abs() < 0.001);
            assert!(s.ps_current.abs() < 0.001);
            assert!(s.power_online.abs() < 0.001);
        }
    }

    #[test]
    fn ps_goes_stale_after_threshold() {
        // STALE_TICKS = 5 ticks of unrefreshed reading before the filter trips.
        const STALE: u32 = 5;
        let mut sd = SensorData::new();
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.5));
        sd.tick(Some(1000));
        assert_eq!(sd.history().len(), 1);
        assert_eq!(sd.history()[0].ps_current, 2.5);
        assert_eq!(sd.history()[0].power_online, 1.0);
        let ps0 = sd.ps_reading().expect("ps fresh after first update");
        assert_eq!(ps0.current, 2.5);
        assert_eq!(ps0.voltage, 13.0);

        for i in 1..STALE {
            sd.update_battery(bat_reading(13.0, 1.0));
            sd.tick(Some(1000 + i));
        }
        let ps_late = sd.ps_reading().expect("PS still fresh at STALE_TICKS");
        assert_eq!(ps_late.current, 2.5);

        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick(Some(1000 + STALE));
        assert!(sd.ps_reading().is_none());
        let latest = sd.history().last().unwrap();
        assert_eq!(latest.ps_current, 0.0);
        assert_eq!(latest.power_online, 0.0);
    }

    #[test]
    fn battery_stale_commits_zeros() {
        const STALE: u32 = 5;
        let mut sd = SensorData::new();
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.tick(Some(2000));
        assert_eq!(sd.history().len(), 1);
        assert_eq!(sd.history()[0].voltage, 13.0);

        let ticks = STALE + 3;
        for i in 1..=ticks {
            sd.update_ps(ps_reading(13.0, 2.0));
            sd.tick(Some(2000 + i));
        }
        assert_eq!(sd.history().len(), 1 + ticks as usize);
        assert!(sd.battery_reading().is_none());
        let latest = sd.history().last().unwrap();
        assert_eq!(latest.voltage, 0.0);
        assert_eq!(latest.battery_current, 0.0);
        assert_eq!(latest.ps_current, 2.0);
    }
}
