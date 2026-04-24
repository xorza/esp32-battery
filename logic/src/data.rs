const FORMAT_VERSION: u32 = 6;
const HEADER_SIZE: usize = 4 + 4 + 4; // version + interval + count
const SAMPLE_SIZE: usize = 4 + 4 * 4; // u32 + 4×f32 = 20 bytes
/// Minimum XY output voltage (V) to consider the PS "online". Uses voltage,
/// not current, so an enabled PSU with no load (fully-charged battery) still
/// registers as online. ~2 V covers noise/leakage while staying well below
/// any real rail.
const POWER_ONLINE_VOLTAGE_THRESHOLD: f32 = 2.0;
/// Upper bound on the serialized history blob — also the in-memory scratch size.
pub const SERIALIZED_MAX_BYTES: usize = 4096;
/// Max samples that fit in SERIALIZED_MAX_BYTES. 204 × 1024s ≈ 58 hours of history.
pub const HISTORY_CAPACITY: usize = (SERIALIZED_MAX_BYTES - HEADER_SIZE) / SAMPLE_SIZE / 2 * 2;
// Once interval reaches this, drop old samples instead of compacting further.
// 204 samples × 1024s ≈ 58h (covers 24h with margin).
const MAX_INTERVAL: u32 = 1024;
const _: () = assert!(
    HISTORY_CAPACITY.is_multiple_of(2),
    "HISTORY_CAPACITY must be even"
);
const _: () = assert!(
    HEADER_SIZE + HISTORY_CAPACITY * SAMPLE_SIZE <= SERIALIZED_MAX_BYTES,
    "serialized history must fit in SERIALIZED_MAX_BYTES"
);
const SAVE_INTERVAL_S: u32 = 600;

/// Source of wall-clock time. Separated from persistence so callers can drive
/// NVS I/O outside the `SensorData` mutex.
pub trait Clock {
    fn epoch_s(&self) -> Option<u32>;
}

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

#[derive(Default)]
struct SampleAccum {
    voltage: f32,
    battery_current: f32,
    ps_current: f32,
    power_online: f32,
}

impl SampleAccum {
    fn add(&mut self, s: &Sample) {
        self.voltage += s.voltage;
        self.battery_current += s.battery_current;
        self.ps_current += s.ps_current;
        self.power_online += s.power_online;
    }

    fn average(&self, n: u32, time_s: u32) -> Sample {
        assert!(n > 0, "cannot average zero samples");
        let n = n as f32;
        Sample {
            time_s,
            voltage: self.voltage / n,
            battery_current: self.battery_current / n,
            ps_current: self.ps_current / n,
            power_online: self.power_online / n,
        }
    }
}

/// Ticks a sensor's reading can go unrefreshed before `tick` treats it as
/// absent. At 1 Hz ticks this is ~5 s — enough to ride out a single missed
/// poll, short enough that a stuck producer flips the dashboard / history
/// to its zero fallback before the user notices.
const STALE_TICKS: u32 = 5;

/// Central data store with adaptive-resolution history.
///
/// Stores up to `HISTORY_CAPACITY` samples. When full, pairs are averaged
/// (halving the count) and the sampling interval doubles. This gives
/// exponentially growing time coverage in fixed memory (~4 KB).
pub struct SensorData<C: Clock> {
    /// Latest readings published by producer threads. Raw fields — readers
    /// should go through `battery_reading()` / `ps_reading()` which apply
    /// the staleness filter. Kept private so the filter can't be bypassed.
    latest_battery: Option<Ina228Reading>,
    latest_ps: Option<PsReading>,
    /// Ticks since the last `update_*`. Initialised to `u32::MAX` so a
    /// fresh or just-loaded `SensorData` treats both sensors as absent
    /// until the first live reading lands.
    battery_ticks_stale: u32,
    ps_ticks_stale: u32,
    history: heapless::Vec<Sample, HISTORY_CAPACITY>,
    /// Current sampling interval: how many ticks per stored sample.
    interval: u32,
    acc: SampleAccum,
    acc_count: u32,
    clock: C,
    /// Has the save-interval elapsed since the last `take_save_payload` call?
    /// Set by `tick`; cleared when the caller drains the payload.
    save_pending: bool,
    /// `None` until the first successful commit anchors it. Non-anchoring
    /// ensures we don't immediately re-save a just-loaded blob.
    last_save_s: Option<u32>,
}

struct BufReader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl BufReader<'_> {
    fn u32(&mut self) -> u32 {
        let v = u32::from_le_bytes(self.buf[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        v
    }
    fn f32(&mut self) -> f32 {
        let v = f32::from_le_bytes(self.buf[self.pos..self.pos + 4].try_into().unwrap());
        self.pos += 4;
        v
    }
    fn sample(&mut self) -> Sample {
        Sample {
            time_s: self.u32(),
            voltage: self.f32(),
            battery_current: self.f32(),
            ps_current: self.f32(),
            power_online: self.f32(),
        }
    }
}

impl<C: Clock> SensorData<C> {
    pub fn new(clock: C) -> Self {
        Self {
            latest_battery: None,
            latest_ps: None,
            battery_ticks_stale: u32::MAX,
            ps_ticks_stale: u32::MAX,
            history: heapless::Vec::new(),
            interval: 1,
            acc: SampleAccum::default(),
            acc_count: 0,
            clock,
            save_pending: false,
            last_save_s: None,
        }
    }

    /// Called by the INA thread once per cycle. Stores the latest reading
    /// and resets its staleness counter. Does NOT commit — the main-loop
    /// 1 Hz `tick` drives commits so one dead producer can't halt history.
    pub fn update_battery(&mut self, bat: Ina228Reading) {
        self.latest_battery = Some(bat);
        self.battery_ticks_stale = 0;
    }

    /// Called by the XY thread once per poll. Same shape as `update_battery`.
    pub fn update_ps(&mut self, ps: PsReading) {
        self.latest_ps = Some(ps);
        self.ps_ticks_stale = 0;
    }

    /// Latest battery reading, filtered by staleness. `None` before the
    /// first update, or once `STALE_TICKS` have passed without a refresh.
    pub fn battery_reading(&self) -> Option<Ina228Reading> {
        if self.battery_ticks_stale > STALE_TICKS {
            return None;
        }
        self.latest_battery
    }

    pub fn ps_reading(&self) -> Option<PsReading> {
        if self.ps_ticks_stale > STALE_TICKS {
            return None;
        }
        self.latest_ps
    }

    /// `1.0` when a fresh PS reading shows measurable voltage, `0.0` otherwise
    /// (including before the first reading and after PS goes stale).
    pub fn power_online(&self) -> f32 {
        match self.ps_reading() {
            Some(ps) if ps.voltage > POWER_ONLINE_VOLTAGE_THRESHOLD => 1.0,
            _ => 0.0,
        }
    }

    /// Restore history from a previously-saved blob. Call at startup before
    /// the first `tick`. Returns false if the blob is malformed.
    pub fn load_from_bytes(&mut self, bytes: &[u8]) -> bool {
        let ok = self.deserialize(bytes);
        if ok {
            log::info!("Loaded {} samples from blob", self.history.len());
        } else {
            log::warn!("Failed to parse history blob ({} bytes)", bytes.len());
        }
        ok
    }

    /// Drain the pending save payload, if any. Returns `Some(bytes)` when
    /// `SAVE_INTERVAL_S` has elapsed since the previous drain. Caller performs
    /// the actual NVS write outside the `SensorData` lock.
    pub fn take_save_payload(&mut self) -> Option<Vec<u8>> {
        if !self.save_pending {
            return None;
        }
        self.save_pending = false;
        let out = self.serialize();
        log::info!(
            "Emitting save payload: {} samples ({} bytes)",
            self.history.len(),
            out.len()
        );
        Some(out)
    }

    /// Drive the history pipeline forward by one tick. Called once per
    /// second by the main-loop supervisor. Commits one raw sample per call
    /// using whichever readings are currently fresh — stale producers
    /// contribute zeros, so history stays continuous and the dead side
    /// surfaces as flat-line zero on the dashboard.
    ///
    /// Replaces the old "commit when both producers tick" rendezvous, which
    /// coupled history liveness to the least-available sensor. Sets
    /// `save_pending` when a save-interval has elapsed; the caller drains
    /// via `take_save_payload` to keep NVS I/O out of the data mutex.
    pub fn tick(&mut self) {
        self.battery_ticks_stale = self.battery_ticks_stale.saturating_add(1);
        self.ps_ticks_stale = self.ps_ticks_stale.saturating_add(1);

        let Some(time_s) = self.clock.epoch_s() else {
            return;
        };

        // First commit anchors the save timer so we don't immediately dump a
        // just-loaded blob back to flash.
        if self.last_save_s.is_none() {
            self.last_save_s = Some(time_s);
        }

        // Enforce monotonic time_s on the history. An NTP step-back (or a clock
        // correction on first sync) can hand us a time earlier than the last
        // committed sample — pushing it would break every downstream consumer
        // that assumes chronological order.
        if let Some(last) = self.history.last()
            && time_s <= last.time_s
        {
            log::warn!(
                "skipping out-of-order commit: time_s={time_s} <= last={}",
                last.time_s
            );
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

        self.acc.add(&sample);
        self.acc_count += 1;

        if self.acc_count >= self.interval {
            let averaged = self.acc.average(self.acc_count, time_s);
            self.acc = SampleAccum::default();
            self.acc_count = 0;

            self.compact_if_needed();
            assert!(self.history.push(averaged).is_ok(), "history overflow");
        }

        if let Some(last) = self.last_save_s
            && time_s.saturating_sub(last) >= SAVE_INTERVAL_S
        {
            self.last_save_s = Some(time_s);
            self.save_pending = true;
        }
    }

    fn compact_if_needed(&mut self) {
        if self.history.len() < HISTORY_CAPACITY {
            return;
        }
        if self.interval >= MAX_INTERVAL {
            // At max interval (~41h of history) — drop oldest sample to make room.
            self.history.remove(0);
            return;
        }
        let len = self.history.len();
        let half = len / 2;
        for i in 0..half {
            let a = self.history[2 * i];
            let b = self.history[2 * i + 1];
            self.history[i] = Sample {
                time_s: b.time_s,
                voltage: (a.voltage + b.voltage) / 2.0,
                battery_current: (a.battery_current + b.battery_current) / 2.0,
                ps_current: (a.ps_current + b.ps_current) / 2.0,
                power_online: (a.power_online + b.power_online) / 2.0,
            };
        }
        self.history.truncate(half);
        self.interval *= 2;
    }

    /// Borrow the history buffer. Always at most `HISTORY_CAPACITY` entries.
    pub fn history(&self) -> &[Sample] {
        &self.history
    }

    /// Current sampling interval: how many raw updates per stored sample.
    /// Starts at 1, doubles on each compaction.
    pub fn interval(&self) -> u32 {
        self.interval
    }

    /// Serialize history + metadata into a fresh `Vec`.
    fn serialize(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER_SIZE + self.history.len() * SAMPLE_SIZE);
        out.extend_from_slice(&FORMAT_VERSION.to_le_bytes());
        out.extend_from_slice(&self.interval.to_le_bytes());
        out.extend_from_slice(&(self.history.len() as u32).to_le_bytes());
        for s in &self.history {
            out.extend_from_slice(&s.time_s.to_le_bytes());
            out.extend_from_slice(&s.voltage.to_le_bytes());
            out.extend_from_slice(&s.battery_current.to_le_bytes());
            out.extend_from_slice(&s.ps_current.to_le_bytes());
            out.extend_from_slice(&s.power_online.to_le_bytes());
        }
        out
    }

    /// Restore history from a byte slice. Returns false on malformed input.
    fn deserialize(&mut self, bytes: &[u8]) -> bool {
        if bytes.len() < HEADER_SIZE {
            return false;
        }

        let mut r = BufReader { buf: bytes, pos: 0 };
        let version = r.u32();
        if version != FORMAT_VERSION {
            return false;
        }
        let interval = r.u32();
        let count = r.u32() as usize;

        if interval == 0 || count == 0 || bytes.len() < HEADER_SIZE + count * SAMPLE_SIZE {
            return false;
        }

        self.history.clear();
        let skip = count.saturating_sub(HISTORY_CAPACITY);
        for i in 0..count {
            let sample = r.sample();
            if i >= skip {
                assert!(self.history.push(sample).is_ok(), "history overflow");
            }
        }

        self.interval = interval;
        self.acc = SampleAccum::default();
        self.acc_count = 0;

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use std::rc::Rc;

    const CAP: usize = HISTORY_CAPACITY;
    const HALF: usize = CAP / 2;

    type TestClock = Rc<Cell<u32>>;

    /// `Clock` impl over `Rc<Cell<u32>>` so tests can tick time deterministically.
    struct TestClockSrc(TestClock);
    impl Clock for TestClockSrc {
        fn epoch_s(&self) -> Option<u32> {
            Some(self.0.get())
        }
    }

    fn new_sd() -> (TestClock, SensorData<TestClockSrc>) {
        let time = Rc::new(Cell::new(0u32));
        (time.clone(), SensorData::new(TestClockSrc(time)))
    }

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

    /// Shorthand: publish battery + PS readings and run one supervisor tick.
    fn update(sd: &mut SensorData<TestClockSrc>, bat: Ina228Reading, p: PsReading) {
        sd.update_battery(bat);
        sd.update_ps(p);
        sd.tick();
    }

    /// Push n uniform samples (v=13, c1=1, c2=2). Returns the next time_s value.
    fn fill(sd: &mut SensorData<TestClockSrc>, n: u32, start_t: u32, time: &TestClock) -> u32 {
        for i in 0..n {
            time.set(start_t + i);
            update(sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        }
        start_t + n
    }

    // --- Default / basic update ---

    #[test]
    fn default_is_empty() {
        let (_time, sd) = new_sd();
        assert!(sd.history.is_empty());
        assert_eq!(sd.interval, 1);
    }

    #[test]
    fn single_update() {
        let (time, mut sd) = new_sd();
        time.set(100);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));

        assert_eq!(sd.history.len(), 1);
        let s = &sd.history[0];
        assert_eq!(s.time_s, 100);
        // voltage = avg(13.0, 13.0) = 13.0
        assert!((s.voltage - 13.0).abs() < 0.001);
        assert!((s.battery_current - 1.0).abs() < 0.001);
        assert!((s.ps_current - 2.0).abs() < 0.001);
    }

    #[test]
    fn voltage_from_battery_only() {
        let (_time, mut sd) = new_sd();
        // Voltage now comes solely from the battery INA — PS voltage is ignored.
        update(&mut sd, bat_reading(12.0, 1.0), ps_reading(14.0, 2.0));
        assert!((sd.history[0].voltage - 12.0).abs() < 0.001);
    }

    #[test]
    fn latest_readings_visible_after_commit() {
        // Readings stay visible via the getters after a tick so HTTP/LCD can
        // always snapshot live values.
        let (time, mut sd) = new_sd();
        time.set(10);
        update(&mut sd, bat_reading(13.0, 1.5), ps_reading(13.1, 2.5));
        assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading().unwrap().current - 2.5).abs() < 0.001);
    }

    #[test]
    fn one_commit_per_tick_regardless_of_update_order() {
        // tick-driven commits: one tick → at most one history row, regardless
        // of how many update_* calls landed in between.
        let (time, mut sd) = new_sd();
        time.set(1);
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick();
        assert_eq!(sd.history.len(), 1);

        // Next tick produces exactly one more row — whatever the latest
        // readings are — not a backlog of the earlier updates.
        time.set(2);
        sd.tick();
        assert_eq!(sd.history.len(), 2);
    }

    #[test]
    fn history_returns_all_entries() {
        let (time, mut sd) = new_sd();
        for i in 0..10u32 {
            time.set(i);
            update(&mut sd, bat_reading(13.0, i as f32), ps_reading(13.0, 0.0));
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
        struct NoClock;
        impl Clock for NoClock {
            fn epoch_s(&self) -> Option<u32> {
                None
            }
        }
        let mut sd = SensorData::new(NoClock);
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.5));
        sd.tick();
        assert!(sd.history.is_empty());
        assert!(sd.last_save_s.is_none());
        // Latest readings must still be visible to HTTP/LCD before NTP sync.
        assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading().unwrap().current - 2.0).abs() < 0.001);
    }

    #[test]
    fn multiple_updates_accumulate() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 10, 0, &time);
        assert_eq!(sd.history.len(), 10);
    }

    // --- Power online tracking ---

    #[test]
    fn power_online_threshold() {
        // Online when PS voltage is above the threshold (load-independent).
        let (_time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 0.0));
        assert!((sd.history[0].power_online - 1.0).abs() < 0.001);

        // Below threshold: voltage=1 → offline.
        let (_time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(1.0, 2.0));
        assert!(sd.history[0].power_online.abs() < 0.001);

        // Exactly zero voltage: → offline.
        let (_time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(0.0, 0.0));
        assert!(sd.history[0].power_online.abs() < 0.001);
    }

    #[test]
    fn power_online_averaged_during_compaction() {
        let (time, mut sd) = new_sd();
        // Alternating online (v=13) / offline (v=0).
        for i in 0..(CAP as u32 + 1) {
            time.set(i);
            let v = if i % 2 == 0 { 13.0 } else { 0.0 };
            update(&mut sd, bat_reading(13.0, 1.0), ps_reading(v, 1.0));
        }
        assert_eq!(sd.interval, 2);
        // Each compacted pair averages one online (1.0) and one offline (0.0) → 0.5
        for s in &sd.history[..HALF] {
            assert!((s.power_online - 0.5).abs() < 0.01);
        }
    }

    #[test]
    fn power_online_roundtrips_through_persistence() {
        let (time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0)); // online
        time.set(1);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(0.0, 0.0)); // offline
        let blob = sd.serialize();

        let (_time2, mut sd2) = new_sd();
        assert!(sd2.deserialize(&blob));
        assert!((sd2.history[0].power_online - 1.0).abs() < 0.001);
        assert!(sd2.history[1].power_online.abs() < 0.001);
    }

    // --- Compaction ---

    #[test]
    fn no_compaction_below_capacity() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, CAP as u32 - 1, 0, &time);
        assert_eq!(sd.history.len(), CAP - 1);
        assert_eq!(sd.interval, 1);
    }

    #[test]
    fn compaction_at_capacity() {
        // CAP+1 updates: fills CAP, compacts to HALF, pushes 1 → HALF+1, interval=2.
        let (time, mut sd) = new_sd();
        fill(&mut sd, CAP as u32 + 1, 0, &time);
        assert_eq!(sd.history.len(), HALF + 1);
        assert_eq!(sd.interval, 2);
    }

    #[test]
    fn compaction_averages_all_fields_and_uses_later_timestamp() {
        let (time, mut sd) = new_sd();
        // Alternating two value sets for CAP+1 updates (triggers compaction).
        for i in 0..(CAP as u32 + 1) {
            time.set(i * 10);
            if i % 2 == 0 {
                update(&mut sd, bat_reading(12.0, 1.0), ps_reading(12.0, 2.0));
            } else {
                update(&mut sd, bat_reading(14.0, 3.0), ps_reading(14.0, 4.0));
            }
        }
        assert_eq!(sd.history.len(), HALF + 1);
        assert_eq!(sd.interval, 2);

        // Each compacted pair averages: voltage=(12+14)/2=13, c1=(1+3)/2=2, c2=(2+4)/2=3
        for s in &sd.history[..HALF] {
            assert!((s.voltage - 13.0).abs() < 0.01);
            assert!((s.battery_current - 2.0).abs() < 0.01);
            assert!((s.ps_current - 3.0).abs() < 0.01);
        }
        // Compaction keeps the later timestamp of each pair.
        assert_eq!(sd.history[0].time_s, 10); // pair t=0,t=10 → keeps t=10
        assert_eq!(sd.history[1].time_s, 30); // pair t=20,t=30 → keeps t=30
        assert_eq!(sd.history[HALF - 1].time_s, (CAP as u32 - 1) * 10);
    }

    #[test]
    fn after_compaction_samples_at_new_interval() {
        let (time, mut sd) = new_sd();
        // CAP+1 fills → compacts to HALF, pushes 1 = HALF+1 at interval=2
        let t = fill(&mut sd, CAP as u32 + 1, 0, &time);
        assert_eq!(sd.interval, 2);

        // Next raw update: acc_count=1, not yet interval=2
        time.set(t);
        update(&mut sd, bat_reading(13.0, 5.0), ps_reading(13.0, 0.0));
        assert_eq!(sd.history.len(), HALF + 1); // not yet pushed

        // Second raw update: acc_count=2 >= interval=2, push averaged
        time.set(t + 1);
        update(&mut sd, bat_reading(13.0, 7.0), ps_reading(13.0, 0.0));
        assert_eq!(sd.history.len(), HALF + 2);
        // Averaged: battery_current = (5+7)/2 = 6
        let last = sd.history.last().unwrap();
        assert!((last.battery_current - 6.0).abs() < 0.01);
        assert_eq!(last.time_s, t + 1);
    }

    #[test]
    fn interval_doubles_each_compaction() {
        let (time, mut sd) = new_sd();
        assert_eq!(sd.interval, 1);

        // CAP=204: Compaction 1 at 205 raw updates (interval 1→2).
        // Compaction 2 at 205+204=409 raw updates (interval 2→4).
        // Compaction 3 at 409+408=817 raw updates (interval 4→8).
        fill(&mut sd, 820, 0, &time);
        assert_eq!(sd.interval, 8);
    }

    #[test]
    fn long_run_stays_bounded_and_chronological() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 10000, 0, &time);
        assert!(sd.history.len() <= CAP);
        assert!(sd.history.len() >= HALF);
        for i in 1..sd.history.len() {
            assert!(
                sd.history[i].time_s >= sd.history[i - 1].time_s,
                "not chronological at {}: {} < {}",
                i,
                sd.history[i].time_s,
                sd.history[i - 1].time_s
            );
        }
    }

    // --- Persistence ---

    #[test]
    fn write_read_roundtrip_empty() {
        let (_time, sd) = new_sd();
        let blob = sd.serialize();
        assert_eq!(blob.len(), HEADER_SIZE);

        let (_time2, mut sd2) = new_sd();
        // Empty blob (count=0) is rejected — history must have at least one sample.
        assert!(!sd2.deserialize(&blob));
    }

    #[test]
    fn write_read_roundtrip() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 10, 1000, &time);
        let blob = sd.serialize();
        assert_eq!(blob.len(), HEADER_SIZE + 10 * SAMPLE_SIZE);

        let (_time2, mut sd2) = new_sd();
        assert!(sd2.deserialize(&blob));
        assert_eq!(sd2.history.len(), 10);
        assert_eq!(sd2.interval, 1);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert_eq!(sd2.history[9].time_s, 1009);
    }

    /// Hand-build a header with the given version/interval/count and return the
    /// padded blob of `total_len` bytes (remaining space zeroed).
    fn header_blob(version: u32, interval: u32, count: u32, total_len: usize) -> Vec<u8> {
        let mut out = vec![0u8; total_len];
        out[0..4].copy_from_slice(&version.to_le_bytes());
        out[4..8].copy_from_slice(&interval.to_le_bytes());
        out[8..12].copy_from_slice(&count.to_le_bytes());
        out
    }

    #[test]
    fn read_rejects_truncated() {
        let (_time, mut sd) = new_sd();
        assert!(!sd.deserialize(&[0u8; 10]));
    }

    #[test]
    fn read_rejects_zero_interval() {
        let (_time, mut sd) = new_sd();
        let blob = header_blob(FORMAT_VERSION, 0, 0, HEADER_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_rejects_wrong_version() {
        let (_time, mut sd) = new_sd();
        let blob = header_blob(99, 1, 0, HEADER_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_rejects_count_without_enough_data() {
        let (_time, mut sd) = new_sd();
        // Header claims more samples than the payload carries.
        let blob = header_blob(FORMAT_VERSION, 1, HISTORY_CAPACITY as u32 + 1, HEADER_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_rejects_truncated_samples() {
        // Valid header claiming 10 samples, but only 5 samples worth of bytes after.
        let (_time, mut sd) = new_sd();
        let blob = header_blob(FORMAT_VERSION, 1, 10, HEADER_SIZE + 5 * SAMPLE_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_single_sample() {
        let (_time, mut sd) = new_sd();
        let mut blob = header_blob(FORMAT_VERSION, 1, 1, HEADER_SIZE + SAMPLE_SIZE);
        // Sample at offset HEADER_SIZE: time=1000, v=13.0, b_i=1.0, p_i=2.0, online=1.0.
        blob[12..16].copy_from_slice(&1000u32.to_le_bytes());
        blob[16..20].copy_from_slice(&13.0f32.to_le_bytes());
        blob[20..24].copy_from_slice(&1.0f32.to_le_bytes());
        blob[24..28].copy_from_slice(&2.0f32.to_le_bytes());
        blob[28..32].copy_from_slice(&1.0f32.to_le_bytes());

        assert!(sd.deserialize(&blob));
        assert_eq!(sd.history.len(), 1);
        assert_eq!(sd.history[0].time_s, 1000);
        assert!((sd.history[0].voltage - 13.0).abs() < 0.001);
    }

    #[test]
    #[should_panic(expected = "cannot average zero samples")]
    fn sample_accum_average_panics_on_zero() {
        let acc = SampleAccum::default();
        acc.average(0, 0);
    }

    #[test]
    fn read_resets_accumulator() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, CAP as u32 + 1, 0, &time);
        assert_eq!(sd.interval, 2);
        let blob = sd.serialize();

        let (time2, mut sd2) = new_sd();
        sd2.acc.voltage = 999.0;
        sd2.acc_count = 1;
        assert!(sd2.deserialize(&blob));
        assert_eq!(sd2.acc_count, 0);

        // With interval=2, first raw update should NOT push a new sample
        let base_len = sd2.history.len();
        time2.set(5000);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(sd2.history.len(), base_len);

        // Second raw update pushes averaged sample
        time2.set(5001);
        update(&mut sd2, bat_reading(13.0, 3.0), ps_reading(13.0, 4.0));
        assert_eq!(sd2.history.len(), base_len + 1);
        // battery_current = (1+3)/2 = 2.0
        assert!((sd2.history.last().unwrap().battery_current - 2.0).abs() < 0.01);
    }

    #[test]
    fn out_of_order_commits_are_rejected() {
        // NTP correction steps the clock backwards. The pre-jump sample was
        // committed; the post-jump one must be skipped so history stays ordered.
        let (time, mut sd) = new_sd();
        time.set(2000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(sd.history.len(), 1);

        time.set(1500);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(
            sd.history.len(),
            1,
            "backward-jump sample must not be pushed"
        );

        // Equal time_s also rejected (strictly increasing required).
        time.set(2000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(sd.history.len(), 1);

        // Forward again: accepted.
        time.set(2001);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(sd.history.len(), 2);
    }

    #[test]
    fn no_commit_before_ntp_sync() {
        // epoch_s() returning None gates history commits AND save_pending entirely.
        struct NoClock;
        impl Clock for NoClock {
            fn epoch_s(&self) -> Option<u32> {
                None
            }
        }
        let mut sd = SensorData::new(NoClock);
        for _ in 0..100 {
            sd.update_ps(ps_reading(13.0, 2.0));
            sd.update_battery(bat_reading(13.0, 1.0));
            sd.tick();
        }
        assert!(sd.history.is_empty(), "no samples before NTP sync");
        assert!(
            sd.take_save_payload().is_none(),
            "no save payload before NTP sync"
        );
    }

    // --- Auto-load on first update ---

    /// Helper: serialize `sd` and load it into a fresh `SensorData`.
    fn sd_with_blob(sd: &SensorData<TestClockSrc>) -> (TestClock, SensorData<TestClockSrc>) {
        let blob = sd.serialize();
        let (time, mut fresh) = new_sd();
        assert!(fresh.load_from_bytes(&blob));
        (time, fresh)
    }

    #[test]
    fn loads_from_platform_on_first_update() {
        let (time, mut sd) = new_sd();
        for i in 0..10u32 {
            time.set(1000 + i);
            update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        }

        let (time2, mut sd2) = sd_with_blob(&sd);
        time2.set(1010);
        update(&mut sd2, bat_reading(14.0, 3.0), ps_reading(14.0, 4.0));

        // 10 restored + 1 new = 11. Sample time must be > last restored (1009).
        assert_eq!(sd2.history.len(), 11);
        assert_eq!(sd2.interval, 1);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert!((sd2.history[0].battery_current - 1.0).abs() < 0.001);
        assert_eq!(sd2.history[10].time_s, 1010);
        assert!((sd2.history[10].voltage - 14.0).abs() < 0.001);
        assert!((sd2.history[10].battery_current - 3.0).abs() < 0.001);
    }

    #[test]
    fn load_restores_old_blob_via_update() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 100, 1000, &time);

        let (time2, mut sd2) = sd_with_blob(&sd);
        time2.set(5000);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));

        assert_eq!(sd2.history.len(), 101);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert_eq!(sd2.history[100].time_s, 5000);
    }

    #[test]
    fn load_rejects_corrupt_blob() {
        let (_time, mut sd) = new_sd();
        assert!(!sd.load_from_bytes(&[0xFF; 10]));
        assert!(sd.history.is_empty());
    }

    // --- Periodic save ---

    #[test]
    fn save_payload_fires_after_interval() {
        let (time, mut sd) = new_sd();
        time.set(1000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd.take_save_payload().is_none());

        time.set(1000 + SAVE_INTERVAL_S - 1);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd.take_save_payload().is_none());

        time.set(1000 + SAVE_INTERVAL_S);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd.take_save_payload().unwrap().len() > HEADER_SIZE);
    }

    #[test]
    fn save_timer_anchors_to_first_commit() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 10, 1000, &time);

        let (time2, mut sd2) = sd_with_blob(&sd);
        // First commit at t=1010 anchors last_save_s — no immediate save.
        time2.set(1010);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd2.take_save_payload().is_none());

        time2.set(1010 + SAVE_INTERVAL_S - 1);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd2.take_save_payload().is_none());

        time2.set(1010 + SAVE_INTERVAL_S);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd2.take_save_payload().is_some());
    }

    #[test]
    fn saved_blob_roundtrips_with_correct_values() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 100, 1000, &time);
        time.set(1100);
        update(&mut sd, bat_reading(12.0, 1.0), ps_reading(12.0, 2.0));
        time.set(1101);
        update(&mut sd, bat_reading(14.0, 3.0), ps_reading(14.0, 4.0));
        time.set(1000 + SAVE_INTERVAL_S);
        update(&mut sd, bat_reading(15.0, 9.0), ps_reading(15.0, 0.0));
        let blob = sd.take_save_payload().unwrap();

        let (_time2, mut sd2) = new_sd();
        assert!(sd2.load_from_bytes(&blob));
        assert_eq!(sd2.history.len(), 103);
        assert!((sd2.history[100].voltage - 12.0).abs() < 0.001);
        assert!((sd2.history[100].battery_current - 1.0).abs() < 0.001);
        assert!((sd2.history[101].voltage - 14.0).abs() < 0.001);
        assert_eq!(sd2.history[102].time_s, 1000 + SAVE_INTERVAL_S);
        assert!((sd2.history[102].battery_current - 9.0).abs() < 0.001);
    }

    #[test]
    fn save_payload_fires_repeatedly() {
        let (time, mut sd) = new_sd();
        time.set(1000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));

        let t1 = 1000 + SAVE_INTERVAL_S;
        time.set(t1);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        let blob1 = sd.take_save_payload().unwrap();

        time.set(t1 + SAVE_INTERVAL_S / 2);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd.take_save_payload().is_none());

        time.set(t1 + SAVE_INTERVAL_S);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd.take_save_payload().unwrap().len() > blob1.len());
    }

    #[test]
    fn save_includes_restored_and_new_data() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 100, 1000, &time);

        let (time2, mut sd2) = sd_with_blob(&sd);
        // fill() ended at t=1099, so the next commit must be > 1099.
        time2.set(1100);
        update(&mut sd2, bat_reading(14.0, 7.0), ps_reading(14.0, 8.0));
        assert_eq!(sd2.history.len(), 101);

        let trigger_t = 1100 + SAVE_INTERVAL_S;
        time2.set(trigger_t);
        update(&mut sd2, bat_reading(13.0, 9.0), ps_reading(13.0, 2.0));
        let blob = sd2.take_save_payload().unwrap();

        let (_time3, mut sd3) = new_sd();
        assert!(sd3.load_from_bytes(&blob));
        assert_eq!(sd3.history.len(), 102);
        assert!((sd3.history[0].battery_current - 1.0).abs() < 0.001);
        assert!((sd3.history[100].battery_current - 7.0).abs() < 0.001);
        assert_eq!(sd3.history[101].time_s, trigger_t);
        assert!((sd3.history[101].battery_current - 9.0).abs() < 0.001);
    }

    // --- Max interval cap ---

    /// Directly push samples into history, bypassing the accumulator.
    fn push_samples(sd: &mut SensorData<TestClockSrc>, n: usize, start_t: u32) {
        for i in 0..n {
            assert!(
                sd.history
                    .push(Sample {
                        time_s: start_t + i as u32,
                        voltage: 13.0,
                        battery_current: 1.0,
                        ps_current: 2.0,
                        power_online: 1.0,
                    })
                    .is_ok(),
                "history overflow"
            );
        }
    }

    #[test]
    fn at_max_interval_drops_oldest_via_update() {
        let (time, mut sd) = new_sd();
        sd.interval = MAX_INTERVAL;
        push_samples(&mut sd, CAP, 0);
        let oldest_before = sd.history[0].time_s;

        // One update at interval=MAX_INTERVAL needs MAX_INTERVAL raw calls.
        let base_t = 100_000;
        for i in 0..MAX_INTERVAL {
            time.set(base_t + i);
            update(&mut sd, bat_reading(13.0, 5.0), ps_reading(13.0, 3.0));
        }

        // Dropped oldest, pushed new — still at CAP.
        assert_eq!(sd.history.len(), CAP);
        assert_eq!(sd.interval, MAX_INTERVAL);
        assert!(sd.history[0].time_s > oldest_before);
        assert!((sd.history.last().unwrap().battery_current - 5.0).abs() < 0.01);
    }

    #[test]
    fn transition_from_compaction_to_dropping() {
        // Start at MAX_INTERVAL/2, fill to CAP. compact_if_needed compacts to
        // HALF at MAX_INTERVAL. Then fill to CAP again — should drop oldest.
        let (_time, mut sd) = new_sd();
        sd.interval = MAX_INTERVAL / 2;
        push_samples(&mut sd, CAP, 0);

        sd.compact_if_needed();
        assert_eq!(sd.history.len(), HALF);
        assert_eq!(sd.interval, MAX_INTERVAL);

        // Fill remaining slots to reach CAP again.
        let first_after_compact = sd.history[0].time_s;
        push_samples(&mut sd, HALF, CAP as u32);

        sd.compact_if_needed();

        // At MAX_INTERVAL — drops oldest instead of compacting.
        assert_eq!(sd.history.len(), CAP - 1);
        assert_eq!(sd.interval, MAX_INTERVAL);
        assert!(sd.history[0].time_s > first_after_compact);
    }

    // --- Producer-independence (F1): a dead sensor must not halt history ---

    #[test]
    fn battery_only_still_commits_with_ps_zeros() {
        // XY thread never publishes. Battery keeps flowing. History must
        // still grow — the ps_current / power_online fields report 0.
        let (time, mut sd) = new_sd();
        for i in 0..10u32 {
            time.set(100 + i);
            sd.update_battery(bat_reading(13.0, 1.5));
            sd.tick();
        }
        assert_eq!(sd.history.len(), 10, "battery-only ticks must still commit");
        for s in sd.history() {
            assert!((s.battery_current - 1.5).abs() < 0.001);
            assert!(s.ps_current.abs() < 0.001, "no PS reading → 0 A");
            assert!(s.power_online.abs() < 0.001, "no PS reading → offline");
        }
    }

    #[test]
    fn ps_goes_stale_after_threshold() {
        // PS updates once, then stops. Within STALE_TICKS the reading is
        // still visible and the sample carries the last-known ps_current.
        // Past STALE_TICKS the getter returns None and samples fall back to
        // zeros — surfacing the stuck producer on the dashboard / history.
        let (time, mut sd) = new_sd();
        time.set(1000);
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.5));
        sd.tick();
        // First tick increments ps_ticks_stale from 0 to 1 before the commit,
        // so we've already "used" one unit of the budget.
        assert_eq!(sd.history.len(), 1);
        assert!((sd.history[0].ps_current - 2.5).abs() < 0.001);
        assert!((sd.history[0].power_online - 1.0).abs() < 0.001);
        assert!(sd.ps_reading().is_some());

        // Keep battery live, don't touch PS, run up to the boundary: one
        // tick past the first takes stale to 2, ..., STALE_TICKS-1 more
        // ticks take it to STALE_TICKS (still fresh).
        for i in 1..STALE_TICKS {
            time.set(1000 + i);
            sd.update_battery(bat_reading(13.0, 1.0));
            sd.tick();
        }
        assert!(sd.ps_reading().is_some(), "PS still fresh at STALE_TICKS");
        let last_fresh = sd.history.last().unwrap();
        assert!((last_fresh.ps_current - 2.5).abs() < 0.001);

        // One more tick past the boundary: PS reported stale, sample zeros.
        time.set(1000 + STALE_TICKS);
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick();
        assert!(sd.ps_reading().is_none(), "PS should be stale now");
        let latest = sd.history.last().unwrap();
        assert!(latest.ps_current.abs() < 0.001);
        assert!(latest.power_online.abs() < 0.001);
    }

    #[test]
    fn battery_stale_commits_zeros() {
        // Symmetry: a stale battery contributes zeros too (not a commit
        // skip). Timeline stays continuous so a dead INA shows as flat-line
        // zero on the dashboard rather than frozen last-known values.
        let (time, mut sd) = new_sd();
        time.set(2000);
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.tick();
        assert_eq!(sd.history.len(), 1);
        assert!((sd.history[0].voltage - 13.0).abs() < 0.001);

        // Run long enough for battery to go stale; keep PS fresh.
        let ticks = STALE_TICKS + 3;
        for i in 1..=ticks {
            time.set(2000 + i);
            sd.update_ps(ps_reading(13.0, 2.0));
            sd.tick();
        }
        assert_eq!(
            sd.history.len(),
            1 + ticks as usize,
            "history must keep growing even with a dead battery sensor"
        );
        assert!(sd.battery_reading().is_none());
        let latest = sd.history.last().unwrap();
        assert!(latest.voltage.abs() < 0.001);
        assert!(latest.battery_current.abs() < 0.001);
        // PS side still fresh → still shows its reading.
        assert!((latest.ps_current - 2.0).abs() < 0.001);
    }
}
