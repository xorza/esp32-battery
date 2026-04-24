const FORMAT_VERSION: u32 = 6;
const HEADER_SIZE: usize = 4 + 4 + 4; // version + interval + count
const SAMPLE_SIZE: usize = 4 + 4 * 4; // u32 + 4×f32 = 20 bytes
const POWER_ONLINE_THRESHOLD: f32 = 0.1;
const MAX_BLOB_SIZE: usize = 4096;
/// Max samples that fit in MAX_BLOB_SIZE. 204 × 1024s ≈ 58 hours of history.
pub const HISTORY_CAPACITY: usize = (MAX_BLOB_SIZE - HEADER_SIZE) / SAMPLE_SIZE / 2 * 2;
// Once interval reaches this, drop old samples instead of compacting further.
// 204 samples × 1024s ≈ 58h (covers 24h with margin).
const MAX_INTERVAL: u32 = 1024;
const _: () = assert!(
    HISTORY_CAPACITY.is_multiple_of(2),
    "HISTORY_CAPACITY must be even"
);
const _: () = assert!(
    HEADER_SIZE + HISTORY_CAPACITY * SAMPLE_SIZE <= MAX_BLOB_SIZE,
    "serialized history must fit in MAX_BLOB_SIZE"
);
const SAVE_INTERVAL_S: u32 = 600;

pub trait Platform {
    fn epoch_s(&self) -> Option<u32>;
    fn save_blob(&self, data: &[u8]);
    /// Load persisted blob into `buf`. Returns the number of bytes written, or None if no blob.
    fn load_blob(&self, buf: &mut [u8]) -> Option<usize>;
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

/// Central data store with adaptive-resolution history.
///
/// Stores up to `HISTORY_CAPACITY` samples. When full, pairs are averaged
/// (halving the count) and the sampling interval doubles. This gives
/// exponentially growing time coverage in fixed memory (~4 KB).
pub struct SensorData<P: Platform> {
    /// Latest reading from each side. `None` only until the first real reading
    /// from that sensor — then always `Some` so HTTP/LCD can snapshot live values.
    pub battery_reading: Option<Ina228Reading>,
    pub ps_reading: Option<PsReading>,
    /// Updated-since-last-commit flags. Each `update_*` sets its flag; a history
    /// row is committed only once both are set, after which both are cleared.
    /// Without this we'd commit twice per cycle (once per thread), each time
    /// pairing a fresh reading with the other side's previous value.
    battery_updated: bool,
    ps_updated: bool,
    history: heapless::Vec<Sample, HISTORY_CAPACITY>,
    /// Current sampling interval: how many raw updates per stored sample.
    interval: u32,
    acc: SampleAccum,
    acc_count: u32,
    platform: P,
    loaded: bool,
    last_save_s: u32,
    buf: Box<[u8; MAX_BLOB_SIZE]>,
}

struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl BufWriter<'_> {
    fn u32(&mut self, v: u32) {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
    }
    fn f32(&mut self, v: f32) {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
    }
    fn sample(&mut self, s: &Sample) {
        self.u32(s.time_s);
        self.f32(s.voltage);
        self.f32(s.battery_current);
        self.f32(s.ps_current);
        self.f32(s.power_online);
    }
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

impl<P: Platform> SensorData<P> {
    pub fn new(platform: P) -> Self {
        Self {
            battery_reading: None,
            ps_reading: None,
            battery_updated: false,
            ps_updated: false,
            history: heapless::Vec::new(),
            interval: 1,
            acc: SampleAccum::default(),
            acc_count: 0,
            platform,
            loaded: false,
            last_save_s: 0,
            buf: Box::new([0u8; MAX_BLOB_SIZE]),
        }
    }

    /// Called by the INA thread once per cycle. Publishes the latest reading
    /// and attempts a history commit.
    pub fn update_battery(&mut self, bat: Ina228Reading) {
        self.battery_reading = Some(bat);
        self.battery_updated = true;
        self.try_commit();
    }

    /// Called by the XY thread once per poll. Publishes the latest reading
    /// and attempts a history commit.
    pub fn update_ps(&mut self, ps: PsReading) {
        self.ps_reading = Some(ps);
        self.ps_updated = true;
        self.try_commit();
    }

    /// `1.0` when the latest PS reading shows measurable current, `0.0` otherwise.
    /// Returns `0.0` before the first PS reading arrives.
    pub fn power_online(&self) -> f32 {
        match self.ps_reading {
            Some(ps) if ps.current.abs() > POWER_ONLINE_THRESHOLD => 1.0,
            _ => 0.0,
        }
    }

    /// Commit one history sample when both sides have a reading that has been
    /// updated since the last commit. Clears the updated flags (but keeps the
    /// readings themselves) so HTTP/LCD still see live values between commits.
    fn try_commit(&mut self) {
        if !(self.battery_updated && self.ps_updated) {
            return;
        }
        let (Some(bat), Some(ps)) = (self.battery_reading, self.ps_reading) else {
            return;
        };
        let Some(time_s) = self.platform.epoch_s() else {
            return;
        };
        self.battery_updated = false;
        self.ps_updated = false;

        if !self.loaded {
            self.loaded = true;
            if let Some(len) = self.platform.load_blob(&mut *self.buf) {
                if self.read(len) {
                    log::info!("Loaded {} samples from NVS", self.history.len());
                } else {
                    log::warn!("Failed to read history from NVS ({} bytes)", len);
                }
            }
            self.last_save_s = time_s;
        }

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

        // NOTE: save_blob runs while the SensorData mutex is held by the caller,
        // stalling other readers for ~50–100 ms every SAVE_INTERVAL_S (10 min).
        // Moving the save outside the lock would require removing `platform` from
        // SensorData — a large refactor touching every test. The stall is rare
        // and brief enough that the current design is accepted.
        if time_s.saturating_sub(self.last_save_s) >= SAVE_INTERVAL_S {
            self.last_save_s = time_s;
            let len = self.write();
            log::info!(
                "Saved {} samples to NVS ({} bytes)",
                self.history.len(),
                len
            );
            self.platform.save_blob(&self.buf[..len]);
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

    /// Serialize history + metadata into internal buffer.
    /// Returns the number of bytes written.
    fn write(&mut self) -> usize {
        let mut w = BufWriter {
            buf: &mut *self.buf,
            pos: 0,
        };
        w.u32(FORMAT_VERSION);
        w.u32(self.interval);
        w.u32(self.history.len() as u32);
        for s in &self.history {
            w.sample(s);
        }
        w.pos
    }

    /// Restore history from internal buffer.
    /// Returns false if data is invalid.
    fn read(&mut self, len: usize) -> bool {
        if len < HEADER_SIZE {
            return false;
        }

        let mut r = BufReader {
            buf: &self.buf[..len],
            pos: 0,
        };
        let version = r.u32();
        if version != FORMAT_VERSION {
            return false;
        }
        let interval = r.u32();
        let count = r.u32() as usize;

        if interval == 0 || count == 0 || len < HEADER_SIZE + count * SAMPLE_SIZE {
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
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    const CAP: usize = HISTORY_CAPACITY;
    const HALF: usize = CAP / 2;

    type Clock = Rc<Cell<u32>>;

    struct TestPlatform {
        time: Clock,
        blob: RefCell<Option<Vec<u8>>>,
    }

    impl TestPlatform {
        fn new() -> (Clock, Self) {
            let time = Rc::new(Cell::new(0u32));
            (
                Rc::clone(&time),
                Self {
                    time,
                    blob: RefCell::new(None),
                },
            )
        }

        fn has_blob(&self) -> bool {
            self.blob.borrow().is_some()
        }

        fn take_blob(&self) -> Option<Vec<u8>> {
            self.blob.borrow().clone()
        }

        fn clear_blob(&self) {
            *self.blob.borrow_mut() = None;
        }
    }

    impl Platform for TestPlatform {
        fn epoch_s(&self) -> Option<u32> {
            Some(self.time.get())
        }

        fn save_blob(&self, data: &[u8]) {
            *self.blob.borrow_mut() = Some(data.to_vec());
        }

        fn load_blob(&self, buf: &mut [u8]) -> Option<usize> {
            let blob = self.blob.borrow();
            let data = blob.as_ref()?;
            let len = data.len();
            buf[..len].copy_from_slice(data);
            Some(len)
        }
    }

    fn new_sd() -> (Clock, SensorData<TestPlatform>) {
        let (time, platform) = TestPlatform::new();
        (time, SensorData::new(platform))
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

    /// Shorthand: update with battery + PS readings.
    fn update(sd: &mut SensorData<TestPlatform>, bat: Ina228Reading, p: PsReading) {
        sd.update_ps(p);
        sd.update_battery(bat);
    }

    /// Push n uniform samples (v=13, c1=1, c2=2). Returns the next time_s value.
    fn fill(sd: &mut SensorData<TestPlatform>, n: u32, start_t: u32, time: &Clock) -> u32 {
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
        // Readings persist across commits so HTTP/LCD can always snapshot
        // live values — only the `_updated` flags get cleared.
        let (time, mut sd) = new_sd();
        time.set(10);
        update(&mut sd, bat_reading(13.0, 1.5), ps_reading(13.1, 2.5));
        assert!((sd.battery_reading.unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading.unwrap().current - 2.5).abs() < 0.001);
        assert!(!sd.battery_updated && !sd.ps_updated);
    }

    #[test]
    fn only_one_commit_per_pair_of_updates() {
        // With both sides at 1 Hz we want 1 history row per second, not 2.
        let (time, mut sd) = new_sd();
        time.set(1);
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0)); // first commit
        assert_eq!(sd.history.len(), 1);
        // A second update_ps without a new update_battery must NOT commit.
        time.set(2);
        sd.update_ps(ps_reading(13.0, 2.0));
        assert_eq!(sd.history.len(), 1);
        sd.update_battery(bat_reading(13.0, 1.0)); // second commit
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
        struct NoTimePlatform;
        impl Platform for NoTimePlatform {
            fn epoch_s(&self) -> Option<u32> {
                None
            }
            fn save_blob(&self, _: &[u8]) {}
            fn load_blob(&self, _: &mut [u8]) -> Option<usize> {
                None
            }
        }
        let mut sd = SensorData::new(NoTimePlatform);
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.5));
        assert!(sd.history.is_empty());
        assert!(!sd.loaded);
        // Latest readings must still be visible to HTTP/LCD before NTP sync.
        assert!((sd.battery_reading.unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading.unwrap().current - 2.0).abs() < 0.001);
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
        // Above threshold: s2.current = 2.0 > 0.01 → 1.0
        let (_time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!((sd.history[0].power_online - 1.0).abs() < 0.001);

        // Below threshold: s2.current = 0.005 < 0.01 → 0.0
        let (_time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 0.005));
        assert!(sd.history[0].power_online.abs() < 0.001);

        // Exactly zero: → 0.0
        let (_time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 0.0));
        assert!(sd.history[0].power_online.abs() < 0.001);

        // Negative current: s2.current = -0.5 → abs > threshold → 1.0
        let (_time, mut sd) = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, -0.5));
        assert!((sd.history[0].power_online - 1.0).abs() < 0.001);
    }

    #[test]
    fn power_online_averaged_during_compaction() {
        let (time, mut sd) = new_sd();
        // Alternating online/offline: even=online (c2=2.0), odd=offline (c2=0.0)
        for i in 0..(CAP as u32 + 1) {
            time.set(i);
            let c2 = if i % 2 == 0 { 2.0 } else { 0.0 };
            update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, c2));
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
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 0.0)); // offline
        let len = sd.write();

        let (_time2, mut sd2) = new_sd();
        sd2.buf[..len].copy_from_slice(&sd.buf[..len]);
        assert!(sd2.read(len));
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
        let (_time, mut sd) = new_sd();
        let len = sd.write();
        assert_eq!(len, HEADER_SIZE);

        let (_time2, mut sd2) = new_sd();
        sd2.buf[..len].copy_from_slice(&sd.buf[..len]);
        assert!(!sd2.read(len));
    }

    #[test]
    fn write_read_roundtrip() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 10, 1000, &time);
        let len = sd.write();
        assert_eq!(len, HEADER_SIZE + 10 * SAMPLE_SIZE);

        let (_time2, mut sd2) = new_sd();
        sd2.buf[..len].copy_from_slice(&sd.buf[..len]);
        assert!(sd2.read(len));
        assert_eq!(sd2.history.len(), 10);
        assert_eq!(sd2.interval, 1);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert_eq!(sd2.history[9].time_s, 1009);
    }

    #[test]
    fn read_rejects_truncated() {
        let (_time, mut sd) = new_sd();
        sd.buf[..10].copy_from_slice(&[0u8; 10]);
        assert!(!sd.read(10));
    }

    #[test]
    fn read_rejects_zero_interval() {
        let (_time, mut sd) = new_sd();
        sd.buf[0..4].copy_from_slice(&FORMAT_VERSION.to_le_bytes());
        // interval at offset 4 is already 0
        assert!(!sd.read(HEADER_SIZE));
    }

    #[test]
    fn read_rejects_wrong_version() {
        let (_time, mut sd) = new_sd();
        sd.buf[0..4].copy_from_slice(&99u32.to_le_bytes());
        assert!(!sd.read(HEADER_SIZE));
    }

    #[test]
    fn read_rejects_count_without_enough_data() {
        let (_time, mut sd) = new_sd();
        let mut w = BufWriter {
            buf: &mut *sd.buf,
            pos: 0,
        };
        w.u32(FORMAT_VERSION);
        w.u32(1); // interval
        w.u32(HISTORY_CAPACITY as u32 + 1); // count > capacity but no sample data
        let len = w.pos;
        // Rejected because buffer is too short for claimed sample count.
        assert!(!sd.read(len));
    }

    #[test]
    fn read_rejects_truncated_samples() {
        // Valid header claiming 10 samples, but only provide space for 5
        let (_time, mut sd) = new_sd();
        let mut w = BufWriter {
            buf: &mut *sd.buf,
            pos: 0,
        };
        w.u32(FORMAT_VERSION);
        w.u32(1);
        w.u32(10); // claims 10 samples
        let len = HEADER_SIZE + 5 * SAMPLE_SIZE;
        assert!(!sd.read(len));
    }

    #[test]
    fn read_single_sample() {
        let (_time, mut sd) = new_sd();
        let mut w = BufWriter {
            buf: &mut *sd.buf,
            pos: 0,
        };
        w.u32(FORMAT_VERSION);
        w.u32(1); // interval
        w.u32(1); // count=1
        w.u32(1000);
        w.f32(13.0);
        w.f32(1.0);
        w.f32(2.0);
        w.f32(1.0); // power_online
        let len = w.pos;

        assert!(sd.read(len));
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
        let len = sd.write();

        let (time2, mut sd2) = new_sd();
        sd2.buf[..len].copy_from_slice(&sd.buf[..len]);
        sd2.acc.voltage = 999.0;
        sd2.acc_count = 1;
        assert!(sd2.read(len));
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
    fn save_does_not_panic_on_time_jump_backward() {
        let (time, mut sd) = new_sd();
        time.set(2000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        // Time jumps backward (NTP correction)
        time.set(1500);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        // Should not panic — saturating_sub yields 0, no save triggered
    }

    // --- Auto-load on first update ---

    /// Helper: create a SensorData with a pre-stored blob from `sd`.
    fn sd_with_blob(sd: &mut SensorData<TestPlatform>) -> (Clock, SensorData<TestPlatform>) {
        let len = sd.write();
        let (time, platform) = TestPlatform::new();
        platform.save_blob(&sd.buf[..len]);
        (time, SensorData::new(platform))
    }

    #[test]
    fn loads_from_platform_on_first_update() {
        let (time, mut sd) = new_sd();
        for i in 0..10u32 {
            time.set(1000 + i);
            update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        }

        let (time2, mut sd2) = sd_with_blob(&mut sd);
        time2.set(1009);
        update(&mut sd2, bat_reading(14.0, 3.0), ps_reading(14.0, 4.0));

        // 10 restored + 1 new = 11
        assert_eq!(sd2.history.len(), 11);
        assert_eq!(sd2.interval, 1);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert!((sd2.history[0].battery_current - 1.0).abs() < 0.001);
        // New sample: voltage=avg(14,14)=14, battery_current=3
        assert_eq!(sd2.history[10].time_s, 1009);
        assert!((sd2.history[10].voltage - 14.0).abs() < 0.001);
        assert!((sd2.history[10].battery_current - 3.0).abs() < 0.001);
    }

    #[test]
    fn load_restores_old_blob_via_update() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 100, 1000, &time);

        let (time2, mut sd2) = sd_with_blob(&mut sd);
        time2.set(5000);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));

        assert_eq!(sd2.history.len(), 101);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert_eq!(sd2.history[100].time_s, 5000);
    }

    #[test]
    fn load_with_corrupt_blob_skips_gracefully() {
        let (time, platform) = TestPlatform::new();
        platform.save_blob(&[0xFF; 10]);
        let mut sd = SensorData::new(platform);

        time.set(1000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));

        assert_eq!(sd.history.len(), 1);
        assert_eq!(sd.history[0].time_s, 1000);
        assert!(sd.loaded);
    }

    #[test]
    fn loads_only_once() {
        let (time, mut sd) = new_sd();
        time.set(1000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(sd.history.len(), 1);

        // Store a blob after the first update — should NOT be loaded again
        let (time_src, mut sd_src) = new_sd();
        fill(&mut sd_src, 10, 500, &time_src);
        let len = sd_src.write();
        sd.platform.save_blob(&sd_src.buf[..len]);

        time.set(1001);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(sd.history.len(), 2); // not 10+2
    }

    #[test]
    fn no_blob_skips_load_gracefully() {
        let (time, mut sd) = new_sd();
        time.set(1000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert_eq!(sd.history.len(), 1);
        assert!(sd.loaded);
    }

    // --- Periodic save ---

    #[test]
    fn saves_after_interval() {
        let (time, mut sd) = new_sd();
        time.set(1000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(!sd.platform.has_blob());

        time.set(1000 + SAVE_INTERVAL_S - 1);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(!sd.platform.has_blob());

        time.set(1000 + SAVE_INTERVAL_S);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd.platform.take_blob().unwrap().len() > HEADER_SIZE);
    }

    #[test]
    fn save_timer_anchors_to_load_time() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 10, 1000, &time);

        let (time2, mut sd2) = sd_with_blob(&mut sd);
        // Load happens at t=1009 → last_save_s = 1009
        time2.set(1009);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        sd2.platform.clear_blob();

        time2.set(1009 + SAVE_INTERVAL_S - 1);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(!sd2.platform.has_blob());

        time2.set(1009 + SAVE_INTERVAL_S);
        update(&mut sd2, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd2.platform.has_blob());
    }

    #[test]
    fn saved_blob_roundtrips_with_correct_values() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 100, 1000, &time);
        time.set(1100);
        update(&mut sd, bat_reading(12.0, 1.0), ps_reading(12.0, 2.0));
        time.set(1101);
        update(&mut sd, bat_reading(14.0, 3.0), ps_reading(14.0, 4.0));
        // Trigger save
        time.set(1000 + SAVE_INTERVAL_S);
        update(&mut sd, bat_reading(15.0, 9.0), ps_reading(15.0, 0.0));
        let blob = sd.platform.take_blob().unwrap();

        let (_time2, mut sd2) = new_sd();
        sd2.buf[..blob.len()].copy_from_slice(&blob);
        assert!(sd2.read(blob.len()));
        assert_eq!(sd2.history.len(), 103);
        assert!((sd2.history[100].voltage - 12.0).abs() < 0.001);
        assert!((sd2.history[100].battery_current - 1.0).abs() < 0.001);
        assert!((sd2.history[101].voltage - 14.0).abs() < 0.001);
        assert_eq!(sd2.history[102].time_s, 1000 + SAVE_INTERVAL_S);
        assert!((sd2.history[102].battery_current - 9.0).abs() < 0.001);
    }

    #[test]
    fn saves_repeatedly() {
        let (time, mut sd) = new_sd();
        time.set(1000);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));

        let t1 = 1000 + SAVE_INTERVAL_S;
        time.set(t1);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        let blob1 = sd.platform.take_blob().unwrap();
        sd.platform.clear_blob();

        time.set(t1 + SAVE_INTERVAL_S / 2);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(!sd.platform.has_blob());

        time.set(t1 + SAVE_INTERVAL_S);
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0));
        assert!(sd.platform.take_blob().unwrap().len() > blob1.len());
    }

    #[test]
    fn save_includes_restored_and_new_data() {
        let (time, mut sd) = new_sd();
        fill(&mut sd, 100, 1000, &time);

        let (time2, mut sd2) = sd_with_blob(&mut sd);
        time2.set(1099);
        update(&mut sd2, bat_reading(14.0, 7.0), ps_reading(14.0, 8.0));
        assert_eq!(sd2.history.len(), 101);

        let trigger_t = 1099 + SAVE_INTERVAL_S;
        time2.set(trigger_t);
        update(&mut sd2, bat_reading(13.0, 9.0), ps_reading(13.0, 2.0));
        let blob = sd2.platform.take_blob().unwrap();

        let (_time3, mut sd3) = new_sd();
        sd3.buf[..blob.len()].copy_from_slice(&blob);
        assert!(sd3.read(blob.len()));
        assert_eq!(sd3.history.len(), 102);
        assert!((sd3.history[0].battery_current - 1.0).abs() < 0.001);
        assert!((sd3.history[100].battery_current - 7.0).abs() < 0.001);
        assert_eq!(sd3.history[101].time_s, trigger_t);
        assert!((sd3.history[101].battery_current - 9.0).abs() < 0.001);
    }

    // --- Max interval cap ---

    /// Directly push samples into history, bypassing the accumulator.
    fn push_samples(sd: &mut SensorData<TestPlatform>, n: usize, start_t: u32) {
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
}
