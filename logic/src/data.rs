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
pub struct SensorData {
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
            battery_ticks_stale: u32::MAX,
            ps_ticks_stale: u32::MAX,
            history: heapless::Vec::new(),
            interval: 1,
            acc: SampleAccum::default(),
            acc_count: 0,
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

    /// Drive the history pipeline forward by one tick. `now_epoch` is the
    /// wall-clock second the caller wants stamped on any committed sample;
    /// `None` (e.g. before NTP sync) gates out commits but still ages the
    /// staleness counters.
    ///
    /// Commits one raw sample per call using whichever readings are
    /// currently fresh — stale producers contribute zeros, so history stays
    /// continuous and the dead side surfaces as flat-line zero on the
    /// dashboard. Replaces the old "commit when both producers tick"
    /// rendezvous, which coupled history liveness to the least-available
    /// sensor.
    pub fn tick(&mut self, now_epoch: Option<u32>) {
        self.battery_ticks_stale = self.battery_ticks_stale.saturating_add(1);
        self.ps_ticks_stale = self.ps_ticks_stale.saturating_add(1);

        let Some(time_s) = now_epoch else {
            return;
        };

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

    /// Serialize history + metadata into a fresh `Vec` for NVS storage.
    pub fn serialize(&self) -> Vec<u8> {
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

    const CAP: usize = HISTORY_CAPACITY;
    const HALF: usize = CAP / 2;

    fn new_sd() -> SensorData {
        SensorData::new()
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

    /// Shorthand: publish battery + PS readings and run one supervisor tick
    /// stamped with `now`.
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

    // --- Default / basic update ---

    #[test]
    fn default_is_empty() {
        let sd = new_sd();
        assert!(sd.history.is_empty());
        assert_eq!(sd.interval, 1);
    }

    #[test]
    fn single_update() {
        let mut sd = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 100);

        assert_eq!(sd.history.len(), 1);
        let s = &sd.history[0];
        assert_eq!(s.time_s, 100);
        assert!((s.voltage - 13.0).abs() < 0.001);
        assert!((s.battery_current - 1.0).abs() < 0.001);
        assert!((s.ps_current - 2.0).abs() < 0.001);
    }

    #[test]
    fn voltage_from_battery_only() {
        let mut sd = new_sd();
        // Voltage comes solely from the battery INA — PS voltage is ignored.
        update(&mut sd, bat_reading(12.0, 1.0), ps_reading(14.0, 2.0), 1);
        assert!((sd.history[0].voltage - 12.0).abs() < 0.001);
    }

    #[test]
    fn latest_readings_visible_after_commit() {
        let mut sd = new_sd();
        update(&mut sd, bat_reading(13.0, 1.5), ps_reading(13.1, 2.5), 10);
        assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading().unwrap().current - 2.5).abs() < 0.001);
    }

    #[test]
    fn one_commit_per_tick_regardless_of_update_order() {
        // tick-driven commits: one tick → at most one history row.
        let mut sd = new_sd();
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick(Some(1));
        assert_eq!(sd.history.len(), 1);

        sd.tick(Some(2));
        assert_eq!(sd.history.len(), 2);
    }

    #[test]
    fn history_returns_all_entries() {
        let mut sd = new_sd();
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
        // tick(None) gates history commits but leaves readings snapshot-visible.
        let mut sd = new_sd();
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.5));
        sd.tick(None);
        assert!(sd.history.is_empty());
        assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
        assert!((sd.ps_reading().unwrap().current - 2.0).abs() < 0.001);
    }

    #[test]
    fn multiple_updates_accumulate() {
        let mut sd = new_sd();
        fill(&mut sd, 10, 0);
        assert_eq!(sd.history.len(), 10);
    }

    // --- Power online tracking ---

    #[test]
    fn power_online_threshold() {
        // Online when PS voltage is above the threshold (load-independent).
        let mut sd = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 0.0), 1);
        assert!((sd.history[0].power_online - 1.0).abs() < 0.001);

        let mut sd = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(1.0, 2.0), 1);
        assert!(sd.history[0].power_online.abs() < 0.001);

        let mut sd = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(0.0, 0.0), 1);
        assert!(sd.history[0].power_online.abs() < 0.001);
    }

    #[test]
    fn power_online_averaged_during_compaction() {
        let mut sd = new_sd();
        // Alternating online (v=13) / offline (v=0).
        for i in 0..(CAP as u32 + 1) {
            let v = if i % 2 == 0 { 13.0 } else { 0.0 };
            update(&mut sd, bat_reading(13.0, 1.0), ps_reading(v, 1.0), i);
        }
        assert_eq!(sd.interval, 2);
        for s in &sd.history[..HALF] {
            assert!((s.power_online - 0.5).abs() < 0.01);
        }
    }

    #[test]
    fn power_online_roundtrips_through_persistence() {
        let mut sd = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 0); // online
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(0.0, 0.0), 1); // offline
        let blob = sd.serialize();

        let mut sd2 = new_sd();
        assert!(sd2.deserialize(&blob));
        assert!((sd2.history[0].power_online - 1.0).abs() < 0.001);
        assert!(sd2.history[1].power_online.abs() < 0.001);
    }

    // --- Compaction ---

    #[test]
    fn no_compaction_below_capacity() {
        let mut sd = new_sd();
        fill(&mut sd, CAP as u32 - 1, 0);
        assert_eq!(sd.history.len(), CAP - 1);
        assert_eq!(sd.interval, 1);
    }

    #[test]
    fn compaction_at_capacity() {
        let mut sd = new_sd();
        fill(&mut sd, CAP as u32 + 1, 0);
        assert_eq!(sd.history.len(), HALF + 1);
        assert_eq!(sd.interval, 2);
    }

    #[test]
    fn compaction_averages_all_fields_and_uses_later_timestamp() {
        let mut sd = new_sd();
        for i in 0..(CAP as u32 + 1) {
            let t = i * 10;
            if i % 2 == 0 {
                update(&mut sd, bat_reading(12.0, 1.0), ps_reading(12.0, 2.0), t);
            } else {
                update(&mut sd, bat_reading(14.0, 3.0), ps_reading(14.0, 4.0), t);
            }
        }
        assert_eq!(sd.history.len(), HALF + 1);
        assert_eq!(sd.interval, 2);

        for s in &sd.history[..HALF] {
            assert!((s.voltage - 13.0).abs() < 0.01);
            assert!((s.battery_current - 2.0).abs() < 0.01);
            assert!((s.ps_current - 3.0).abs() < 0.01);
        }
        assert_eq!(sd.history[0].time_s, 10);
        assert_eq!(sd.history[1].time_s, 30);
        assert_eq!(sd.history[HALF - 1].time_s, (CAP as u32 - 1) * 10);
    }

    #[test]
    fn after_compaction_samples_at_new_interval() {
        let mut sd = new_sd();
        let t = fill(&mut sd, CAP as u32 + 1, 0);
        assert_eq!(sd.interval, 2);

        update(&mut sd, bat_reading(13.0, 5.0), ps_reading(13.0, 0.0), t);
        assert_eq!(sd.history.len(), HALF + 1);

        update(
            &mut sd,
            bat_reading(13.0, 7.0),
            ps_reading(13.0, 0.0),
            t + 1,
        );
        assert_eq!(sd.history.len(), HALF + 2);
        let last = sd.history.last().unwrap();
        assert!((last.battery_current - 6.0).abs() < 0.01);
        assert_eq!(last.time_s, t + 1);
    }

    #[test]
    fn interval_doubles_each_compaction() {
        let mut sd = new_sd();
        assert_eq!(sd.interval, 1);
        fill(&mut sd, 820, 0);
        assert_eq!(sd.interval, 8);
    }

    #[test]
    fn long_run_stays_bounded_and_chronological() {
        let mut sd = new_sd();
        fill(&mut sd, 10000, 0);
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
        let sd = new_sd();
        let blob = sd.serialize();
        assert_eq!(blob.len(), HEADER_SIZE);

        let mut sd2 = new_sd();
        assert!(!sd2.deserialize(&blob));
    }

    #[test]
    fn write_read_roundtrip() {
        let mut sd = new_sd();
        fill(&mut sd, 10, 1000);
        let blob = sd.serialize();
        assert_eq!(blob.len(), HEADER_SIZE + 10 * SAMPLE_SIZE);

        let mut sd2 = new_sd();
        assert!(sd2.deserialize(&blob));
        assert_eq!(sd2.history.len(), 10);
        assert_eq!(sd2.interval, 1);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert_eq!(sd2.history[9].time_s, 1009);
    }

    fn header_blob(version: u32, interval: u32, count: u32, total_len: usize) -> Vec<u8> {
        let mut out = vec![0u8; total_len];
        out[0..4].copy_from_slice(&version.to_le_bytes());
        out[4..8].copy_from_slice(&interval.to_le_bytes());
        out[8..12].copy_from_slice(&count.to_le_bytes());
        out
    }

    #[test]
    fn read_rejects_truncated() {
        let mut sd = new_sd();
        assert!(!sd.deserialize(&[0u8; 10]));
    }

    #[test]
    fn read_rejects_zero_interval() {
        let mut sd = new_sd();
        let blob = header_blob(FORMAT_VERSION, 0, 0, HEADER_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_rejects_wrong_version() {
        let mut sd = new_sd();
        let blob = header_blob(99, 1, 0, HEADER_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_rejects_count_without_enough_data() {
        let mut sd = new_sd();
        let blob = header_blob(FORMAT_VERSION, 1, HISTORY_CAPACITY as u32 + 1, HEADER_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_rejects_truncated_samples() {
        let mut sd = new_sd();
        let blob = header_blob(FORMAT_VERSION, 1, 10, HEADER_SIZE + 5 * SAMPLE_SIZE);
        assert!(!sd.deserialize(&blob));
    }

    #[test]
    fn read_single_sample() {
        let mut sd = new_sd();
        let mut blob = header_blob(FORMAT_VERSION, 1, 1, HEADER_SIZE + SAMPLE_SIZE);
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
        let mut sd = new_sd();
        fill(&mut sd, CAP as u32 + 1, 0);
        assert_eq!(sd.interval, 2);
        let blob = sd.serialize();

        let mut sd2 = new_sd();
        sd2.acc.voltage = 999.0;
        sd2.acc_count = 1;
        assert!(sd2.deserialize(&blob));
        assert_eq!(sd2.acc_count, 0);

        let base_len = sd2.history.len();
        update(
            &mut sd2,
            bat_reading(13.0, 1.0),
            ps_reading(13.0, 2.0),
            5000,
        );
        assert_eq!(sd2.history.len(), base_len);

        update(
            &mut sd2,
            bat_reading(13.0, 3.0),
            ps_reading(13.0, 4.0),
            5001,
        );
        assert_eq!(sd2.history.len(), base_len + 1);
        assert!((sd2.history.last().unwrap().battery_current - 2.0).abs() < 0.01);
    }

    #[test]
    fn out_of_order_commits_are_rejected() {
        let mut sd = new_sd();
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2000);
        assert_eq!(sd.history.len(), 1);

        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 1500);
        assert_eq!(
            sd.history.len(),
            1,
            "backward-jump sample must not be pushed"
        );

        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2000);
        assert_eq!(sd.history.len(), 1);

        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2001);
        assert_eq!(sd.history.len(), 2);
    }

    #[test]
    fn no_commit_before_ntp_sync() {
        let mut sd = new_sd();
        for _ in 0..100 {
            sd.update_ps(ps_reading(13.0, 2.0));
            sd.update_battery(bat_reading(13.0, 1.0));
            sd.tick(None);
        }
        assert!(sd.history.is_empty(), "no samples before NTP sync");
    }

    // --- Restore-from-blob ---

    /// Helper: serialize `sd` and load it into a fresh `SensorData`.
    fn sd_with_blob(sd: &SensorData) -> SensorData {
        let blob = sd.serialize();
        let mut fresh = new_sd();
        assert!(fresh.load_from_bytes(&blob));
        fresh
    }

    #[test]
    fn loads_from_platform_on_first_update() {
        let mut sd = new_sd();
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
        let mut sd = new_sd();
        fill(&mut sd, 100, 1000);

        let mut sd2 = sd_with_blob(&sd);
        update(
            &mut sd2,
            bat_reading(13.0, 1.0),
            ps_reading(13.0, 2.0),
            5000,
        );

        assert_eq!(sd2.history.len(), 101);
        assert_eq!(sd2.history[0].time_s, 1000);
        assert_eq!(sd2.history[100].time_s, 5000);
    }

    #[test]
    fn load_rejects_corrupt_blob() {
        let mut sd = new_sd();
        assert!(!sd.load_from_bytes(&[0xFF; 10]));
        assert!(sd.history.is_empty());
    }

    // --- Max interval cap ---

    /// Directly push samples into history, bypassing the accumulator.
    fn push_samples(sd: &mut SensorData, n: usize, start_t: u32) {
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
        let mut sd = new_sd();
        sd.interval = MAX_INTERVAL;
        push_samples(&mut sd, CAP, 0);
        let oldest_before = sd.history[0].time_s;

        let base_t = 100_000;
        for i in 0..MAX_INTERVAL {
            update(
                &mut sd,
                bat_reading(13.0, 5.0),
                ps_reading(13.0, 3.0),
                base_t + i,
            );
        }

        assert_eq!(sd.history.len(), CAP);
        assert_eq!(sd.interval, MAX_INTERVAL);
        assert!(sd.history[0].time_s > oldest_before);
        assert!((sd.history.last().unwrap().battery_current - 5.0).abs() < 0.01);
    }

    #[test]
    fn transition_from_compaction_to_dropping() {
        let mut sd = new_sd();
        sd.interval = MAX_INTERVAL / 2;
        push_samples(&mut sd, CAP, 0);

        sd.compact_if_needed();
        assert_eq!(sd.history.len(), HALF);
        assert_eq!(sd.interval, MAX_INTERVAL);

        let first_after_compact = sd.history[0].time_s;
        push_samples(&mut sd, HALF, CAP as u32);

        sd.compact_if_needed();

        assert_eq!(sd.history.len(), CAP - 1);
        assert_eq!(sd.interval, MAX_INTERVAL);
        assert!(sd.history[0].time_s > first_after_compact);
    }

    // --- Producer-independence (F1): a dead sensor must not halt history ---

    #[test]
    fn battery_only_still_commits_with_ps_zeros() {
        let mut sd = new_sd();
        for i in 0..10u32 {
            sd.update_battery(bat_reading(13.0, 1.5));
            sd.tick(Some(100 + i));
        }
        assert_eq!(sd.history.len(), 10);
        for s in sd.history() {
            assert!((s.battery_current - 1.5).abs() < 0.001);
            assert!(s.ps_current.abs() < 0.001);
            assert!(s.power_online.abs() < 0.001);
        }
    }

    #[test]
    fn ps_goes_stale_after_threshold() {
        let mut sd = new_sd();
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.5));
        sd.tick(Some(1000));
        assert_eq!(sd.history.len(), 1);
        assert!((sd.history[0].ps_current - 2.5).abs() < 0.001);
        assert!((sd.history[0].power_online - 1.0).abs() < 0.001);
        assert!(sd.ps_reading().is_some());

        // STALE_TICKS - 1 more ticks → stale counter exactly at STALE_TICKS.
        for i in 1..STALE_TICKS {
            sd.update_battery(bat_reading(13.0, 1.0));
            sd.tick(Some(1000 + i));
        }
        assert!(sd.ps_reading().is_some(), "PS still fresh at STALE_TICKS");

        // One more tick → past the boundary → zeros.
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick(Some(1000 + STALE_TICKS));
        assert!(sd.ps_reading().is_none());
        let latest = sd.history.last().unwrap();
        assert!(latest.ps_current.abs() < 0.001);
        assert!(latest.power_online.abs() < 0.001);
    }

    #[test]
    fn battery_stale_commits_zeros() {
        let mut sd = new_sd();
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.tick(Some(2000));
        assert_eq!(sd.history.len(), 1);
        assert!((sd.history[0].voltage - 13.0).abs() < 0.001);

        let ticks = STALE_TICKS + 3;
        for i in 1..=ticks {
            sd.update_ps(ps_reading(13.0, 2.0));
            sd.tick(Some(2000 + i));
        }
        assert_eq!(sd.history.len(), 1 + ticks as usize);
        assert!(sd.battery_reading().is_none());
        let latest = sd.history.last().unwrap();
        assert!(latest.voltage.abs() < 0.001);
        assert!(latest.battery_current.abs() < 0.001);
        assert!((latest.ps_current - 2.0).abs() < 0.001);
    }
}
