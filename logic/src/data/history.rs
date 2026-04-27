//! Adaptive-resolution history ring + on-flash codec.
//!
//! Stores up to `HISTORY_CAPACITY` samples. Raw samples flow in via
//! `commit`; once `interval` of them have accumulated, an averaged sample
//! is pushed onto the history. When the buffer is full, pairs of samples
//! are averaged together (halving the count, doubling the interval),
//! giving exponentially growing time coverage in fixed memory. Once the
//! interval reaches `MAX_INTERVAL`, oldest samples are dropped instead.
//!
//! Wire layout (little-endian throughout):
//!
//! ```text
//! [0..4]   FORMAT_VERSION (u32)
//! [4..8]   interval       (u32)   — sampling interval at save time
//! [8..12]  count          (u32)   — number of samples that follow
//! [12..]   count × Sample        — { time_s u32, v f32, c1 f32, c2 f32, online f32 }
//! ```
//!
//! Bumping `FORMAT_VERSION` invalidates older blobs (`deserialize` returns
//! `false`); the firmware then logs a warning and starts fresh — known-good
//! behavior on every persisted-format change.

use super::Sample;

/// Upper bound on the serialized history blob — also the in-memory scratch
/// size.
pub const SERIALIZED_MAX_BYTES: usize = 4096;
const HEADER_SIZE: usize = 4 + 4 + 4; // version + interval + count
const SAMPLE_SIZE: usize = 4 + 4 * 4; // u32 + 4×f32 = 20 bytes
/// Max samples that fit in `SERIALIZED_MAX_BYTES`. 204 × 4 s ≈ 13.6 min
/// of history.
pub const HISTORY_CAPACITY: usize = (SERIALIZED_MAX_BYTES - HEADER_SIZE) / SAMPLE_SIZE / 2 * 2;
/// Once interval reaches this, drop old samples instead of compacting
/// further. 204 samples × 4 s ≈ 13.6 min.
const MAX_INTERVAL: u32 = 4;

const FORMAT_VERSION: u32 = 7;

const _: () = assert!(
    HISTORY_CAPACITY.is_multiple_of(2),
    "HISTORY_CAPACITY must be even"
);
const _: () = assert!(
    HEADER_SIZE + HISTORY_CAPACITY * SAMPLE_SIZE <= SERIALIZED_MAX_BYTES,
    "serialized history must fit in SERIALIZED_MAX_BYTES"
);

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

pub struct History {
    samples: heapless::Vec<Sample, HISTORY_CAPACITY>,
    /// Current sampling interval: how many raw samples per stored entry.
    /// Starts at 1, doubles on each compaction.
    interval: u32,
    acc: SampleAccum,
    acc_count: u32,
}

impl Default for History {
    fn default() -> Self {
        Self::new()
    }
}

impl History {
    pub fn new() -> Self {
        Self {
            samples: heapless::Vec::new(),
            interval: 1,
            acc: SampleAccum::default(),
            acc_count: 0,
        }
    }

    /// Borrow the history buffer. Always at most `HISTORY_CAPACITY` entries.
    pub fn samples(&self) -> &[Sample] {
        &self.samples
    }

    pub fn interval(&self) -> u32 {
        self.interval
    }

    pub fn last_time(&self) -> Option<u32> {
        self.samples.last().map(|s| s.time_s)
    }

    /// Feed one raw sample into the pipeline. Accumulates; when the
    /// running count reaches `interval`, averages and pushes onto the
    /// history (compacting first if needed).
    pub fn commit(&mut self, raw: Sample) {
        self.acc.add(&raw);
        self.acc_count += 1;

        if self.acc_count >= self.interval {
            let averaged = self.acc.average(self.acc_count, raw.time_s);
            self.acc = SampleAccum::default();
            self.acc_count = 0;

            self.compact_if_needed();
            assert!(self.samples.push(averaged).is_ok(), "history overflow");
        }
    }

    fn compact_if_needed(&mut self) {
        if self.samples.len() < HISTORY_CAPACITY {
            return;
        }
        if self.interval >= MAX_INTERVAL {
            // At max interval (~13.6 min of history) — drop oldest sample to
            // make room for the next push.
            self.samples.remove(0);
            return;
        }
        let len = self.samples.len();
        let half = len / 2;
        for i in 0..half {
            let a = self.samples[2 * i];
            let b = self.samples[2 * i + 1];
            self.samples[i] = Sample {
                time_s: b.time_s,
                voltage: (a.voltage + b.voltage) / 2.0,
                battery_current: (a.battery_current + b.battery_current) / 2.0,
                ps_current: (a.ps_current + b.ps_current) / 2.0,
                power_online: (a.power_online + b.power_online) / 2.0,
            };
        }
        self.samples.truncate(half);
        self.interval *= 2;
    }
}

// --- Codec ------------------------------------------------------------------

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

/// Serialize history + metadata into the caller-provided buffer; returns
/// the number of bytes written. `out` must hold at least
/// `HEADER_SIZE + samples.len() * SAMPLE_SIZE` bytes — passing a
/// `[u8; SERIALIZED_MAX_BYTES]` always satisfies that.
pub fn serialize_into(history: &History, out: &mut [u8]) -> usize {
    let samples = history.samples();
    let total = HEADER_SIZE + samples.len() * SAMPLE_SIZE;
    assert!(
        out.len() >= total,
        "serialize_into: buffer too small ({} < {})",
        out.len(),
        total
    );
    out[0..4].copy_from_slice(&FORMAT_VERSION.to_le_bytes());
    out[4..8].copy_from_slice(&history.interval().to_le_bytes());
    out[8..12].copy_from_slice(&(samples.len() as u32).to_le_bytes());
    let mut o = HEADER_SIZE;
    for s in samples {
        out[o..o + 4].copy_from_slice(&s.time_s.to_le_bytes());
        out[o + 4..o + 8].copy_from_slice(&s.voltage.to_le_bytes());
        out[o + 8..o + 12].copy_from_slice(&s.battery_current.to_le_bytes());
        out[o + 12..o + 16].copy_from_slice(&s.ps_current.to_le_bytes());
        out[o + 16..o + 20].copy_from_slice(&s.power_online.to_le_bytes());
        o += SAMPLE_SIZE;
    }
    total
}

/// Restore a `History` from a byte slice into the caller's slot.
/// Returns `false` on malformed input (wrong version, zero count/interval,
/// truncated payload) and leaves `history` untouched in that case.
/// Truncates to the newest `HISTORY_CAPACITY` if the blob is larger.
///
/// Takes `&mut History` rather than returning by value because `History`
/// is ~4 KB and the firmware's `main` task only gets ~12 KB of stack —
/// a temporary on the return path overflows.
pub fn deserialize(bytes: &[u8], history: &mut History) -> bool {
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

    history.samples.clear();
    history.interval = interval;
    history.acc = SampleAccum::default();
    history.acc_count = 0;
    let skip = count.saturating_sub(HISTORY_CAPACITY);
    for i in 0..count {
        let sample = r.sample();
        if i >= skip {
            assert!(history.samples.push(sample).is_ok(), "history overflow");
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    const CAP: usize = HISTORY_CAPACITY;
    const HALF: usize = CAP / 2;

    /// Push n raw samples through `commit` (interval=1 → one history row each).
    fn fill(h: &mut History, n: u32, start_t: u32) -> u32 {
        for i in 0..n {
            h.commit(Sample {
                time_s: start_t + i,
                voltage: 13.0,
                battery_current: 1.0,
                ps_current: 2.0,
                power_online: 1.0,
            });
        }
        start_t + n
    }

    /// Directly push samples into the buffer, bypassing `commit`'s
    /// accumulator. Used by max-interval tests that need a pre-filled
    /// buffer at a chosen interval.
    fn push_direct(h: &mut History, n: usize, start_t: u32) {
        for i in 0..n {
            assert!(
                h.samples
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
    fn no_compaction_below_capacity() {
        let mut h = History::new();
        fill(&mut h, CAP as u32 - 1, 0);
        assert_eq!(h.samples.len(), CAP - 1);
        assert_eq!(h.interval, 1);
    }

    #[test]
    fn compaction_at_capacity() {
        let mut h = History::new();
        fill(&mut h, CAP as u32 + 1, 0);
        assert_eq!(h.samples.len(), HALF + 1);
        assert_eq!(h.interval, 2);
    }

    #[test]
    fn compaction_averages_all_fields_and_uses_later_timestamp() {
        let mut h = History::new();
        for i in 0..(CAP as u32 + 1) {
            let t = i * 10;
            let (v, c1, c2) = if i % 2 == 0 {
                (12.0, 1.0, 2.0)
            } else {
                (14.0, 3.0, 4.0)
            };
            h.commit(Sample {
                time_s: t,
                voltage: v,
                battery_current: c1,
                ps_current: c2,
                power_online: 1.0,
            });
        }
        assert_eq!(h.samples.len(), HALF + 1);
        assert_eq!(h.interval, 2);

        // Pairs (12,1,2) + (14,3,4) average to (13,2,3); all halves are
        // exact in f32 so equality is appropriate.
        for s in &h.samples[..HALF] {
            assert_eq!(s.voltage, 13.0);
            assert_eq!(s.battery_current, 2.0);
            assert_eq!(s.ps_current, 3.0);
        }
        assert_eq!(h.samples[0].time_s, 10);
        assert_eq!(h.samples[1].time_s, 30);
        assert_eq!(h.samples[HALF - 1].time_s, (CAP as u32 - 1) * 10);
    }

    #[test]
    fn after_compaction_samples_at_new_interval() {
        let mut h = History::new();
        let t = fill(&mut h, CAP as u32 + 1, 0);
        assert_eq!(h.interval, 2);

        h.commit(Sample {
            time_s: t,
            voltage: 13.0,
            battery_current: 5.0,
            ps_current: 0.0,
            power_online: 1.0,
        });
        assert_eq!(h.samples.len(), HALF + 1);

        h.commit(Sample {
            time_s: t + 1,
            voltage: 13.0,
            battery_current: 7.0,
            ps_current: 0.0,
            power_online: 1.0,
        });
        assert_eq!(h.samples.len(), HALF + 2);
        let last = h.samples.last().unwrap();
        assert!((last.battery_current - 6.0).abs() < 0.01);
        assert_eq!(last.time_s, t + 1);
    }

    #[test]
    fn interval_doubles_each_compaction() {
        let mut h = History::new();
        assert_eq!(h.interval, 1);
        fill(&mut h, 820, 0);
        assert_eq!(h.interval, MAX_INTERVAL);
    }

    #[test]
    fn long_run_stays_bounded_and_chronological() {
        let mut h = History::new();
        fill(&mut h, 10000, 0);
        assert!(h.samples.len() <= CAP);
        assert!(h.samples.len() >= HALF);
        for i in 1..h.samples.len() {
            assert!(
                h.samples[i].time_s >= h.samples[i - 1].time_s,
                "not chronological at {}: {} < {}",
                i,
                h.samples[i].time_s,
                h.samples[i - 1].time_s
            );
        }
    }

    #[test]
    fn at_max_interval_drops_oldest_via_commit() {
        let mut h = History::new();
        h.interval = MAX_INTERVAL;
        push_direct(&mut h, CAP, 0);
        let oldest_before = h.samples[0].time_s;

        let base_t = 100_000;
        for i in 0..MAX_INTERVAL {
            h.commit(Sample {
                time_s: base_t + i,
                voltage: 13.0,
                battery_current: 5.0,
                ps_current: 3.0,
                power_online: 1.0,
            });
        }

        assert_eq!(h.samples.len(), CAP);
        assert_eq!(h.interval, MAX_INTERVAL);
        assert!(h.samples[0].time_s > oldest_before);
        assert!((h.samples.last().unwrap().battery_current - 5.0).abs() < 0.01);
    }

    #[test]
    fn transition_from_compaction_to_dropping() {
        let mut h = History::new();
        h.interval = MAX_INTERVAL / 2;
        push_direct(&mut h, CAP, 0);

        h.compact_if_needed();
        assert_eq!(h.samples.len(), HALF);
        assert_eq!(h.interval, MAX_INTERVAL);

        let first_after_compact = h.samples[0].time_s;
        push_direct(&mut h, HALF, CAP as u32);

        h.compact_if_needed();

        assert_eq!(h.samples.len(), CAP - 1);
        assert_eq!(h.interval, MAX_INTERVAL);
        assert!(h.samples[0].time_s > first_after_compact);
    }

    #[test]
    #[should_panic(expected = "cannot average zero samples")]
    fn sample_accum_average_panics_on_zero() {
        let acc = SampleAccum::default();
        acc.average(0, 0);
    }

    // --- Codec ---

    fn serialize_to_vec(history: &History) -> Vec<u8> {
        let mut buf = vec![0u8; SERIALIZED_MAX_BYTES];
        let n = serialize_into(history, &mut buf);
        buf.truncate(n);
        buf
    }

    fn header_blob(version: u32, interval: u32, count: u32, total_len: usize) -> Vec<u8> {
        let mut out = vec![0u8; total_len];
        out[0..4].copy_from_slice(&version.to_le_bytes());
        out[4..8].copy_from_slice(&interval.to_le_bytes());
        out[8..12].copy_from_slice(&count.to_le_bytes());
        out
    }

    fn filled_history(n: u32, start_t: u32) -> History {
        let mut h = History::new();
        for i in 0..n {
            h.commit(Sample {
                time_s: start_t + i,
                voltage: 13.0,
                battery_current: 1.0,
                ps_current: 2.0,
                power_online: 1.0,
            });
        }
        h
    }

    fn fresh() -> History {
        History::new()
    }

    #[test]
    fn write_read_roundtrip_empty() {
        let h = History::new();
        let blob = serialize_to_vec(&h);
        assert_eq!(blob.len(), HEADER_SIZE);
        // Empty blob (count=0) is rejected — no useful state to restore.
        let mut out = fresh();
        assert!(!deserialize(&blob, &mut out));
    }

    #[test]
    fn write_read_roundtrip() {
        let h = filled_history(10, 1000);
        let blob = serialize_to_vec(&h);
        assert_eq!(blob.len(), HEADER_SIZE + 10 * SAMPLE_SIZE);

        let mut h2 = fresh();
        assert!(deserialize(&blob, &mut h2));
        assert_eq!(h2.samples().len(), 10);
        assert_eq!(h2.interval(), 1);
        assert_eq!(h2.samples()[0].time_s, 1000);
        assert_eq!(h2.samples()[9].time_s, 1009);
    }

    #[test]
    fn read_rejects_truncated() {
        let mut out = fresh();
        assert!(!deserialize(&[0u8; 10], &mut out));
    }

    #[test]
    fn read_rejects_zero_interval() {
        let blob = header_blob(FORMAT_VERSION, 0, 0, HEADER_SIZE);
        let mut out = fresh();
        assert!(!deserialize(&blob, &mut out));
    }

    #[test]
    fn read_rejects_wrong_version() {
        let blob = header_blob(99, 1, 0, HEADER_SIZE);
        let mut out = fresh();
        assert!(!deserialize(&blob, &mut out));
    }

    #[test]
    fn read_rejects_count_without_enough_data() {
        let blob = header_blob(FORMAT_VERSION, 1, HISTORY_CAPACITY as u32 + 1, HEADER_SIZE);
        let mut out = fresh();
        assert!(!deserialize(&blob, &mut out));
    }

    #[test]
    fn read_rejects_truncated_samples() {
        let blob = header_blob(FORMAT_VERSION, 1, 10, HEADER_SIZE + 5 * SAMPLE_SIZE);
        let mut out = fresh();
        assert!(!deserialize(&blob, &mut out));
    }

    #[test]
    fn read_single_sample() {
        let mut blob = header_blob(FORMAT_VERSION, 1, 1, HEADER_SIZE + SAMPLE_SIZE);
        blob[12..16].copy_from_slice(&1000u32.to_le_bytes());
        blob[16..20].copy_from_slice(&13.0f32.to_le_bytes());
        blob[20..24].copy_from_slice(&1.0f32.to_le_bytes());
        blob[24..28].copy_from_slice(&2.0f32.to_le_bytes());
        blob[28..32].copy_from_slice(&1.0f32.to_le_bytes());

        let mut h = fresh();
        assert!(deserialize(&blob, &mut h));
        assert_eq!(h.samples().len(), 1);
        assert_eq!(h.samples()[0].time_s, 1000);
        assert!((h.samples()[0].voltage - 13.0).abs() < 0.001);
    }

    #[test]
    fn load_from_bytes_resets_running_accumulator() {
        use crate::data::{Ina228Reading, PsReading, SensorData};

        // Two raw commits at interval=2 before save: leaves acc_count=0
        // (one history row pushed, fresh acc). To probe "load resets acc"
        // we save at interval=2 then re-load on a SensorData whose acc has
        // been dirtied by a partial run, and confirm post-load behavior
        // matches "acc cleared".
        let mut h = History::new();
        // Force interval=2 the legitimate way (CAP+1 commits at i=1
        // triggers compaction). Then snapshot.
        for i in 0..(HISTORY_CAPACITY as u32 + 1) {
            h.commit(Sample {
                time_s: i,
                voltage: 13.0,
                battery_current: 1.0,
                ps_current: 2.0,
                power_online: 1.0,
            });
        }
        assert_eq!(h.interval(), 2);
        let blob = serialize_to_vec(&h);

        let mut sd = SensorData::new();
        assert!(sd.load_from_bytes(&blob));
        assert_eq!(sd.interval(), 2);

        // Post-load: at interval=2, the first commit must accumulate
        // (no row yet), the second commit must push. If load forgot to
        // reset acc_count, the first commit could push prematurely.
        let base_len = sd.history().len();
        sd.update_battery(Ina228Reading {
            voltage: 13.0,
            current: 1.0,
            power: 0.0,
        });
        sd.update_ps(PsReading {
            voltage: 13.0,
            current: 2.0,
            power: 0.0,
        });
        sd.tick(Some(5000));
        assert_eq!(
            sd.history().len(),
            base_len,
            "first post-load tick should accumulate, not push"
        );

        sd.update_battery(Ina228Reading {
            voltage: 13.0,
            current: 3.0,
            power: 0.0,
        });
        sd.update_ps(PsReading {
            voltage: 13.0,
            current: 4.0,
            power: 0.0,
        });
        sd.tick(Some(5001));
        assert_eq!(sd.history().len(), base_len + 1);
        assert!((sd.history().last().unwrap().battery_current - 2.0).abs() < 0.01);
    }
}
