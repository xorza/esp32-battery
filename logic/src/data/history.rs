//! Adaptive-resolution history ring.
//!
//! Stores up to `HISTORY_CAPACITY` samples. Raw samples flow in via
//! `commit`; once `interval` of them have accumulated, an averaged sample
//! is pushed onto the history. When the buffer is full, pairs of samples
//! are averaged together (halving the count, doubling the interval),
//! giving exponentially growing time coverage in fixed memory. Once the
//! interval reaches `MAX_INTERVAL`, oldest samples are dropped instead.

use super::Sample;

/// 204 samples × 4 s ≈ 13.6 min of history at max interval.
pub const HISTORY_CAPACITY: usize = 204;
/// Once interval reaches this, drop old samples instead of compacting
/// further.
const MAX_INTERVAL: u32 = 4;

const _: () = assert!(
    HISTORY_CAPACITY.is_multiple_of(2),
    "HISTORY_CAPACITY must be even"
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

}
