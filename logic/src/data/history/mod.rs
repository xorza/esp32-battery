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
mod tests;
