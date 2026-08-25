//! Adaptive-resolution history ring.
//!
//! Stores up to `HISTORY_CAPACITY` samples. Raw samples flow in via
//! `commit`; once `interval` of them have accumulated, an averaged sample
//! is pushed onto the history. When the buffer is full, pairs of samples
//! are averaged together (halving the count, doubling the interval),
//! giving exponentially growing time coverage in fixed memory. Once the
//! interval reaches `MAX_INTERVAL`, oldest samples are dropped instead.

use super::Sample;

pub const HISTORY_CAPACITY: usize = 204;

/// Once the interval reaches this, the window slides (oldest sample dropped)
/// instead of coarsening further. 204 samples × 64 s ≈ 3.6 h, which covers a
/// full `MAX_ABSORB` charge cycle with margin — a chart that cannot span one
/// absorb cannot show the taper that ends it.
const MAX_INTERVAL: u32 = 64;

const _: () = assert!(
    HISTORY_CAPACITY.is_multiple_of(2),
    "HISTORY_CAPACITY must be even"
);
const _: () = assert!(
    MAX_INTERVAL.is_power_of_two(),
    "the interval doubles from 1, so the cap has to be a power of two to be hit exactly"
);

#[derive(Default, Debug)]
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

    /// Mean of the `n` samples added so far, stamped `time_s`. Used both for
    /// the running accumulator and for pairwise compaction, so the two paths
    /// cannot drift apart on which fields get averaged.
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

#[derive(Debug)]
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

    pub fn last_time(&self) -> Option<u32> {
        self.samples.last().map(|s| s.time_s)
    }

    /// Feed one raw sample into the pipeline. Accumulates; when the
    /// running count reaches `interval`, averages and pushes onto the
    /// history, making room first if the buffer is full.
    pub fn commit(&mut self, raw: Sample) {
        self.acc.add(&raw);
        self.acc_count += 1;
        if self.acc_count < self.interval {
            return;
        }

        let averaged = self.acc.average(self.acc_count, raw.time_s);
        self.acc = SampleAccum::default();
        self.acc_count = 0;

        if self.samples.is_full() {
            if self.interval < MAX_INTERVAL {
                self.compact();
            } else {
                // At the cap the window slides rather than coarsening: one
                // sample's worth of memmove per `MAX_INTERVAL` seconds buys a
                // contiguous buffer for every reader and a chart whose left
                // edge advances smoothly instead of in jumps.
                self.samples.remove(0);
            }
        }
        self.samples
            .push(averaged)
            .expect("buffer has room after compacting or dropping");
    }

    /// Halve the buffer by averaging adjacent pairs, doubling the interval
    /// each entry now represents. Time coverage grows exponentially in fixed
    /// memory; resolution is what pays for it.
    fn compact(&mut self) {
        let half = self.samples.len() / 2;
        for i in 0..half {
            let mut pair = SampleAccum::default();
            pair.add(&self.samples[2 * i]);
            pair.add(&self.samples[2 * i + 1]);
            // The later of the two stamps: like an accumulated sample, a
            // compacted one is labelled by the end of the span it covers.
            let time_s = self.samples[2 * i + 1].time_s;
            self.samples[i] = pair.average(2, time_s);
        }
        self.samples.truncate(half);
        self.interval *= 2;
    }
}

/// The compaction interval, for tests outside this module. `history`'s own
/// tests read the field directly; `data`'s cannot, and nothing in production
/// needs to know the interval.
#[cfg(test)]
pub(crate) mod internals {
    use crate::data::history::History;

    pub(crate) trait HistoryInternals {
        fn interval(&self) -> u32;
    }

    impl HistoryInternals for History {
        fn interval(&self) -> u32 {
            self.interval
        }
    }
}

#[cfg(test)]
mod tests;
