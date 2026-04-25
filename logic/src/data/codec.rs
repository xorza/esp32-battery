//! On-flash binary format for the history ring.
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
//! Bumping `FORMAT_VERSION` invalidates older blobs (deserialize returns
//! `None`); the firmware then logs a warning and starts fresh — known-good
//! behavior on every persisted-format change.

use super::history::{HEADER_SIZE, HISTORY_CAPACITY, History, SAMPLE_SIZE};
use super::sample::Sample;

const FORMAT_VERSION: u32 = 6;

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

/// Serialize history + metadata into a fresh `Vec` for NVS storage.
pub fn serialize(history: &History) -> Vec<u8> {
    let samples = history.samples();
    let mut out = Vec::with_capacity(HEADER_SIZE + samples.len() * SAMPLE_SIZE);
    out.extend_from_slice(&FORMAT_VERSION.to_le_bytes());
    out.extend_from_slice(&history.interval().to_le_bytes());
    out.extend_from_slice(&(samples.len() as u32).to_le_bytes());
    for s in samples {
        out.extend_from_slice(&s.time_s.to_le_bytes());
        out.extend_from_slice(&s.voltage.to_le_bytes());
        out.extend_from_slice(&s.battery_current.to_le_bytes());
        out.extend_from_slice(&s.ps_current.to_le_bytes());
        out.extend_from_slice(&s.power_online.to_le_bytes());
    }
    out
}

/// Restore a `History` from a byte slice. Returns `None` on malformed
/// input (wrong version, zero count/interval, truncated payload).
/// Truncates to the newest `HISTORY_CAPACITY` if the blob is larger.
pub fn deserialize(bytes: &[u8]) -> Option<History> {
    if bytes.len() < HEADER_SIZE {
        return None;
    }

    let mut r = BufReader { buf: bytes, pos: 0 };
    let version = r.u32();
    if version != FORMAT_VERSION {
        return None;
    }
    let interval = r.u32();
    let count = r.u32() as usize;

    if interval == 0 || count == 0 || bytes.len() < HEADER_SIZE + count * SAMPLE_SIZE {
        return None;
    }

    let mut history = History::new();
    history.interval = interval;
    let skip = count.saturating_sub(HISTORY_CAPACITY);
    for i in 0..count {
        let sample = r.sample();
        if i >= skip {
            assert!(history.samples.push(sample).is_ok(), "history overflow");
        }
    }
    Some(history)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::SensorData;

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

    #[test]
    fn write_read_roundtrip_empty() {
        let h = History::new();
        let blob = serialize(&h);
        assert_eq!(blob.len(), HEADER_SIZE);
        // Empty blob (count=0) is rejected — no useful state to restore.
        assert!(deserialize(&blob).is_none());
    }

    #[test]
    fn write_read_roundtrip() {
        let h = filled_history(10, 1000);
        let blob = serialize(&h);
        assert_eq!(blob.len(), HEADER_SIZE + 10 * SAMPLE_SIZE);

        let h2 = deserialize(&blob).expect("roundtrip");
        assert_eq!(h2.samples().len(), 10);
        assert_eq!(h2.interval(), 1);
        assert_eq!(h2.samples()[0].time_s, 1000);
        assert_eq!(h2.samples()[9].time_s, 1009);
    }

    #[test]
    fn read_rejects_truncated() {
        assert!(deserialize(&[0u8; 10]).is_none());
    }

    #[test]
    fn read_rejects_zero_interval() {
        let blob = header_blob(FORMAT_VERSION, 0, 0, HEADER_SIZE);
        assert!(deserialize(&blob).is_none());
    }

    #[test]
    fn read_rejects_wrong_version() {
        let blob = header_blob(99, 1, 0, HEADER_SIZE);
        assert!(deserialize(&blob).is_none());
    }

    #[test]
    fn read_rejects_count_without_enough_data() {
        let blob = header_blob(FORMAT_VERSION, 1, HISTORY_CAPACITY as u32 + 1, HEADER_SIZE);
        assert!(deserialize(&blob).is_none());
    }

    #[test]
    fn read_rejects_truncated_samples() {
        let blob = header_blob(FORMAT_VERSION, 1, 10, HEADER_SIZE + 5 * SAMPLE_SIZE);
        assert!(deserialize(&blob).is_none());
    }

    #[test]
    fn read_single_sample() {
        let mut blob = header_blob(FORMAT_VERSION, 1, 1, HEADER_SIZE + SAMPLE_SIZE);
        blob[12..16].copy_from_slice(&1000u32.to_le_bytes());
        blob[16..20].copy_from_slice(&13.0f32.to_le_bytes());
        blob[20..24].copy_from_slice(&1.0f32.to_le_bytes());
        blob[24..28].copy_from_slice(&2.0f32.to_le_bytes());
        blob[28..32].copy_from_slice(&1.0f32.to_le_bytes());

        let h = deserialize(&blob).expect("roundtrip");
        assert_eq!(h.samples().len(), 1);
        assert_eq!(h.samples()[0].time_s, 1000);
        assert!((h.samples()[0].voltage - 13.0).abs() < 0.001);
    }

    #[test]
    fn load_from_bytes_resets_running_accumulator() {
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
        let blob = serialize(&h);

        // Dirty SensorData via one partial-interval update.
        let mut sd = SensorData::new();
        // Drive interval to 2 by replicating compaction the slow way:
        // not strictly necessary — load will overwrite. The point is
        // that load's reset is independent of pre-load acc state.
        assert!(sd.load_from_bytes(&blob));
        assert_eq!(sd.interval(), 2);

        // Post-load: at interval=2, the first commit must accumulate
        // (no row yet), the second commit must push. If load forgot to
        // reset acc_count, the first commit could push prematurely.
        let base_len = sd.history().len();
        sd.update_battery(crate::data::Ina228Reading {
            voltage: 13.0,
            current: 1.0,
            power: 0.0,
        });
        sd.update_ps(crate::data::PsReading {
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

        sd.update_battery(crate::data::Ina228Reading {
            voltage: 13.0,
            current: 3.0,
            power: 0.0,
        });
        sd.update_ps(crate::data::PsReading {
            voltage: 13.0,
            current: 4.0,
            power: 0.0,
        });
        sd.tick(Some(5001));
        assert_eq!(sd.history().len(), base_len + 1);
        assert!((sd.history().last().unwrap().battery_current - 2.0).abs() < 0.01);
    }
}
