//! Wall-clock source backed by ESP-IDF's SNTP-driven system time.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// Plausibility bounds on the system clock. The SNTP callback fires even when
/// the system time hasn't actually been set to something sensible (e.g. a
/// poisoned reply, a captive-portal NTP spoof, or a pre-sync tick), and once
/// we commit a bogus epoch value it poisons every later sample.
pub const VALID_EPOCH_S: std::ops::Range<u64> = 1_700_000_000..4_102_444_800;

/// Cheap-to-clone wall-clock source. `epoch_s` returns `None` until the SNTP
/// callback validates a real time and calls `mark_synced`.
#[derive(Clone)]
pub struct EspClock {
    ntp_synced: Arc<AtomicBool>,
}

impl EspClock {
    pub fn new() -> Self {
        Self {
            ntp_synced: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn mark_synced(&self) {
        self.ntp_synced.store(true, Ordering::Relaxed);
    }

    pub fn epoch_s(&self) -> Option<u32> {
        if !self.ntp_synced.load(Ordering::Relaxed) {
            return None;
        }
        let t = esp_idf_svc::systime::EspSystemTime.now().as_secs();
        if !VALID_EPOCH_S.contains(&t) {
            return None;
        }
        Some(t as u32)
    }
}
