//! Firmware-side clock + NVS glue for the logic crate. The logic crate only
//! depends on the `Clock` trait; persistence is a plain firmware concern
//! handled by `HistoryStore` and driven from the producer threads outside the
//! `SensorData` mutex.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use esp32_battery_logic::data::Clock;

const NAMESPACE: &str = "data";
const NVS_KEY: &str = "hist";

/// Plausibility bounds on the system clock. The SNTP callback fires even when
/// the system time hasn't actually been set to something sensible (e.g. a
/// poisoned reply, a captive-portal NTP spoof, or a pre-sync tick), and once
/// we commit a bogus epoch value it poisons every later sample.
pub const VALID_EPOCH_S: std::ops::Range<u64> = 1_700_000_000..4_102_444_800;

/// Cheap-to-clone wall-clock source. `epoch_s` returns `None` until the SNTP
/// callback validates a real time and calls `mark_synced`.
#[derive(Clone)]
pub struct EspClock {
    ntp_synced: std::sync::Arc<AtomicBool>,
}

impl EspClock {
    pub fn new() -> Self {
        Self {
            ntp_synced: std::sync::Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn mark_synced(&self) {
        self.ntp_synced.store(true, Ordering::Relaxed);
    }
}

impl Clock for EspClock {
    fn epoch_s(&self) -> Option<u32> {
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

/// NVS-backed history persistence. Owned externally to `SensorData` so save
/// I/O happens outside the data mutex.
pub struct HistoryStore {
    nvs: Mutex<EspNvs<NvsDefault>>,
}

impl HistoryStore {
    pub fn new(partition: EspDefaultNvsPartition) -> Self {
        Self {
            nvs: Mutex::new(EspNvs::new(partition, NAMESPACE, true).unwrap()),
        }
    }

    pub fn save(&self, data: &[u8]) {
        let nvs = self.nvs.lock().unwrap();
        // Erase first to free NVS pages — overwriting a blob requires space
        // for both old and new copies simultaneously, which exhausts our 16 KB partition.
        let _ = nvs.remove(NVS_KEY);
        if let Err(e) = nvs.set_blob(NVS_KEY, data) {
            log::warn!("HistoryStore save failed: {e}");
        }
    }

    /// Load the persisted blob into `buf`. Returns `Some(len)` if present.
    pub fn load(&self, buf: &mut [u8]) -> Option<usize> {
        let nvs = self.nvs.lock().unwrap();
        match nvs.get_blob(NVS_KEY, buf) {
            Ok(Some(data)) => Some(data.len()),
            _ => None,
        }
    }
}
