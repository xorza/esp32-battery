use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use esp32_battery_logic::data::Platform;

const NAMESPACE: &str = "data";
const NVS_KEY: &str = "hist";

pub struct EspPlatform {
    nvs: EspNvs<NvsDefault>,
    ntp_synced: Arc<AtomicBool>,
}

impl EspPlatform {
    pub fn new(partition: EspDefaultNvsPartition, ntp_synced: Arc<AtomicBool>) -> Self {
        Self {
            nvs: EspNvs::new(partition, NAMESPACE, true).unwrap(),
            ntp_synced,
        }
    }
}

impl Platform for EspPlatform {
    fn epoch_s(&self) -> Option<u32> {
        if !self.ntp_synced.load(Ordering::Relaxed) {
            return None;
        }
        Some(esp_idf_svc::systime::EspSystemTime.now().as_secs() as u32)
    }

    fn save_blob(&self, data: &[u8]) {
        // Erase first to free NVS pages — overwriting a blob requires space
        // for both old and new copies simultaneously, which exhausts our 16KB partition.
        let _ = self.nvs.remove(NVS_KEY);
        if let Err(e) = self.nvs.set_blob(NVS_KEY, data) {
            log::warn!("Failed to save history: {}", e);
        }
    }

    fn load_blob(&self, buf: &mut [u8]) -> Option<usize> {
        match self.nvs.get_blob(NVS_KEY, buf) {
            Ok(Some(data)) => Some(data.len()),
            _ => None,
        }
    }
}
