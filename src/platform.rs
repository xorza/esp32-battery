use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use esp32_battery_logic::data::Platform;

const NAMESPACE: &str = "data";
const NVS_KEY: &str = "hist";

pub struct EspPlatform {
    nvs: EspNvs<NvsDefault>,
}

impl EspPlatform {
    pub fn new(partition: EspDefaultNvsPartition) -> Self {
        Self {
            nvs: EspNvs::new(partition, NAMESPACE, true).unwrap(),
        }
    }
}

impl Platform for EspPlatform {
    fn epoch_s(&self) -> Option<u32> {
        crate::epoch_s()
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
