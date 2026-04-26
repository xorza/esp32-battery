//! NVS-backed persistence for the serialized `SensorData` history blob.
//! Owned externally to `SensorData` so flash I/O happens outside the data
//! mutex — producer threads never stall on the 50–100 ms erase/write.

use std::sync::Mutex;

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

const NAMESPACE: &str = "data";
const NVS_KEY: &str = "hist";

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

    pub fn load(&self, buf: &mut [u8]) -> Option<usize> {
        let nvs = self.nvs.lock().unwrap();
        match nvs.get_blob(NVS_KEY, buf) {
            Ok(Some(data)) => Some(data.len()),
            Ok(None) => None,
            Err(e) => {
                log::warn!("HistoryStore load failed: {e}");
                None
            }
        }
    }
}
