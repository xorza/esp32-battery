//! NVS-backed persistence for the serialized `SensorData` history blob.
//! Owned externally to `SensorData` so flash I/O happens outside the data
//! mutex — producer threads never stall on the 50–100 ms erase/write.

use std::sync::Mutex;

use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};

use esp32_battery_logic::data::SERIALIZED_MAX_BYTES;

const NAMESPACE: &str = "data";
const NVS_KEY: &str = "hist";

/// NVS handle plus a single 4 KiB scratch buffer reused across save and
/// load. Boxing keeps `HistoryStore` a thin pointer on stack while the
/// buffer lives on the heap once at boot — no per-call allocations.
struct Inner {
    nvs: EspNvs<NvsDefault>,
    buf: [u8; SERIALIZED_MAX_BYTES],
}

pub struct HistoryStore {
    inner: Mutex<Inner>,
}

impl HistoryStore {
    pub fn new(partition: EspDefaultNvsPartition) -> Self {
        Self {
            inner: Mutex::new(Inner {
                nvs: EspNvs::new(partition, NAMESPACE, true).unwrap(),
                buf: [0u8; SERIALIZED_MAX_BYTES],
            }),
        }
    }

    /// Fill the scratch buffer via `fill` (which returns the byte count it
    /// wrote) and write that prefix to NVS. The closure runs under the
    /// store's mutex — callers can take additional locks (e.g. SensorData)
    /// inside it; release them before the closure returns so the slow
    /// flash write isn't gated by them.
    pub fn save_with<F: FnOnce(&mut [u8]) -> usize>(&self, fill: F) {
        let mut inner = self.inner.lock().unwrap();
        let n = fill(&mut inner.buf);
        // Erase first to free NVS pages — overwriting a blob requires
        // space for both old and new copies simultaneously, which exhausts
        // our 16 KiB partition.
        let _ = inner.nvs.remove(NVS_KEY);
        let inner = &mut *inner;
        if let Err(e) = inner.nvs.set_blob(NVS_KEY, &inner.buf[..n]) {
            log::warn!("HistoryStore save failed: {e}");
        }
    }

    /// Load the persisted blob into the scratch buffer and hand it to
    /// `consume` as a borrowed slice. Calls `consume` only when a blob
    /// was found and read successfully.
    pub fn load_with<F: FnOnce(&[u8])>(&self, consume: F) {
        let mut inner = self.inner.lock().unwrap();
        let inner = &mut *inner;
        match inner.nvs.get_blob(NVS_KEY, &mut inner.buf) {
            Ok(Some(data)) => consume(data),
            Ok(None) => {}
            Err(e) => log::warn!("HistoryStore load failed: {e}"),
        }
    }
}
