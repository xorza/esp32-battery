//! Shared application state. One `Arc<AppState>` is cloned into every thread.

use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use esp32_battery_logic::data::SensorData;

use crate::nvs_creds::WifiCredentials;
use crate::platform::{EspClock, HistoryStore};

pub struct AppState {
    pub sensor_data: Mutex<SensorData<EspClock>>,
    pub history_store: HistoryStore,
    captive_portal_active: AtomicBool,
    /// Set by the captive portal `/save` handler when fresh credentials land.
    /// Drained by the main loop, which then drives the live STA reconnect.
    pub pending_creds: Mutex<Option<WifiCredentials>>,
}

impl AppState {
    pub fn new(sensor_data: SensorData<EspClock>, history_store: HistoryStore) -> std::sync::Arc<Self> {
        std::sync::Arc::new(Self {
            sensor_data: Mutex::new(sensor_data),
            history_store,
            captive_portal_active: AtomicBool::new(false),
            pending_creds: Mutex::new(None),
        })
    }

    pub fn set_captive(&self, active: bool) {
        self.captive_portal_active.store(active, Ordering::Relaxed);
    }

    pub fn is_captive(&self) -> bool {
        self.captive_portal_active.load(Ordering::Relaxed)
    }
}

pub fn uptime_s() -> u32 {
    (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000) as u32
}
