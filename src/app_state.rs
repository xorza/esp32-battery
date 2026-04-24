//! Shared application state. One `Arc<AppState>` is cloned into every thread —
//! replaces the prior module-level `static AtomicBool` globals.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use esp32_battery_logic::data::SensorData;

use crate::platform::EspPlatform;

pub struct AppState {
    pub sensor_data: Mutex<SensorData<EspPlatform>>,
    captive_portal_active: AtomicBool,
    /// Set by the SNTP callback once system time has been synchronized.
    /// Shared with `EspPlatform` so `epoch_s()` returns `None` until sync.
    pub ntp_synced: Arc<AtomicBool>,
}

impl AppState {
    pub fn new(ntp_synced: Arc<AtomicBool>, sensor_data: SensorData<EspPlatform>) -> Arc<Self> {
        Arc::new(Self {
            sensor_data: Mutex::new(sensor_data),
            captive_portal_active: AtomicBool::new(false),
            ntp_synced,
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
