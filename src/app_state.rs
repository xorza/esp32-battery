//! Application state. One `AppState` value lives on the main thread for the
//! life of the program. Worker threads (xy, ina, lcd, http) get an
//! `Arc<Shared>` clone — the cross-thread, `Send + Sync` subset.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, Ordering};

use esp_idf_svc::http::server::EspHttpServer;

use esp32_battery_logic::data::SensorData;

use crate::dns::DnsHandle;
use crate::nvs_creds::WifiCredentials;
use crate::platform::{EspClock, HistoryStore};

/// HTTP server held for `Drop`. Two shapes — captive portal pairs HTTP with
/// a DNS hijack so the device's own SSID resolves; host mode is HTTPS only.
/// `EspHttpServer` is `!Send`, which is why this lives in `AppState` (main
/// thread only) rather than `Shared`.
pub enum Server {
    Captive {
        #[allow(dead_code)]
        http: EspHttpServer<'static>,
        #[allow(dead_code)]
        dns: DnsHandle,
    },
    Host {
        #[allow(dead_code)]
        http: EspHttpServer<'static>,
    },
}

impl Server {
    pub fn kind(&self) -> ServerKind {
        match self {
            Server::Captive { .. } => ServerKind::Captive,
            Server::Host { .. } => ServerKind::Host,
        }
    }
}

/// Three-state classifier for the active server: nothing yet, captive portal,
/// or host dashboard. Used by the main loop to decide swaps without juggling
/// `Option<bool>`.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ServerKind {
    None,
    Captive,
    Host,
}

/// Cross-thread subset. Cloned (as `Arc<Shared>`) into every worker thread.
pub struct Shared {
    pub sensor_data: Mutex<SensorData<EspClock>>,
    /// Set by the captive `/save` handler when fresh credentials land.
    /// Drained by the main loop, which then drives the live STA reconnect.
    pub pending_creds: Mutex<Option<WifiCredentials>>,
    /// Send-projection of `AppState::server`'s variant. Written only via
    /// `AppState::set_server`; the field is private so no other code can
    /// desync the mirror.
    captive_active: AtomicBool,
}

impl Shared {
    pub fn is_captive(&self) -> bool {
        self.captive_active.load(Ordering::Relaxed)
    }
}

pub struct AppState {
    pub shared: Arc<Shared>,
    pub history_store: HistoryStore,
    /// Private — only `set_server` can write it, which keeps the
    /// `Shared::captive_active` mirror in sync.
    server: Option<Server>,
}

impl AppState {
    pub fn new(sensor_data: SensorData<EspClock>, history_store: HistoryStore) -> Self {
        Self {
            shared: Arc::new(Shared {
                sensor_data: Mutex::new(sensor_data),
                pending_creds: Mutex::new(None),
                captive_active: AtomicBool::new(false),
            }),
            history_store,
            server: None,
        }
    }

    pub fn server_kind(&self) -> ServerKind {
        self.server.as_ref().map_or(ServerKind::None, Server::kind)
    }

    /// Single write path for `server` and the cross-thread mirror.
    pub fn set_server(&mut self, new: Option<Server>) {
        let captive = matches!(new, Some(Server::Captive { .. }));
        self.shared.captive_active.store(captive, Ordering::Relaxed);
        self.server = new;
    }
}

pub fn uptime_s() -> u32 {
    (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000) as u32
}
