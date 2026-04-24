//! Application state. One `AppState` value lives on the main thread for the
//! life of the program. Worker threads (xy, ina, lcd, http) get an
//! `Arc<Shared>` clone — the cross-thread, `Send + Sync` subset.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};

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

/// Cross-thread network status — what readers (LCD, etc.) should display.
/// `Connecting` is a transient between Captive (user submitted creds) and
/// Host (STA actually came up). The captive HTTP server is still running
/// during `Connecting` so the user can resubmit if STA never connects.
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum NetStatus {
    Captive = 0,
    Connecting = 1,
    Host = 2,
}

impl NetStatus {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => NetStatus::Captive,
            1 => NetStatus::Connecting,
            2 => NetStatus::Host,
            _ => unreachable!("NetStatus byte out of range"),
        }
    }
}

/// Cross-thread subset. Cloned (as `Arc<Shared>`) into every worker thread.
pub struct Shared {
    pub sensor_data: Mutex<SensorData<EspClock>>,
    /// Set by the captive `/save` handler when fresh credentials land.
    /// Drained by the main loop, which then drives the live STA reconnect.
    pub pending_creds: Mutex<Option<WifiCredentials>>,
    /// Cross-thread net status. Written only via `AppState::set_server`
    /// and `AppState::set_status`; the field is private so callers can't
    /// desync from the actual server.
    status: AtomicU8,
}

impl Shared {
    pub fn status(&self) -> NetStatus {
        NetStatus::from_u8(self.status.load(Ordering::Relaxed))
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
                status: AtomicU8::new(NetStatus::Captive as u8),
            }),
            history_store,
            server: None,
        }
    }

    pub fn server_kind(&self) -> ServerKind {
        self.server.as_ref().map_or(ServerKind::None, Server::kind)
    }

    /// Single write path for `server`. The status mirror tracks the new
    /// server's variant — but `set_status(Connecting)` may immediately
    /// overwrite to surface the post-creds-submit transient.
    pub fn set_server(&mut self, new: Option<Server>) {
        let status = match new {
            Some(Server::Captive { .. }) => NetStatus::Captive,
            Some(Server::Host { .. }) => NetStatus::Host,
            None => NetStatus::Captive,
        };
        self.set_status(status);
        self.server = new;
    }

    pub fn set_status(&self, status: NetStatus) {
        self.shared.status.store(status as u8, Ordering::Relaxed);
    }
}

pub fn uptime_s() -> u32 {
    (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000) as u32
}
