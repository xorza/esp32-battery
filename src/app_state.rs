//! Application state. One `AppState` value lives on the main thread for the
//! life of the program. Worker threads (xy, ina, lcd, http) get an
//! `Arc<Shared>` clone — the cross-thread, `Send + Sync` subset.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};

use esp_idf_svc::http::server::EspHttpServer;
use log::{info, warn};

use esp32_battery_logic::data::SensorData;

use crate::clock::EspClock;
use crate::dns::DnsHandle;
use crate::history_store::HistoryStore;
use crate::nvs_creds::WifiCredentials;

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

/// Cross-thread net status — what readers (LCD, etc.) should display.
/// `Connecting` is a transient: STA has been started (boot with creds, or
/// captive `/save` submission) but hasn't associated yet, OR was previously
/// Host and dropped the link but is still inside the reconnect grace window.
/// The currently-mounted server may not match the status during `Connecting`.
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
    server: Option<Server>,
    reconnect_failures: u32,
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
            reconnect_failures: 0,
        }
    }

    /// Drop the previous server (releasing its resources) and install the new
    /// one. Mirrors the variant into `Shared::status` so cross-thread readers
    /// see the same mode the main thread does.
    pub fn replace_server(&mut self, new: Server) {
        let status = match &new {
            Server::Captive { .. } => NetStatus::Captive,
            Server::Host { .. } => NetStatus::Host,
        };
        self.set_status(status);
        self.server = Some(new);
    }

    pub fn set_status(&self, status: NetStatus) {
        self.shared.status.store(status as u8, Ordering::Relaxed);
    }

    pub fn reset_reconnect_failures(&mut self) {
        self.reconnect_failures = 0;
    }

    /// Increment and return the new count. Saturates so a permanently-down
    /// link can't wrap around and re-enter the grace window.
    pub fn bump_reconnect_failures(&mut self) -> u32 {
        self.reconnect_failures = self.reconnect_failures.saturating_add(1);
        self.reconnect_failures
    }

    /// Idempotent: keep the active server as Host. If we're already there,
    /// just clear any leftover `Connecting` status from a grace-window blip.
    /// Otherwise build the new server and swap it in.
    pub fn ensure_host(&mut self, build: impl FnOnce() -> EspHttpServer<'static>) {
        if matches!(self.server, Some(Server::Host { .. })) {
            self.set_status(NetStatus::Host);
        } else {
            info!("WiFi connected, starting main server");
            self.replace_server(Server::Host { http: build() });
        }
    }

    /// Idempotent: keep the active server as Captive, building it if needed.
    pub fn ensure_captive(
        &mut self,
        build: impl FnOnce() -> (EspHttpServer<'static>, DnsHandle),
    ) {
        if !matches!(self.server, Some(Server::Captive { .. })) {
            warn!("WiFi disconnected, starting captive portal");
            let (http, dns) = build();
            self.replace_server(Server::Captive { http, dns });
        }
    }
}

pub fn uptime_s() -> u32 {
    (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000) as u32
}
