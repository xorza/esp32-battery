//! Application state. One `AppState` value lives on the main thread for the
//! life of the program. Worker threads each receive only the cross-thread
//! handles they actually use — sensor data, the credential mailbox, or the
//! current net status — instead of an aggregate `Shared` blob.

use std::sync::Arc;
use std::sync::Mutex;

use esp_idf_svc::http::server::EspHttpServer;
use log::{info, warn};

use esp32_battery_logic::data::SensorData;

use crate::dns::DnsHandle;
use crate::history_store::HistoryStore;
use crate::nvs_creds::WifiCredentials;

/// Cross-thread net status — what readers (LCD, etc.) should display.
/// Derived from `NetPhase` at every transition.
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum NetStatus {
    Captive,
    Connecting,
    Host,
}

pub type SensorDataHandle = Arc<Mutex<SensorData>>;
/// Set by the captive `/save` handler when fresh credentials land. Drained
/// by the main loop, which then drives the live STA reconnect.
pub type CredsMailbox = Arc<Mutex<Option<WifiCredentials>>>;
pub type NetStatusHandle = Arc<Mutex<NetStatus>>;

/// Single source of truth for network phase + mounted HTTP server. Lives on
/// the main thread — `EspHttpServer` is `!Send`.
///
/// `Host.grace` carries a tick counter when the WiFi link has dropped but
/// we're keeping the dashboard server mounted in case the link comes back
/// within the grace window. `None` means actively hosting; `Some(n)` means
/// disconnected for `n` ticks and about to tear down. `Bootstrap` covers the
/// pre-server state (boot + post-creds reapply) with its own tick counter
/// so a slow first associate doesn't immediately flap to captive.
pub enum NetPhase {
    Bootstrap {
        ticks: u32,
    },
    Host {
        server: EspHttpServer<'static>,
        grace: Option<u32>,
    },
    Captive {
        #[allow(dead_code)]
        server: EspHttpServer<'static>,
        #[allow(dead_code)]
        dns: DnsHandle,
    },
}

impl NetPhase {
    fn status(&self) -> NetStatus {
        match self {
            NetPhase::Bootstrap { .. } | NetPhase::Host { grace: Some(_), .. } => {
                NetStatus::Connecting
            }
            NetPhase::Host { grace: None, .. } => NetStatus::Host,
            NetPhase::Captive { .. } => NetStatus::Captive,
        }
    }
}

pub struct AppState {
    pub sensor_data: SensorDataHandle,
    pub pending_creds: CredsMailbox,
    pub status: NetStatusHandle,
    pub history_store: HistoryStore,
    phase: NetPhase,
}

impl AppState {
    pub fn new(sensor_data: SensorData, history_store: HistoryStore) -> Self {
        Self {
            sensor_data: Arc::new(Mutex::new(sensor_data)),
            pending_creds: Arc::new(Mutex::new(None)),
            status: Arc::new(Mutex::new(NetStatus::Connecting)),
            history_store,
            phase: NetPhase::Bootstrap { ticks: 0 },
        }
    }

    fn set_phase(&mut self, phase: NetPhase) {
        *self.status.lock().unwrap() = phase.status();
        self.phase = phase;
    }

    /// New credentials applied to the WiFi hardware (boot load, or captive
    /// `/save`). Drops any live server and resets to `Bootstrap` so the
    /// supervisor begins the grace count from zero.
    pub fn on_creds_applied(&mut self) {
        self.set_phase(NetPhase::Bootstrap { ticks: 0 });
    }

    /// Supervisor tick: WiFi reports associated. No-op when actively
    /// hosting; clears `grace` (reusing the existing server) when within
    /// the post-disconnect grace window; otherwise builds a fresh server.
    pub fn on_tick_connected(&mut self, build_host: impl FnOnce() -> EspHttpServer<'static>) {
        let current = std::mem::replace(&mut self.phase, NetPhase::Bootstrap { ticks: 0 });
        let server = match current {
            NetPhase::Host {
                server,
                grace: None,
            } => {
                self.phase = NetPhase::Host {
                    server,
                    grace: None,
                };
                return;
            }
            NetPhase::Host {
                server,
                grace: Some(_),
            } => {
                info!("WiFi reassociated within grace window, reusing main server");
                server
            }
            NetPhase::Bootstrap { .. } | NetPhase::Captive { .. } => {
                info!("WiFi connected, starting main server");
                build_host()
            }
        };
        self.set_phase(NetPhase::Host {
            server,
            grace: None,
        });
    }

    /// Supervisor tick: WiFi is not associated. Drives the transitions
    /// between Host (via grace) and Captive.
    ///
    /// - No creds: mount captive (or stay in it).
    /// - `Host { grace: None }` → `Host { grace: Some(1) }`.
    /// - `Host { grace: Some(n) }` / `Bootstrap`: bump ticks; mount captive
    ///   once `grace_ticks` is reached, dropping the leftover server.
    pub fn on_tick_disconnected(
        &mut self,
        has_creds: bool,
        grace_ticks: u32,
        build_captive: impl FnOnce() -> (EspHttpServer<'static>, DnsHandle),
    ) {
        if !has_creds {
            if !matches!(self.phase, NetPhase::Captive { .. }) {
                self.mount_captive(build_captive);
            }
            return;
        }
        let current = std::mem::replace(&mut self.phase, NetPhase::Bootstrap { ticks: 0 });
        match current {
            NetPhase::Host { server, grace } => {
                let next = grace.unwrap_or(0).saturating_add(1);
                if next >= grace_ticks {
                    drop(server);
                    self.mount_captive(build_captive);
                } else {
                    self.set_phase(NetPhase::Host {
                        server,
                        grace: Some(next),
                    });
                }
            }
            NetPhase::Bootstrap { ticks } => {
                let next = ticks.saturating_add(1);
                if next >= grace_ticks {
                    self.mount_captive(build_captive);
                } else {
                    self.set_phase(NetPhase::Bootstrap { ticks: next });
                }
            }
            NetPhase::Captive { server, dns } => {
                self.set_phase(NetPhase::Captive { server, dns });
            }
        }
    }

    fn mount_captive(&mut self, build: impl FnOnce() -> (EspHttpServer<'static>, DnsHandle)) {
        warn!("WiFi disconnected, starting captive portal");
        let (server, dns) = build();
        self.set_phase(NetPhase::Captive { server, dns });
    }
}
