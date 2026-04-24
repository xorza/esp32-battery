//! Application state. One `AppState` value lives on the main thread for the
//! life of the program. Worker threads (xy, ina, lcd, http) get an
//! `Arc<Shared>` clone — the cross-thread, `Send + Sync` subset.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};

use esp_idf_svc::http::server::EspHttpServer;
use log::{info, warn};

use esp32_battery_logic::data::SensorData;

use crate::dns::DnsHandle;
use crate::history_store::HistoryStore;
use crate::nvs_creds::WifiCredentials;

/// Cross-thread net status — what readers (LCD, etc.) should display.
/// Derived from `NetPhase` at every transition; lives in `Shared` because
/// `NetPhase` contains `!Send` servers and stays on the main thread.
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

/// Single source of truth for network phase + mounted HTTP server. Lives on
/// the main thread — `EspHttpServer` is `!Send`.
///
/// `Connecting` carries an optional leftover Host server so a brief WiFi
/// blip doesn't tear down the dashboard; the server is reused on
/// reassociation, or dropped once the grace window expires and we fall back
/// to `Captive`.
pub enum NetPhase {
    /// Bootstrap: no server mounted, STA not yet configured.
    Idle,
    Connecting {
        ticks: u32,
        #[allow(dead_code)] // held for Drop during the grace window
        host_server: Option<EspHttpServer<'static>>,
    },
    Host {
        #[allow(dead_code)]
        server: EspHttpServer<'static>,
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
            NetPhase::Idle | NetPhase::Connecting { .. } => NetStatus::Connecting,
            NetPhase::Host { .. } => NetStatus::Host,
            NetPhase::Captive { .. } => NetStatus::Captive,
        }
    }
}

/// Cross-thread subset. Cloned (as `Arc<Shared>`) into every worker thread.
pub struct Shared {
    pub sensor_data: Mutex<SensorData>,
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
    phase: NetPhase,
}

impl AppState {
    pub fn new(sensor_data: SensorData, history_store: HistoryStore) -> Self {
        Self {
            shared: Arc::new(Shared {
                sensor_data: Mutex::new(sensor_data),
                pending_creds: Mutex::new(None),
                status: AtomicU8::new(NetStatus::Connecting as u8),
            }),
            history_store,
            phase: NetPhase::Idle,
        }
    }

    fn set_phase(&mut self, phase: NetPhase) {
        self.shared
            .status
            .store(phase.status() as u8, Ordering::Relaxed);
        self.phase = phase;
    }

    /// New credentials applied to the WiFi hardware (boot load, or captive
    /// `/save`). Drops any live server and enters `Connecting` so the
    /// supervisor begins the grace count from zero.
    pub fn on_creds_applied(&mut self) {
        self.set_phase(NetPhase::Connecting {
            ticks: 0,
            host_server: None,
        });
    }

    /// Supervisor tick: WiFi reports associated. Idempotent in `Host`;
    /// reuses the leftover server from a grace-window `Connecting` so a
    /// brief blip doesn't rebuild HTTPS state.
    pub fn on_tick_connected(&mut self, build_host: impl FnOnce() -> EspHttpServer<'static>) {
        if matches!(self.phase, NetPhase::Host { .. }) {
            return;
        }
        let current = std::mem::replace(&mut self.phase, NetPhase::Idle);
        let server = match current {
            NetPhase::Connecting {
                host_server: Some(s),
                ..
            } => {
                info!("WiFi reassociated within grace window, reusing main server");
                s
            }
            _ => {
                info!("WiFi connected, starting main server");
                build_host()
            }
        };
        self.set_phase(NetPhase::Host { server });
    }

    /// Supervisor tick: WiFi is not associated. Drives the transitions
    /// between Host (via Connecting grace) and Captive.
    ///
    /// - No creds: mount captive (or stay in it).
    /// - `Host` → `Connecting { ticks: 1, host_server: Some(..) }`.
    /// - `Connecting`: bump ticks; fall through to Captive once `grace_ticks`
    ///   is reached, dropping the leftover server.
    /// - `Idle` / unhandled: mount captive.
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
        let current = std::mem::replace(&mut self.phase, NetPhase::Idle);
        match current {
            NetPhase::Host { server } => {
                self.set_phase(NetPhase::Connecting {
                    ticks: 1,
                    host_server: Some(server),
                });
            }
            NetPhase::Connecting { ticks, host_server } => {
                let next = ticks.saturating_add(1);
                if next >= grace_ticks {
                    drop(host_server);
                    self.mount_captive(build_captive);
                } else {
                    self.set_phase(NetPhase::Connecting {
                        ticks: next,
                        host_server,
                    });
                }
            }
            NetPhase::Captive { server, dns } => {
                self.set_phase(NetPhase::Captive { server, dns });
            }
            NetPhase::Idle => {
                self.mount_captive(build_captive);
            }
        }
    }

    fn mount_captive(&mut self, build: impl FnOnce() -> (EspHttpServer<'static>, DnsHandle)) {
        warn!("WiFi disconnected, starting captive portal");
        let (server, dns) = build();
        self.set_phase(NetPhase::Captive { server, dns });
    }
}

pub fn uptime_s() -> u32 {
    (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000) as u32
}
