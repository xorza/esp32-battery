//! Supervisor state owned by the main thread. Worker threads receive only the
//! cross-thread handles they actually use — `SensorDataHandle` and (for the
//! LCD) `NetStatusHandle` — neither of which lives on `Supervisor`.
//!
//! `Supervisor` itself is `!Send` because `NetPhase` carries an `EspHttpServer`.
//! It owns the captive→main credential channel: the captive `/save` handler
//! gets a `Sender<WifiCredentials>` clone, and the main loop drains the
//! `Receiver` once per tick.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, Sender, channel};

use esp_idf_svc::http::server::EspHttpServer;
use log::{info, warn};

use esp32_battery_logic::data::SensorData;

use crate::dns::DnsHandle;
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
pub type NetStatusHandle = Arc<Mutex<NetStatus>>;

/// Single source of truth for network phase + mounted HTTP server.
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

pub struct Supervisor {
    pub status: NetStatusHandle,
    creds_tx: Sender<WifiCredentials>,
    creds_rx: Receiver<WifiCredentials>,
    phase: NetPhase,
}

impl Supervisor {
    pub fn new() -> Self {
        let (creds_tx, creds_rx) = channel();
        Self {
            status: Arc::new(Mutex::new(NetStatus::Connecting)),
            creds_tx,
            creds_rx,
            phase: NetPhase::Bootstrap { ticks: 0 },
        }
    }

    /// Cloneable handle for the captive `/save` handler to deliver new
    /// credentials back to the main loop.
    pub fn creds_sender(&self) -> Sender<WifiCredentials> {
        self.creds_tx.clone()
    }

    /// Drain any credentials posted by the captive portal since the last
    /// tick, returning the most recent (later submissions supersede earlier
    /// ones — same "latest wins" semantics as the previous mailbox). On a
    /// hit, resets the phase to `Bootstrap` so the post-reconnect grace
    /// counter starts from zero.
    pub fn take_pending_creds(&mut self) -> Option<WifiCredentials> {
        let mut latest = None;
        while let Ok(c) = self.creds_rx.try_recv() {
            latest = Some(c);
        }
        if latest.is_some() {
            info!("Applying credentials submitted via captive portal");
            self.on_creds_applied();
        }
        latest
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
