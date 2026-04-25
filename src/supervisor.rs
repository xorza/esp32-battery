//! ESP-side wrapper around the host-testable `net_supervisor::Phase`
//! state machine. Pins the generic phase to the real handle types
//! (`EspHttpServer`, `CaptiveBundle`), mirrors status to the LCD-visible
//! `NetStatusHandle`, and owns the captive→main credential channel.
//!
//! `Supervisor` is `!Send` because the wrapped `Phase` carries an
//! `EspHttpServer`. Worker threads only get the small `Clone` handles
//! defined here (`SensorDataHandle`, `EventLogHandle`, `NetStatusHandle`).

use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;

use esp_idf_svc::http::server::EspHttpServer;

use esp32_battery_logic::data::SensorData;
use esp32_battery_logic::error_log::EventLog;
pub use esp32_battery_logic::net_supervisor::{HostTransition, NetStatus};
use esp32_battery_logic::net_supervisor::Phase;

use crate::captive_api::SaveStateHandle;
use crate::clock::uptime;
use crate::dns::DnsHandle;
use crate::nvs_creds::WifiCredentials;

pub type SensorDataHandle = Arc<std::sync::Mutex<SensorData>>;
pub type EventLogHandle = Arc<std::sync::Mutex<EventLog>>;

#[derive(Clone)]
pub struct NetStatusHandle(Arc<AtomicU8>);

impl NetStatusHandle {
    fn new(status: NetStatus) -> Self {
        Self(Arc::new(AtomicU8::new(status as u8)))
    }

    fn store(&self, status: NetStatus) {
        self.0.store(status as u8, Ordering::Relaxed);
    }

    pub fn load(&self) -> NetStatus {
        match self.0.load(Ordering::Relaxed) {
            0 => NetStatus::Captive,
            1 => NetStatus::Connecting,
            2 => NetStatus::Host,
            v => unreachable!("invalid NetStatus discriminant: {v}"),
        }
    }
}

type ServerHandle = EspHttpServer<'static>;

/// Server + DNS responder for the captive portal, plus the shared
/// `SaveState` the supervisor reads/writes to coordinate the
/// captive→host handoff with the captive page's `/status` poll.
/// `server` and `dns` are held only for their `Drop` side effects (stop
/// the server, kill the DNS thread); the supervisor never reads them.
pub struct CaptiveBundle {
    #[allow(dead_code)]
    pub server: EspHttpServer<'static>,
    #[allow(dead_code)]
    pub dns: DnsHandle,
    pub save_state: SaveStateHandle,
}

type EspPhase = Phase<ServerHandle, CaptiveBundle>;

pub struct Supervisor {
    pub status: NetStatusHandle,
    creds_tx: Sender<WifiCredentials>,
    creds_rx: Receiver<WifiCredentials>,
    phase: EspPhase,
}

impl Supervisor {
    pub fn new() -> Self {
        let (creds_tx, creds_rx) = channel();
        Self {
            status: NetStatusHandle::new(NetStatus::Connecting),
            creds_tx,
            creds_rx,
            phase: Phase::bootstrap(uptime()),
        }
    }

    pub fn creds_sender(&self) -> Sender<WifiCredentials> {
        self.creds_tx.clone()
    }

    /// Drain any credentials posted by the captive portal since the last
    /// tick, returning the most recent (later submissions supersede
    /// earlier ones). The captive phase stays alive — main loop applies
    /// the new creds via a live STA-config update (so the AP doesn't
    /// blip) and waits for the STA to associate before transitioning to
    /// host mode.
    pub fn take_pending_creds(&mut self) -> Option<WifiCredentials> {
        let mut latest = None;
        while let Ok(c) = self.creds_rx.try_recv() {
            latest = Some(c);
        }
        latest
    }

    /// Snapshot the SaveState handle when the supervisor is in `Captive`.
    /// Returns an owned `Arc` clone so the caller can use it after
    /// dropping the supervisor borrow and re-borrowing for `on_tick_*`.
    pub fn captive_save_state(&self) -> Option<SaveStateHandle> {
        match &self.phase {
            Phase::Captive { bundle } => Some(bundle.save_state.clone()),
            _ => None,
        }
    }

    pub fn on_tick_connected(
        &mut self,
        now: Duration,
        handoff_grace: Duration,
        build_host: impl FnOnce(HostTransition) -> ServerHandle,
    ) {
        self.transition(|p| p.tick_connected(now, handoff_grace, build_host));
    }

    pub fn on_tick_disconnected(
        &mut self,
        now: Duration,
        has_creds: bool,
        captive_grace: Duration,
        build_captive: impl FnOnce() -> CaptiveBundle,
    ) {
        self.transition(|p| p.tick_disconnected(now, has_creds, captive_grace, build_captive));
    }

    fn transition(&mut self, f: impl FnOnce(EspPhase) -> EspPhase) {
        // Placeholder is overwritten by `f` before anyone observes it —
        // the entered_at value is throwaway.
        let placeholder = Phase::bootstrap(Duration::ZERO);
        let next = f(std::mem::replace(&mut self.phase, placeholder));
        self.replace_phase(next);
    }

    fn replace_phase(&mut self, phase: EspPhase) {
        self.status.store(phase.status());
        self.phase = phase;
    }
}
