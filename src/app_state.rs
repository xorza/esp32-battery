//! Supervisor state owned by the main thread. Worker threads receive only the
//! cross-thread handles they actually use — `SensorDataHandle` and (for the
//! LCD) `NetStatusHandle` — neither of which lives on `Supervisor`.
//!
//! `Supervisor` itself is `!Send` because the wrapped `Phase` carries an
//! `EspHttpServer`. It owns the captive→main credential channel: the captive
//! `/save` handler gets a `Sender<WifiCredentials>` clone, and the main loop
//! drains the `Receiver` once per tick.
//!
//! The state-machine core lives in `esp32_battery_logic::net_supervisor` so it
//! can be tested on the host with trivial handle types. This file is the thin
//! ESP-bound shell: it pins the generic to the real handle types, mirrors
//! status to the LCD-visible `AtomicU8`, and owns the creds channel.

use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use std::sync::mpsc::{Receiver, Sender, channel};

use esp_idf_svc::http::server::EspHttpServer;

use esp32_battery_logic::data::SensorData;
use esp32_battery_logic::error_log::{Event, EventLog};
pub use esp32_battery_logic::net_supervisor::NetStatus;
use esp32_battery_logic::net_supervisor::Phase;

use crate::captive_api::SaveStateHandle;
use crate::clock::EspClock;
use crate::dns::DnsHandle;
use crate::nvs_creds::WifiCredentials;

pub type SensorDataHandle = Arc<std::sync::Mutex<SensorData>>;
pub type EventLogHandle = Arc<std::sync::Mutex<EventLog>>;

/// Pairs the event log with the wall clock used to timestamp entries.
/// Sensor threads always need both together — bundling them here removes
/// the per-thread `record(log, clock, kind)` helper duplicated in `ina.rs`
/// and `xy.rs`. Cheap to clone (two `Arc`s).
#[derive(Clone)]
pub struct EventRecorder {
    log: EventLogHandle,
    clock: EspClock,
}

impl EventRecorder {
    pub fn new(log: EventLogHandle, clock: EspClock) -> Self {
        Self { log, clock }
    }

    pub fn record(&self, event: Event) {
        let ts = self.clock.epoch_s().unwrap_or(0);
        self.log.lock().unwrap().record(ts, event);
    }
}

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
            phase: Phase::bootstrap(),
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

    pub fn on_tick_connected(&mut self, build_host: impl FnOnce() -> ServerHandle) {
        self.transition(|p| p.tick_connected(build_host));
    }

    pub fn on_tick_disconnected(
        &mut self,
        has_creds: bool,
        grace_ticks: u32,
        build_captive: impl FnOnce() -> CaptiveBundle,
    ) {
        self.transition(|p| p.tick_disconnected(has_creds, grace_ticks, build_captive));
    }

    fn transition(&mut self, f: impl FnOnce(EspPhase) -> EspPhase) {
        let next = f(std::mem::replace(&mut self.phase, Phase::bootstrap()));
        self.replace_phase(next);
    }

    fn replace_phase(&mut self, phase: EspPhase) {
        self.status.store(phase.status());
        self.phase = phase;
    }
}
