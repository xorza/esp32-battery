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
use log::info;

use esp32_battery_logic::data::SensorData;
use esp32_battery_logic::net_supervisor::Phase;
pub use esp32_battery_logic::net_supervisor::NetStatus;

use crate::dns::DnsHandle;
use crate::nvs_creds::WifiCredentials;

pub type SensorDataHandle = Arc<std::sync::Mutex<SensorData>>;

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
type CaptiveBundle = (EspHttpServer<'static>, DnsHandle);
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
    /// earlier ones — same "latest wins" semantics as the previous
    /// mailbox). On a hit, resets the phase to `Bootstrap` so the
    /// post-reconnect grace counter starts from zero.
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

    /// New credentials applied to the WiFi hardware (boot load, or
    /// captive `/save`). Drops any live server and resets to `Bootstrap`
    /// so the supervisor begins the grace count from zero.
    pub fn on_creds_applied(&mut self) {
        self.replace_phase(Phase::bootstrap());
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
