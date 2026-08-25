//! Firmware-side network resources: the handles the HTTP threads share
//! with the supervisor, and the resource shell the supervisor's phase
//! drives.
//!
//! The state machine itself is `esp32_battery_logic::net` — pure, and
//! tested on the host. What lives here is everything that cannot be:
//! the live radio, the running servers, and the atomics the request
//! handlers poke.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::time::Duration;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::mdns::EspMdns;
use strum::IntoStaticStr;

use esp32_battery_logic::WifiCredentials;
use esp32_battery_logic::{NetPhase, NetStatus};

use crate::dns::DnsHandle;
use crate::wifi::{MixedWifi, StaWifi};

#[derive(Clone)]
pub struct NetStatusHandle(Arc<AtomicU8>);

impl NetStatusHandle {
    pub fn new() -> Self {
        Self(Arc::new(AtomicU8::new(NetStatus::Connecting as u8)))
    }

    pub fn store(&self, s: NetStatus) {
        self.0.store(s as u8, Ordering::Relaxed);
    }

    /// Read by the LCD thread, which is the only consumer — so a build
    /// without a panel does not carry it.
    #[cfg(feature = "lcd")]
    pub fn load(&self) -> NetStatus {
        let v = self.0.load(Ordering::Relaxed);
        NetStatus::from_repr(v).expect("invalid NetStatus discriminant")
    }
}

/// Reported by `/status` to the captive page so its spinner / error UI
/// can track the lifecycle of a `/save` submission. Stored as an
/// `AtomicU8` shared between the HTTP handler and the supervisor — the
/// supervisor owns transitions, the handler is read-only.
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, IntoStaticStr, strum::FromRepr)]
#[strum(serialize_all = "lowercase")]
pub enum SubmissionStatus {
    Idle = 0,
    Pending = 1,
    Trying = 2,
    Failed = 3,
    Connected = 4,
}

#[derive(Clone)]
pub struct SubmissionStatusHandle(Arc<AtomicU8>);

impl SubmissionStatusHandle {
    pub fn new() -> Self {
        Self(Arc::new(AtomicU8::new(SubmissionStatus::Idle as u8)))
    }

    pub fn store(&self, s: SubmissionStatus) {
        self.0.store(s as u8, Ordering::Relaxed);
    }

    pub fn load(&self) -> SubmissionStatus {
        let v = self.0.load(Ordering::Relaxed);
        SubmissionStatus::from_repr(v).expect("invalid SubmissionStatus discriminant")
    }
}

/// One-shot signal raised by the `/wifi-reset` handler and consumed by
/// the supervisor on its next tick. The handler clears NVS creds; the
/// supervisor drops the live association and returns the FSM to
/// `CaptiveIdle`. An atomic so the HTTP handler thread can signal
/// without holding any FSM lock.
#[derive(Clone)]
pub struct ResetSignal(Arc<AtomicBool>);

impl ResetSignal {
    pub fn new() -> Self {
        Self(Arc::new(AtomicBool::new(false)))
    }

    pub fn raise(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    /// Atomically reads-and-clears the flag.
    pub fn take(&self) -> bool {
        self.0.swap(false, Ordering::Relaxed)
    }
}

/// Single-slot creds mailbox. `/save` writes; the supervisor `take`s on
/// the next captive-arm tick. Latest-submission-wins — a second `/save`
/// before the supervisor drains overwrites the first. Wrapped in
/// `Arc<Mutex<…>>` because the HTTP handler closure needs `Send`.
pub type CredsMailbox = Arc<Mutex<Option<WifiCredentials>>>;

pub fn new_creds_mailbox() -> CredsMailbox {
    Arc::new(Mutex::new(None))
}

pub struct CaptiveBundle {
    pub _server: EspHttpServer<'static>,
    pub _dns: DnsHandle,
    pub mailbox: CredsMailbox,
    pub status: SubmissionStatusHandle,
}

impl CaptiveBundle {
    /// Pop a freshly-submitted creds payload, if any. Called once per
    /// captive-arm tick.
    pub fn take_creds(&self) -> Option<WifiCredentials> {
        self.mailbox.lock().unwrap().take()
    }

    pub fn set_status(&self, s: SubmissionStatus) {
        self.status.store(s);
    }
}

/// The resources a phase owns. Only two shapes exist, because the five
/// phases collapse to two: Mixed radio with the captive bundle, or STA-only
/// with the dashboard. Which shape is live is *not* enforced against the
/// phase by construction — see [`Self::warn_out_of_step`].
pub enum NetResources {
    Mixed {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
    },
    Sta {
        wifi: StaWifi<'static>,
        server: EspHttpServer<'static>,
        /// Taken on the first association of a session; `None` while
        /// `StaConnecting`, since mDNS needs a live netif.
        mdns: Option<EspMdns>,
    },
}

impl NetResources {
    /// Run this tick's association attempt, if the phase has credentials
    /// to attempt one with.
    pub fn try_connect(&mut self, phase: &NetPhase) -> bool {
        if !phase.polls_association() {
            return false;
        }
        match self {
            Self::Mixed { wifi, .. } => wifi.try_connect(),
            Self::Sta { wifi, .. } => wifi.try_connect(),
        }
    }

    /// Pop a freshly-submitted credentials payload, if the captive
    /// bundle is up to have received one.
    pub fn take_creds(&self) -> Option<WifiCredentials> {
        match self {
            Self::Mixed { bundle, .. } => bundle.take_creds(),
            Self::Sta { .. } => None,
        }
    }

    /// Refresh the AP scan cache if it has gone stale. The TTL lives with
    /// the cache; the supervisor only decides *when* scanning is safe.
    pub fn refresh_scan_if_stale(&mut self, now: Duration) {
        if let Self::Mixed { wifi, .. } = self {
            wifi.refresh_scan_if_stale(now);
        }
    }

    /// The phase-to-resource mapping is total: the three captive phases share
    /// the Mixed shape, the two STA phases share the Sta shape. The
    /// pure/impure split gave up enforcing that by construction, so a site
    /// handed an action its resources cannot carry out reports here. The radio
    /// is stale either way; silence would leave nothing to explain why.
    pub fn warn_out_of_step(&self, action: &str) {
        let live = match self {
            Self::Mixed { .. } => "Mixed",
            Self::Sta { .. } => "Sta",
        };
        log::warn!(
            "net: {action} does not match the live {live} resources; the two are out of step"
        );
    }
}
