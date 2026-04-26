//! Flat-state network FSM. One enum, one variant per state, no `Option`s
//! used as state flags. Each variant carries exactly the resources alive
//! in that state (radio mode wrapper + servers); transitions consume a
//! variant and produce another. See `wifi_fsm.md` for the spec.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Duration;

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::mdns::EspMdns;
use strum::IntoStaticStr;

use crate::dns::DnsHandle;
use crate::nvs_creds::WifiCredentials;
use crate::wifi::{MixedWifi, StaWifi};

/// LCD-visible status. Computed from the FSM variant + clock each tick.
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug, strum::FromRepr)]
pub enum NetStatus {
    Captive = 0,
    CaptiveTrying = 1,
    Connecting = 2,
    Host = 3,
}

#[derive(Clone)]
pub struct NetStatusHandle(Arc<AtomicU8>);

impl NetStatusHandle {
    pub fn new() -> Self {
        Self(Arc::new(AtomicU8::new(NetStatus::Connecting as u8)))
    }

    pub fn store(&self, s: NetStatus) {
        self.0.store(s as u8, Ordering::Relaxed);
    }

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
        SubmissionStatus::from_repr(self.0.load(Ordering::Relaxed)).unwrap()
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

/// Flat FSM over network state. The variant is the source of truth —
/// radio mode, alive servers, and legal transitions are all bounded by
/// the type. See `wifi_fsm.md` for the state table and transitions.
///
/// Credentials live in NVS (`nvs_creds`) and on the radio config; only
/// `CaptiveSubmitted` / `CaptiveTrying` carry an in-memory copy, for the
/// window between `/save` and the success that persists them.
pub enum NetState {
    BootNoCreds {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
    },
    CaptiveSubmitted {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
        creds: WifiCredentials,
        since: Duration,
    },
    CaptiveTrying {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
        creds: WifiCredentials,
        since: Duration,
    },
    CaptiveFailed {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
    },
    CaptiveFallbackRetrying {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
    },
    StaConnecting {
        wifi: StaWifi<'static>,
        server: EspHttpServer<'static>,
        session_start: Duration,
    },
    StaHost {
        wifi: StaWifi<'static>,
        server: EspHttpServer<'static>,
        mdns: EspMdns,
        last_assoc: Duration,
    },
    StaReassociating {
        wifi: StaWifi<'static>,
        server: EspHttpServer<'static>,
        mdns: EspMdns,
        last_assoc: Duration,
    },
}

impl NetState {
    pub fn lcd_status(&self) -> NetStatus {
        match self {
            NetState::BootNoCreds { .. }
            | NetState::CaptiveFailed { .. }
            | NetState::CaptiveFallbackRetrying { .. } => NetStatus::Captive,
            NetState::CaptiveSubmitted { .. } | NetState::CaptiveTrying { .. } => {
                NetStatus::CaptiveTrying
            }
            NetState::StaConnecting { .. } | NetState::StaReassociating { .. } => {
                NetStatus::Connecting
            }
            NetState::StaHost { .. } => NetStatus::Host,
        }
    }
}
