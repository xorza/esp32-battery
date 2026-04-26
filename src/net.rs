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
pub struct ResetSignal(Arc<std::sync::atomic::AtomicBool>);

impl ResetSignal {
    pub fn new() -> Self {
        Self(Arc::new(std::sync::atomic::AtomicBool::new(false)))
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

/// Flat FSM over network state. The variant is the source of truth —
/// radio mode, alive servers, and legal transitions are all bounded by
/// the type. See `wifi_fsm.md` for the state table and transitions.
///
/// Every variant that has credentials at runtime carries them in the
/// variant. NVS is the durable store; the FSM doesn't read NVS per
/// tick. `CaptiveIdle` is the only state without creds — cold boot
/// before any /save, or after a `CaptiveFailed`-style timeout where
/// the last attempt's creds are intentionally dropped so the captive
/// page is the source of truth on retry.
pub enum NetState {
    /// Captive AP up, no in-flight submission. Covers cold boot
    /// (NVS empty) and post-timeout retry. The captive page reads
    /// `bundle.status` to know whether to show a "wrong creds" error.
    CaptiveIdle {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
    },
    /// /save creds applied to the radio, association in flight (≤ 20 s).
    CaptiveTrying {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
        creds: WifiCredentials,
        since: Duration,
    },
    /// STA→Captive carry-over. Radio is Mixed with the known
    /// (last-good) creds; STA half retries in the background while
    /// the captive page lets the user re-enter creds if needed.
    CaptiveFallbackRetrying {
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
        creds: WifiCredentials,
    },
    /// STA-only, never associated this session. Dashboard server is
    /// up but mDNS isn't (mDNS needs the netif live, only true after
    /// first associated tick).
    StaConnecting {
        wifi: StaWifi<'static>,
        server: EspHttpServer<'static>,
        creds: WifiCredentials,
        session_start: Duration,
    },
    /// STA-only, dashboard + mDNS up. `link` is the most recent
    /// `is_connected()` result; mDNS stays up across `Down` windows
    /// since it'll be valid again on re-link without re-init.
    StaServing {
        wifi: StaWifi<'static>,
        server: EspHttpServer<'static>,
        mdns: EspMdns,
        creds: WifiCredentials,
        link: LinkState,
    },
}

/// `StaServing` link status. `Up` means the radio is associated this
/// tick; `Down { since }` means it's not, and the captive-fallback
/// timer counts from `since`. The variant encodes the invariant
/// "we only need a timer when we're disconnected" — no
/// always-present `last_assoc` field whose meaning depends on a
/// sibling boolean.
pub enum LinkState {
    Up,
    Down { since: Duration },
}

impl NetState {
    pub fn lcd_status(&self) -> NetStatus {
        match self {
            NetState::CaptiveIdle { .. } | NetState::CaptiveFallbackRetrying { .. } => {
                NetStatus::Captive
            }
            NetState::CaptiveTrying { .. } => NetStatus::CaptiveTrying,
            NetState::StaConnecting { .. } => NetStatus::Connecting,
            NetState::StaServing {
                link: LinkState::Up,
                ..
            } => NetStatus::Host,
            NetState::StaServing {
                link: LinkState::Down { .. },
                ..
            } => NetStatus::Connecting,
        }
    }
}
