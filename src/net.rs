//! Network state — two-mode core: either we're trying to be on the
//! user's network (`Sta`) or asking the user for credentials (`Captive`).
//!
//! The captive bundle (HTTP server + DNS responder + shared submission
//! state) is owned by `Net::Captive` for as long as the captive AP is
//! up; dropping it stops the server and joins the DNS thread. The host
//! server is owned by `Net::Sta` for as long as STA is in service.
//! Transition logic lives in `main` — there's no separate state machine
//! to maintain in lockstep.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU8, Ordering};
use std::time::Duration;

use esp_idf_svc::http::server::EspHttpServer;
use strum::IntoStaticStr;

use crate::dns::DnsHandle;
use crate::nvs_creds::WifiCredentials;

/// LCD-visible status. Derived once per supervisor tick from `(Net,
/// connected)`; not stored inside `Net`.
///
/// `CaptiveTrying` distinguishes "captive AP up, STA mid-association on
/// the user's freshly-submitted creds" from plain `Captive` — the LCD
/// keeps showing the AP credentials (so the user can reconnect on
/// failure) and overlays a connecting indicator.
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum NetStatus {
    Captive = 0,
    CaptiveTrying = 1,
    Connecting = 2,
    Host = 3,
}

/// Window during which a brief STA drop is hidden from the LCD — keeps
/// `Host` displayed across a single missed `is_connected()` sample (sub-second
/// deauth, beacon-loss false negative, scan blip) instead of flickering
/// through `Connecting`. Sustained drops past this window honestly read as
/// `Connecting`; past `CAPTIVE_AFTER_DISCONNECT` the supervisor falls back
/// to captive AP entirely.
const LCD_HOST_HYSTERESIS: Duration = Duration::from_secs(3);

impl NetStatus {
    pub fn derive(net: &Net, connected: bool, now: Duration) -> Self {
        match net {
            Net::Sta {
                last_associated,
                ever_connected,
                ..
            } => {
                if connected
                    || (*ever_connected
                        && now.saturating_sub(*last_associated) < LCD_HOST_HYSTERESIS)
                {
                    NetStatus::Host
                } else {
                    NetStatus::Connecting
                }
            }
            Net::Captive { bundle } => match &*bundle.state.lock().unwrap() {
                Submission::Pending { .. } | Submission::Trying { .. } => NetStatus::CaptiveTrying,
                Submission::Idle | Submission::Failed => NetStatus::Captive,
            },
        }
    }
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
        match self.0.load(Ordering::Relaxed) {
            0 => NetStatus::Captive,
            1 => NetStatus::CaptiveTrying,
            2 => NetStatus::Connecting,
            3 => NetStatus::Host,
            v => unreachable!("invalid NetStatus discriminant: {v}"),
        }
    }
}

/// Shared state between `/save` (producer) and the main loop (consumer).
///
/// Lifecycle: `Idle` → `Pending { creds, since }` (set by `/save`) →
/// `Trying { since }` (supervisor consumed creds and called
/// `set_sta_creds_live`) → `Failed` on timeout, or the whole captive
/// bundle is dropped on association success — the page's `/status` poll
/// then errors, which it treats as success.
///
/// `Pending` carries the one-shot creds payload; `Trying` carries only
/// the deadline. Splitting them keeps the lifecycle visible at the type
/// level instead of through an `Option<WifiCredentials>` in `Trying`.
#[derive(IntoStaticStr)]
#[strum(serialize_all = "lowercase")]
pub enum Submission {
    Idle,
    Pending {
        creds: WifiCredentials,
        since: Duration,
    },
    Trying {
        since: Duration,
    },
    Failed,
}

pub type CaptiveStateHandle = Arc<Mutex<Submission>>;

pub struct CaptiveBundle {
    pub _server: EspHttpServer<'static>,
    pub _dns: DnsHandle,
    pub state: CaptiveStateHandle,
}

pub enum Net {
    /// Trying to be on the user's network. `last_associated` is the
    /// monotonic timestamp of the most recent associated tick (or the
    /// arm's construction time if never associated yet). Once
    /// `now - last_associated` exceeds the captive grace, we fall back
    /// to `Captive`. `ever_connected` distinguishes "fresh boot, still
    /// trying" from "we've been associated and may briefly drop" — only
    /// the latter gets LCD-side hysteresis.
    Sta {
        server: EspHttpServer<'static>,
        last_associated: Duration,
        ever_connected: bool,
    },
    /// Serving the captive portal AP.
    Captive { bundle: CaptiveBundle },
}
