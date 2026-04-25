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

/// LCD-visible status. Derived from `Net` + the most recent link
/// observation each tick — not stored inside `Net` itself.
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum NetStatus {
    Captive = 0,
    Connecting = 1,
    Host = 2,
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
            1 => NetStatus::Connecting,
            2 => NetStatus::Host,
            v => unreachable!("invalid NetStatus discriminant: {v}"),
        }
    }
}

/// Shared state between `/save` (producer) and the main loop
/// (consumer). `Trying { pending: Some(_) }` means /save just landed
/// fresh creds for main to apply; main `take()`s the inner `Option`
/// and leaves the `Trying` window running. Timeout flips to `Failed`.
/// "Connected" isn't a variant — once STA associates the main loop
/// drops the captive bundle, the AP goes down, and the page's
/// `/status` poll just fails (its cue to assume success).
#[derive(IntoStaticStr)]
#[strum(serialize_all = "lowercase")]
pub enum Submission {
    Idle,
    Trying {
        since: Duration,
        pending: Option<WifiCredentials>,
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
    /// monotonic timestamp of the most recent associated tick (or boot
    /// time if never associated yet). Once `now - last_associated`
    /// exceeds the captive grace, we fall back to `Captive`.
    Sta {
        server: EspHttpServer<'static>,
        last_associated: Duration,
    },
    /// Serving the captive portal AP.
    Captive { bundle: CaptiveBundle },
}
