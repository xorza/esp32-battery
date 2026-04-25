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
use esp_idf_svc::mdns::EspMdns;
use strum::IntoStaticStr;

use crate::dns::DnsHandle;
use crate::nvs_creds::WifiCredentials;
use crate::wifi::{MixedWifi, StaWifi};

/// LCD-visible status. Computed by the supervisor inside each tick arm
/// (so `bundle.state` is locked at most once per tick).
///
/// `CaptiveTrying` distinguishes "captive AP up, STA mid-association on
/// the user's freshly-submitted creds" from plain `Captive` — the LCD
/// keeps showing the AP credentials (so the user can reconnect on
/// failure) and overlays a connecting indicator.
#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq, Debug, strum::FromRepr)]
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
/// `Connecting`; past the supervisor's captive-fallback grace the
/// supervisor falls back to captive AP entirely.
const LCD_HOST_HYSTERESIS: Duration = Duration::from_secs(3);

impl NetStatus {
    /// LCD reading for `Net::Sta`. `Host` while associated, or within
    /// the hysteresis window after the last associated tick (so a single
    /// missed `is_connected()` sample doesn't flicker the screen);
    /// `Connecting` otherwise. The hysteresis only fires once we've ever
    /// associated this session — cold boot does not lie.
    pub fn for_sta(link_seen: &LinkSeen, connected: bool, now: Duration) -> Self {
        if connected {
            return NetStatus::Host;
        }
        match link_seen {
            LinkSeen::At(t) if now.saturating_sub(*t) < LCD_HOST_HYSTERESIS => NetStatus::Host,
            _ => NetStatus::Connecting,
        }
    }

    /// LCD reading for `Net::Captive`. `CaptiveTrying` during the
    /// `Pending`/`Trying` window so the captive page's "Connecting…"
    /// overlay stays visible; otherwise `Captive`.
    pub fn for_captive(s: &Submission) -> Self {
        match s {
            Submission::Pending { .. } | Submission::Trying { .. } => NetStatus::CaptiveTrying,
            Submission::Idle | Submission::Failed => NetStatus::Captive,
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
        let v = self.0.load(Ordering::Relaxed);
        NetStatus::from_repr(v).expect("invalid NetStatus discriminant")
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
    /// Trying to be on the user's network. `creds` is always present
    /// (constructing `Sta` requires them); `link_seen` carries the
    /// monotonic timestamp the captive-fallback timer counts from, and
    /// also gates LCD-side hysteresis: only `LinkSeen::At` (we've
    /// associated at least once this session) qualifies.
    Sta {
        /// STA-only radio — the type bounds the legal operations to
        /// `try_connect` / `into_mixed`. Moves with the variant on
        /// fallback; no shared `Arc<Mutex<…>>`.
        wifi: StaWifi<'static>,
        // Alive-for-Drop only — reassigning `net` away from `Sta` drops
        // the server, which stops the dashboard. Same convention as
        // `CaptiveBundle::_server` / `_dns`.
        server: EspHttpServer<'static>,
        // `None` until the first associated tick — `EspMdns::take()`
        // requires a live netif. Reassignment-on-fallback drops it so a
        // later promote can `take()` again.
        mdns: Option<EspMdns>,
        creds: WifiCredentials,
        link_seen: LinkSeen,
    },
    /// Serving the captive portal AP. `creds` is `None` pre-first-save
    /// (cold boot with no stored creds) or `Some` after the captive
    /// `/save` produced a `Pending` that the supervisor drained — and
    /// also after a `Sta → Captive` fallback (creds carry over so STA
    /// can keep retrying while the user re-enters them).
    Captive {
        /// Mixed AP+STA radio — the type bounds the legal operations to
        /// `try_connect` / `set_sta_creds` / `refresh_scan_if_stale` /
        /// `into_sta`. Moves with the variant on promote.
        wifi: MixedWifi<'static>,
        bundle: CaptiveBundle,
        creds: Option<WifiCredentials>,
    },
}

/// Tracks STA association history within a single `Net::Sta` session.
/// `Never` carries the arm's construction time so the captive-fallback
/// timer has a deadline even when STA has never associated; `At` carries
/// the most recent associated-tick time so both the fallback timer and
/// the LCD hysteresis read off one value.
#[derive(Copy, Clone)]
pub enum LinkSeen {
    Never { session_start: Duration },
    At(Duration),
}

impl LinkSeen {
    /// Time the captive-fallback grace counts from — either the most
    /// recent association or the session start when we've never
    /// associated.
    pub fn timestamp(&self) -> Duration {
        match self {
            LinkSeen::Never { session_start } => *session_start,
            LinkSeen::At(t) => *t,
        }
    }
}
