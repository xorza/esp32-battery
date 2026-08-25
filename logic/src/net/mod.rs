//! Network supervisor: a flat state machine over the WiFi lifecycle.
//!
//! The phase alone determines radio mode (STA-only vs Mixed AP+STA),
//! which servers should be alive (dashboard vs captive bundle), what the
//! LCD shows, and which transitions are legal. Each variant carries only
//! the data meaningful to it — no `Option`s used as state flags, and no
//! illegal combinations to represent.
//!
//! Pure logic: no I/O, no clock, no radio. The firmware gathers a
//! [`NetPoll`], calls [`NetSupervisor::tick`], and performs the returned
//! [`NetAction`] against the resources it owns. Timing policy (the 20 s
//! association budget, the 2 h fallback grace) lives here so it is
//! testable on the host; resource ownership stays in firmware.

use std::time::Duration;

pub(crate) mod net_action;
pub(crate) mod net_phase;
pub(crate) mod net_poll;
pub(crate) mod net_supervisor;
pub(crate) mod wifi_credentials;

/// How long `associated == false` may persist before the dashboard comes
/// down and the captive AP takes over. The AP is a fallback for "the
/// saved creds no longer work" (rotated password, SSID gone), so the wait
/// is long enough that a real outage of the user's router — ISP reboot,
/// scheduled maintenance — doesn't flap us into captive mode and break
/// the dashboard for everyone on the LAN.
pub const CAPTIVE_AFTER_DISCONNECT: Duration = Duration::from_secs(2 * 60 * 60);

/// How long the captive page's "Connecting…" spinner may run before the
/// submitted credentials are declared a failure and the user gets to
/// re-enter them. ESP-IDF associates good creds in 3–8 s typically; 20 s
/// is comfortably past that.
pub const CAPTIVE_TRYING_TIMEOUT: Duration = Duration::from_secs(20);

#[cfg(test)]
mod tests;
