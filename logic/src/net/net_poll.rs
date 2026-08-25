//! One tick's view of the world, as gathered by the firmware.

use std::time::Duration;

use crate::net::wifi_credentials::WifiCredentials;

/// One tick's view of the world.
#[derive(Clone, Debug, Default)]
pub struct NetPoll {
    /// Monotonic uptime. Compared against the phases' own timestamps, so
    /// only differences matter.
    pub now: Duration,
    /// Result of this tick's association attempt. Meaningless — and not
    /// gathered — where [`NetPhase::polls_association`] is false.
    pub associated: bool,
    /// Credentials drained from the captive `/save` mailbox, if any.
    pub submitted: Option<WifiCredentials>,
    /// `/wifi-reset` was raised since the last tick.
    pub reset_requested: bool,
}
