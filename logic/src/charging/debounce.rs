//! Time-based debouncer shared by every gate the supervisor clocks.

use std::time::Duration;

/// Time-based debouncer: counts elapsed while `cond` holds, resets when it
/// doesn't. One per condition we care about (OV, absorb cap, exit taper,
/// missing battery, modbus errors).
#[derive(Default)]
pub(super) struct Debounce {
    pub(super) elapsed: Duration,
}

impl Debounce {
    /// Add `dt` if `cond`, else reset. Returns `true` once accumulated
    /// `>= timeout`.
    pub(super) fn step(&mut self, cond: bool, dt: Duration, timeout: Duration) -> bool {
        if cond {
            self.elapsed = self.elapsed.saturating_add(dt);
            self.elapsed >= timeout
        } else {
            self.elapsed = Duration::ZERO;
            false
        }
    }

    /// Clear the accumulated window.
    pub(super) fn reset(&mut self) {
        self.elapsed = Duration::ZERO;
    }

    /// Like `step`, but a false `cond` *drains* the accumulator by `dt`
    /// (floored at zero) instead of zeroing it. Firing at `>= timeout` then
    /// means "net time-true exceeded the window" — equivalently, `cond` held
    /// for more than half the recent window on average. Used for the
    /// Absorb-exit taper gate: a nearly-full pack drives the XY7025 into burst
    /// pulses (0 → several amps every few seconds), so the instantaneous
    /// charging current keeps poking back above the tail threshold. Under a
    /// hard reset each pulse re-arms the full window forever and pins the
    /// supervisor in Absorb; draining lets the mostly-below-tail average still
    /// reach the timeout, while a genuine *sustained* return to charging
    /// drains it back to zero and blocks the exit.
    pub(super) fn step_leaky(&mut self, cond: bool, dt: Duration, timeout: Duration) -> bool {
        if cond {
            self.elapsed = self.elapsed.saturating_add(dt);
        } else {
            self.elapsed = self.elapsed.saturating_sub(dt);
        }
        // Firing is supposed to make the caller transition and reset this,
        // so the accumulator can overshoot by at most the tick that
        // crossed the line. Running further means a fired gate went
        // unacted-on and the window no longer means what it says.
        debug_assert!(
            self.elapsed <= timeout.saturating_add(dt),
            "leaky debounce ran past its window — a fired gate went unhandled"
        );
        self.elapsed >= timeout
    }
}
