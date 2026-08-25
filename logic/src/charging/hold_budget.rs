//! Rate limit on self-clearing buck protections.

use std::time::Duration;

use crate::charging::{FLAP_WINDOW, MAX_HOLDS};

/// How often the buck has dropped into a self-clearing hold lately.
///
/// One hold is routine — the supply was unplugged, the case got warm — and
/// waiting it out is exactly right. A *stream* of them is something else: a
/// rail that cannot carry the charge current sags, the buck drops on LVP,
/// the rail recovers unloaded, the buck comes back on, and it sags again.
/// That loop is stable and unbounded, and every turn of it takes the UPS
/// output away and gives it back. Counting the holds is what turns "wait it
/// out" into "wait it out a few times, then stop".
#[derive(Debug, Default)]
pub(super) struct HoldBudget {
    holds: u8,
    /// Time since the most recent hold. Past `FLAP_WINDOW` the run is over
    /// and the count starts again.
    since_last: Duration,
}

impl HoldBudget {
    /// Count a hold the machine has just entered.
    pub(super) fn record(&mut self) {
        self.holds = self.holds.saturating_add(1);
        self.since_last = Duration::ZERO;
    }

    /// Advance the quiet timer, and answer whether the current run of holds
    /// is over budget.
    ///
    /// A run ends when `FLAP_WINDOW` passes with no new hold, so what this
    /// counts is `MAX_HOLDS` holds with no quiet stretch that long between
    /// any two of them — not `MAX_HOLDS` inside one fixed window. The
    /// difference matters: a rail that sags every four minutes for a day
    /// never fills a five-minute window, and it is still flapping.
    pub(super) fn step(&mut self, dt: Duration) -> bool {
        self.since_last = self.since_last.saturating_add(dt);
        if self.since_last > FLAP_WINDOW {
            self.holds = 0;
        }
        self.holds > MAX_HOLDS
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TICK: Duration = Duration::from_secs(1);

    /// The budget is `MAX_HOLDS` tolerated, so the one after it trips.
    #[test]
    fn trips_one_hold_past_the_budget() {
        let mut h = HoldBudget::default();
        for turn in 0..MAX_HOLDS {
            h.record();
            assert!(!h.step(TICK), "turn {turn} tripped early");
        }
        h.record();
        assert!(h.step(TICK));
    }

    /// A quiet stretch ends the run, and the next one starts from zero —
    /// including its full budget.
    #[test]
    fn a_quiet_stretch_ends_the_run() {
        let mut h = HoldBudget::default();
        for _ in 0..MAX_HOLDS {
            h.record();
            assert!(!h.step(TICK));
        }
        // One tick *past* the window, since the clear is on strictly-greater.
        assert!(!h.step(FLAP_WINDOW + TICK));
        for turn in 0..MAX_HOLDS {
            h.record();
            assert!(!h.step(TICK), "turn {turn} did not get a fresh budget");
        }
        h.record();
        assert!(h.step(TICK));
    }

    /// Exactly the window is not enough to end a run — it is the strictly
    /// longer gap that does, and the boundary is worth pinning because a
    /// run that ends one tick early forgives a genuine flap.
    #[test]
    fn the_window_boundary_is_strict() {
        let mut h = HoldBudget::default();
        h.record();
        // `record` zeroes the timer, so this lands exactly on the window.
        assert!(!h.step(FLAP_WINDOW), "exactly the window must not end a run");
        // The first hold therefore still counts, leaving room for only
        // `MAX_HOLDS - 1` more before the budget is spent.
        for turn in 0..(MAX_HOLDS - 1) {
            h.record();
            assert!(!h.step(TICK), "turn {turn} tripped early");
        }
        h.record();
        assert!(h.step(TICK), "the retained first hold was forgiven");
    }
}
