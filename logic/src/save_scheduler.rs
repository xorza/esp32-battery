//! Periodic save timer driven by wall-clock epochs.
//!
//! Extracted from `SensorData` so the data store only models
//! history-and-readings; this decides *when* to persist. Typical flow in the
//! 1 Hz main loop: call `tick()` with the current epoch, and — in the same
//! critical section — call `SensorData::serialize()` to get a snapshot when
//! this returns `true`.

/// Default interval between flash writes. Matches the prior hard-coded
/// `SAVE_INTERVAL_S`. Kept as a const so call sites can use it without
/// threading a config value through.
pub const DEFAULT_SAVE_INTERVAL_S: u32 = 600;

pub struct SaveScheduler {
    interval_s: u32,
    /// `None` until the first non-`None` tick anchors the timer. Without
    /// this anchor we'd fire on the first tick after a fresh boot — which
    /// would immediately rewrite a just-loaded blob.
    last_save_s: Option<u32>,
}

impl SaveScheduler {
    pub fn new(interval_s: u32) -> Self {
        assert!(interval_s > 0, "SaveScheduler interval must be > 0");
        Self {
            interval_s,
            last_save_s: None,
        }
    }

    /// Advance the timer. Returns `true` when the caller should emit a save
    /// payload (i.e. `interval_s` has elapsed since the last fire).
    ///
    /// The first call with `Some(_)` anchors the timer without firing, so a
    /// fresh or just-restored state never triggers an immediate rewrite.
    pub fn tick(&mut self, now_epoch: Option<u32>) -> bool {
        let Some(t) = now_epoch else {
            return false;
        };
        match self.last_save_s {
            None => {
                self.last_save_s = Some(t);
                false
            }
            Some(last) if t.saturating_sub(last) >= self.interval_s => {
                self.last_save_s = Some(t);
                true
            }
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_tick_anchors_without_firing() {
        let mut s = SaveScheduler::new(600);
        assert!(!s.tick(Some(1000)));
        // Same epoch immediately after: still below the interval.
        assert!(!s.tick(Some(1000)));
    }

    #[test]
    fn fires_exactly_at_interval_boundary() {
        let mut s = SaveScheduler::new(600);
        assert!(!s.tick(Some(1000)));
        assert!(!s.tick(Some(1599)));
        assert!(s.tick(Some(1600)));
    }

    #[test]
    fn fires_repeatedly_with_steady_cadence() {
        let mut s = SaveScheduler::new(100);
        assert!(!s.tick(Some(0)));
        assert!(s.tick(Some(100)));
        // Halfway through the next interval: no fire.
        assert!(!s.tick(Some(150)));
        assert!(s.tick(Some(200)));
        assert!(s.tick(Some(300)));
    }

    #[test]
    fn none_epoch_never_fires_or_anchors() {
        // Before NTP sync we get None; the timer must stay un-anchored so
        // the first real epoch doesn't prematurely fire.
        let mut s = SaveScheduler::new(600);
        for _ in 0..1000 {
            assert!(!s.tick(None));
        }
        // First real time just anchors; doesn't fire.
        assert!(!s.tick(Some(1000)));
        assert!(!s.tick(Some(1500)));
        assert!(s.tick(Some(1600)));
    }

    #[test]
    fn backwards_clock_does_not_fire() {
        // An NTP step-back (say wall clock jumps from 2000 to 500) must not
        // produce a spurious fire — saturating_sub yields 0 < interval.
        let mut s = SaveScheduler::new(600);
        assert!(!s.tick(Some(2000)));
        assert!(!s.tick(Some(500)));
        assert!(!s.tick(Some(1000)));
        // Forward again past last anchor (2000) + interval.
        assert!(s.tick(Some(2600)));
    }

    #[test]
    #[should_panic(expected = "SaveScheduler interval must be > 0")]
    fn zero_interval_panics() {
        SaveScheduler::new(0);
    }
}
