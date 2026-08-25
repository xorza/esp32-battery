//! Wall-clock source backed by ESP-IDF's SNTP-driven system time.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use log::{info, warn};

use esp32_battery_logic::{Event, EventLog};

/// Plausibility bounds on the system clock. The SNTP callback fires even when
/// the system time hasn't actually been set to something sensible (e.g. a
/// poisoned reply, a captive-portal NTP spoof, or a pre-sync tick), and once
/// we commit a bogus epoch value it poisons every later sample.
pub const VALID_EPOCH_S: std::ops::Range<u64> = 1_700_000_000..4_102_444_800;

/// Cheap-to-clone wall-clock source. `epoch_s` returns `None` until the SNTP
/// callback validates a real time and calls `mark_synced`.
#[derive(Clone)]
pub struct EspClock {
    ntp_synced: Arc<AtomicBool>,
}

impl EspClock {
    pub fn new() -> Self {
        Self {
            ntp_synced: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn mark_synced(&self) {
        self.ntp_synced.store(true, Ordering::Relaxed);
    }

    pub fn epoch_s(&self) -> Option<u32> {
        if !self.ntp_synced.load(Ordering::Relaxed) {
            return None;
        }
        let t = esp_idf_svc::systime::EspSystemTime.now().as_secs();
        if !VALID_EPOCH_S.contains(&t) {
            return None;
        }
        Some(t as u32)
    }
}

/// Monotonic time since boot. Backed by `esp_timer_get_time`
/// (microseconds), same hardware counter that backs `Instant::now()` —
/// preferred over `Instant` because no baseline needs to be threaded
/// through call sites and the value is directly meaningful as "elapsed
/// since boot".
pub fn uptime() -> Duration {
    let micros = unsafe { esp_idf_svc::sys::esp_timer_get_time() } as u64;
    Duration::from_micros(micros)
}

/// Measures one loop iteration against the next.
///
/// Both supervisor loops charge time-based windows — the charge supervisor's
/// debounces, the sensor staleness clocks — and neither runs at its nominal
/// period: an xy iteration also pays that tick's Modbus traffic, and a main
/// iteration can block for seconds inside a slow association. Handing those
/// windows the nominal period instead of the measured one makes every single
/// one of them fire late.
#[derive(Debug)]
pub struct LoopTimer {
    last: Duration,
}

impl LoopTimer {
    pub fn start() -> Self {
        Self { last: uptime() }
    }

    /// Advance to the current instant, returning the interval since the
    /// previous advance. Nothing clamps the result: both callers feed a task
    /// watchdog that reboots the device long before an interval could grow
    /// large enough to matter.
    pub fn lap(&mut self) -> Duration {
        let now = uptime();
        let elapsed = now.saturating_sub(self.last);
        self.last = now;
        elapsed
    }

    /// The instant the last [`lap`](Self::lap) observed, so a caller needing
    /// both the interval and the timestamp reads the hardware counter once.
    pub fn now(&self) -> Duration {
        self.last
    }
}

/// Pairs the event log with the wall clock used to timestamp entries.
/// Sensor threads always need both together — bundling them here removes
/// the per-thread `record(log, clock, kind)` helper duplicated in `ina.rs`
/// and `xy.rs`. Cheap to clone (two `Arc`s).
#[derive(Clone)]
pub struct EventRecorder {
    log: Arc<Mutex<EventLog>>,
    clock: EspClock,
}

impl EventRecorder {
    pub fn new(log: Arc<Mutex<EventLog>>, clock: EspClock) -> Self {
        Self { log, clock }
    }

    pub fn record(&self, event: Event) {
        let ts = self.clock.epoch_s().unwrap_or(0);
        self.log.lock().unwrap().record(ts, event);
    }
}

/// Start the SNTP client and validate every callback fire against
/// `VALID_EPOCH_S` before flipping the synced flag. The client lives for
/// the whole process lifetime — it handles WiFi flaps internally.
pub fn start_sntp(clock: EspClock) -> esp_idf_svc::sntp::EspSntp<'static> {
    info!("Starting NTP sync");
    // Smooth sync gradually nudges the clock instead of snapping it, so EventLog
    // timestamps stay monotonic even after a long unsynced stretch.
    let conf = esp_idf_svc::sntp::SntpConf {
        sync_mode: esp_idf_svc::sntp::SyncMode::Smooth,
        ..Default::default()
    };
    esp_idf_svc::sntp::EspSntp::new_with_callback(&conf, move |synced_at| {
        // The SNTP callback can fire with a bogus time (bad server, DNS
        // hijack, pre-sync tick). Only flip the flag once the reported
        // epoch is within the plausibility window — otherwise a bogus
        // value reaches SensorData and poisons history.
        let secs = synced_at.as_secs();
        if VALID_EPOCH_S.contains(&secs) {
            info!("NTP synced: epoch={secs}");
            clock.mark_synced();
        } else {
            warn!("NTP sync ignored: implausible epoch={secs}");
        }
    })
    .unwrap()
}
