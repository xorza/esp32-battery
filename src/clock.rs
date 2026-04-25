//! Wall-clock source backed by ESP-IDF's SNTP-driven system time.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use log::{info, warn};

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

/// Monotonic seconds since boot, from `esp_timer_get_time` (microseconds).
pub fn uptime_s() -> u32 {
    (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000) as u32
}

/// Start the SNTP client and validate every callback fire against
/// `VALID_EPOCH_S` before flipping the synced flag. The client lives for
/// the whole process lifetime — it handles WiFi flaps internally.
pub fn start_sntp(clock: EspClock) -> esp_idf_svc::sntp::EspSntp<'static> {
    info!("Starting NTP sync");
    esp_idf_svc::sntp::EspSntp::new_with_callback(
        &esp_idf_svc::sntp::SntpConf::default(),
        move |synced_at| {
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
        },
    )
    .unwrap()
}
