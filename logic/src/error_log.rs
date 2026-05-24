//! Structured event log for non-fatal sensor + buck failures.
//!
//! Recent entries in a fixed-capacity ring, plus per-kind lifetime
//! counters that survive ring eviction. Pure data — producers pass an
//! epoch timestamp; logic does no I/O and no clock access.

use heapless::Deque;
use strum::{EnumCount, EnumIter, IntoEnumIterator, IntoStaticStr};
use xy_modbus::ProtectionStatus;

/// Failures from the INA228 / I²C bus path.
#[derive(Copy, Clone, Debug, PartialEq, Eq, EnumCount, EnumIter, IntoStaticStr)]
#[strum(serialize_all = "snake_case")]
pub enum InaError {
    /// Probe / calibration during thread startup failed.
    Init,
    BusVoltageRead,
    CurrentRead,
    PowerRead,
}

impl InaError {
    /// Stable index 0..COUNT — relies on declaration order being the
    /// numeric discriminant order (true for unit-only enums in Rust).
    #[inline]
    pub fn index(self) -> usize {
        self as usize
    }
    #[inline]
    pub fn name(self) -> &'static str {
        self.into()
    }
}

/// Failures from the XY7025 / Modbus-RTU path.
#[derive(Copy, Clone, Debug, PartialEq, Eq, EnumCount, EnumIter, IntoStaticStr)]
#[strum(serialize_all = "snake_case")]
pub enum XyError {
    ReadStatus,
    SetVoltage,
    SetOutput,
    BootSequence,
    // Latched buck protection causes (PROTECT register). One per
    // `ProtectionStatus` fault so the cause survives in counts + recent.
    ProtectOvp,
    ProtectOcp,
    ProtectOpp,
    ProtectLvp,
    ProtectOah,
    ProtectOhp,
    ProtectOtp,
    ProtectOep,
    ProtectOwh,
    ProtectIcp,
    ProtectUnknown,
}

impl XyError {
    #[inline]
    pub fn index(self) -> usize {
        self as usize
    }
    #[inline]
    pub fn name(self) -> &'static str {
        self.into()
    }

    /// Event kind for a latched protection cause. `Normal` is not a fault,
    /// so it maps to `None`.
    pub fn from_protection(p: ProtectionStatus) -> Option<Self> {
        Some(match p {
            ProtectionStatus::Normal => return None,
            ProtectionStatus::Ovp => Self::ProtectOvp,
            ProtectionStatus::Ocp => Self::ProtectOcp,
            ProtectionStatus::Opp => Self::ProtectOpp,
            ProtectionStatus::Lvp => Self::ProtectLvp,
            ProtectionStatus::Oah => Self::ProtectOah,
            ProtectionStatus::Ohp => Self::ProtectOhp,
            ProtectionStatus::Otp => Self::ProtectOtp,
            ProtectionStatus::Oep => Self::ProtectOep,
            ProtectionStatus::Owh => Self::ProtectOwh,
            ProtectionStatus::Icp => Self::ProtectIcp,
            ProtectionStatus::Unknown(_) => Self::ProtectUnknown,
        })
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Event {
    Ina(InaError),
    Xy(XyError),
}

#[derive(Copy, Clone)]
pub struct TimedEvent {
    /// Epoch seconds. `0` means the entry was recorded before NTP sync —
    /// readers can't distinguish ordering of pre-sync entries beyond
    /// ring position.
    pub ts: u32,
    pub event: Event,
}

const CAPACITY: usize = 32;

/// Shared via `Arc<Mutex<_>>` across producers (INA/XY threads via
/// `EventRecorder`) and readers (HTTP, LCD). Same poison contract as
/// `SensorData`: `.lock().unwrap()` is intentional — the panic hook in
/// `src/main.rs` reboots on any thread panic, so a poisoned lock is
/// unreachable.
pub struct EventLog {
    recent: Deque<TimedEvent, CAPACITY>,
    ina_counts: [u32; <InaError as EnumCount>::COUNT],
    xy_counts: [u32; <XyError as EnumCount>::COUNT],
}

impl Default for EventLog {
    fn default() -> Self {
        Self::new()
    }
}

impl EventLog {
    pub const CAPACITY: usize = CAPACITY;

    pub fn new() -> Self {
        Self {
            recent: Deque::new(),
            ina_counts: [0; <InaError as EnumCount>::COUNT],
            xy_counts: [0; <XyError as EnumCount>::COUNT],
        }
    }

    /// Append an event. Evicts the oldest ring entry when full; counters
    /// always increment (saturating at `u32::MAX`).
    pub fn record(&mut self, ts: u32, event: Event) {
        if self.recent.is_full() {
            self.recent.pop_front();
        }
        self.recent
            .push_back(TimedEvent { ts, event })
            .ok()
            .expect("ring has a free slot after pop_front");
        match event {
            Event::Ina(k) => {
                let i = k.index();
                self.ina_counts[i] = self.ina_counts[i].saturating_add(1);
            }
            Event::Xy(k) => {
                let i = k.index();
                self.xy_counts[i] = self.xy_counts[i].saturating_add(1);
            }
        }
    }

    /// Iterate ring entries oldest-first.
    pub fn recent(&self) -> impl Iterator<Item = &TimedEvent> {
        self.recent.iter()
    }

    pub fn ina_count(&self, k: InaError) -> u32 {
        self.ina_counts[k.index()]
    }

    pub fn xy_count(&self, k: XyError) -> u32 {
        self.xy_counts[k.index()]
    }

    /// Iterate `(name, count)` pairs for every INA error kind. Lets API
    /// callers serialize without depending on `strum` themselves.
    pub fn ina_counts_iter(&self) -> impl Iterator<Item = (&'static str, u32)> + '_ {
        InaError::iter().map(|k| (k.name(), self.ina_count(k)))
    }

    pub fn xy_counts_iter(&self) -> impl Iterator<Item = (&'static str, u32)> + '_ {
        XyError::iter().map(|k| (k.name(), self.xy_count(k)))
    }

    pub fn len(&self) -> usize {
        self.recent.len()
    }

    pub fn is_empty(&self) -> bool {
        self.recent.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use strum::IntoEnumIterator;

    #[test]
    fn new_is_empty() {
        let log = EventLog::new();
        assert!(log.is_empty());
        assert_eq!(log.len(), 0);
        for k in InaError::iter() {
            assert_eq!(log.ina_count(k), 0);
        }
        for k in XyError::iter() {
            assert_eq!(log.xy_count(k), 0);
        }
    }

    #[test]
    fn record_bumps_counter_and_appends() {
        let mut log = EventLog::new();
        log.record(100, Event::Ina(InaError::CurrentRead));
        assert_eq!(log.len(), 1);
        assert_eq!(log.ina_count(InaError::CurrentRead), 1);
        assert_eq!(log.ina_count(InaError::Init), 0);
        let e = log.recent().next().unwrap();
        assert_eq!(e.ts, 100);
        assert!(matches!(e.event, Event::Ina(InaError::CurrentRead)));
    }

    #[test]
    fn distinct_kinds_within_source_counted_separately() {
        let mut log = EventLog::new();
        log.record(1, Event::Ina(InaError::CurrentRead));
        log.record(2, Event::Ina(InaError::CurrentRead));
        log.record(3, Event::Ina(InaError::PowerRead));
        assert_eq!(log.ina_count(InaError::CurrentRead), 2);
        assert_eq!(log.ina_count(InaError::PowerRead), 1);
        assert_eq!(log.ina_count(InaError::BusVoltageRead), 0);
    }

    #[test]
    fn ina_and_xy_counters_independent() {
        let mut log = EventLog::new();
        log.record(1, Event::Ina(InaError::CurrentRead));
        log.record(2, Event::Xy(XyError::ReadStatus));
        log.record(3, Event::Xy(XyError::ReadStatus));
        assert_eq!(log.ina_count(InaError::CurrentRead), 1);
        assert_eq!(log.xy_count(XyError::ReadStatus), 2);
    }

    #[test]
    fn iteration_order_is_oldest_first() {
        let mut log = EventLog::new();
        for ts in 1..=5 {
            log.record(ts, Event::Xy(XyError::ReadStatus));
        }
        let timestamps: heapless::Vec<u32, 5> = log.recent().map(|e| e.ts).collect();
        assert_eq!(&timestamps[..], &[1, 2, 3, 4, 5]);
    }

    #[test]
    fn ring_evicts_oldest_when_full() {
        let mut log = EventLog::new();
        // Fill capacity + 5 — first 5 must be evicted.
        for ts in 0..(EventLog::CAPACITY as u32 + 5) {
            log.record(ts, Event::Xy(XyError::ReadStatus));
        }
        assert_eq!(log.len(), EventLog::CAPACITY);
        let first_ts = log.recent().next().unwrap().ts;
        let last_ts = log.recent().last().unwrap().ts;
        assert_eq!(first_ts, 5);
        assert_eq!(last_ts, EventLog::CAPACITY as u32 + 4);
    }

    #[test]
    fn counters_survive_ring_eviction() {
        // Counter is the lifetime total — ring overflow must not lose it.
        let mut log = EventLog::new();
        let pushes = EventLog::CAPACITY as u32 + 17;
        for _ in 0..pushes {
            log.record(1, Event::Xy(XyError::ReadStatus));
        }
        assert_eq!(log.len(), EventLog::CAPACITY);
        assert_eq!(log.xy_count(XyError::ReadStatus), pushes);
    }

    #[test]
    fn pre_ntp_zero_timestamp_is_valid() {
        let mut log = EventLog::new();
        log.record(0, Event::Ina(InaError::Init));
        let e = log.recent().next().unwrap();
        assert_eq!(e.ts, 0);
        assert_eq!(log.ina_count(InaError::Init), 1);
    }

    #[test]
    fn names_are_unique_across_sources() {
        let mut seen: heapless::Vec<
            &str,
            { <InaError as EnumCount>::COUNT + <XyError as EnumCount>::COUNT },
        > = heapless::Vec::new();
        for k in InaError::iter() {
            assert!(seen.push(k.name()).is_ok());
        }
        for k in XyError::iter() {
            assert!(seen.push(k.name()).is_ok());
        }
        for i in 0..seen.len() {
            for j in (i + 1)..seen.len() {
                assert_ne!(seen[i], seen[j], "duplicate name at {i}/{j}");
            }
        }
    }

    #[test]
    fn indices_are_dense_and_match_count() {
        let mut ina_seen = [false; <InaError as EnumCount>::COUNT];
        for k in InaError::iter() {
            assert!(!ina_seen[k.index()], "duplicate index {}", k.index());
            ina_seen[k.index()] = true;
        }
        assert!(ina_seen.iter().all(|&b| b));

        let mut xy_seen = [false; <XyError as EnumCount>::COUNT];
        for k in XyError::iter() {
            assert!(!xy_seen[k.index()], "duplicate index {}", k.index());
            xy_seen[k.index()] = true;
        }
        assert!(xy_seen.iter().all(|&b| b));
    }

    #[test]
    fn from_protection_maps_every_cause() {
        // Normal is not a fault.
        assert_eq!(XyError::from_protection(ProtectionStatus::Normal), None);
        // Each fault cause maps to its own kind, names are cause-tagged.
        let cases = [
            (ProtectionStatus::Ovp, XyError::ProtectOvp, "protect_ovp"),
            (ProtectionStatus::Ocp, XyError::ProtectOcp, "protect_ocp"),
            (ProtectionStatus::Opp, XyError::ProtectOpp, "protect_opp"),
            (ProtectionStatus::Lvp, XyError::ProtectLvp, "protect_lvp"),
            (ProtectionStatus::Oah, XyError::ProtectOah, "protect_oah"),
            (ProtectionStatus::Ohp, XyError::ProtectOhp, "protect_ohp"),
            (ProtectionStatus::Otp, XyError::ProtectOtp, "protect_otp"),
            (ProtectionStatus::Oep, XyError::ProtectOep, "protect_oep"),
            (ProtectionStatus::Owh, XyError::ProtectOwh, "protect_owh"),
            (ProtectionStatus::Icp, XyError::ProtectIcp, "protect_icp"),
            (ProtectionStatus::Unknown(99), XyError::ProtectUnknown, "protect_unknown"),
        ];
        for (status, expected, name) in cases {
            assert_eq!(XyError::from_protection(status), Some(expected));
            assert_eq!(expected.name(), name);
        }
    }

    #[test]
    fn mixed_workload_keeps_each_source_consistent() {
        let mut log = EventLog::new();
        for i in 0..50 {
            if i % 3 == 0 {
                log.record(i, Event::Ina(InaError::CurrentRead));
            } else {
                log.record(i, Event::Xy(XyError::ReadStatus));
            }
        }
        // 0,3,6,…,48 → 17 INA. 50 - 17 = 33 XY.
        assert_eq!(log.ina_count(InaError::CurrentRead), 17);
        assert_eq!(log.xy_count(XyError::ReadStatus), 33);
        assert_eq!(log.len(), EventLog::CAPACITY);
    }
}
