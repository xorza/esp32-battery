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
}

/// Supervisor latch-state transitions. Recorded on every change so the
/// log shows the *route* into a state, not just the destination: "flapped
/// protect-hold four times in ninety seconds, then latched" reads very
/// differently from a bare latch. The specific `FaultReason` behind a
/// `Latched` is live on `SensorData::charge_fault`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, EnumCount, EnumIter, IntoStaticStr)]
#[strum(serialize_all = "snake_case")]
pub enum ChargeTransition {
    /// Bring-up committed — the buck is on and the phase machine starts.
    Energised,
    /// A regulating buck self-disabled on a self-clearing protection
    /// (input UVLO / over-temp); the supervisor stepped back to bring-up
    /// to wait it out.
    ProtectHold,
    /// The protection cause cleared and the buck came back on by itself.
    ProtectCleared,
    /// A fault latched. Reboot-only recovery from here.
    Latched,
}

impl ChargeTransition {
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
        })
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Event {
    Ina(InaError),
    Xy(XyError),
    Charge(ChargeTransition),
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
    charge_counts: [u32; <ChargeTransition as EnumCount>::COUNT],
}

impl Default for EventLog {
    fn default() -> Self {
        Self::new()
    }
}

impl EventLog {
    pub fn new() -> Self {
        Self {
            recent: Deque::new(),
            ina_counts: [0; <InaError as EnumCount>::COUNT],
            xy_counts: [0; <XyError as EnumCount>::COUNT],
            charge_counts: [0; <ChargeTransition as EnumCount>::COUNT],
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
            Event::Charge(k) => {
                let i = k.index();
                self.charge_counts[i] = self.charge_counts[i].saturating_add(1);
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

    pub fn charge_count(&self, k: ChargeTransition) -> u32 {
        self.charge_counts[k.index()]
    }

    /// Iterate `(name, count)` pairs for every INA error kind. Lets API
    /// callers serialize without depending on `strum` themselves.
    pub fn ina_counts_iter(&self) -> impl Iterator<Item = (&'static str, u32)> + '_ {
        InaError::iter().map(|k| (k.name(), self.ina_count(k)))
    }

    pub fn xy_counts_iter(&self) -> impl Iterator<Item = (&'static str, u32)> + '_ {
        XyError::iter().map(|k| (k.name(), self.xy_count(k)))
    }

    pub fn charge_counts_iter(&self) -> impl Iterator<Item = (&'static str, u32)> + '_ {
        ChargeTransition::iter().map(|k| (k.name(), self.charge_count(k)))
    }

    pub fn len(&self) -> usize {
        self.recent.len()
    }

    pub fn is_empty(&self) -> bool {
        self.recent.is_empty()
    }
}

#[cfg(test)]
mod tests;
