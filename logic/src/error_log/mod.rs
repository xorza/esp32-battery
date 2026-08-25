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

/// One source's event kinds, as `EventLog` counts them.
///
/// The three sources are otherwise unrelated types — `InaError` and `XyError`
/// are real error types their producers return — so this exists only to let
/// the log keep one flat counter array and one set of accessors instead of a
/// parallel set per source. `OFFSET` chains off the previous source's, so a
/// fourth one only has to continue the chain.
pub trait EventKind:
    // `Iterator: 'static` is strum's own shape — a plain owned struct over the
    // variants. Stating it here rather than at each use keeps the bound off
    // every caller's signature.
    Copy + Into<&'static str> + IntoEnumIterator<Iterator: 'static> + EnumCount
{
    /// Where this source's block starts in the flat counter array.
    const OFFSET: usize;

    /// Position within this source's block. Relies on declaration order being
    /// the numeric discriminant order (true for unit-only enums in Rust).
    fn index(self) -> usize;

    /// Slot in the flat counter array.
    fn slot(self) -> usize {
        Self::OFFSET + self.index()
    }

    fn name(self) -> &'static str {
        self.into()
    }
}

impl EventKind for InaError {
    const OFFSET: usize = 0;
    fn index(self) -> usize {
        self as usize
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
    /// A fault stopped the charge but left the output up: the buck holds
    /// the float target and the load stays fed. Reboot-only, like a latch.
    Parked,
}

impl EventKind for XyError {
    const OFFSET: usize = <InaError as EnumCount>::COUNT;
    fn index(self) -> usize {
        self as usize
    }
}

impl EventKind for ChargeTransition {
    const OFFSET: usize = <XyError as EventKind>::OFFSET + <XyError as EnumCount>::COUNT;
    fn index(self) -> usize {
        self as usize
    }
}

impl XyError {
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

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Event {
    Ina(InaError),
    Xy(XyError),
    Charge(ChargeTransition),
}

/// How an event names itself on the wire: which producer it came from, and
/// which kind within that producer. One struct rather than two accessors so
/// the source tags live beside the kinds they label and `/api/errors` needs
/// no match of its own.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EventName {
    pub source: &'static str,
    pub kind: &'static str,
}

impl Event {
    pub fn name(self) -> EventName {
        match self {
            Self::Ina(k) => EventName {
                source: "ina",
                kind: k.name(),
            },
            Self::Xy(k) => EventName {
                source: "xy",
                kind: k.name(),
            },
            Self::Charge(k) => EventName {
                source: "charge",
                kind: k.name(),
            },
        }
    }

    fn slot(self) -> usize {
        match self {
            Self::Ina(k) => k.slot(),
            Self::Xy(k) => k.slot(),
            Self::Charge(k) => k.slot(),
        }
    }
}

#[derive(Copy, Clone, Debug)]
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
    /// Lifetime totals, one slot per kind across all sources — see
    /// [`EventKind::slot`]. Flat rather than one array per source so the
    /// record/read paths are written once.
    counts: [u32; TOTAL_KINDS],
}

/// Width of the flat counter array: every source's block, end to end.
const TOTAL_KINDS: usize =
    <ChargeTransition as EventKind>::OFFSET + <ChargeTransition as EnumCount>::COUNT;

impl Default for EventLog {
    fn default() -> Self {
        Self::new()
    }
}

impl EventLog {
    pub fn new() -> Self {
        Self {
            recent: Deque::new(),
            counts: [0; TOTAL_KINDS],
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
        let slot = event.slot();
        self.counts[slot] = self.counts[slot].saturating_add(1);
    }

    /// Iterate ring entries oldest-first.
    pub fn recent(&self) -> impl Iterator<Item = &TimedEvent> {
        self.recent.iter()
    }

    /// Lifetime total for one kind, saturating at `u32::MAX`. Survives ring
    /// eviction.
    pub fn count<K: EventKind>(&self, kind: K) -> u32 {
        self.counts[kind.slot()]
    }

    /// Iterate `(name, count)` pairs for every kind of one source. Lets API
    /// callers serialize without depending on `strum` themselves.
    pub fn counts_iter<K: EventKind>(&self) -> impl Iterator<Item = (&'static str, u32)> + '_ {
        K::iter().map(|k| (k.name(), self.count(k)))
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
