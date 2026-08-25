//! What the charge supervisor publishes about itself.

use crate::charging::fault_reason::FaultReason;
use crate::charging::inhibit_reason::InhibitReason;
use crate::charging::phase::Phase;

/// Supervisor → UI mailbox: written by the XY thread once per poll, read by
/// `/api` and the LCD.
///
/// Behind its own mutex rather than sharing [`crate::SensorData`]'s, because
/// the two have opposite access shapes. This is five `Copy` fields written
/// together every second; the reading store carries a 4 KB history that
/// `/api` holds for the length of a serialization. Sharing one lock made the
/// poll thread queue behind that serialization every tick.
///
/// `Copy` on purpose: every reader takes the lock, copies the whole struct
/// out, and releases it — so this lock is never held across another one and
/// cannot deadlock against the sensor-data lock whichever order a caller
/// wants them in.
///
/// Same poison contract as `SensorData`: `.lock().unwrap()` is deliberate,
/// since the panic hook reboots the device on any thread panic.
#[derive(Clone, Copy, Debug, Default)]
pub struct ChargeStatus {
    /// XY `MODEL` register (`0x0016`) read once at boot. `0` = not yet read.
    /// Diagnostic only — confirms the configured `Model`'s scale family.
    pub model_code: u16,
    /// `true` while the buck reports input UVLO (`ProtectionStatus::Lvp`) —
    /// the DC supply was disconnected or sagged. Set live each XY poll, so
    /// it self-clears when the supply returns. Surfaced to LCD/web as a
    /// benign "PS offline" status rather than a fault, since it recovers on
    /// its own without operator action.
    pub ps_offline: bool,
    /// Current charging phase, or `None` while the supervisor is still in
    /// Pending bring-up / latched off.
    pub phase: Option<Phase>,
    /// Latched supervisor fault, if any. `None` during normal operation;
    /// `Some(reason)` once the buck has been latched off, and it stays set
    /// until a reboot. Conditions that recover on their own report through
    /// [`Self::inhibit`] instead and never reach this field.
    pub fault: Option<FaultReason>,
    /// Why the supervisor is holding the buck off without having latched.
    /// `None` while regulating normally or once a fault has latched. Unlike
    /// [`Self::fault`] this self-clears, so it distinguishes "waiting for the
    /// input rail" from "the INA228 is dead" — both of which otherwise look
    /// like a dark output with no phase.
    pub inhibit: Option<InhibitReason>,
}
