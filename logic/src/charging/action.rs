//! What the poll loop should do this tick, and the tickets that
//! authorise committing each one.

use crate::charging::fault_reason::FaultReason;
use crate::charging::phase::Phase;

/// Proof that this tick asked for output-on, and the only key that opens
/// [`ChargeSupervisor::commit_enable`]. Neither `Copy` nor `Clone`, and
/// its fields are private to this module, so a caller cannot commit an
/// enable it was never handed, commit the same one twice, or supply a
/// `resume_absorb` of its own invention — the supervisor's answer rides
/// along inside.
#[derive(Debug)]
pub struct EnableTicket {
    pub(super) resume_absorb: bool,
}

impl EnableTicket {
    /// Whether the first regulating tick steps V_SET straight to absorb
    /// (the pack's resting SoC was short of full) or parks in Float.
    /// Exposed for logging; committing uses it either way.
    pub fn resume_absorb(&self) -> bool {
        self.resume_absorb
    }
}

/// Proof that this tick asked for a V_SET change, and the key to
/// [`ChargeSupervisor::commit_voltage`]. Carries the phase being moved
/// to, so [`apply_update_voltage`] can name it in logs without reaching
/// back into the supervisor, and so the commit can check the ticket
/// against the retarget actually outstanding.
#[derive(Debug)]
pub struct VoltageTicket {
    /// Phase this write transitions into once committed.
    pub(super) phase: Phase,
    pub(super) target_v: f32,
    /// `true` when `target_v` is *below* the live V_SET, meaning
    /// [`apply_update_voltage`] must disable output before writing V_SET
    /// and re-enable after, in that order. Stepping V_SET down with
    /// output enabled drives reverse current through the buck's
    /// synchronous low-side FET (the battery sources back into the buck
    /// as the control loop pulls V_OUT down to the new setpoint), which
    /// can destroy the FET and propagate upstream through the input rail
    /// — the XY7025 has no anti-backup protection on either port.
    /// `false` means a step-up, safe to do live.
    pub(super) cycle_output: bool,
}

/// Proof that this tick latched a fault, and the key to
/// [`ChargeSupervisor::commit_disable`].
#[derive(Debug)]
pub struct DisableTicket {
    pub(super) reason: FaultReason,
}

impl DisableTicket {
    pub fn reason(&self) -> FaultReason {
        self.reason
    }
}
/// What the poll loop should do this tick.
///
/// The supervisor boots output-OFF, having not yet decided it is safe to
/// enable. Each tick re-runs the same safety checks as the sourcing path;
/// once all clear it emits `EnableOutput` and stays where it is until the
/// caller commits the ticket. After that the phase machine, drift check
/// and fault paths run. A fault ends the charge one of two ways. If it
/// disables, only `DisableOutput` is emitted until the write is committed
/// and the supervisor then sits in `Action::None`. If it parks, one last
/// `UpdateVoltage` steps down to the float target and the supervisor sits
/// in `Action::None` there with the output still up. Both are reboot-only;
/// transient protection causes LVP/OTP are neither, and get waited out in
/// a hold.
///
/// Every non-`None` variant carries a ticket. Perform the write, then
/// commit the ticket only if the write succeeded: dropping it instead is
/// how a failed Modbus write becomes a retry on the next tick.
#[derive(Debug)]
pub enum Action {
    None,
    /// Write `set_output(true)`, then [`ChargeSupervisor::commit_enable`].
    EnableOutput(EnableTicket),
    /// Hand the ticket to [`apply_update_voltage`], then
    /// [`ChargeSupervisor::commit_voltage`] if it reports `Committed`.
    /// Re-emitted every tick until committed, so a transient Modbus
    /// glitch on the write retries instead of latching `SettingsDrift`.
    UpdateVoltage(VoltageTicket),
    /// Write `set_output(false)`, then [`ChargeSupervisor::commit_disable`].
    DisableOutput(DisableTicket),
}
