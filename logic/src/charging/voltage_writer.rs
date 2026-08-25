//! Firmware-side V_SET sequencing: the one I/O-shaped helper, kept here
//! so the safe step-down is host-testable against a mock.

use log::{error, info, warn};

use crate::charging::action::VoltageTicket;
use crate::error_log::XyError;
use xy_modbus::XyError as BusError;

/// Minimal device interface used by [`apply_update_voltage`] — only the
/// two writes the safe step-down sequence needs. Lives here (not in
/// firmware) so the sequencing is host-testable with a mock.
pub trait VoltageWriter {
    fn set_voltage(&mut self, volts: f32) -> Result<(), BusError>;
    fn set_output(&mut self, on: bool) -> Result<(), BusError>;
}

/// Whether [`apply_update_voltage`] got V_SET onto the device, and so
/// whether its ticket should be committed.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VoltageWriteOutcome {
    /// V_SET is on the device — commit the ticket so the supervisor's
    /// drift check switches to the new target next tick. This includes
    /// the case where a step-down's re-enable failed: the setpoint
    /// really did change, and the next tick latches
    /// `OutputUnexpectedlyOff` for the dark buck.
    Committed,
    /// V_SET was not written — drop the ticket. The supervisor stays on
    /// the old phase, its drift check keeps matching the old V_SET, and
    /// the next tick re-emits `UpdateVoltage` for another attempt.
    Retry,
}

/// Execute one `Action::UpdateVoltage`. For a step-up
/// (`cycle_output == false`) writes V_SET live. For a step-down runs
/// `set_output(false)` → settle → `set_voltage` → `set_output(true)`.
/// Partial failures attempt a best-effort restore so a single transient
/// Modbus glitch doesn't drop the UPS load:
///
/// - Step-2 failure (`set_voltage` after the disable): re-enable output
///   and report `Retry`, so the supervisor re-runs the whole sequence
///   next tick instead of latching.
/// - Step-3 failure (`set_output(true)` after V_SET landed): retried
///   once inline. If it still fails the outcome is `Committed` anyway —
///   V_SET did change — and the supervisor latches on the next tick.
///
/// Takes the ticket by reference and returns an outcome rather than
/// touching the supervisor, so control flows one direction and this
/// stays a pure sequence of device writes. `settle` is the quiet window
/// between the disable and the V_SET write (a no-op in tests); it is
/// passed as a closure so this module needs no clock of its own.
/// `on_error` is invoked once per Modbus error for the firmware's event log.
pub fn apply_update_voltage<W: VoltageWriter>(
    xy: &mut W,
    ticket: &VoltageTicket,
    settle: impl FnOnce(),
    mut on_error: impl FnMut(XyError),
) -> VoltageWriteOutcome {
    let target_v = ticket.target_v;
    let phase = ticket.phase.label();

    if !ticket.cycle_output {
        return match xy.set_voltage(target_v) {
            Ok(()) => {
                info!("charge phase → {phase}: V_set = {target_v:.2} V");
                VoltageWriteOutcome::Committed
            }
            Err(e) => {
                warn!("XY set_voltage({target_v}): {e} — supervisor will retry next tick");
                on_error(XyError::SetVoltage);
                VoltageWriteOutcome::Retry
            }
        };
    }

    info!("charge phase step-down → V_set = {target_v:.2} V (cycling output)");
    if let Err(e) = xy.set_output(false) {
        warn!("XY safe-step-down set_output(false): {e} — will retry next tick");
        on_error(XyError::SetOutput);
        return VoltageWriteOutcome::Retry;
    }
    settle();
    if let Err(e) = xy.set_voltage(target_v) {
        warn!("XY safe-step-down set_voltage({target_v}): {e} — attempting output restore");
        on_error(XyError::SetVoltage);
        // Best-effort restore so the UPS load stays powered. The ticket
        // goes uncommitted, so the supervisor re-emits UpdateVoltage.
        if let Err(e) = xy.set_output(true) {
            error!("XY safe-step-down restore set_output(true): {e} — buck OFF, will latch");
            on_error(XyError::SetOutput);
        }
        return VoltageWriteOutcome::Retry;
    }
    // Step 3: re-enable. Retry once inline to ride out a single transient
    // glitch; persistent failure is safe because V_SET is already on the
    // device, so the next tick sees a dark buck and latches.
    let enable = xy.set_output(true).or_else(|e| {
        warn!("XY safe-step-down set_output(true) attempt 1: {e} — retrying");
        xy.set_output(true)
    });
    match enable {
        Ok(()) => info!("charge phase → {phase}: V_set = {target_v:.2} V (step-down complete)"),
        Err(e) => {
            error!("XY safe-step-down set_output(true): {e} — buck OFF after voltage commit");
            on_error(XyError::SetOutput);
        }
    }
    VoltageWriteOutcome::Committed
}
