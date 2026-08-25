//! Whether this board can see the pack's temperature at all.

/// Whether a pack temperature sensor is fitted.
///
/// Declared rather than inferred, because a sensor that has failed and a
/// sensor that was never fitted produce the same absent reading — and only
/// one of them is safe to charge through. Inferring would mean either
/// refusing to charge on every board that has no sensor, or charging
/// blindly on a board whose sensor just died. Neither is acceptable, so the
/// board says which it is.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PackTemp {
    /// No sensor fitted. Charging proceeds with no temperature check, which
    /// is an **accepted risk**: charging lithium below freezing plates
    /// metal onto the anode, and the damage is cumulative and invisible
    /// until the cell fails. The only thing standing between this board and
    /// that is the pack's own BMS, which is therefore not optional.
    Absent,
    /// Sensor fitted. An absent or stale reading is a fault, the same way a
    /// dead INA228 is — the supervisor refuses to charge on a measurement
    /// it does not have.
    Fitted,
}
