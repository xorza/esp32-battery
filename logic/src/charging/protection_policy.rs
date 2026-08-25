//! What the supervisor makes of a buck `PROTECT` cause.

use xy_modbus::ProtectionStatus;

/// The two questions anything asks about a `PROTECT` register value.
///
/// A trait on the foreign enum rather than a scattering of `Lvp | Otp`
/// patterns: the supervisor decides from these whether to latch or wait, and
/// the firmware decides from them what to show and what to log. They have to
/// agree, so they read the same answer rather than each spelling out the rule.
///
/// Both default to `false` for a cause not named below, which is the
/// fail-safe direction: an unrecognised protection latches the buck off
/// rather than being waited out forever.
pub trait ProtectionPolicy {
    /// The buck is holding itself off but is otherwise healthy, and will
    /// re-enable `OUTPUT_EN` by itself once the condition lifts. Sensor-driven
    /// rather than a true latch, so the supervisor waits it out instead of
    /// latching, and declines to bring up into one.
    fn is_self_clearing(self) -> bool;

    /// The DC input went away, or sagged below the buck's LVP setpoint. A
    /// subset of [`Self::is_self_clearing`]: the routine "supply unplugged"
    /// case, which the dashboard reports as a status and the event log
    /// ignores — unlike a thermal hold, which is worth recording.
    fn is_input_loss(self) -> bool;
}

impl ProtectionPolicy for ProtectionStatus {
    fn is_self_clearing(self) -> bool {
        matches!(self, ProtectionStatus::Lvp | ProtectionStatus::Otp)
    }

    fn is_input_loss(self) -> bool {
        matches!(self, ProtectionStatus::Lvp)
    }
}

#[cfg(test)]
mod tests {
    use xy_modbus::ProtectionStatus;

    use crate::charging::protection_policy::ProtectionPolicy;

    #[test]
    fn only_sensor_side_causes_are_waited_out() {
        // Every cause the driver can report, with what the supervisor must
        // make of it. Otp is the case that keeps the two predicates apart:
        // waited out like an input sag, but recorded, because a buck that ran
        // hot enough to shut down is worth a log entry and an unplugged
        // supply is not.
        const CASES: [(ProtectionStatus, bool, bool); 11] = [
            (ProtectionStatus::Normal, false, false),
            (ProtectionStatus::Ovp, false, false),
            (ProtectionStatus::Ocp, false, false),
            (ProtectionStatus::Opp, false, false),
            (ProtectionStatus::Lvp, true, true),
            (ProtectionStatus::Oah, false, false),
            (ProtectionStatus::Ohp, false, false),
            (ProtectionStatus::Otp, true, false),
            (ProtectionStatus::Oep, false, false),
            (ProtectionStatus::Owh, false, false),
            (ProtectionStatus::Icp, false, false),
        ];
        for (cause, self_clearing, input_loss) in CASES {
            assert_eq!(cause.is_self_clearing(), self_clearing, "{cause}");
            assert_eq!(cause.is_input_loss(), input_loss, "{cause}");
            assert!(
                !input_loss || self_clearing,
                "{cause}: input loss has to be a subset of self-clearing, or the \
                 supervisor would latch on a cause the dashboard calls benign"
            );
        }
    }
}
