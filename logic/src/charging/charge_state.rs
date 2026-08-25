//! The supervisor's state machine: every state it can occupy, every event
//! that moves it, and the flat table that says where each pair lands.

use strum::{EnumCount, VariantArray};

use crate::charging::phase::Phase;
use crate::error_log::ChargeTransition;

/// Every state the charge supervisor can occupy.
///
/// Flat and payload-free on purpose: the variant list *is* the state
/// space, so [`TRANSITIONS`] reads as a matrix whose cells can be pinned
/// one by one. The two questions every safety decision asks — is the buck
/// meant to be sourcing, and which V_SET is the device holding — are total
/// functions of the variant, so no second field can disagree with it.
#[derive(Copy, Clone, Debug, PartialEq, Eq, EnumCount, VariantArray)]
#[repr(u8)]
pub(crate) enum ChargeState {
    /// Cold start. Output off, device at the float target — what
    /// `boot_sequence` wrote and verified. Nothing should be sourcing, so
    /// a buck reporting `On` here is an anomaly that latches.
    Boot,
    /// Buck self-disabled out of [`ChargeState::Float`] on a cause it
    /// clears by itself. Output off, device still at the float target; a
    /// buck reporting `On` is the expected recovery, not a fault.
    HoldFloat,
    /// As [`ChargeState::HoldFloat`], entered from [`ChargeState::Absorb`],
    /// so the device is still at the absorb target.
    HoldAbsorb,
    /// Sourcing at the float target, watching for `enter_absorb_a`.
    Float,
    /// Sourcing at the absorb target, watching the taper and clocking
    /// `MAX_ABSORB`.
    Absorb,
    /// Sourcing, device still at the float target, a live step-up to
    /// absorb outstanding. Re-emitted every tick until committed.
    ToAbsorb,
    /// Sourcing, device still at the absorb target, an off→write→on
    /// step-down to float outstanding.
    ToFloat,
    /// A fault latched. `set_output(false)` not yet confirmed, so the
    /// disable is re-emitted every tick.
    Tripping,
    /// The disable landed. Terminal until a reboot.
    Latched,
}

/// What one tick concluded. At most one is produced per tick, and
/// [`TRANSITIONS`] maps `(state, event)` to where it lands.
#[derive(Copy, Clone, Debug, PartialEq, Eq, EnumCount, VariantArray)]
#[repr(u8)]
pub(super) enum ChargeEvent {
    /// The safety gauntlet latched a fault.
    Fault,
    /// The buck self-disabled on a self-clearing cause while sourcing.
    SelfDisabled,
    /// The buck's output came back on without us asking. Out of a hold
    /// that is the recovery being waited for; out of `Latched` it is a
    /// pack charging under a supervisor that already gave up, and the
    /// answer is to go disable it again.
    SelfEnabled,
    /// Caller confirmed `set_output(true)`, with the pack resting at the
    /// CV plateau — it is full, so regulation starts at the target the
    /// device already holds.
    Enabled,
    /// Caller confirmed `set_output(true)` with the pack resting below the
    /// plateau, so a step up to absorb is owed unless the device is
    /// already there.
    EnabledBelowFull,
    /// Charging current rose past `enter_absorb_a`.
    TaperRose,
    /// Charging current held under `exit_absorb_a` for `EXIT_DEBOUNCE`.
    TaperFell,
    /// Caller confirmed the V_SET write for the outstanding retarget.
    VoltageWritten,
    /// Caller confirmed `set_output(false)` after a latch.
    Disabled,
}

/// `TRANSITIONS[state][event]` is where that pair lands, or `None` when
/// the pair cannot arise. Rows follow [`ChargeState`]'s declaration order,
/// columns [`ChargeEvent`]'s — which is what makes this a table rather
/// than a nest of matches, and what lets the tests below walk every cell.
///
/// Two rows carry the rules that used to live in prose. `HoldAbsorb`
/// lands in `Absorb` on *either* enable event: the device is already at
/// the absorb target, so a drained pack has nothing to write and a full
/// one is left to the exit taper. And `ToAbsorb`/`ToFloat` fall back to
/// the hold for the target the device still holds, not the one they were
/// moving to, because an uncommitted ticket never reached the register.
/// The third is `Latched` accepting `SelfEnabled` back into `Tripping`: a
/// latch is only as good as the output actually being off, so a buck that
/// resurfaces is re-disabled rather than ignored. Nothing in the column
/// leads anywhere that sources.
#[rustfmt::skip]
const TRANSITIONS: [[Option<ChargeState>; ChargeEvent::COUNT]; ChargeState::COUNT] = {
    const X: Option<ChargeState> = None;
    const TRIP: Option<ChargeState> = Some(ChargeState::Tripping);
    const LTCH: Option<ChargeState> = Some(ChargeState::Latched);
    const FLT: Option<ChargeState> = Some(ChargeState::Float);
    const ABS: Option<ChargeState> = Some(ChargeState::Absorb);
    const TOA: Option<ChargeState> = Some(ChargeState::ToAbsorb);
    const TOF: Option<ChargeState> = Some(ChargeState::ToFloat);
    const HLDF: Option<ChargeState> = Some(ChargeState::HoldFloat);
    const HLDA: Option<ChargeState> = Some(ChargeState::HoldAbsorb);
    [
        //          Fault SelfDis SelfEn Enabled BelowFull TaperUp TaperDn VWritten Disabled
        /* Boot   */ [TRIP,    X,     X,   FLT,     TOA,      X,      X,      X,       X],
        /* HoldF  */ [TRIP,    X,   FLT,   FLT,     TOA,      X,      X,      X,       X],
        /* HoldA  */ [TRIP,    X,   ABS,   ABS,     ABS,      X,      X,      X,       X],
        /* Float  */ [TRIP, HLDF,     X,     X,       X,    TOA,      X,      X,       X],
        /* Absorb */ [TRIP, HLDA,     X,     X,       X,      X,    TOF,      X,       X],
        /* ToAbs  */ [TRIP, HLDF,     X,     X,       X,      X,      X,    ABS,       X],
        /* ToFlt  */ [TRIP, HLDA,     X,     X,       X,      X,      X,    FLT,       X],
        /* Tripng */ [   X,    X,     X,     X,       X,      X,      X,      X,    LTCH],
        /* Latchd */ [   X,    X,  TRIP,     X,       X,      X,      X,      X,       X],
    ]
};

impl ChargeState {
    /// Where `event` takes this state, or `None` if the table says the
    /// pair cannot arise.
    pub(super) fn next(self, event: ChargeEvent) -> Option<ChargeState> {
        TRANSITIONS[self as usize][event as usize]
    }

    /// The buck is meant to be sourcing. Every safety decision keys off
    /// this, and it is total: the hold and latch states answer `false` for
    /// the same reason `Boot` does — the output is off, or on its way off.
    pub(super) fn sourcing(self) -> bool {
        matches!(self, Self::Float | Self::Absorb | Self::ToAbsorb | Self::ToFloat)
    }

    /// Output is off and the supervisor is deciding whether to bring it up.
    pub(super) fn bringing_up(self) -> bool {
        matches!(self, Self::Boot | Self::HoldFloat | Self::HoldAbsorb)
    }

    /// Waiting out a self-clearing buck protection. A buck reporting `On`
    /// here is the recovery we are waiting for; the same reading from
    /// `Boot` is a fault, which is the whole reason the two are distinct
    /// states rather than one "output off" flag.
    pub(super) fn holding(self) -> bool {
        matches!(self, Self::HoldFloat | Self::HoldAbsorb)
    }

    /// Which target the device's V_SET is holding — what the per-tick
    /// drift check compares readback against. `None` once a fault has
    /// latched: the supervisor stops comparing setpoints there, so the
    /// value is deliberately not carried past the latch.
    pub(super) fn setpoint_phase(self) -> Option<Phase> {
        match self {
            // A retarget's ticket is uncommitted until the write lands, so
            // `To*` names the target the device still holds, not the one
            // it is moving to.
            Self::Boot | Self::HoldFloat | Self::Float | Self::ToAbsorb => Some(Phase::Float),
            Self::HoldAbsorb | Self::Absorb | Self::ToFloat => Some(Phase::Absorb),
            Self::Tripping | Self::Latched => None,
        }
    }

    /// The phase to show a dashboard: `Some` only while the buck is
    /// actually regulating to it, so "Float" / "Absorb" never label a dark
    /// output.
    pub(super) fn regulating_phase(self) -> Option<Phase> {
        self.sourcing().then(|| {
            self.setpoint_phase()
                .expect("a sourcing state holds a setpoint")
        })
    }

    /// The phase an outstanding V_SET write is moving to, if one is
    /// outstanding.
    pub(super) fn retarget_to(self) -> Option<Phase> {
        match self {
            Self::ToAbsorb => Some(Phase::Absorb),
            Self::ToFloat => Some(Phase::Float),
            _ => None,
        }
    }

    /// What moving from `self` to `next` means to the event log, or `None`
    /// for a move not worth an entry. Derived from the pair rather than
    /// named at each call site: coming up out of `Boot` is the unit
    /// energising, the same move out of a hold is the protection clearing,
    /// and no caller has to remember which of the two it is performing.
    pub(super) fn logged_as(self, next: Self) -> Option<ChargeTransition> {
        match (self, next) {
            (Self::Boot, n) if n.sourcing() => Some(ChargeTransition::Energised),
            (s, n) if s.holding() && n.sourcing() => Some(ChargeTransition::ProtectCleared),
            (s, n) if s.sourcing() && n.holding() => Some(ChargeTransition::ProtectHold),
            (_, Self::Tripping) => Some(ChargeTransition::Latched),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `next` indexes the table by discriminant, and `VARIANTS` is in
    /// declaration order, so this is what pins a row to the state that
    /// labels it — without it every lookup could be off by a row.
    #[test]
    fn discriminants_match_declaration_order() {
        assert_eq!(ChargeState::VARIANTS.len(), ChargeState::COUNT);
        assert_eq!(ChargeEvent::VARIANTS.len(), ChargeEvent::COUNT);
        for (i, s) in ChargeState::VARIANTS.iter().enumerate() {
            assert_eq!(*s as usize, i, "{s:?}");
        }
        for (i, e) in ChargeEvent::VARIANTS.iter().enumerate() {
            assert_eq!(*e as usize, i, "{e:?}");
        }
    }

    /// The properties that make the flat enum worth having: each state
    /// belongs to exactly one of the three groups the gauntlet dispatches
    /// on, and holds a setpoint iff it is not latched.
    #[test]
    fn state_groups_partition_the_machine() {
        for &s in ChargeState::VARIANTS {
            let latched = matches!(s, ChargeState::Tripping | ChargeState::Latched);
            let groups = [s.sourcing(), s.bringing_up(), latched];
            assert_eq!(
                groups.iter().filter(|g| **g).count(),
                1,
                "{s:?} is in {groups:?}, not exactly one group"
            );
            assert_eq!(s.holding(), s.bringing_up() && s != ChargeState::Boot, "{s:?}");
            assert_eq!(s.setpoint_phase().is_some(), !latched, "{s:?}");
            assert_eq!(s.regulating_phase().is_some(), s.sourcing(), "{s:?}");
            // A retarget is in flight exactly where the state names one,
            // and it always moves *away* from the target being held.
            assert_eq!(
                s.retarget_to().is_some(),
                matches!(s, ChargeState::ToAbsorb | ChargeState::ToFloat),
                "{s:?}"
            );
            if let Some(to) = s.retarget_to() {
                assert_ne!(Some(to), s.setpoint_phase(), "{s:?} retargets to itself");
            }
        }
    }

    /// Walk every cell. A reachable transition must land somewhere the
    /// event makes sense, and every state except `Boot` must be reachable
    /// — an unreachable one is dead code the table would otherwise hide.
    #[test]
    fn every_transition_is_coherent_and_every_state_reachable() {
        let mut reached = [false; ChargeState::COUNT];
        reached[ChargeState::Boot as usize] = true;
        for &from in ChargeState::VARIANTS {
            for &event in ChargeEvent::VARIANTS {
                let Some(to) = from.next(event) else { continue };
                reached[to as usize] = true;
                assert_ne!(from, to, "{from:?} on {event:?} is a self-loop");
                if matches!(from, ChargeState::Tripping | ChargeState::Latched) {
                    assert!(
                        !to.sourcing(),
                        "{from:?} on {event:?} re-energises after a latch"
                    );
                }
                match event {
                    // A fault always disables, and only a sourcing or
                    // bring-up state can still take one.
                    ChargeEvent::Fault => {
                        assert_eq!(to, ChargeState::Tripping, "{from:?}");
                        assert!(from.sourcing() || from.bringing_up(), "{from:?}");
                    }
                    // Only a sourcing buck can drop out, and it lands in
                    // the hold for the target it was holding.
                    ChargeEvent::SelfDisabled => {
                        assert!(from.sourcing(), "{from:?}");
                        assert!(to.holding(), "{from:?} → {to:?}");
                        assert_eq!(to.setpoint_phase(), from.setpoint_phase(), "{from:?}");
                    }
                    // Out of a hold this is recovery — start regulating,
                    // without changing V_SET, which no write has moved.
                    // Out of `Latched` it is the opposite: go re-disable.
                    ChargeEvent::SelfEnabled => {
                        if from.holding() {
                            assert!(to.sourcing(), "{from:?} → {to:?}");
                            assert_eq!(to.setpoint_phase(), from.setpoint_phase(), "{from:?}");
                        } else {
                            assert_eq!(from, ChargeState::Latched, "{from:?}");
                            assert_eq!(to, ChargeState::Tripping, "{from:?}");
                        }
                    }
                    ChargeEvent::Enabled | ChargeEvent::EnabledBelowFull => {
                        assert!(from.bringing_up(), "{from:?}");
                        assert!(to.sourcing(), "{from:?} → {to:?}");
                        assert_eq!(to.setpoint_phase(), from.setpoint_phase(), "{from:?}");
                    }
                    // A taper crossing arms a retarget; the device's own
                    // V_SET does not move until the write is committed.
                    ChargeEvent::TaperRose | ChargeEvent::TaperFell => {
                        assert!(to.retarget_to().is_some(), "{from:?} → {to:?}");
                        assert_eq!(to.setpoint_phase(), from.setpoint_phase(), "{from:?}");
                    }
                    // Committing the write is what finally moves it.
                    ChargeEvent::VoltageWritten => {
                        assert!(from.retarget_to().is_some(), "{from:?}");
                        assert!(to.sourcing() && to.retarget_to().is_none(), "{to:?}");
                        assert_eq!(to.setpoint_phase(), from.retarget_to(), "{from:?}");
                    }
                    ChargeEvent::Disabled => {
                        assert_eq!(from, ChargeState::Tripping, "{from:?}");
                        assert_eq!(to, ChargeState::Latched, "{from:?}");
                    }
                }
            }
        }
        for (i, r) in reached.iter().enumerate() {
            assert!(*r, "{:?} is unreachable", ChargeState::VARIANTS[i]);
        }
    }

    /// The event log's four entries against the moves that produce them.
    /// Hand-listed rather than derived from `logged_as`, so a change to
    /// the rule has to be restated here to pass.
    #[test]
    fn transitions_log_the_move_they_mean() {
        use ChargeState::*;
        let cases: [(ChargeState, ChargeState, Option<ChargeTransition>); 11] = [
            (Boot, Float, Some(ChargeTransition::Energised)),
            (Boot, ToAbsorb, Some(ChargeTransition::Energised)),
            (HoldFloat, Float, Some(ChargeTransition::ProtectCleared)),
            (HoldAbsorb, Absorb, Some(ChargeTransition::ProtectCleared)),
            (Float, HoldFloat, Some(ChargeTransition::ProtectHold)),
            (ToFloat, HoldAbsorb, Some(ChargeTransition::ProtectHold)),
            (Absorb, Tripping, Some(ChargeTransition::Latched)),
            (Boot, Tripping, Some(ChargeTransition::Latched)),
            // Each re-disable is its own episode — that a buck keeps
            // resurfacing is exactly what the log should show.
            (Latched, Tripping, Some(ChargeTransition::Latched)),
            // Retargets are already covered by the phase log, and the
            // disable ack is covered by the latch entry that preceded it.
            (Float, ToAbsorb, None),
            (Tripping, Latched, None),
        ];
        for (from, to, want) in cases {
            assert_eq!(from.logged_as(to), want, "{from:?} → {to:?}");
        }
    }
}
