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
    /// As [`ChargeState::HoldFloat`], entered from [`ChargeState::Parked`],
    /// and resuming to it rather than to `Float`. Without this a parked
    /// unit that lost its rail would come back charging, undoing a fault
    /// nobody has looked at — which is why the hold has to remember that
    /// it was parked, not merely that it held the float target.
    HoldParked,
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
    /// Sourcing, device still at the absorb target, an off→write→on
    /// step-down to float outstanding *because a fault said to stop
    /// charging* — the same write as `ToFloat`, landing somewhere else.
    ToParked,
    /// Sourcing at the float target with a fault latched and the phase
    /// machine frozen: charging has stopped, but the load is still fed.
    ///
    /// The response to a fault whose hazard is overcharge and whose control
    /// of the buck is intact. Killing the output would stop the overcharge
    /// too, and also drop the load onto the pack to drain for however long
    /// it takes someone to notice — which is the worse of the two on a UPS.
    Parked,
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
    /// The safety gauntlet raised a fault whose answer is to stop
    /// sourcing entirely.
    Fault,
    /// The safety gauntlet raised a fault whose answer is to stop charging
    /// but keep the load fed — see [`ChargeState::Parked`].
    Park,
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
///
/// The `Park` column carries the fourth: the two states already holding
/// the float target go straight to `Parked`, because there is no write to
/// make, while the two holding absorb go via `ToParked` for the same
/// off→write→on step-down `ToFloat` performs. Nothing parks from a
/// bring-up state — every park-class fault is raised from a sourcing path —
/// and nothing parks *out of* a park, which the supervisor escalates to a
/// disable instead.
///
/// The `SelfDisabled` column doubles as the answer to "can this state wait
/// a protection out?", which `reconcile` reads back rather than deciding
/// for itself. `ToParked` is the one sourcing state with no cell: it is
/// mid-step-down, so the output state a hold would claim is not settled
/// yet, and it latches instead.
#[rustfmt::skip]
const TRANSITIONS: [[Option<ChargeState>; ChargeEvent::COUNT]; ChargeState::COUNT] = {
    const X: Option<ChargeState> = None;
    const TRIP: Option<ChargeState> = Some(ChargeState::Tripping);
    const LTCH: Option<ChargeState> = Some(ChargeState::Latched);
    const FLT: Option<ChargeState> = Some(ChargeState::Float);
    const ABS: Option<ChargeState> = Some(ChargeState::Absorb);
    const TOA: Option<ChargeState> = Some(ChargeState::ToAbsorb);
    const TOF: Option<ChargeState> = Some(ChargeState::ToFloat);
    const TOP: Option<ChargeState> = Some(ChargeState::ToParked);
    const PRK: Option<ChargeState> = Some(ChargeState::Parked);
    const HLDF: Option<ChargeState> = Some(ChargeState::HoldFloat);
    const HLDA: Option<ChargeState> = Some(ChargeState::HoldAbsorb);
    const HLDP: Option<ChargeState> = Some(ChargeState::HoldParked);
    [
        //          Fault  Park SelfDis SelfEn Enabled BelowFull TaperUp TaperDn VWritten Disabled
        /* Boot   */ [TRIP,   X,    X,     X,   FLT,     TOA,      X,      X,      X,       X],
        /* HoldF  */ [TRIP,   X,    X,   FLT,   FLT,     TOA,      X,      X,      X,       X],
        /* HoldA  */ [TRIP,   X,    X,   ABS,   ABS,     ABS,      X,      X,      X,       X],
        /* HoldP  */ [TRIP,   X,    X,   PRK,   PRK,     PRK,      X,      X,      X,       X],
        /* Float  */ [TRIP, PRK, HLDF,     X,     X,       X,    TOA,      X,      X,       X],
        /* Absorb */ [TRIP, TOP, HLDA,     X,     X,       X,      X,    TOF,      X,       X],
        /* ToAbs  */ [TRIP, PRK, HLDF,     X,     X,       X,      X,      X,    ABS,       X],
        /* ToFlt  */ [TRIP, TOP, HLDA,     X,     X,       X,      X,      X,    FLT,       X],
        /* ToPark */ [TRIP,   X,    X,     X,     X,       X,      X,      X,    PRK,       X],
        /* Parked */ [TRIP,   X, HLDP,     X,     X,       X,      X,      X,      X,       X],
        /* Tripng */ [   X,   X,    X,     X,     X,       X,      X,      X,      X,    LTCH],
        /* Latchd */ [   X,   X,    X,  TRIP,     X,       X,      X,      X,      X,       X],
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
        matches!(
            self,
            Self::Float
                | Self::Absorb
                | Self::ToAbsorb
                | Self::ToFloat
                | Self::ToParked
                | Self::Parked
        )
    }

    /// Output is off and the supervisor is deciding whether to bring it up.
    pub(super) fn bringing_up(self) -> bool {
        matches!(
            self,
            Self::Boot | Self::HoldFloat | Self::HoldAbsorb | Self::HoldParked
        )
    }

    /// Parked on a fault, or on the way there: the buck is up and the load
    /// is fed, but charging has stopped and will not resume without a
    /// reboot. Deliberately excludes `HoldParked`, where the output is
    /// down and the load is *not* fed — that is the distinction the
    /// dashboard's `parked` flag exists to draw.
    pub(super) fn parked(self) -> bool {
        matches!(self, Self::ToParked | Self::Parked)
    }

    /// Waiting out a self-clearing buck protection. A buck reporting `On`
    /// here is the recovery we are waiting for; the same reading from
    /// `Boot` is a fault, which is the whole reason the two are distinct
    /// states rather than one "output off" flag.
    pub(super) fn holding(self) -> bool {
        matches!(self, Self::HoldFloat | Self::HoldAbsorb | Self::HoldParked)
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
            Self::Boot
            | Self::HoldFloat
            | Self::HoldParked
            | Self::Float
            | Self::ToAbsorb
            | Self::Parked => Some(Phase::Float),
            Self::HoldAbsorb | Self::Absorb | Self::ToFloat | Self::ToParked => Some(Phase::Absorb),
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
            Self::ToFloat | Self::ToParked => Some(Phase::Float),
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
            // One entry per park episode: `Absorb → ToParked` records it,
            // and the `ToParked → Parked` that completes the write does not.
            (s, n) if !s.parked() && n.parked() => Some(ChargeTransition::Parked),
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
                matches!(
                    s,
                    ChargeState::ToAbsorb | ChargeState::ToFloat | ChargeState::ToParked
                ),
                "{s:?}"
            );
            // Parked is a sourcing state, so it is judged like any other —
            // it is not a blind spot the gauntlet stops looking at.
            assert!(!s.parked() || s.sourcing(), "{s:?}");
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
                    // A park keeps the buck up, so it is only reachable from
                    // a sourcing state — and never from one already parked,
                    // which the supervisor escalates to a `Fault` instead.
                    // The V_SET does not move until a write is committed.
                    ChargeEvent::Park => {
                        assert!(from.sourcing() && !from.parked(), "{from:?}");
                        assert!(to.parked(), "{from:?} → {to:?}");
                        assert_eq!(to.setpoint_phase(), from.setpoint_phase(), "{from:?}");
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
        let cases: [(ChargeState, ChargeState, Option<ChargeTransition>); 16] = [
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
            // A park is one episode: the move into it records, the write
            // that completes it does not.
            (Absorb, ToParked, Some(ChargeTransition::Parked)),
            (Float, Parked, Some(ChargeTransition::Parked)),
            (ToParked, Parked, None),
            // A hold out of a park is the same protection story as any
            // other hold, and resuming it is not a fresh park.
            (Parked, HoldParked, Some(ChargeTransition::ProtectHold)),
            (HoldParked, Parked, Some(ChargeTransition::ProtectCleared)),
        ];
        for (from, to, want) in cases {
            assert_eq!(from.logged_as(to), want, "{from:?} → {to:?}");
        }
    }
}
