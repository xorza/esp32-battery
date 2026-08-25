//! The charge supervisor: safety gauntlet, bring-up, and the
//! Float/Absorb phase machine.

use std::time::Duration;

use heapless::Deque;

use crate::charging::action::{Action, DisableTicket, EnableTicket, VoltageTicket};
use crate::charging::debounce::Debounce;
use crate::charging::fault_reason::FaultReason;
use crate::charging::inhibit_reason::InhibitReason;
use crate::charging::phase::Phase;
use crate::charging::poll_result::{BatterySample, BuckOutput, PollResult};
use crate::charging::profile::Profile;
use crate::charging::{
    ABSORB_CV_BAND_V, BATTERY_MISSING_TIMEOUT, EXIT_DEBOUNCE, MAX_ABSORB,
    MODBUS_UNHEALTHY_TIMEOUT, OV_DURATION, OV_MARGIN_V, SETPOINT_DRIFT_TOL, TRANSITION_BUFFER,
};
use crate::error_log::ChargeTransition;
use xy_modbus::ProtectionStatus;
use xy_modbus::ProtectionStatus::{Lvp, Otp};
use xy_modbus::Setpoints;

/// Latch state.
/// - `Pending`: output is OFF and we haven't yet emitted EnableOutput, or
///   we have but its `EnableTicket` hasn't been committed yet (the write
///   may have failed). Same safety checks as `Active`, but tick emits
///   `EnableOutput` instead of running the phase machine.
/// - `Active { pending_voltage }`: output is on, phase machine + drift +
///   fault paths run. `pending_voltage` is `Some(next)` while a
///   Float↔Absorb V_SET write is in flight: tick re-emits `UpdateVoltage`
///   each cycle (so a transient Modbus glitch retries instead of latching
///   `SettingsDrift`), and `target_voltage` keeps reporting the **old**
///   phase's voltage until the `VoltageTicket` is committed.
/// - `Tripped { acked: false }`: a fault latched; emit `DisableOutput`.
/// - `Tripped { acked: true }`: caller successfully disabled. Reboot-only
///   recovery — `tick` returns `Action::None` from here on.
enum LatchState {
    Pending { reason: PendingReason },
    Active { pending_voltage: Option<Phase> },
    Tripped { reason: FaultReason, acked: bool },
}

impl LatchState {
    /// Output is off and the supervisor is deciding whether to bring it up.
    fn pending(&self) -> bool {
        matches!(self, LatchState::Pending { .. })
    }

    /// The buck is sourcing. This is what every safety decision keys off,
    /// and it is total: `Tripped` answers `false` for the same reason
    /// `Pending` does — the output is off or on its way off.
    fn regulating(&self) -> bool {
        matches!(self, LatchState::Active { .. })
    }

    /// The latch/inhibit rule in one place: the same condition disables a
    /// sourcing buck and merely blocks bring-up of an idle one.
    fn fault(&self, latched: FaultReason, inhibited: InhibitReason) -> Verdict {
        if self.regulating() {
            Verdict::Latch(latched)
        } else {
            Verdict::Inhibit(inhibited)
        }
    }
}

/// Why the supervisor is in `Pending`. Determines how an unexpected
/// `buck output ON in Pending` is handled.
///
/// - `Boot`: cold start. `boot_sequence` just wrote `set_output(false)`
///   and verified `OUTPUT_EN=0`. If a poll then shows On, something is
///   genuinely off (firmware/EMI/panel) — latch immediately.
/// - `ProtectRecovery`: the supervisor was Active when the buck
///   self-disabled on a transient protection (input UVLO / over-temp);
///   we dropped here to wait for the condition to clear. The XY7025
///   may auto-re-enable `OUTPUT_EN` when the cause clears (LVP/OTP are
///   sensor-driven, not true latches), so seeing buck=On is the
///   *expected* recovery — transition straight back to Active rather
///   than latching. Setpoints are still what we programmed before the
///   self-disable, so drift check covers regulation safety.
#[derive(Copy, Clone, Debug)]
enum PendingReason {
    Boot,
    ProtectRecovery,
}
/// Outcome of the ordered safety gauntlet, in descending authority:
/// a `Latch` beats a latch-state change, which beats an `Inhibit`, which beats
/// `Clear`. `safety_verdict` returns the first one it reaches, so the
/// order of the checks inside it *is* the precedence.
enum Verdict {
    /// Disable the buck and stay disabled until a reboot.
    Latch(FaultReason),
    /// Buck self-disabled on a self-clearing protection while
    /// regulating — step back to Pending and wait it out.
    EnterProtectRecovery(ProtectionStatus),
    /// Buck re-enabled itself once the protection cause cleared.
    ResumeRegulating,
    /// Hold the buck off without latching; re-checked next tick.
    Inhibit(InhibitReason),
    /// Every check passed; carries the validated battery sample so
    /// `tick` doesn't re-filter it.
    Clear(BatterySample),
}
pub struct ChargeSupervisor {
    profile: Profile,
    phase: Phase,
    ov: Debounce,
    absorb: Debounce,
    exit: Debounce,
    battery_missing: Debounce,
    modbus_err: Debounce,
    latch: LatchState,
    inhibit: Option<InhibitReason>,
    transitions: Deque<ChargeTransition, TRANSITION_BUFFER>,
}
impl ChargeSupervisor {
    pub fn new(profile: Profile) -> Self {
        assert!(profile.absorb_v > profile.float_v);
        // Boot conservative: Phase::Float and LatchState::Pending (output
        // stays OFF until the first healthy tick — bringing up the buck is
        // the supervisor's job, so cold-boot can't bypass safety). We never
        // trust a *stored* phase across a reset, but the Pending bring-up
        // re-derives it from the pack's resting voltage: a pack below the CV
        // plateau isn't full, so the enable ticket resumes Absorb rather than
        // stalling in Float.
        Self {
            profile,
            phase: Phase::Float,
            ov: Debounce::default(),
            absorb: Debounce::default(),
            exit: Debounce::default(),
            battery_missing: Debounce::default(),
            modbus_err: Debounce::default(),
            latch: LatchState::Pending {
                reason: PendingReason::Boot,
            },
            inhibit: None,
            transitions: Deque::new(),
        }
    }

    /// Float→Absorb and the LVP/OTP intercept both clear these so the
    /// next CV-plateau dwell starts fresh and the exit-taper isn't
    /// pre-armed from a load transient that happened before the
    /// transition.
    fn reset_phase_timers(&mut self) {
        self.absorb.reset();
        self.exit.reset();
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// Phase only while the supervisor is actually regulating (output ON).
    /// `None` in Pending (output still off, waiting to enable) and Tripped
    /// (latched fault). Surfaced to the dashboard so "Float" / "Absorb"
    /// labels appear only when they describe a live charging state.
    pub fn active_phase(&self) -> Option<Phase> {
        self.latch.regulating().then_some(self.phase)
    }

    fn target_voltage(&self) -> f32 {
        self.voltage_for_phase(self.phase)
    }

    /// Pack voltage within `ABSORB_CV_BAND_V` of `absorb_v` — i.e. at/above
    /// the CV plateau. Doubles as "full" at bring-up and "clock the absorb
    /// timeout" once in Absorb.
    fn at_cv_plateau(&self, voltage: f32) -> bool {
        voltage >= self.profile.absorb_v - ABSORB_CV_BAND_V
    }

    fn voltage_for_phase(&self, phase: Phase) -> f32 {
        match phase {
            Phase::Float => self.profile.float_v,
            Phase::Absorb => self.profile.absorb_v,
        }
    }

    /// Build the `UpdateVoltage` action for a phase transition to `next`.
    /// `cycle_output` is set when the new V_SET is below the current
    /// one — see `Action::UpdateVoltage` for why. Stable across re-emits
    /// because `self.phase` only changes on `commit_voltage`.
    fn update_voltage_for(&self, next: Phase) -> Action {
        let target_v = self.voltage_for_phase(next);
        Action::UpdateVoltage(VoltageTicket {
            phase: next,
            target_v,
            cycle_output: target_v < self.voltage_for_phase(self.phase),
        })
    }

    /// Why the supervisor is holding the buck off without having latched,
    /// if it is. `None` while regulating normally, and `None` once a fault
    /// has latched — `fault()` covers that case. Unlike a fault, every
    /// inhibit clears by itself when its cause does.
    pub fn inhibit(&self) -> Option<InhibitReason> {
        self.inhibit
    }

    /// Pop the oldest un-drained latch transition. The caller loops this
    /// once per tick and writes each into its event log — the supervisor
    /// has no clock of its own, so timestamping is the caller's job.
    pub fn pop_transition(&mut self) -> Option<ChargeTransition> {
        self.transitions.pop_front()
    }

    pub fn fault(&self) -> Option<FaultReason> {
        match self.latch {
            LatchState::Tripped { reason, .. } => Some(reason),
            _ => None,
        }
    }

    /// What setpoints the supervisor currently expects the buck to be
    /// regulating to. Used by the caller to construct `Setpoints` for tests
    /// and as documentation for what `tick` will compare readbacks against.
    ///
    /// `i_set` is the constant `regulation_a` from the profile — the
    /// drift check relies on this never changing at runtime. If a future
    /// feature ever varies the current setpoint (CC tapering, dynamic
    /// limits, etc.), it must use the same defer-and-ack pattern as
    /// `pending_voltage` for V_SET, otherwise a successful write to a new
    /// I_SET will trip `SettingsDrift` on the very next tick.
    fn expected_setpoints(&self) -> Setpoints {
        Setpoints {
            v_set: self.target_voltage(),
            i_set: self.profile.regulation_a,
        }
    }

    /// Commit the disable named by `ticket`, after a successful
    /// `set_output(false)`. Until then the supervisor keeps emitting
    /// `DisableOutput` so a failed write is retried every tick.
    ///
    /// The assert cannot fire through the public API — a `DisableTicket`
    /// is only minted by a tick that latched — but it still guards
    /// against a ticket stashed across ticks.
    pub fn commit_disable(&mut self, ticket: DisableTicket) {
        let LatchState::Tripped { reason, acked } = &mut self.latch else {
            panic!("disable ticket committed while no fault is latched");
        };
        assert_eq!(
            *reason, ticket.reason,
            "disable ticket does not match the latched fault"
        );
        *acked = true;
    }

    /// Commit the bring-up named by `ticket`, after a successful
    /// `set_output(true)`. Transitions Pending → Active; the phase
    /// machine starts on the next tick. Until committed the supervisor
    /// keeps emitting `EnableOutput` so a failed write is retried.
    ///
    /// The ticket carries `resume_absorb`, so the caller can no longer
    /// disagree with the supervisor about it: `true` means the pack
    /// rested below the CV plateau and the first Active tick steps V_SET
    /// float_v → absorb_v. A pack power-cycled above ~75% rests too near
    /// `float_v` to ever draw `enter_absorb_a`, so without this it would
    /// stall in Float and never finish charging.
    pub fn commit_enable(&mut self, ticket: EnableTicket) {
        let LatchState::Pending { reason } = self.latch else {
            panic!("enable ticket committed outside Pending");
        };
        // Arming the phase we are already in would emit an UpdateVoltage
        // whose target equals the live V_SET — a wasted Modbus write, and
        // a tick where the phase machine is skipped for nothing. Reachable
        // after a protect-hold: the pack can drain below the CV plateau
        // during a long input outage while the phase is still Absorb.
        let resume = ticket
            .resume_absorb
            .then_some(Phase::Absorb)
            .filter(|&p| p != self.phase);
        // A protect-hold ends where it began — the buck came back on once
        // its cause cleared — so that route reads as a resume, not a boot.
        let transition = match reason {
            PendingReason::Boot => ChargeTransition::Energised,
            PendingReason::ProtectRecovery => ChargeTransition::ProtectCleared,
        };
        self.set_latch(
            LatchState::Active {
                pending_voltage: resume,
            },
            Some(transition),
        );
    }

    /// Commit the phase transition named by `ticket`, after
    /// [`apply_update_voltage`] reported `Committed`. The new phase
    /// becomes `target_voltage()` — so the drift check switches to the
    /// new value on the next tick — and the absorb/exit debouncers reset.
    ///
    /// If the write failed the caller drops the ticket instead: the
    /// supervisor stays on the old phase, the drift check keeps matching
    /// the old V_SET, and the next tick re-emits `UpdateVoltage`.
    pub fn commit_voltage(&mut self, ticket: VoltageTicket) {
        let LatchState::Active {
            pending_voltage: Some(pending),
        } = self.latch
        else {
            panic!("voltage ticket committed without a pending phase");
        };
        assert_eq!(
            pending, ticket.phase,
            "voltage ticket does not match the pending phase"
        );
        self.phase = ticket.phase;
        self.set_latch(
            LatchState::Active {
                pending_voltage: None,
            },
            None,
        );
        // A Float→Absorb transition can immediately follow an
        // Absorb→Float, with no intervening Float dwell to clear stale counts.
        self.reset_phase_timers();
    }

    /// Drive one poll cycle. `p` carries the buck readback and latest fresh
    /// battery sample; `elapsed` is wall time since the previous tick.
    /// Returns the action the caller should take.
    ///
    /// `p.setpoints.is_some()` doubles as the modbus-healthy signal — a
    /// successful read means the link is up. Drift (commanded vs.
    /// reported V_SET / I_SET) latches `SettingsDrift` immediately; no
    /// debounce, the read itself succeeded so this isn't transport noise.
    ///
    /// Battery samples with NaN/Inf in either field are treated as
    /// **missing** — a sensor reporting non-finite values can't be used
    /// to supervise charging, and silently ignoring NaN would let a
    /// stuck sensor mask overvoltage. Routes through the same
    /// `BatterySensorStale` debounce as a truly absent sample.
    pub fn tick(&mut self, p: PollResult, elapsed: Duration) -> Action {
        match self.latch {
            LatchState::Tripped {
                reason,
                acked: false,
            } => return Action::DisableOutput(DisableTicket { reason }),
            // Tripped+acked: reboot-only recovery, supervisor parks here.
            LatchState::Tripped { acked: true, .. } => return Action::None,
            _ => {}
        }

        // The gauntlet steps debouncers but never writes `self.latch`, so the
        // dispatch below still reads the state its verdict was formed against.
        let battery = match self.safety_verdict(&p, elapsed) {
            Verdict::Latch(reason) => return self.latch(reason),
            Verdict::EnterProtectRecovery(cause) => {
                self.set_latch(
                    LatchState::Pending {
                        reason: PendingReason::ProtectRecovery,
                    },
                    Some(ChargeTransition::ProtectHold),
                );
                self.reset_phase_timers();
                // Output is off for the duration of the hold, so the pack
                // voltage decays. A partly-accumulated OV window from before
                // the self-disable would otherwise carry across and trip the
                // next regulating stretch early.
                self.ov.reset();
                self.inhibit = Some(InhibitReason::BuckProtection(cause));
                return Action::None;
            }
            Verdict::ResumeRegulating => {
                self.set_latch(
                    LatchState::Active {
                        pending_voltage: None,
                    },
                    Some(ChargeTransition::ProtectCleared),
                );
                self.inhibit = None;
                return Action::None;
            }
            Verdict::Inhibit(reason) => {
                self.inhibit = Some(reason);
                return Action::None;
            }
            Verdict::Clear(b) => {
                self.inhibit = None;
                b
            }
        };

        // Output has been OFF throughout Pending, so `battery.voltage` is the
        // pack's resting voltage — the true SoC signal. Below the CV plateau
        // means not full, so the caller acks with resume_absorb = true. The
        // supervisor stays Pending until it does.
        if self.latch.pending() {
            return Action::EnableOutput(EnableTicket {
                resume_absorb: !self.at_cv_plateau(battery.voltage),
            });
        }
        self.regulate(battery, elapsed)
    }

    /// The ordered safety gauntlet. **The order of the checks below is the
    /// specification** — each one may only be moved past checks it commutes
    /// with, and `tests.rs` pins the precedence where two can fire on the
    /// same tick.
    ///
    /// Whether a failure latches or merely inhibits is decided by the latch
    /// state and nothing else. A fault latches only while the buck is
    /// sourcing; in `Pending` the output is already off, so a latch would
    /// disable nothing and cost a reboot to clear. `OutputOnInPending` is the
    /// one exception, because there the output really is on.
    ///
    /// Debouncers are stepped in both states so their windows stay coherent
    /// across a move between them. `self.latch` is only read here, never
    /// written — `tick` relies on that to dispatch on it afterwards.
    fn safety_verdict(&mut self, p: &PollResult, elapsed: Duration) -> Verdict {
        // 1. Commanded vs. reported setpoints. No debounce: the read itself
        //    succeeded, so a mismatch is the device disagreeing with us
        //    rather than transport noise.
        if let Some(sp) = p.setpoints {
            let want = self.expected_setpoints();
            if (sp.v_set - want.v_set).abs() >= SETPOINT_DRIFT_TOL
                || (sp.i_set - want.i_set).abs() >= SETPOINT_DRIFT_TOL
            {
                return self
                    .latch
                    .fault(FaultReason::SettingsDrift, InhibitReason::SettingsDrift);
            }
        }

        // 2. Latch state vs. what OUTPUT_EN reports. Regulating expects ON:
        //    any OFF means the buck self-disabled (its own hardware OVP/OCP,
        //    a panel toggle). Pending expects OFF: an ON means our boot
        //    disable / S_INI=0 didn't stick.
        //
        //    LVP (input UVLO) and OTP (over-temp) are sensor-driven, not true
        //    latches: the buck is healthy and waiting on a condition to
        //    clear, and it may re-enable OUTPUT_EN by itself once it does.
        //    So we step back to Pending and treat a later ON as the expected
        //    recovery. Setpoints are untouched through the wait — check 1
        //    just verified them — so regulation resumes at known targets.
        match (&self.latch, p.output) {
            (
                LatchState::Active { .. },
                Some(BuckOutput::Off {
                    cause: cause @ (Lvp | Otp),
                }),
            ) => {
                return Verdict::EnterProtectRecovery(cause);
            }
            (LatchState::Active { .. }, Some(BuckOutput::Off { cause })) => {
                return Verdict::Latch(FaultReason::OutputUnexpectedlyOff(cause));
            }
            (
                LatchState::Pending {
                    reason: PendingReason::ProtectRecovery,
                },
                Some(BuckOutput::On),
            ) => {
                return Verdict::ResumeRegulating;
            }
            // Boot + ON: `boot_sequence` wrote set_output(false) and verified
            // OUTPUT_EN=0, so an ON reading is a real anomaly (firmware bug,
            // panel toggle, EMI on the button GPIO). Unlike every other
            // Pending check there IS something sourcing to disable, so this
            // one latches.
            (
                LatchState::Pending {
                    reason: PendingReason::Boot,
                },
                Some(BuckOutput::On),
            ) => {
                return Verdict::Latch(FaultReason::OutputOnInPending);
            }
            _ => {}
        }

        // 3. Modbus health. `p.setpoints.is_none()` doubles as the read-failed
        //    signal — a successful read means the link is up.
        if self
            .modbus_err
            .step(p.setpoints.is_none(), elapsed, MODBUS_UNHEALTHY_TIMEOUT)
        {
            return self
                .latch
                .fault(FaultReason::ModbusUnhealthy, InhibitReason::ModbusUnhealthy);
        }

        // 4. Battery sample freshness. NaN/Inf counts as missing: a sensor
        //    reporting non-finite values can't supervise charging, and
        //    silently ignoring it would let a stuck sensor mask overvoltage.
        let battery = p
            .battery
            .filter(|b| b.voltage.is_finite() && b.current.is_finite());
        if self
            .battery_missing
            .step(battery.is_none(), elapsed, BATTERY_MISSING_TIMEOUT)
        {
            return self.latch.fault(
                FaultReason::BatterySensorStale,
                InhibitReason::BatterySensorStale,
            );
        }
        let Some(b) = battery else {
            return Verdict::Inhibit(InhibitReason::NoBatterySample);
        };

        // 5. Overvoltage. Regulating needs the 3 s debounce so switching
        //    noise and load steps don't trip a healthy charge. Pending needs
        //    none: a single sample over the line is reason enough not to
        //    energise, and since that only inhibits, one noisy reading can no
        //    longer strand the unit off until a reboot.
        let ov = b.voltage > self.profile.absorb_v + OV_MARGIN_V;
        let ov_debounced = self.ov.step(ov, elapsed, OV_DURATION);
        if self.latch.regulating() {
            if ov_debounced {
                return Verdict::Latch(FaultReason::Overvoltage);
            }
        } else if ov {
            return Verdict::Inhibit(InhibitReason::Overvoltage);
        }

        // 6. Bring-up-only gates. Not faults — they say "not yet", and only
        //    mean anything while the output is off.
        if self.latch.pending() {
            // Demand a fresh setpoint readback before energising.
            // `boot_sequence` already verified the writes, but requiring
            // closed-loop confirmation here means we never ask for output-on
            // until the link is demonstrably alive. Check 3 eventually
            // inhibits on sustained failure, but takes 5 s; this covers the gap.
            if p.setpoints.is_none() {
                return Verdict::Inhibit(InhibitReason::ModbusUnhealthy);
            }
            // Enabling into a live LVP/OTP hold would succeed at the Modbus
            // layer while the buck stayed off, flapping EnableOutput every poll.
            if let Some(BuckOutput::Off {
                cause: cause @ (Lvp | Otp),
            }) = p.output
            {
                return Verdict::Inhibit(InhibitReason::BuckProtection(cause));
            }
        }

        Verdict::Clear(b)
    }

    /// Active arm: output is on and every safety check just cleared. Runs the
    /// deferred V_SET write, then the Float-Absorb phase machine and the
    /// absorb time cap.
    fn regulate(&mut self, b: BatterySample, elapsed: Duration) -> Action {
        // Re-emit UpdateVoltage until the caller acks the previous one. The
        // phase machine and absorb cap don't run while a write is in flight —
        // the drift check keeps matching the old V_SET (since `target_voltage`
        // reflects the still-current phase), and the caller retries on every
        // tick by writing again.
        if let LatchState::Active {
            pending_voltage: Some(next),
        } = self.latch
        {
            return self.update_voltage_for(next);
        }

        // Charging current as a positive number.
        let charging_a = -b.current;
        let below_exit = self.phase == Phase::Absorb && charging_a < self.profile.exit_absorb_a;
        // Leaky, not hard-reset: a full pack makes the buck pulse current in
        // bursts that briefly exceed the tail threshold; those pulses must
        // shave the gate, not re-arm it from scratch (see `step_leaky`).
        let exit_done = self.exit.step_leaky(below_exit, elapsed, EXIT_DEBOUNCE);

        let next = match self.phase {
            Phase::Float if charging_a > self.profile.enter_absorb_a => Phase::Absorb,
            Phase::Absorb if exit_done => Phase::Float,
            p => p,
        };
        if next != self.phase {
            // Defer the phase commit until the caller commits the
            // ticket — keeps `target_voltage` matching the buck's actual
            // V_SET so a failed write doesn't trigger SettingsDrift on the
            // next tick.
            self.set_latch(
                LatchState::Active {
                    pending_voltage: Some(next),
                },
                None,
            );
            return self.update_voltage_for(next);
        }

        // Clock the absorb timeout only while the pack sits at the CV plateau.
        // A CC dip (load transient pulling voltage back below absorb_v) resets
        // it via Debounce — that's genuine charging, not a stuck taper.
        let at_cv = self.at_cv_plateau(b.voltage);
        if self.phase == Phase::Absorb && self.absorb.step(at_cv, elapsed, MAX_ABSORB) {
            return self.latch(FaultReason::AbsorbTimeout);
        }
        Action::None
    }

    /// Single write point for `self.latch`. `transition` is what the move
    /// means to the event log, named by the caller rather than recovered
    /// from `from × to`: `Pending → Active` is `Energised` or
    /// `ProtectCleared` depending on why we were Pending, and only the
    /// caller is holding that. `None` records nothing, which is what an
    /// `Active → Active` move wants — that is `pending_voltage` being armed
    /// or cleared, already covered by the phase log.
    fn set_latch(&mut self, next: LatchState, transition: Option<ChargeTransition>) {
        if let Some(t) = transition {
            // Oldest-out when full: a caller that stopped draining is
            // better served by the recent history than the stale head.
            if self.transitions.is_full() {
                self.transitions.pop_front();
            }
            self.transitions
                .push_back(t)
                .ok()
                .expect("ring has a free slot after pop_front");
        }
        self.latch = next;
    }

    fn latch(&mut self, reason: FaultReason) -> Action {
        self.inhibit = None;
        self.set_latch(
            LatchState::Tripped {
                reason,
                acked: false,
            },
            Some(ChargeTransition::Latched),
        );
        Action::DisableOutput(DisableTicket { reason })
    }
}

/// What the charging tests read out of a supervisor, gated so none of it
/// widens the production surface. `LatchState` stays private: the tests ask
/// about it through predicates rather than matching the type.
#[cfg(test)]
pub(crate) mod internals {
    use xy_modbus::Setpoints;

    use crate::charging::charge_supervisor::ChargeSupervisor;

    /// A trait, not an inherent `impl`, because two of these names are
    /// already private inherent methods and a second inherent definition
    /// of a name is a duplicate-definition error however it is gated.
    /// Tests reach them by importing this; the bodies below spell out
    /// `ChargeSupervisor::…` so it is clear they forward rather than recurse.
    pub(crate) trait SupervisorInternals {
        /// Output is off and the supervisor is deciding whether to bring it up.
        fn is_pending(&self) -> bool;
        /// Output is on and the phase machine is running.
        fn is_active(&self) -> bool;
        fn target_voltage(&self) -> f32;
        fn expected_setpoints(&self) -> Setpoints;
    }

    impl SupervisorInternals for ChargeSupervisor {
        fn is_pending(&self) -> bool {
            self.latch.pending()
        }

        fn is_active(&self) -> bool {
            self.latch.regulating()
        }

        fn target_voltage(&self) -> f32 {
            ChargeSupervisor::target_voltage(self)
        }

        fn expected_setpoints(&self) -> Setpoints {
            ChargeSupervisor::expected_setpoints(self)
        }
    }
}
