//! The charge supervisor: safety gauntlet, bring-up, and the
//! Float/Absorb phase machine.

use std::time::Duration;

use heapless::Deque;

use crate::charging::action::{Action, DisableTicket, EnableTicket, VoltageTicket};
use crate::charging::charge_state::{ChargeEvent, ChargeState};
use crate::charging::debounce::Debounce;
use crate::charging::fault_reason::FaultReason;
use crate::charging::inhibit_reason::InhibitReason;
use crate::charging::phase::Phase;
use crate::charging::poll_result::{BatterySample, BuckOutput, PollResult};
use crate::charging::profile::Profile;
use crate::charging::protection_policy::ProtectionPolicy;
use crate::charging::{
    ABSORB_CV_BAND_V, BATTERY_MISSING_TIMEOUT, EXIT_DEBOUNCE, MAX_ABSORB,
    MODBUS_UNHEALTHY_TIMEOUT, OV_DURATION, OV_MARGIN_V, SETPOINT_DRIFT_TOL, TRANSITION_BUFFER,
};
use crate::error_log::ChargeTransition;
use xy_modbus::Setpoints;

/// Outcome of the ordered safety gauntlet, in descending authority: a
/// `Latch` beats an `Inhibit`, which beats `Clear`. `gauntlet` returns the
/// first one it reaches, so the order of the checks inside it *is* the
/// precedence.
enum Verdict {
    /// Disable the buck and stay disabled until a reboot.
    Latch(FaultReason),
    /// Hold the buck off without latching; re-checked next tick.
    Inhibit(InhibitReason),
    /// Every check passed; carries the validated battery sample so
    /// `tick` doesn't re-filter it.
    Clear(BatterySample),
}

pub struct ChargeSupervisor {
    profile: Profile,
    state: ChargeState,
    ov: Debounce,
    absorb: Debounce,
    exit: Debounce,
    battery_missing: Debounce,
    modbus_err: Debounce,
    /// Set once, when the machine enters `Tripping`. Reboot-only recovery
    /// means it is never cleared.
    fault: Option<FaultReason>,
    inhibit: Option<InhibitReason>,
    transitions: Deque<ChargeTransition, TRANSITION_BUFFER>,
}

impl ChargeSupervisor {
    pub fn new(profile: Profile) -> Self {
        assert!(profile.absorb_v > profile.float_v);
        // Boot conservative: output stays OFF until the first healthy tick,
        // because bringing up the buck is the supervisor's job and a cold
        // boot must not bypass safety. We never trust a *stored* phase
        // across a reset; `ChargeState::Boot` holds the float target that
        // `boot_sequence` wrote, and the bring-up re-derives from the pack's
        // resting voltage whether a step up to absorb is owed.
        Self {
            profile,
            state: ChargeState::Boot,
            ov: Debounce::default(),
            absorb: Debounce::default(),
            exit: Debounce::default(),
            battery_missing: Debounce::default(),
            modbus_err: Debounce::default(),
            fault: None,
            inhibit: None,
            transitions: Deque::new(),
        }
    }

    /// The phase the buck is regulating to, or `None` when it isn't
    /// regulating. Surfaced to the dashboard so "Float" / "Absorb" labels
    /// appear only when they describe a live charging state.
    pub fn phase(&self) -> Option<Phase> {
        self.state.regulating_phase()
    }

    pub fn fault(&self) -> Option<FaultReason> {
        self.fault
    }

    /// Why the supervisor is holding the buck off without having latched,
    /// if it is. `None` while regulating normally, and `None` once a fault
    /// has latched — `fault()` covers that case. Unlike a fault, every
    /// inhibit clears by itself when its cause does.
    pub fn inhibit(&self) -> Option<InhibitReason> {
        self.inhibit
    }

    /// Pop the oldest un-drained transition. The caller loops this once
    /// per tick and writes each into its event log — the supervisor has no
    /// clock of its own, so timestamping is the caller's job.
    pub fn pop_transition(&mut self) -> Option<ChargeTransition> {
        self.transitions.pop_front()
    }

    /// Pack voltage within `ABSORB_CV_BAND_V` of `absorb_v` — i.e. at or
    /// above the CV plateau. Doubles as "full" at bring-up and "clock the
    /// absorb timeout" once in Absorb.
    fn at_cv_plateau(&self, voltage: f32) -> bool {
        voltage >= self.profile.absorb_v - ABSORB_CV_BAND_V
    }

    fn voltage_for_phase(&self, phase: Phase) -> f32 {
        match phase {
            Phase::Float => self.profile.float_v,
            Phase::Absorb => self.profile.absorb_v,
        }
    }

    /// The pair a buck holding `phase` would be regulating to.
    ///
    /// `i_set` is the constant `regulation_a` from the profile — the drift
    /// check relies on this never changing at runtime. If a future feature
    /// ever varies the current setpoint (CC tapering, dynamic limits), it
    /// must use the same arm-then-commit pattern the `To*` states give
    /// V_SET, otherwise a successful write to a new I_SET trips
    /// `SettingsDrift` on the very next tick.
    fn setpoints_for(&self, phase: Phase) -> Setpoints {
        Setpoints {
            v_set: self.voltage_for_phase(phase),
            i_set: self.profile.regulation_a,
        }
    }

    /// What the supervisor expects the buck to be regulating to. Required:
    /// a latched supervisor regulates to nothing, and `tick` returns before
    /// anything here can run.
    fn expected_setpoints(&self) -> Setpoints {
        self.setpoints_for(
            self.state
                .setpoint_phase()
                .expect("a latched supervisor regulates to no setpoint"),
        )
    }

    /// The V_SET half of [`Self::expected_setpoints`].
    fn target_voltage(&self) -> f32 {
        self.expected_setpoints().v_set
    }

    /// Build the `UpdateVoltage` action for a retarget to `next`.
    /// `cycle_output` is set when the new V_SET is below the live one —
    /// see `Action::UpdateVoltage` for why. Stable across re-emits because
    /// `setpoint_phase` only moves when the ticket is committed.
    fn update_voltage_for(&self, next: Phase) -> Action {
        let target_v = self.voltage_for_phase(next);
        Action::UpdateVoltage(VoltageTicket {
            phase: next,
            target_v,
            cycle_output: target_v < self.target_voltage(),
        })
    }

    /// The only writer of `self.state`. Routing every move through the
    /// table keeps the machine's shape in one place, and hangs the log
    /// entry and the debounce resets on *where we land* rather than on
    /// which caller got us there.
    ///
    /// A pair the table says cannot arise panics here, which is what turns
    /// a ticket committed out of turn — stashed across ticks, or minted by
    /// a caller that invented one — into a failure at the call rather than
    /// silent drift.
    fn step(&mut self, event: ChargeEvent) {
        let from = self.state;
        let to = from
            .next(event)
            .unwrap_or_else(|| panic!("no transition from {from:?} on {event:?}"));
        if let Some(t) = from.logged_as(to) {
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
        // Both windows measure a dwell inside one phase at one V_SET, so
        // any move at all invalidates them: a hold takes the output away,
        // and a retarget changes the target they were accumulated against.
        if !matches!(to, ChargeState::Tripping | ChargeState::Latched) {
            self.absorb.reset();
            self.exit.reset();
        }
        // Output is off for the whole hold, so the pack decays. A partly
        // accumulated OV window would otherwise carry across and trip the
        // next regulating stretch early.
        if to.holding() {
            self.ov.reset();
        }
        self.state = to;
    }

    /// Commit the bring-up named by `ticket`, after a successful
    /// `set_output(true)`. Until committed the supervisor keeps emitting
    /// `EnableOutput`, so a failed write is retried.
    ///
    /// The ticket carries `resume_absorb`, so the caller cannot disagree
    /// with the supervisor about it: `true` means the pack rested below
    /// the CV plateau, and the first regulating tick steps V_SET up to
    /// `absorb_v`. A pack power-cycled above ~75% rests too near `float_v`
    /// to ever draw `enter_absorb_a`, so without this it would stall in
    /// Float and never finish charging. Stepping to a target the device
    /// already holds is the table's business, not the caller's — see the
    /// `HoldAbsorb` row.
    pub fn commit_enable(&mut self, ticket: EnableTicket) {
        self.step(if ticket.resume_absorb {
            ChargeEvent::EnabledBelowFull
        } else {
            ChargeEvent::Enabled
        });
    }

    /// Commit the retarget named by `ticket`, after [`apply_update_voltage`]
    /// reported `Committed`. The new target becomes what the drift check
    /// compares against from the next tick, and the absorb/exit windows
    /// restart.
    ///
    /// If the write failed the caller drops the ticket instead: the
    /// supervisor stays in its `To*` state, the drift check keeps matching
    /// the old V_SET, and the next tick re-emits `UpdateVoltage`.
    pub fn commit_voltage(&mut self, ticket: VoltageTicket) {
        assert_eq!(
            self.state.retarget_to(),
            Some(ticket.phase),
            "voltage ticket does not match the outstanding retarget"
        );
        self.step(ChargeEvent::VoltageWritten);
    }

    /// Commit the disable named by `ticket`, after a successful
    /// `set_output(false)`. Until then the supervisor keeps emitting
    /// `DisableOutput` so a failed write is retried every tick.
    pub fn commit_disable(&mut self, ticket: DisableTicket) {
        assert_eq!(
            self.fault,
            Some(ticket.reason),
            "disable ticket does not match the latched fault"
        );
        self.step(ChargeEvent::Disabled);
    }

    /// Drive one poll cycle. `p` carries the buck readback and latest fresh
    /// battery sample; `elapsed` is wall time since the previous tick.
    /// Returns the action the caller should take.
    ///
    /// Two phases, in this order and for a reason. [`Self::reconcile`] asks
    /// what the buck is *actually* doing and moves the machine to agree
    /// with it; [`Self::gauntlet`] then asks whether that is safe. Running
    /// them the other way round evaluates against a state the device has
    /// already left — which is how a buck that re-enabled itself used to
    /// resume for a whole tick before anything checked on it.
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
        if let Some(a) = self.reconcile(&p) {
            return a;
        }

        // The gauntlet steps debouncers but never writes `self.state`, so
        // the dispatch below still reads the state its verdict was formed
        // against — which `reconcile` has already brought into agreement
        // with the device.
        let battery = match self.gauntlet(&p, elapsed) {
            Verdict::Latch(reason) => return self.on_fault(reason),
            Verdict::Inhibit(reason) => {
                self.inhibit = Some(reason);
                return Action::None;
            }
            Verdict::Clear(b) => {
                self.inhibit = None;
                b
            }
        };

        // Output has been OFF throughout bring-up, so `battery.voltage` is
        // the pack's resting voltage — the true SoC signal. Below the CV
        // plateau means not full, so the caller acks with
        // resume_absorb = true. The supervisor stays put until it does.
        if self.state.bringing_up() {
            return Action::EnableOutput(EnableTicket {
                resume_absorb: !self.at_cv_plateau(battery.voltage),
            });
        }
        self.regulate(battery, elapsed)
    }

    /// The disable owed for the fault already latched. Both terminal
    /// states produce it: `Tripping` every tick until the write is
    /// confirmed, and `Latched` whenever the output turns up on again.
    fn disable_for_latched_fault(&self) -> Action {
        Action::DisableOutput(DisableTicket {
            reason: self
                .fault
                .expect("a latched state carries the fault that put it there"),
        })
    }

    /// A fault latched. Emits the disable and keeps emitting it until the
    /// caller confirms — dropping the ticket is how a failed write retries.
    fn on_fault(&mut self, reason: FaultReason) -> Action {
        self.fault = Some(reason);
        self.inhibit = None;
        self.step(ChargeEvent::Fault);
        self.disable_for_latched_fault()
    }

    /// Bring the machine into agreement with what OUTPUT_EN reports, before
    /// anything is evaluated against it.
    ///
    /// `Some` means the tick is already decided: the state is terminal, or
    /// the disagreement was itself the fault. `None` means the machine now
    /// agrees with the device and [`Self::gauntlet`] should run — so a buck
    /// that just came back on gets *this* tick's safety checks applied to
    /// it as a sourcing buck, not the next one's.
    ///
    /// This is also the only place a state moves before the gauntlet, which
    /// is what lets the gauntlet document itself as read-only.
    fn reconcile(&mut self, p: &PollResult) -> Option<Action> {
        match (self.state, p.output) {
            // Re-emit until the caller confirms the write; dropping the
            // ticket is how a failed disable becomes a retry.
            (ChargeState::Tripping, _) => Some(self.disable_for_latched_fault()),
            // A latch is only as good as the output actually being off. A
            // buck that comes back on — front panel, a device-side
            // re-enable — has not made the fault go away, so say it again
            // rather than watch a pack charge under a supervisor that gave
            // up. Back to `Tripping`, which re-emits until confirmed.
            (ChargeState::Latched, Some(BuckOutput::On)) => {
                self.step(ChargeEvent::SelfEnabled);
                Some(self.disable_for_latched_fault())
            }
            // Output confirmed off: reboot-only recovery, nothing to do.
            (ChargeState::Latched, _) => Some(Action::None),
            // A self-clearing cause (see `ProtectionPolicy`) is the buck
            // waiting on a condition rather than failing, and it may
            // re-enable OUTPUT_EN by itself once the condition lifts. Step
            // back to the hold for the target it is still holding; the
            // gauntlet's bring-up gate turns the cause into this tick's
            // inhibit, so nothing has to set one by hand here.
            //
            // Any other cause is the buck's own hardware OVP/OCP or a panel
            // toggle — it is not coming back on its own.
            (s, Some(BuckOutput::Off { cause })) if s.sourcing() => {
                if cause.is_self_clearing() {
                    self.step(ChargeEvent::SelfDisabled);
                    None
                } else {
                    Some(self.on_fault(FaultReason::OutputUnexpectedlyOff(cause)))
                }
            }
            // Output on while we believe it off. Out of a hold that is the
            // recovery being waited for: setpoints went untouched through
            // it — the gauntlet's first check verifies that on this very
            // tick — so regulation resumes at known targets. This is why
            // `Boot` and `Hold*` are separate states rather than one
            // "output off" flag: the same reading means recovery from one
            // and an anomaly from the other.
            //
            // Anywhere else in bring-up it is an anomaly (firmware bug,
            // panel toggle, EMI on the button GPIO) — `boot_sequence` wrote
            // set_output(false) and verified OUTPUT_EN=0. Unlike every
            // other bring-up condition there IS something sourcing under
            // setpoints we never confirmed, so that one latches. Guarding
            // on `bringing_up` rather than naming `Boot` is what makes a
            // bring-up state added later fail closed instead of falling
            // through to the catch-all and being ignored while it sources.
            (s, Some(BuckOutput::On)) if s.bringing_up() => {
                if s.holding() {
                    self.step(ChargeEvent::SelfEnabled);
                    None
                } else {
                    Some(self.on_fault(FaultReason::OutputOnInPending))
                }
            }
            _ => None,
        }
    }

    /// The latch/inhibit rule in one place: the same condition disables a
    /// sourcing buck and merely blocks bring-up of an idle one.
    fn fault_or_inhibit(&self, latched: FaultReason, inhibited: InhibitReason) -> Verdict {
        if self.state.sourcing() {
            Verdict::Latch(latched)
        } else {
            Verdict::Inhibit(inhibited)
        }
    }

    /// The ordered safety gauntlet, run against the state
    /// [`Self::reconcile`] has already agreed with the device. **The order
    /// of the checks below is the specification** — each one may only be
    /// moved past checks it commutes with, and `tests/` pins the precedence
    /// where two can fire on the same tick.
    ///
    /// Whether a failure latches or merely inhibits is decided by the state
    /// and nothing else. A fault latches only while the buck is sourcing;
    /// in a bring-up state the output is already off, so a latch would
    /// disable nothing and cost a reboot to clear. The one condition that
    /// latches an idle machine is a buck reporting output ON, and that
    /// belongs to `reconcile` rather than here — by the time the gauntlet
    /// runs, "not sourcing" really does mean nothing is sourcing.
    ///
    /// Debouncers are stepped in both cases so their windows stay coherent
    /// across a move between them. `self.state` is only read here, never
    /// written — `tick` relies on that to dispatch on it afterwards.
    fn gauntlet(&mut self, p: &PollResult, elapsed: Duration) -> Verdict {
        // 1. Commanded vs. reported setpoints. No debounce: the read itself
        //    succeeded, so a mismatch is the device disagreeing with us
        //    rather than transport noise.
        if let Some(sp) = p.setpoints {
            let want = self.expected_setpoints();
            if (sp.v_set - want.v_set).abs() >= SETPOINT_DRIFT_TOL
                || (sp.i_set - want.i_set).abs() >= SETPOINT_DRIFT_TOL
            {
                return self
                    .fault_or_inhibit(FaultReason::SettingsDrift, InhibitReason::SettingsDrift);
            }
        }

        // 2. Modbus health. `p.setpoints.is_none()` doubles as the read-failed
        //    signal — a successful read means the link is up.
        if self
            .modbus_err
            .step(p.setpoints.is_none(), elapsed, MODBUS_UNHEALTHY_TIMEOUT)
        {
            return self
                .fault_or_inhibit(FaultReason::ModbusUnhealthy, InhibitReason::ModbusUnhealthy);
        }

        // 3. Battery sample freshness. NaN/Inf counts as missing: a sensor
        //    reporting non-finite values can't supervise charging, and
        //    silently ignoring it would let a stuck sensor mask overvoltage.
        let battery = p
            .battery
            .filter(|b| b.voltage.is_finite() && b.current.is_finite());
        if self
            .battery_missing
            .step(battery.is_none(), elapsed, BATTERY_MISSING_TIMEOUT)
        {
            return self.fault_or_inhibit(
                FaultReason::BatterySensorStale,
                InhibitReason::BatterySensorStale,
            );
        }
        let Some(b) = battery else {
            return Verdict::Inhibit(InhibitReason::NoBatterySample);
        };

        // 4. Overvoltage. Regulating needs the 3 s debounce so switching
        //    noise and load steps don't trip a healthy charge. Bring-up
        //    needs none: a single sample over the line is reason enough not
        //    to energise, and since that only inhibits, one noisy reading
        //    can no longer strand the unit off until a reboot.
        let ov = b.voltage > self.profile.absorb_v + OV_MARGIN_V;
        let ov_debounced = self.ov.step(ov, elapsed, OV_DURATION);
        if self.state.sourcing() {
            if ov_debounced {
                return Verdict::Latch(FaultReason::Overvoltage);
            }
        } else if ov {
            return Verdict::Inhibit(InhibitReason::Overvoltage);
        }

        // 5. Bring-up-only gates. Not faults — they say "not yet", and only
        //    mean anything while the output is off.
        if self.state.bringing_up() {
            // Demand a fresh setpoint readback before energising.
            // `boot_sequence` already verified the writes, but requiring
            // closed-loop confirmation here means we never ask for output-on
            // until the link is demonstrably alive. Check 2 eventually
            // inhibits on sustained failure, but takes 5 s; this covers the gap.
            if p.setpoints.is_none() {
                return Verdict::Inhibit(InhibitReason::ModbusUnhealthy);
            }
            // Enabling into a live self-clearing hold would succeed at the
            // Modbus layer while the buck stayed off, flapping EnableOutput
            // every poll.
            if let Some(BuckOutput::Off { cause }) = p.output
                && cause.is_self_clearing()
            {
                return Verdict::Inhibit(InhibitReason::BuckProtection(cause));
            }
        }

        Verdict::Clear(b)
    }

    /// Sourcing arm: output is on and every safety check just cleared. Runs
    /// the deferred V_SET write, then the Float↔Absorb phase machine and
    /// the absorb time cap.
    fn regulate(&mut self, b: BatterySample, elapsed: Duration) -> Action {
        // Re-emit UpdateVoltage until the caller acks the previous one. The
        // phase machine and absorb cap don't run while a write is in flight
        // — `setpoint_phase` still names the old target, so the drift check
        // keeps matching the live V_SET, and the caller retries on every
        // tick by writing again.
        if let Some(next) = self.state.retarget_to() {
            return self.update_voltage_for(next);
        }

        // Charging current as a positive number.
        let charging_a = -b.current;
        let below_exit =
            self.state == ChargeState::Absorb && charging_a < self.profile.exit_absorb_a;
        // Leaky, not hard-reset: a full pack makes the buck pulse current in
        // bursts that briefly exceed the tail threshold; those pulses must
        // shave the gate, not re-arm it from scratch (see `step_leaky`).
        let exit_done = self.exit.step_leaky(below_exit, elapsed, EXIT_DEBOUNCE);

        // Arming a retarget defers the V_SET move until the caller commits
        // the ticket, which keeps `setpoint_phase` matching the buck's
        // actual V_SET so a failed write doesn't trip SettingsDrift on the
        // next tick.
        match self.state {
            ChargeState::Float if charging_a > self.profile.enter_absorb_a => {
                self.step(ChargeEvent::TaperRose);
                return self.update_voltage_for(Phase::Absorb);
            }
            ChargeState::Absorb if exit_done => {
                self.step(ChargeEvent::TaperFell);
                return self.update_voltage_for(Phase::Float);
            }
            _ => {}
        }

        // Clock the absorb timeout only while the pack sits at the CV plateau.
        // A CC dip (load transient pulling voltage back below absorb_v) resets
        // it via Debounce — that's genuine charging, not a stuck taper.
        let at_cv = self.at_cv_plateau(b.voltage);
        if self.state == ChargeState::Absorb && self.absorb.step(at_cv, elapsed, MAX_ABSORB) {
            return self.on_fault(FaultReason::AbsorbTimeout);
        }
        Action::None
    }
}

/// What the charging tests read out of a supervisor, gated so none of it
/// widens the production surface.
#[cfg(test)]
pub(crate) mod internals {
    use xy_modbus::Setpoints;

    use crate::charging::charge_state::ChargeState;
    use crate::charging::charge_supervisor::ChargeSupervisor;
    use crate::charging::phase::Phase;

    /// A trait, not an inherent `impl`, because several of these names are
    /// already private inherent methods and a second inherent definition
    /// of a name is a duplicate-definition error however it is gated.
    /// Tests reach them by importing this; the bodies below spell out
    /// `ChargeSupervisor::…` so it is clear they forward rather than recurse.
    pub(crate) trait SupervisorInternals {
        fn state(&self) -> ChargeState;
        /// The V_SET the device is holding. Panics for a latched
        /// supervisor, which no longer regulates to one.
        fn target_voltage(&self) -> f32;
        /// As above, as the pair the drift check compares against.
        fn expected_setpoints(&self) -> Setpoints;
        /// What a healthy buck would report this tick: the supervisor's own
        /// expectation while it has one, and the float pair once it has
        /// latched and stopped keeping one — where all a fixture needs is a
        /// read that succeeded.
        fn readback_setpoints(&self) -> Setpoints;
    }

    impl SupervisorInternals for ChargeSupervisor {
        fn state(&self) -> ChargeState {
            self.state
        }

        fn target_voltage(&self) -> f32 {
            ChargeSupervisor::target_voltage(self)
        }

        fn expected_setpoints(&self) -> Setpoints {
            ChargeSupervisor::expected_setpoints(self)
        }

        fn readback_setpoints(&self) -> Setpoints {
            self.setpoints_for(self.state.setpoint_phase().unwrap_or(Phase::Float))
        }
    }
}
