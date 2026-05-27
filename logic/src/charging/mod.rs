//! Two-phase CV charging strategy with hysteresis + supervisor latch.
//!
//! Sits in Float (low CV) by default. When the battery draws more than
//! `enter_absorb_a` of charging current, switches to Absorb (high CV) to
//! finish the pack. Once current tapers below `exit_absorb_a`, drops back to
//! Float. Profiles are per-chemistry constants.
//!
//! Wraps the phase logic with a latch that disables the buck on
//! overvoltage, stuck-absorb, missing-battery, or unhealthy-Modbus
//! conditions. Once latched, only a reboot clears it.
//!
//! Sign convention: battery current is **negative when charging** (matches
//! the INA228 wiring on this board). The supervisor takes signed amps and
//! negates internally, so profile thresholds stay positive and read
//! naturally.
//!
//! Pure logic: no I/O. The firmware calls `tick()` each poll and writes
//! the returned action to the buck converter.

use std::time::Duration;

use strum::IntoStaticStr;

use crate::battery::{self, Chemistry};

pub use xy_modbus::{ProtectionStatus, SafetyLimits, Setpoints};

// ─── Tunables ────────────────────────────────────────────────────────────────

/// CC charge rate as a fraction of pack capacity. 0.2C is the
/// longevity-tuned value; manufacturer max is 0.5C. Stay conservative.
pub const REGULATION_C: f32 = 0.2;
/// Tail-current threshold for ending Absorb, as a fraction of capacity.
/// 0.05C (= C/20) is the cell-manufacturer-standard termination current
/// for LFP — consensus across Battle Born, Victron, and Nordkyn Design
/// references.
pub const EXIT_ABSORB_C: f32 = 0.05;
/// Threshold for entering Absorb. Sits just above `EXIT_ABSORB_C` so the
/// hysteresis band straddles the manufacturer tail current — no flap once
/// the pack tapers near 0.05C.
pub const ENTER_ABSORB_C: f32 = 0.06;
const _: () = assert!(REGULATION_C > ENTER_ABSORB_C);
const _: () = assert!(ENTER_ABSORB_C > EXIT_ABSORB_C);

/// How far above `absorb_v` the pack must sit before the firmware's
/// debounced OV trip starts counting. Pub so callers (and the hardware-OVP
/// derivation below) reference one number, not a literal.
pub const OV_MARGIN_V: f32 = 0.2;
/// Margin above `absorb_v` programmed into the buck's own OVP register.
/// Must strictly exceed `OV_MARGIN_V` so the supervisor's debounced trip
/// fires first; the const-block below enforces it at compile time.
pub const HARDWARE_OVP_MARGIN_V: f32 = OV_MARGIN_V * 3.0;
const _: () = assert!(HARDWARE_OVP_MARGIN_V > OV_MARGIN_V);

/// Headroom below the DC input rail before the buck cuts output on input sag.
/// 2 V tolerates ~8% droop and stays well above the XY7025's 12 V minimum.
pub const INPUT_LVP_MARGIN_V: f32 = 2.0;

/// How long the pack must hold above `absorb_v + OV_MARGIN_V` before tripping.
/// Time-based so the debounce isn't sensitive to poll cadence.
const OV_DURATION: Duration = Duration::from_secs(3);
/// Cap on how long absorb can run continuously. With the manufacturer-spec
/// 0.05C tail (`EXIT_ABSORB_C`), a healthy pack tapers under 30 min — 2 h
/// is generous headroom while keeping a stuck-current scenario from sitting
/// at CV indefinitely.
const MAX_ABSORB: Duration = Duration::from_secs(2 * 60 * 60);
/// How long charging current must hold below `exit_absorb_a` before the
/// supervisor accepts the taper as real and drops back to Float. Filters
/// brief sags from switching noise or transient loads that would otherwise
/// finish absorb prematurely.
const EXIT_DEBOUNCE: Duration = Duration::from_secs(60);
/// How long `battery.is_none()` must persist before we fail closed.
/// Counts *after* the data layer has already flipped to `None` per its
/// own `STALE_TICKS` debounce — total time from last INA reading to a
/// latched buck-off is `data::STALE_TICKS + BATTERY_MISSING_TIMEOUT`.
const BATTERY_MISSING_TIMEOUT: Duration = Duration::from_secs(10);
/// How long Modbus reads to the XY can keep failing before we fail closed.
const MODBUS_UNHEALTHY_TIMEOUT: Duration = Duration::from_secs(5);

/// Allowed drift between commanded and observed setpoint. One register
/// quantum is 0.01; two-quantum slack absorbs IEEE-float round-trip
/// quirks on values like 14.4 V whose binary repr isn't exact.
const SETPOINT_DRIFT_TOL: f32 = 0.02;

/// How long the world must look healthy after `OutputUnexpectedlyOff`
/// before the supervisor signals the caller to restart. Long enough for
/// transient causes (input LVP from AC sag, over-temp cooldown) to
/// genuinely clear; short enough that operationally a brief input glitch
/// doesn't require a manual reboot.
const OUTPUT_RECOVERY_HEALTHY: Duration = Duration::from_secs(60);
/// Total recoveries from `OutputUnexpectedlyOff` allowed since boot. After
/// this many flap cycles, the caller stops restarting and leaves the buck
/// off — flapping is a real signal that something underlying is wrong.
/// Tracked by the caller (the supervisor is recreated on each restart and
/// can't carry the count itself).
pub const OUTPUT_RECOVERY_MAX_ATTEMPTS: u32 = 3;

// ─── Types ───────────────────────────────────────────────────────────────────

#[derive(Copy, Clone, Debug)]
pub struct Profile {
    pub chemistry: Chemistry,
    pub cells: u8,
    pub absorb_v: f32,
    pub float_v: f32,
    /// Constant-current setpoint sent to the buck during normal charging.
    pub regulation_a: f32,
    pub enter_absorb_a: f32,
    pub exit_absorb_a: f32,
}

/// One poll cycle's view of the world for the supervisor.
/// `setpoints` is from the V_SET/I_SET readback; `setpoints.is_some()`
/// doubles as the modbus-healthy signal. `battery` is independent —
/// it's the latest fresh INA228 reading.
#[derive(Copy, Clone, Default)]
pub struct PollResult {
    pub battery: Option<BatterySample>,
    pub setpoints: Option<Setpoints>,
    /// `None` means the OUTPUT_EN read itself failed.
    pub output: Option<BuckOutput>,
}

/// What the buck's OUTPUT_EN register reported this poll, plus the
/// PROTECT (0x0010) cause when output is off. The two were separate
/// fields once but they covary: PROTECT is necessarily Normal while
/// output is on, and is read only when output is off — so the relation
/// belongs in the type.
#[derive(Copy, Clone)]
pub enum BuckOutput {
    /// OUTPUT_EN reads 1.
    On,
    /// OUTPUT_EN reads 0. `cause` carries the PROTECT register value if
    /// we managed to read it; `None` means the PROTECT read itself
    /// failed.
    Off { cause: Option<ProtectionStatus> },
}

/// Policy: whether a self-disable for a given protection cause is safe
/// to auto-recover from. Conservative — only causes that are *likely*
/// transient and pose no fresh risk if they re-fire after a wait.
/// OCP/OPP can signal a real downstream fault (short, sticky FET); OVP
/// means pack-side trouble; energy/time limits hit programmed budgets.
/// Reboot-required for those.
fn is_recoverable(cause: ProtectionStatus) -> bool {
    match cause {
        // Input-side and over-temp issues that genuinely clear with
        // time (AC sag returns, fan cools the case).
        ProtectionStatus::Lvp | ProtectionStatus::Otp => true,
        // Output went off but the device reports no protection.
        // Means a front-panel toggle, external Modbus write, or
        // EMI on the panel button GPIO — all human/environmental
        // causes that warrant someone looking. Auto-recovering
        // would just burn the restart budget on something that
        // didn't fix itself.
        ProtectionStatus::Normal => false,
        // Output / pack / load problems that need someone to look.
        ProtectionStatus::Ovp
        | ProtectionStatus::Ocp
        | ProtectionStatus::Opp
        | ProtectionStatus::Oah
        | ProtectionStatus::Ohp
        | ProtectionStatus::Oep
        | ProtectionStatus::Owh
        | ProtectionStatus::Icp => false,
        // Off-spec read — don't trust ourselves.
        ProtectionStatus::Unknown(_) => false,
    }
}

#[derive(Copy, Clone, PartialEq, Eq, IntoStaticStr)]
#[strum(serialize_all = "lowercase")]
pub enum Phase {
    Float,
    Absorb,
}

impl Phase {
    pub fn label(self) -> &'static str {
        self.into()
    }
}

/// Why the supervisor latched the buck off. Once latched, only a reboot
/// clears it — auto-recovery on a battery charger means trying again
/// under the same conditions. `OutputUnexpectedlyOff` carries the
/// device-reported protection cause when we managed to read it (`None`
/// means the PROTECT register read itself failed).
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum FaultReason {
    /// No fresh battery reading for `BATTERY_MISSING_TIMEOUT.as_secs()` consecutive ticks.
    /// Without current/voltage we cannot supervise charging — fail closed.
    BatterySensorStale,
    /// Modbus reads to the XY7025 have been failing for `MODBUS_UNHEALTHY_TIMEOUT`
    /// continuously. We've lost closed-loop control over the buck; disable
    /// while we still can.
    ModbusUnhealthy,
    /// Pack voltage exceeded `absorb_v + OV_MARGIN_V` for `OV_DURATION.as_secs()` ticks.
    /// Catches drift below the XY's hardware OVP trip but above the profile target.
    Overvoltage,
    /// Absorb ran for `MAX_ABSORB.as_secs()` ticks. Under a parasitic load
    /// pinning current above `exit_absorb_a` we'd otherwise sit at CV
    /// forever.
    AbsorbTimeout,
    /// XY7025 setpoint readback (V_SET or I_SET) disagreed with what we
    /// commanded. The buck is sourcing under unknown setpoints — disable
    /// before it can do damage. Triggers immediately, no debounce: the
    /// caller already verified the read itself succeeded, so this isn't
    /// a transport glitch.
    SettingsDrift,
    /// Buck's OUTPUT_EN register read 0 while the supervisor was Active.
    /// The buck self-disabled — its own hardware OVP / OCP / LVP /
    /// over-temp tripped, or someone toggled the front panel. Payload is
    /// the cause from PROTECT (0x0010); `None` if that read failed.
    OutputUnexpectedlyOff(Option<ProtectionStatus>),
    /// Buck's OUTPUT_EN register read 1 while the supervisor was Pending —
    /// output is supposed to be off until the supervisor itself enables it.
    /// Means the boot disable / S_INI=0 didn't stick or the front panel
    /// toggled it on. We don't know what setpoints regulation is using;
    /// fail closed and reboot.
    OutputOnInPending,
}

impl std::fmt::Display for FaultReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::BatterySensorStale => f.write_str("battery sensor stale"),
            Self::ModbusUnhealthy => f.write_str("modbus link unhealthy"),
            Self::Overvoltage => f.write_str("pack overvoltage"),
            Self::AbsorbTimeout => f.write_str("absorb time cap reached"),
            Self::SettingsDrift => f.write_str("setpoint readback drift"),
            Self::OutputUnexpectedlyOff(Some(s)) => write!(f, "buck self-disabled ({s})"),
            Self::OutputUnexpectedlyOff(None) => f.write_str("buck self-disabled (cause unread)"),
            Self::OutputOnInPending => f.write_str("buck output on while supervisor pending"),
        }
    }
}

impl FaultReason {
    /// How long the world must look healthy before the caller may restart
    /// the supervise loop. `None` means reboot-only recovery; only
    /// `OutputUnexpectedlyOff` is currently recoverable since its common
    /// causes (input LVP, over-temp, transient panel toggle) genuinely
    /// clear without operator intervention. Hard safety faults (OV, drift,
    /// absorb timeout) stay reboot-only.
    pub fn recovery_healthy_for(self) -> Option<Duration> {
        match self {
            // Only recover when the device-reported cause is one we
            // believe is transient (LVP/OTP). OCP/OVP/etc. would be
            // re-energizing into a possibly-still-tripped condition.
            // Normal means human/environmental cause (panel toggle,
            // external write, EMI on the button GPIO) — not transient
            // in the sense that matters. Cause=None (PROTECT read
            // failed): conservative, treat as non-recoverable.
            Self::OutputUnexpectedlyOff(Some(cause)) if is_recoverable(cause) => {
                Some(OUTPUT_RECOVERY_HEALTHY)
            }
            _ => None,
        }
    }
}

/// What the poll loop should do this tick.
///
/// The supervisor boots in a `Pending` latch state — output is OFF and we
/// haven't decided it's safe to enable yet. Each tick re-runs the same
/// safety checks as the active path; once all clear, the supervisor emits
/// `EnableOutput` and stays Pending until the caller `ack_enable`s. After
/// that it transitions to active operation: phase machine + drift +
/// fault paths. After a fault latches, only `DisableOutput` is ever
/// emitted until the disable is ACKed. Once acked, recoverable faults
/// accumulate a healthy window and eventually emit `RestartSupervisor`.
pub enum Action {
    None,
    /// Caller should write `set_output(true)`. V_SET is untouched —
    /// `boot_sequence` already programmed it to `float_v`, which is
    /// always the supervisor's target voltage in Pending.
    EnableOutput,
    /// Caller should write V_SET to `target_v` then call
    /// `ack_voltage_update`. Emitted while the phase machine wants to
    /// transition Float ↔ Absorb but the new voltage hasn't been
    /// successfully written yet — re-emits each tick until acked, so a
    /// transient Modbus glitch on the write retries instead of latching
    /// `SettingsDrift`.
    UpdateVoltage {
        target_v: f32,
    },
    DisableOutput(FaultReason),
    /// Latched fault is recoverable, the world has looked healthy
    /// (Modbus up, output off, finite battery below OV) for the
    /// fault's `recovery_healthy_for` window. Caller should tear this
    /// supervisor down, re-run `boot_sequence`, and construct a fresh
    /// `ChargeSupervisor`. Re-emits each tick while the conditions
    /// hold; the caller's per-boot restart budget is what stops a flap
    /// loop, not the supervisor.
    RestartSupervisor,
}

/// Latest fresh battery reading fed to the supervisor. Voltage is used for
/// OV detection, current drives the phase machine. Power isn't needed.
#[derive(Copy, Clone, Debug)]
pub struct BatterySample {
    pub voltage: f32,
    pub current: f32,
}

/// Latch state.
/// - `Pending`: output is OFF and we haven't yet emitted EnableOutput, or
///   we have but `ack_enable` hasn't been called yet (write may have
///   failed). Same safety checks as `Active`, but tick emits
///   `EnableOutput` instead of running the phase machine.
/// - `Active { pending_voltage }`: output is on, phase machine + drift +
///   fault paths run. `pending_voltage` is `Some(next)` while a
///   Float↔Absorb V_SET write is in flight: tick re-emits `UpdateVoltage`
///   each cycle (so a transient Modbus glitch retries instead of latching
///   `SettingsDrift`), and `target_voltage` keeps reporting the **old**
///   phase's voltage until `ack_voltage_update` commits the transition.
/// - `Tripped { acked: false }`: a fault latched; emit `DisableOutput`.
/// - `Tripped { acked: true }`: caller successfully disabled. For
///   recoverable faults the supervisor accumulates a healthy-window
///   timer and, once met, returns `Action::RestartSupervisor` from
///   `tick`. Non-recoverable faults stay parked in `Action::None`.
enum LatchState {
    Pending,
    Active { pending_voltage: Option<Phase> },
    Tripped { reason: FaultReason, acked: bool },
}

/// Time-based debouncer: counts elapsed while `cond` holds, resets when it
/// doesn't. One per condition we care about (OV, absorb cap, exit taper,
/// missing battery, modbus errors).
#[derive(Default)]
struct Debounce {
    elapsed: Duration,
}

impl Debounce {
    /// Add `dt` if `cond`, else reset. Returns `true` once accumulated
    /// `>= timeout`.
    fn step(&mut self, cond: bool, dt: Duration, timeout: Duration) -> bool {
        if cond {
            self.elapsed = self.elapsed.saturating_add(dt);
            self.elapsed >= timeout
        } else {
            self.elapsed = Duration::ZERO;
            false
        }
    }
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
    /// Healthy-state debouncer advanced only by `tick_recovery`. Reset on
    /// each new latch. Only meaningful when the latched fault has a
    /// recovery policy.
    recovery: Debounce,
}

// ─── Impls ───────────────────────────────────────────────────────────────────

impl Profile {
    /// Build a pack-level profile from chemistry, series cell count, and
    /// pack capacity. Voltages scale with `cells`; charge/taper currents
    /// scale with `capacity_ah` via the `*_C` constants above. Same C-rates
    /// across chemistries — the LFP literature is the basis, but the
    /// fractions are conservative enough that NMC/LCO are also safe.
    pub const fn for_pack(chemistry: Chemistry, cells: u8, capacity_ah: f32) -> Self {
        assert!(cells > 0);
        assert!(capacity_ah > 0.0);
        let v = chemistry.charge_voltages();
        let s = cells as f32;
        Self {
            chemistry,
            cells,
            absorb_v: v.absorb_v * s,
            float_v: v.float_v * s,
            regulation_a: capacity_ah * REGULATION_C,
            enter_absorb_a: capacity_ah * ENTER_ABSORB_C,
            exit_absorb_a: capacity_ah * EXIT_ABSORB_C,
        }
    }

    /// Estimated state-of-charge (0.0–100.0) from pack bus voltage, using
    /// this pack's chemistry and cell count.
    pub fn soc(&self, pack_voltage_v: f32) -> f32 {
        battery::ocv_soc(self.chemistry, self.cells, pack_voltage_v)
    }

    /// Derive hard trip thresholds for the buck's own protection. The buck
    /// fires these only when regulation has already failed — the supervisor's
    /// debounced OV at `absorb_v + OV_MARGIN_V` should catch problems first.
    /// Hardware OVP sits at 3× that margin so the supervisor always wins
    /// (the const-block above enforces this at compile time). OCP is 50%
    /// over the CC setpoint. LVP on the XY7025 is **input** UVLO, not a
    /// pack-side cutoff — it's tied to the supply rail, not the profile.
    pub const fn safety_limits(&self, input_nominal_v: f32) -> SafetyLimits {
        SafetyLimits {
            ovp_v: self.absorb_v + HARDWARE_OVP_MARGIN_V,
            ocp_a: self.regulation_a * 1.5,
            lvp_v: input_nominal_v - INPUT_LVP_MARGIN_V,
        }
    }
}

impl ChargeSupervisor {
    pub fn new(profile: Profile) -> Self {
        assert!(profile.absorb_v > profile.float_v);
        // Boot conservative: Phase::Float (re-derive from observed current,
        // never resume Absorb across a reset) and LatchState::Pending
        // (output stays OFF until the first healthy tick — bringing up the
        // buck is the supervisor's job, so cold-boot can't bypass safety).
        Self {
            profile,
            phase: Phase::Float,
            ov: Debounce::default(),
            absorb: Debounce::default(),
            exit: Debounce::default(),
            battery_missing: Debounce::default(),
            modbus_err: Debounce::default(),
            latch: LatchState::Pending,
            recovery: Debounce::default(),
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    fn target_voltage(&self) -> f32 {
        self.voltage_for_phase(self.phase)
    }

    fn voltage_for_phase(&self, phase: Phase) -> f32 {
        match phase {
            Phase::Float => self.profile.float_v,
            Phase::Absorb => self.profile.absorb_v,
        }
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

    /// Caller invokes this after a successful `set_output(false)` Modbus write.
    /// Until then, the supervisor will keep emitting `DisableOutput` so a
    /// failed disable write gets retried on every tick.
    pub fn ack_disable(&mut self) {
        match &mut self.latch {
            LatchState::Tripped { acked, .. } => *acked = true,
            _ => panic!("ack_disable without latched fault"),
        }
    }

    /// Caller invokes this after a successful `set_output(true)` Modbus write.
    /// Transitions Pending → Active; the supervisor's phase machine starts
    /// running on the next tick. Until acked, the supervisor keeps emitting
    /// `EnableOutput` so a failed enable write gets retried.
    pub fn ack_enable(&mut self) {
        match self.latch {
            LatchState::Pending => {
                self.latch = LatchState::Active {
                    pending_voltage: None,
                }
            }
            _ => panic!("ack_enable from non-Pending state"),
        }
    }

    /// Caller invokes this after a successful `set_voltage(target)` Modbus
    /// write that resulted from `Action::UpdateVoltage`. Commits the
    /// pending phase transition: the new phase becomes the supervisor's
    /// `target_voltage()` (drift check switches to the new value on the
    /// next tick) and the absorb/exit debouncers reset. If the write
    /// fails, the caller does NOT call this — the supervisor stays at
    /// the old phase, drift check keeps matching old V_SET, and the next
    /// tick re-emits `UpdateVoltage` for retry.
    pub fn ack_voltage_update(&mut self) {
        let LatchState::Active {
            pending_voltage: Some(next),
        } = self.latch
        else {
            panic!("ack_voltage_update without pending phase");
        };
        self.phase = next;
        self.latch = LatchState::Active {
            pending_voltage: None,
        };
        // Reset both timers explicitly: a Float→Absorb transition can
        // immediately follow an Absorb→Float, with no intervening Float
        // dwell to clear stale counts.
        self.absorb.elapsed = Duration::ZERO;
        self.exit.elapsed = Duration::ZERO;
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
        let pre_enable = match self.latch {
            LatchState::Tripped {
                reason,
                acked: true,
            } => return self.tick_recovery(reason, &p, elapsed),
            LatchState::Tripped {
                reason,
                acked: false,
            } => return Action::DisableOutput(reason),
            LatchState::Pending => true,
            LatchState::Active { .. } => false,
        };

        if let Some(sp) = p.setpoints {
            let want = self.expected_setpoints();
            if (sp.v_set - want.v_set).abs() >= SETPOINT_DRIFT_TOL
                || (sp.i_set - want.i_set).abs() >= SETPOINT_DRIFT_TOL
            {
                return self.latch(FaultReason::SettingsDrift);
            }
        }

        // Mismatch between latch state and what the buck reports.
        // Active expects ON: any OFF means the buck self-disabled
        // (hardware OVP/OCP/LVP, over-temp, panel toggle).
        // Pending expects OFF: any ON means our boot disable / S_INI=0
        // didn't stick — fail closed and reboot rather than trust
        // unknown regulation.
        match (pre_enable, p.output) {
            (false, Some(BuckOutput::Off { cause })) => {
                return self.latch(FaultReason::OutputUnexpectedlyOff(cause));
            }
            (true, Some(BuckOutput::On)) => {
                return self.latch(FaultReason::OutputOnInPending);
            }
            _ => {}
        }

        if self
            .modbus_err
            .step(p.setpoints.is_none(), elapsed, MODBUS_UNHEALTHY_TIMEOUT)
        {
            return self.latch(FaultReason::ModbusUnhealthy);
        }

        // NaN-poisoned samples are treated as missing — they can't safely
        // drive OV or the phase machine, and the sensor-stale debounce is
        // the right place to fail closed.
        let battery = p
            .battery
            .filter(|b| b.voltage.is_finite() && b.current.is_finite());
        if self
            .battery_missing
            .step(battery.is_none(), elapsed, BATTERY_MISSING_TIMEOUT)
        {
            return self.latch(FaultReason::BatterySensorStale);
        }
        let Some(b) = battery else {
            return Action::None;
        };

        // OV is undebounced in Pending — a pack already over the threshold
        // at boot must never see EnableOutput. In Active the 3 s debounce
        // filters transients caused by switching noise / load steps. Always
        // step the debouncer so its state stays coherent for Active.
        let ov = b.voltage > self.profile.absorb_v + OV_MARGIN_V;
        let ov_debounced = self.ov.step(ov, elapsed, OV_DURATION);
        if (pre_enable && ov) || ov_debounced {
            return self.latch(FaultReason::Overvoltage);
        }

        // All safety checks clear. In Pending we haven't enabled output yet
        // — emit EnableOutput and stay Pending until the caller acks.
        // Phase machine doesn't run yet (output is OFF, no current
        // measurement is meaningful).
        //
        // Require at least one successful setpoint readback before
        // energizing — `boot_sequence` already verified the writes, but
        // demanding fresh closed-loop confirmation here means we never
        // ask for output-on until the Modbus link is demonstrably alive.
        // The modbus_err debounce above eventually fails closed on
        // sustained read failures, but takes 5 s; this gate avoids
        // emitting EnableOutput in the meantime.
        if pre_enable {
            return if p.setpoints.is_some() {
                Action::EnableOutput
            } else {
                Action::None
            };
        }

        // Re-emit UpdateVoltage until the caller acks the previous one.
        // The phase machine and absorb-cap don't run while a write is in
        // flight — drift check keeps matching the old V_SET (since
        // `target_voltage` reflects the still-current phase), and the
        // caller retries on every tick by writing again.
        if let LatchState::Active {
            pending_voltage: Some(next),
        } = self.latch
        {
            return Action::UpdateVoltage {
                target_v: self.voltage_for_phase(next),
            };
        }

        // Charging current as a positive number.
        let charging_a = -b.current;
        let below_exit = self.phase == Phase::Absorb && charging_a < self.profile.exit_absorb_a;
        let exit_done = self.exit.step(below_exit, elapsed, EXIT_DEBOUNCE);

        let next = match self.phase {
            Phase::Float if charging_a > self.profile.enter_absorb_a => Phase::Absorb,
            Phase::Absorb if exit_done => Phase::Float,
            p => p,
        };
        if next != self.phase {
            // Defer the phase commit until the caller acks — keeps
            // `target_voltage` matching the buck's actual V_SET so a
            // failed write doesn't trigger SettingsDrift on the next
            // tick. Caller invokes `ack_voltage_update` on success.
            self.latch = LatchState::Active {
                pending_voltage: Some(next),
            };
            return Action::UpdateVoltage {
                target_v: self.voltage_for_phase(next),
            };
        }

        if self.phase == Phase::Absorb && self.absorb.step(true, elapsed, MAX_ABSORB) {
            return self.latch(FaultReason::AbsorbTimeout);
        }
        Action::None
    }

    fn latch(&mut self, reason: FaultReason) -> Action {
        self.latch = LatchState::Tripped {
            reason,
            acked: false,
        };
        // New latch — reset recovery clock.
        self.recovery.elapsed = Duration::ZERO;
        Action::DisableOutput(reason)
    }

    /// Recovery path for `Tripped { acked: true }` — accumulates a
    /// healthy window and emits `RestartSupervisor` once the fault's
    /// `recovery_healthy_for` budget is met. Non-recoverable faults
    /// stay parked in `Action::None`. "Healthy" here: Modbus up, battery
    /// present and finite, pack below the OV threshold, and the buck
    /// still reporting output OFF (any spontaneous re-enable is
    /// unmodeled — reset the clock until we see a clean stable state).
    fn tick_recovery(&mut self, reason: FaultReason, p: &PollResult, elapsed: Duration) -> Action {
        let Some(healthy_for) = reason.recovery_healthy_for() else {
            return Action::None;
        };
        let battery_ok = p
            .battery
            .map(|b| {
                b.voltage.is_finite()
                    && b.current.is_finite()
                    && b.voltage <= self.profile.absorb_v + OV_MARGIN_V
            })
            .unwrap_or(false);
        let healthy =
            p.setpoints.is_some() && matches!(p.output, Some(BuckOutput::Off { .. })) && battery_ok;
        if self.recovery.step(healthy, elapsed, healthy_for) {
            Action::RestartSupervisor
        } else {
            Action::None
        }
    }
}

#[cfg(test)]
mod tests;
