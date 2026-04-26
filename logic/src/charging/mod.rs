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

/// Nominal DC input feeding the XY7025 buck. Used to derive the buck's input
/// UVLO (LVP register) — it has nothing to do with the pack profile.
pub const INPUT_NOMINAL_V: f32 = 24.0;
/// Headroom below `INPUT_NOMINAL_V` before the buck cuts output on input sag.
/// 2 V tolerates ~8% droop and stays well above the XY7025's 12 V minimum.
pub const INPUT_LVP_MARGIN_V: f32 = 2.0;
const _: () = assert!(INPUT_NOMINAL_V - INPUT_LVP_MARGIN_V > 12.0);

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
/// How long battery readings can stay absent before we fail closed.
const BATTERY_MISSING_TIMEOUT: Duration = Duration::from_secs(10);
/// How long Modbus reads to the XY can keep failing before we fail closed.
const MODBUS_UNHEALTHY_TIMEOUT: Duration = Duration::from_secs(5);

/// Allowed drift between commanded and observed setpoint. One register
/// quantum is 0.01; two-quantum slack absorbs IEEE-float round-trip
/// quirks on values like 14.4 V whose binary repr isn't exact.
const SETPOINT_DRIFT_TOL: f32 = 0.02;

// ─── Types ───────────────────────────────────────────────────────────────────

#[derive(Copy, Clone)]
pub enum Chemistry {
    /// Daily-cycling LFP: 3.60 V/cell absorb, 3.375 V/cell float.
    /// Matches Victron / Battle Born defaults — gentler on cells than 3.65 V,
    /// reaches ~99% SoC either way (Battery University BU-808b, Off-Grid Garage tests).
    LiFePo4,
    /// Top-balance variant for LFP: 3.65 V/cell absorb (manufacturer max).
    /// Use sparingly when the BMS needs the high voltage to balance cells.
    LiFePo4TopBalance,
    /// Longevity-tuned Li-ion (NMC/LCO): 4.10 V/cell absorb, 4.00 V/cell float.
    /// 4.10 V trades ~15% capacity for dramatically more cycles vs. 4.20 V.
    LiIon,
}

#[derive(Copy, Clone)]
pub struct Profile {
    pub absorb_v: f32,
    pub float_v: f32,
    /// Constant-current setpoint sent to the buck during normal charging.
    pub regulation_a: f32,
    pub enter_absorb_a: f32,
    pub exit_absorb_a: f32,
}

/// Hard trip limits programmed into the buck's own protection registers.
/// Last-resort backstops above the supervisor's debounced fault thresholds.
#[derive(Copy, Clone)]
pub struct SafetyLimits {
    pub ovp_v: f32,
    pub ocp_a: f32,
    pub lvp_v: f32,
}

/// Setpoints read back from the buck (V_SET / I_SET register pair). Fed
/// to the supervisor each tick so it can detect drift between what we
/// commanded and what the buck claims to be regulating to.
#[derive(Copy, Clone)]
pub struct Setpoints {
    pub v_set: f32,
    pub i_set: f32,
}

/// One poll cycle's view of the world for the supervisor: the buck's
/// readback (`None` if the Modbus read failed) and the latest fresh
/// battery sample (`None` if stale or absent). `setpoints.is_some()`
/// doubles as the modbus-healthy signal.
#[derive(Copy, Clone, Default)]
pub struct PollResult {
    pub setpoints: Option<Setpoints>,
    pub battery: Option<BatterySample>,
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
/// under the same conditions.
#[derive(Copy, Clone, PartialEq, Eq, IntoStaticStr)]
pub enum FaultReason {
    /// No fresh battery reading for `BATTERY_MISSING_TIMEOUT.as_secs()` consecutive ticks.
    /// Without current/voltage we cannot supervise charging — fail closed.
    #[strum(serialize = "battery sensor stale")]
    BatterySensorStale,
    /// Modbus reads to the XY7025 have been failing for `MODBUS_UNHEALTHY_TIMEOUT`
    /// continuously. We've lost closed-loop control over the buck; disable
    /// while we still can.
    #[strum(serialize = "modbus link unhealthy")]
    ModbusUnhealthy,
    /// Pack voltage exceeded `absorb_v + OV_MARGIN_V` for `OV_DURATION.as_secs()` ticks.
    /// Catches drift below the XY's hardware OVP trip but above the profile target.
    #[strum(serialize = "pack overvoltage")]
    Overvoltage,
    /// Absorb ran for `MAX_ABSORB.as_secs()` ticks. Under a parasitic load
    /// pinning current above `exit_absorb_a` we'd otherwise sit at CV
    /// forever.
    #[strum(serialize = "absorb time cap reached")]
    AbsorbTimeout,
    /// XY7025 setpoint readback (V_SET or I_SET) disagreed with what we
    /// commanded. The buck is sourcing under unknown setpoints — disable
    /// before it can do damage. Triggers immediately, no debounce: the
    /// caller already verified the read itself succeeded, so this isn't
    /// a transport glitch.
    #[strum(serialize = "setpoint readback drift")]
    SettingsDrift,
}

impl FaultReason {
    pub fn label(self) -> &'static str {
        self.into()
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
/// emitted until the disable is ACKed.
pub enum Action {
    None,
    /// Caller should write `set_output(true)`. V_SET is untouched —
    /// `boot_sequence` already programmed it to `float_v`, which is
    /// always the supervisor's target voltage in Pending.
    EnableOutput,
    /// Caller should write V_SET to `supervisor.target_voltage()`.
    /// Emitted after the phase machine transitions Float ↔ Absorb.
    UpdateVoltage,
    DisableOutput(FaultReason),
}

/// Latest fresh battery reading fed to the supervisor. Voltage is used for
/// OV detection, current drives the phase machine. Power isn't needed.
#[derive(Copy, Clone)]
pub struct BatterySample {
    pub voltage: f32,
    pub current: f32,
}

/// Latch state.
/// - `Pending`: output is OFF and we haven't yet emitted EnableOutput, or
///   we have but `ack_enable` hasn't been called yet (write may have
///   failed). Same safety checks as `Active`, but tick emits
///   `EnableOutput` instead of running the phase machine.
/// - `Active`: output is on, phase machine + drift + fault paths run.
/// - `Tripped { acked: false }`: a fault latched; emit `DisableOutput`.
/// - `Tripped { acked: true }`: caller successfully disabled; emit None.
enum LatchState {
    Pending,
    Active,
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
}

// ─── Impls ───────────────────────────────────────────────────────────────────

impl Chemistry {
    /// Per-cell (absorb_v, float_v). Scaled by cell count in `Profile::for_pack`.
    const fn per_cell(self) -> (f32, f32) {
        match self {
            Chemistry::LiFePo4 => (3.60, 3.375),
            Chemistry::LiFePo4TopBalance => (3.65, 3.375),
            Chemistry::LiIon => (4.10, 4.00),
        }
    }
}

impl Profile {
    /// Build a pack-level profile from chemistry, series cell count, and
    /// pack capacity. Voltages scale with `cells`; charge/taper currents
    /// scale with `capacity_ah` via the `*_C` constants above. Same C-rates
    /// across chemistries — the LFP literature is the basis, but the
    /// fractions are conservative enough that NMC/LCO are also safe.
    pub const fn for_pack(chemistry: Chemistry, cells: u8, capacity_ah: f32) -> Self {
        assert!(cells > 0);
        assert!(capacity_ah > 0.0);
        let (av, fv) = chemistry.per_cell();
        let s = cells as f32;
        Self {
            absorb_v: av * s,
            float_v: fv * s,
            regulation_a: capacity_ah * REGULATION_C,
            enter_absorb_a: capacity_ah * ENTER_ABSORB_C,
            exit_absorb_a: capacity_ah * EXIT_ABSORB_C,
        }
    }

    /// Derive hard trip thresholds for the buck's own protection. The buck
    /// fires these only when regulation has already failed — the supervisor's
    /// debounced OV at `absorb_v + OV_MARGIN_V` should catch problems first.
    /// Hardware OVP sits at 3× that margin so the supervisor always wins
    /// (the const-block above enforces this at compile time). OCP is 50%
    /// over the CC setpoint. LVP on the XY7025 is **input** UVLO, not a
    /// pack-side cutoff — it's tied to the supply rail, not the profile.
    pub const fn safety_limits(&self) -> SafetyLimits {
        SafetyLimits {
            ovp_v: self.absorb_v + HARDWARE_OVP_MARGIN_V,
            ocp_a: self.regulation_a * 1.5,
            lvp_v: INPUT_NOMINAL_V - INPUT_LVP_MARGIN_V,
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
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn target_voltage(&self) -> f32 {
        match self.phase {
            Phase::Float => self.profile.float_v,
            Phase::Absorb => self.profile.absorb_v,
        }
    }

    pub fn fault(&self) -> Option<FaultReason> {
        match self.latch {
            LatchState::Pending | LatchState::Active => None,
            LatchState::Tripped { reason, .. } => Some(reason),
        }
    }

    /// What setpoints the supervisor currently expects the buck to be
    /// regulating to. Used by the caller to construct `Setpoints` for tests
    /// and as documentation for what `tick` will compare readbacks against.
    pub fn expected_setpoints(&self) -> Setpoints {
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
            LatchState::Pending | LatchState::Active => {
                panic!("ack_disable without latched fault")
            }
        }
    }

    /// Caller invokes this after a successful `set_output(true)` Modbus write.
    /// Transitions Pending → Active; the supervisor's phase machine starts
    /// running on the next tick. Until acked, the supervisor keeps emitting
    /// `EnableOutput` so a failed enable write gets retried.
    pub fn ack_enable(&mut self) {
        match self.latch {
            LatchState::Pending => self.latch = LatchState::Active,
            _ => panic!("ack_enable from non-Pending state"),
        }
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
        let pending = match self.latch {
            LatchState::Tripped { acked: true, .. } => return Action::None,
            LatchState::Tripped {
                reason,
                acked: false,
            } => return Action::DisableOutput(reason),
            LatchState::Pending => true,
            LatchState::Active => false,
        };

        if let Some(sp) = p.setpoints {
            let want = self.expected_setpoints();
            if (sp.v_set - want.v_set).abs() >= SETPOINT_DRIFT_TOL
                || (sp.i_set - want.i_set).abs() >= SETPOINT_DRIFT_TOL
            {
                return self.latch(FaultReason::SettingsDrift);
            }
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
        if (pending && ov) || ov_debounced {
            return self.latch(FaultReason::Overvoltage);
        }

        // All safety checks clear. In Pending we haven't enabled output yet
        // — emit EnableOutput and stay Pending until the caller acks.
        // Phase machine doesn't run yet (output is OFF, no current
        // measurement is meaningful).
        if pending {
            return Action::EnableOutput;
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
            self.phase = next;
            // Reset both timers explicitly: a Float→Absorb transition can
            // immediately follow an Absorb→Float, with no intervening Float
            // dwell to clear stale counts.
            self.absorb.elapsed = Duration::ZERO;
            self.exit.elapsed = Duration::ZERO;
            return Action::UpdateVoltage;
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
        Action::DisableOutput(reason)
    }
}

#[cfg(test)]
mod tests;
