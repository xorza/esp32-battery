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
//! The supervisor proper is pure logic: no I/O. The firmware calls
//! `tick()` each poll and writes the returned `Action` to the buck.
//! `apply_update_voltage` is the one I/O-bound helper, hosted here so
//! the safe-step-down sequencing is testable against a `VoltageWriter`
//! mock without an esp-idf target.

use std::thread;
use std::time::Duration;

use log::{error, info, warn};
use strum::IntoStaticStr;

use crate::battery::{self, Chemistry};
use crate::error_log::XyError;

// `XyError` is xy-modbus's top-level failure (input / bad register value /
// transport). Renamed on import: `error_log::XyError` is this crate's
// event kind and the two would otherwise collide in every module that
// touches both.
pub use xy_modbus::{ProtectionStatus, RtuError, SafetyLimits, Setpoints, XyError as BusError};
// Imported by name for terse pattern matching on the buck's PROTECT register —
// LVP and OTP are the only causes the supervisor handles in-place.
use ProtectionStatus::{Lvp, Otp};

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
/// Cap on how long the pack may hold at the CV plateau (`absorb_v`) without
/// the current tapering out. Clocks the CV phase only — *not* the CC ramp,
/// which from a deeply discharged pack at 0.2C can legitimately run several
/// hours before the pack even reaches `absorb_v`. With the manufacturer-spec
/// 0.05C tail (`EXIT_ABSORB_C`), a healthy pack tapers under 30 min once at
/// CV — 2 h is generous headroom while keeping a stuck-current scenario from
/// sitting at CV indefinitely.
const MAX_ABSORB: Duration = Duration::from_secs(2 * 60 * 60);
/// Pack voltage within this of `absorb_v` counts as "at the CV plateau" for
/// the `MAX_ABSORB` clock. Wide enough to absorb sensing noise / IR drop at
/// the knee, narrow enough that the CC ramp (well below `absorb_v`) never
/// arms the timer.
const ABSORB_CV_BAND_V: f32 = 0.1;
/// How long charging current must hold below `exit_absorb_a` before the
/// supervisor accepts the taper as real and drops back to Float. Applied as a
/// *leaky* window (`Debounce::step_leaky`): above-tail pulses drain it rather
/// than resetting it, so the gate fires once the net time below tail crosses
/// this — i.e. once the average charging current sits below the tail. Filters
/// both brief sags (under the old hard reset) and the buck's burst pulses at a
/// full pack (which the hard reset could never get past).
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

// ─── Types ───────────────────────────────────────────────────────────────────

#[derive(Copy, Clone, Debug)]
pub struct Profile {
    pub chemistry: Chemistry,
    pub cells: u8,
    /// Rated pack capacity — the input the `*_a` currents were scaled from.
    /// Kept for display/identity; not used by the supervisor.
    pub capacity_ah: f32,
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
/// output is on, and is read in the same bulk transaction as OUTPUT_EN,
/// so the relation belongs in the type. `cause: Normal` covers the
/// "output is off and the buck reports no protection cause" case
/// (e.g. fresh-off after boot, post-disable, panel toggle).
#[derive(Copy, Clone)]
pub enum BuckOutput {
    /// OUTPUT_EN reads 1.
    On,
    /// OUTPUT_EN reads 0; PROTECT register value carried inline.
    Off { cause: ProtectionStatus },
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
/// device-reported PROTECT cause that was active when the buck
/// self-disabled (or `Normal` if no cause was set).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
    /// Pack held at the CV plateau (`absorb_v`) for `MAX_ABSORB.as_secs()`
    /// ticks without tapering out. Under a parasitic load pinning current
    /// above `exit_absorb_a` we'd otherwise sit at CV forever. The CC ramp
    /// up to `absorb_v` doesn't count — only time spent actually at CV.
    AbsorbTimeout,
    /// XY7025 setpoint readback (V_SET or I_SET) disagreed with what we
    /// commanded. The buck is sourcing under unknown setpoints — disable
    /// before it can do damage. Triggers immediately, no debounce: the
    /// caller already verified the read itself succeeded, so this isn't
    /// a transport glitch.
    SettingsDrift,
    /// Buck's OUTPUT_EN register read 0 while the supervisor was Active.
    /// The buck self-disabled — its own hardware OVP / OCP / over-temp
    /// tripped, or someone toggled the front panel (in which case PROTECT
    /// reads `Normal`). LVP/OTP are intercepted earlier and don't reach
    /// here. Payload is the cause from PROTECT (0x0010).
    OutputUnexpectedlyOff(ProtectionStatus),
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
            Self::OutputUnexpectedlyOff(s) => write!(f, "buck self-disabled ({s})"),
            Self::OutputOnInPending => f.write_str("buck output on while supervisor pending"),
        }
    }
}

impl FaultReason {
    /// Stable snake_case identifier — what API consumers and dashboards
    /// match on. The `Display` impl is the human-readable form for logs.
    pub fn label(self) -> &'static str {
        match self {
            Self::BatterySensorStale => "battery_sensor_stale",
            Self::ModbusUnhealthy => "modbus_unhealthy",
            Self::Overvoltage => "overvoltage",
            Self::AbsorbTimeout => "absorb_timeout",
            Self::SettingsDrift => "settings_drift",
            Self::OutputUnexpectedlyOff(_) => "output_unexpectedly_off",
            Self::OutputOnInPending => "output_on_in_pending",
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
/// emitted until the disable is ACKed; the supervisor then sits in
/// `Action::None` indefinitely (reboot-only recovery — transient
/// protection causes LVP/OTP are handled in-place without latching).
#[derive(Debug)]
pub enum Action {
    None,
    /// Caller should write `set_output(true)` then call
    /// `ack_enable(resume_absorb)`. `resume_absorb` reports whether the
    /// supervisor will jump straight to Absorb on the first Active tick
    /// (pack rests below the CV plateau, so it isn't full) or park in
    /// Float. Surfacing it here keeps the bring-up decision visible at
    /// the call site instead of buried in supervisor state.
    EnableOutput {
        resume_absorb: bool,
    },
    /// Caller should write V_SET to `target_v` then call
    /// `ack_voltage_update`. Emitted while the phase machine wants to
    /// transition Float ↔ Absorb but the new voltage hasn't been
    /// successfully written yet — re-emits each tick until acked, so a
    /// transient Modbus glitch on the write retries instead of latching
    /// `SettingsDrift`.
    ///
    /// `cycle_output: true` means `target_v` is *below* the current
    /// V_SET. The caller MUST disable output before writing V_SET and
    /// re-enable after, in that order. Stepping V_SET down with output
    /// enabled drives reverse current through the buck's synchronous
    /// low-side FET (the battery sources back into the buck as the
    /// control loop tries to pull V_OUT down to the new setpoint),
    /// which can destroy the FET and propagate upstream through the
    /// input rail — the XY7025 has no anti-backup protection on either
    /// the output or the input. `false` means a step-up, which is safe
    /// to do live.
    UpdateVoltage {
        target_v: f32,
        cycle_output: bool,
    },
    DisableOutput(FaultReason),
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
/// - `Tripped { acked: true }`: caller successfully disabled. Reboot-only
///   recovery — `tick` returns `Action::None` from here on.
enum LatchState {
    Pending { reason: PendingReason },
    Active { pending_voltage: Option<Phase> },
    Tripped { reason: FaultReason, acked: bool },
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
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum PendingReason {
    Boot,
    ProtectRecovery,
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

    /// Like `step`, but a false `cond` *drains* the accumulator by `dt`
    /// (floored at zero) instead of zeroing it. Firing at `>= timeout` then
    /// means "net time-true exceeded the window" — equivalently, `cond` held
    /// for more than half the recent window on average. Used for the
    /// Absorb-exit taper gate: a nearly-full pack drives the XY7025 into burst
    /// pulses (0 → several amps every few seconds), so the instantaneous
    /// charging current keeps poking back above the tail threshold. Under a
    /// hard reset each pulse re-arms the full window forever and pins the
    /// supervisor in Absorb; draining lets the mostly-below-tail average still
    /// reach the timeout, while a genuine *sustained* return to charging
    /// drains it back to zero and blocks the exit.
    fn step_leaky(&mut self, cond: bool, dt: Duration, timeout: Duration) -> bool {
        if cond {
            self.elapsed = self.elapsed.saturating_add(dt);
        } else {
            self.elapsed = self.elapsed.saturating_sub(dt);
        }
        self.elapsed >= timeout
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
            capacity_ah,
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

/// Compact pack identity for the LCD / web UI, e.g. `LFP 4S 50Ah`.
impl std::fmt::Display for Profile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{} {}S {:.0}Ah",
            self.chemistry, self.cells, self.capacity_ah
        )
    }
}

impl ChargeSupervisor {
    pub fn new(profile: Profile) -> Self {
        assert!(profile.absorb_v > profile.float_v);
        // Boot conservative: Phase::Float and LatchState::Pending (output
        // stays OFF until the first healthy tick — bringing up the buck is
        // the supervisor's job, so cold-boot can't bypass safety). We never
        // trust a *stored* phase across a reset, but the Pending bring-up
        // re-derives it from the pack's resting voltage: a pack below the CV
        // plateau isn't full, so `ack_enable` resumes Absorb rather than
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
        }
    }

    /// Float→Absorb and the LVP/OTP intercept both clear these so the
    /// next CV-plateau dwell starts fresh and the exit-taper isn't
    /// pre-armed from a load transient that happened before the
    /// transition.
    fn reset_phase_timers(&mut self) {
        self.absorb.elapsed = Duration::ZERO;
        self.exit.elapsed = Duration::ZERO;
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    /// Phase only while the supervisor is actually regulating (output ON).
    /// `None` in Pending (output still off, waiting to enable) and Tripped
    /// (latched fault). Surfaced to the dashboard so "Float" / "Absorb"
    /// labels appear only when they describe a live charging state.
    pub fn active_phase(&self) -> Option<Phase> {
        matches!(self.latch, LatchState::Active { .. }).then_some(self.phase)
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
    /// because `self.phase` only changes on `ack_voltage_update`.
    fn update_voltage_for(&self, next: Phase) -> Action {
        let target_v = self.voltage_for_phase(next);
        Action::UpdateVoltage {
            target_v,
            cycle_output: target_v < self.voltage_for_phase(self.phase),
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
    ///
    /// `resume_absorb` echoes the bool from the `Action::EnableOutput` the
    /// caller is acking — `true` means the pack rested below the CV plateau
    /// so it isn't full and the first Active tick should step V_SET
    /// float_v → absorb_v (otherwise the supervisor parks in Float). A pack
    /// power-cycled above ~75% rests too near `float_v` to ever draw
    /// `enter_absorb_a`, so without this it would stall there and never
    /// finish charging.
    pub fn ack_enable(&mut self, resume_absorb: bool) {
        match self.latch {
            LatchState::Pending { .. } => {
                let pending_voltage = resume_absorb.then_some(Phase::Absorb);
                self.latch = LatchState::Active { pending_voltage };
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
        // A Float→Absorb transition can immediately follow an
        // Absorb→Float, with no intervening Float dwell to clear
        // stale counts.
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
        let pending_reason = match self.latch {
            LatchState::Tripped {
                reason,
                acked: false,
            } => return Action::DisableOutput(reason),
            // Tripped+acked: reboot-only recovery, supervisor parks here.
            LatchState::Tripped { acked: true, .. } => return Action::None,
            LatchState::Pending { reason } => Some(reason),
            LatchState::Active { .. } => None,
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
        // (hardware OVP/OCP, panel toggle, missing PROTECT read).
        // Pending expects OFF: any ON means our boot disable / S_INI=0
        // didn't stick — fail closed rather than trust unknown
        // regulation.
        //
        // LVP (input UVLO) and OTP (over-temp) are exceptions: the buck
        // is healthy, just waiting on a sensor-side condition to clear.
        // Drop back to Pending and let the normal bring-up re-enable
        // when the cause is gone. A multi-hour outage would otherwise
        // require manual reboot. The XY7025 may also auto-re-enable
        // OUTPUT_EN itself once the cause clears (these are sensor-driven
        // protections, not true latches), so seeing On while waiting in
        // ProtectRecovery is the expected recovery — snap to Active
        // without latching. Setpoints are unchanged through the wait
        // (drift check above just verified them), so regulation is at
        // known targets either way.
        match (pending_reason, p.output) {
            (None, Some(BuckOutput::Off { cause: Lvp | Otp })) => {
                self.latch = LatchState::Pending {
                    reason: PendingReason::ProtectRecovery,
                };
                self.reset_phase_timers();
                return Action::None;
            }
            (None, Some(BuckOutput::Off { cause })) => {
                return self.latch(FaultReason::OutputUnexpectedlyOff(cause));
            }
            (Some(PendingReason::ProtectRecovery), Some(BuckOutput::On)) => {
                self.latch = LatchState::Active {
                    pending_voltage: None,
                };
                return Action::None;
            }
            // PendingReason::Boot + buck On: boot_sequence wrote
            // set_output(false) and verified OUTPUT_EN=0, so an ON
            // reading is a real anomaly (firmware bug, panel toggle,
            // EMI on the button GPIO) — latch.
            (Some(PendingReason::Boot), Some(BuckOutput::On)) => {
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
        if ov_debounced || (pending_reason.is_some() && ov) {
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
        if pending_reason.is_some() {
            if p.setpoints.is_none() {
                return Action::None;
            }
            // Don't try to enable while the buck is still holding on a
            // transient protection — set_output(true) would succeed at
            // the Modbus layer but the buck would stay off, and we'd
            // flap EnableOutput on every poll. Wait for LVP/OTP to clear.
            if matches!(p.output, Some(BuckOutput::Off { cause: Lvp | Otp })) {
                return Action::None;
            }
            // Output has been OFF through Pending, so `b.voltage` is the
            // pack's resting voltage — the true SoC signal. Below the CV
            // plateau ⇒ not full ⇒ caller acks with resume_absorb=true.
            let resume_absorb = !self.at_cv_plateau(b.voltage);
            return Action::EnableOutput { resume_absorb };
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
            // Defer the phase commit until the caller acks — keeps
            // `target_voltage` matching the buck's actual V_SET so a
            // failed write doesn't trigger SettingsDrift on the next
            // tick. Caller invokes `ack_voltage_update` on success.
            self.latch = LatchState::Active {
                pending_voltage: Some(next),
            };
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

    fn latch(&mut self, reason: FaultReason) -> Action {
        self.latch = LatchState::Tripped {
            reason,
            acked: false,
        };
        Action::DisableOutput(reason)
    }
}

// ─── Firmware-side V_SET sequencing ──────────────────────────────────────────

/// Minimal device interface used by [`apply_update_voltage`] — only the
/// two writes the safe step-down sequence needs. Lives here (not in
/// firmware) so the sequencing is host-testable with a mock.
pub trait VoltageWriter {
    fn set_voltage(&mut self, volts: f32) -> Result<(), BusError>;
    fn set_output(&mut self, on: bool) -> Result<(), BusError>;
}

/// Execute one `Action::UpdateVoltage`. For `cycle_output: false` (step-up)
/// writes V_SET live. For `cycle_output: true` (step-down) runs
/// `set_output(false)` → settle → `set_voltage` → `ack_voltage_update`
/// → `set_output(true)`. Partial failures attempt best-effort restore so
/// a single transient Modbus glitch doesn't drop the UPS load:
///
/// - Step-2 failure (`set_voltage` after disable): re-enable output so the
///   supervisor can retry the whole sequence next tick instead of latching.
/// - Step-3 failure (`set_output(true)` after V_SET commit): retried once
///   inline. If still failing, the supervisor latches on the next tick
///   — V_SET is already committed, so the failure mode is safe.
///
/// `settle` is the quiet window between disable and the V_SET write
/// (zero in tests). `on_error` is invoked once per Modbus error so the
/// firmware can drop it into its event log.
pub fn apply_update_voltage<W: VoltageWriter>(
    xy: &mut W,
    supervisor: &mut ChargeSupervisor,
    target_v: f32,
    cycle_output: bool,
    settle: Duration,
    mut on_error: impl FnMut(XyError),
) {
    if !cycle_output {
        match xy.set_voltage(target_v) {
            Ok(()) => {
                supervisor.ack_voltage_update();
                info!(
                    "charge phase → {}: V_set = {target_v:.2} V",
                    supervisor.phase().label()
                );
            }
            Err(e) => {
                warn!("XY set_voltage({target_v}): {e} — supervisor will retry next tick");
                on_error(XyError::SetVoltage);
            }
        }
        return;
    }

    info!("charge phase step-down → V_set = {target_v:.2} V (cycling output)");
    if let Err(e) = xy.set_output(false) {
        warn!("XY safe-step-down set_output(false): {e} — will retry next tick");
        on_error(XyError::SetOutput);
        return;
    }
    thread::sleep(settle);
    if let Err(e) = xy.set_voltage(target_v) {
        warn!("XY safe-step-down set_voltage({target_v}): {e} — attempting output restore");
        on_error(XyError::SetVoltage);
        // Best-effort restore so the UPS load stays powered. Supervisor
        // is still in Active+pending_voltage=Some(next) and will
        // re-emit UpdateVoltage on the next tick for another attempt.
        if let Err(e) = xy.set_output(true) {
            error!("XY safe-step-down restore set_output(true): {e} — buck OFF, will latch");
            on_error(XyError::SetOutput);
        }
        return;
    }
    // V_SET is on the buck; commit the phase so the next tick's drift
    // check sees matching setpoints.
    supervisor.ack_voltage_update();
    // Step 3: re-enable. Retry once inline to ride out a single
    // transient glitch; persistent failure latches on the next tick.
    let enable = xy.set_output(true).or_else(|e| {
        warn!("XY safe-step-down set_output(true) attempt 1: {e} — retrying");
        xy.set_output(true)
    });
    match enable {
        Ok(()) => info!(
            "charge phase → {}: V_set = {target_v:.2} V (step-down complete)",
            supervisor.phase().label()
        ),
        Err(e) => {
            error!("XY safe-step-down set_output(true): {e} — buck OFF after voltage commit");
            on_error(XyError::SetOutput);
        }
    }
}

#[cfg(test)]
mod tests;
