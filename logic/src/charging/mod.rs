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
    /// (the const-block below enforces this at compile time). OCP is 50%
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

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Phase {
    Float,
    Absorb,
}

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
/// How long battery readings can stay absent before we fail closed.
const BATTERY_MISSING_TIMEOUT: Duration = Duration::from_secs(10);
/// How long Modbus reads to the XY can keep failing before we fail closed.
const MODBUS_UNHEALTHY_TIMEOUT: Duration = Duration::from_secs(5);

/// Why the supervisor latched the buck off. Once latched, only a reboot
/// clears it — auto-recovery on a battery charger means trying again
/// under the same conditions.
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
}

/// What the poll loop should do this tick. The supervisor never enables the
/// output — `boot_sequence` does that once at startup. After a latch, only
/// `DisableOutput` is ever emitted until the disable is ACKed.
pub enum Action {
    None,
    SetVoltage(f32),
    DisableOutput(FaultReason),
}

/// Latest fresh battery reading fed to the supervisor. Voltage is used for
/// OV detection, current drives the phase machine. Power isn't needed.
#[derive(Copy, Clone)]
pub struct BatterySample {
    pub voltage: f32,
    pub current: f32,
}

/// Latch state. `Tripped { acked: false }` is the only state that emits
/// `DisableOutput`; the unreachable `(None, true)` of the old two-field
/// encoding can't be expressed.
enum LatchState {
    Active,
    Tripped { reason: FaultReason, acked: bool },
}

pub struct ChargeSupervisor {
    profile: Profile,
    phase: Phase,
    ov_elapsed: Duration,
    absorb_elapsed: Duration,
    battery_missing_elapsed: Duration,
    modbus_err_elapsed: Duration,
    latch: LatchState,
}

impl ChargeSupervisor {
    pub fn new(profile: Profile) -> Self {
        assert!(profile.absorb_v > profile.float_v);
        // Always boot in Float — never resume Absorb across a reset, even if
        // we crashed mid-absorb. Conservative by design: re-derive phase from
        // observed current. Don't add NVS-backed phase persistence.
        Self {
            profile,
            phase: Phase::Float,
            ov_elapsed: Duration::ZERO,
            absorb_elapsed: Duration::ZERO,
            battery_missing_elapsed: Duration::ZERO,
            modbus_err_elapsed: Duration::ZERO,
            latch: LatchState::Active,
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
            LatchState::Active => None,
            LatchState::Tripped { reason, .. } => Some(reason),
        }
    }

    /// Caller invokes this after a successful `set_output(false)` Modbus write.
    /// Until then, the supervisor will keep emitting `DisableOutput` so a
    /// failed disable write gets retried on every tick.
    pub fn ack_disable(&mut self) {
        match &mut self.latch {
            LatchState::Tripped { acked, .. } => *acked = true,
            LatchState::Active => panic!("ack_disable without latched fault"),
        }
    }

    /// Drive one poll cycle. `modbus_ok` reflects the most recent read attempt
    /// against the XY7025. `battery` is the latest fresh reading (`None` if
    /// stale or absent). `elapsed` is wall time since the previous tick.
    /// Returns the action the caller should take.
    ///
    /// Non-finite battery inputs are charitable: voltage NaN doesn't count
    /// toward OV; current NaN holds the current phase.
    pub fn tick(
        &mut self,
        modbus_ok: bool,
        battery: Option<BatterySample>,
        elapsed: Duration,
    ) -> Action {
        match self.latch {
            LatchState::Tripped { acked: true, .. } => return Action::None,
            LatchState::Tripped {
                reason,
                acked: false,
            } => return Action::DisableOutput(reason),
            LatchState::Active => {}
        }

        if modbus_ok {
            self.modbus_err_elapsed = Duration::ZERO;
        } else {
            self.modbus_err_elapsed = self.modbus_err_elapsed.saturating_add(elapsed);
            if self.modbus_err_elapsed >= MODBUS_UNHEALTHY_TIMEOUT {
                return self.latch(FaultReason::ModbusUnhealthy);
            }
        }

        let Some(b) = battery else {
            self.battery_missing_elapsed = self.battery_missing_elapsed.saturating_add(elapsed);
            if self.battery_missing_elapsed >= BATTERY_MISSING_TIMEOUT {
                return self.latch(FaultReason::BatterySensorStale);
            }
            return Action::None;
        };
        self.battery_missing_elapsed = Duration::ZERO;

        if b.voltage.is_finite() && b.voltage > self.profile.absorb_v + OV_MARGIN_V {
            self.ov_elapsed = self.ov_elapsed.saturating_add(elapsed);
            if self.ov_elapsed >= OV_DURATION {
                return self.latch(FaultReason::Overvoltage);
            }
        } else {
            self.ov_elapsed = Duration::ZERO;
        }

        if !b.current.is_finite() {
            return Action::None;
        }
        // Charging current as a positive number.
        let charging_a = -b.current;
        let next = match self.phase {
            Phase::Float if charging_a > self.profile.enter_absorb_a => Phase::Absorb,
            Phase::Absorb if charging_a < self.profile.exit_absorb_a => Phase::Float,
            p => p,
        };
        if next != self.phase {
            self.phase = next;
            self.absorb_elapsed = Duration::ZERO;
            return Action::SetVoltage(self.target_voltage());
        }

        if self.phase == Phase::Absorb {
            self.absorb_elapsed = self.absorb_elapsed.saturating_add(elapsed);
            if self.absorb_elapsed >= MAX_ABSORB {
                return self.latch(FaultReason::AbsorbTimeout);
            }
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
