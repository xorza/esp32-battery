//! Two-phase CV charging strategy with hysteresis + supervisor latch.
//!
//! Sits in Float (low CV) by default. When the battery draws more than
//! `enter_absorb_a` of charging current, switches to Absorb (high CV) to
//! finish the pack. Once current tapers below `exit_absorb_a`, drops back to
//! Float. Profiles are per-chemistry constants.
//!
//! Wraps the phase logic with the fault machinery: overvoltage, a stuck
//! absorb, a missing battery, an unhealthy Modbus link and the rest all
//! stop the charge, and only a reboot resumes it. What they do to the
//! output splits two ways — see `FaultReason::response`. Losing control of
//! the buck takes it down; an overcharge with control intact only drops it
//! to the float target, because on a UPS a dark output means the pack
//! starts carrying the load.
//!
//! Both live in one flat state machine — see `charge_state.rs`, whose
//! `TRANSITIONS` table is the whole of it: which V_SET the device holds,
//! whether the buck is meant to be sourcing, and what each event does to
//! either are all functions of a single payload-free enum.
//!
//! A fault latches only while the buck is actually sourcing. The same
//! conditions detected in a bring-up state *inhibit* instead: the output
//! is already off, so latching would disable nothing while still costing
//! a reboot to clear. Inhibits are reported via `inhibit()` and clear on
//! their own when the condition does.
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

use std::time::Duration;

use crate::data::STALE_WINDOW;

pub(crate) mod action;
pub(crate) mod charge_state;
pub(crate) mod charge_supervisor;
pub(crate) mod debounce;
pub(crate) mod fault_reason;
pub(crate) mod hold_budget;
pub(crate) mod inhibit_reason;
pub(crate) mod pack_temp;
pub(crate) mod phase;
pub(crate) mod poll_result;
pub(crate) mod profile;
pub(crate) mod protection_policy;
pub(crate) mod voltage_writer;

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
///
/// Applied as a *leaky* window (`Debounce::step_leaky`), for the same reason
/// `EXIT_DEBOUNCE` is: a UPS load that periodically pulls the buck out of CV
/// would, under a hard reset, erase the whole accumulation on every dip and
/// keep the cap from ever firing. Draining costs a dip exactly the time it
/// lasted, while a genuine sustained return to CC still empties the window
/// and blocks the trip.
const MAX_ABSORB: Duration = Duration::from_secs(2 * 60 * 60);
/// Absolute cap on one charge cycle, counted from entering Absorb and cleared
/// by any state change.
///
/// `MAX_ABSORB` clocks the CV plateau only, and that is right — the CC ramp
/// from a deeply discharged pack legitimately runs for hours. But it leaves a
/// pack that never *reaches* the plateau with no cap at all: a shorted cell,
/// a wiring fault, or a load eating the whole charge current would charge
/// forever. From empty at `REGULATION_C` a healthy pack needs ~5 h of CC plus
/// ~1 h of CV, so 8 h bounds the pathological case with generous headroom
/// over the legitimate one.
const MAX_CHARGE: Duration = Duration::from_secs(8 * 60 * 60);
/// The CV plateau is a subset of the cycle, so its cap has to be the tighter
/// of the two or it could never be the one to fire.
const _: () = assert!(MAX_CHARGE.as_secs() > MAX_ABSORB.as_secs());
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
///
/// This is the last of three windows a dead INA228 has to cross, and the
/// only place their total is written down. In order: the INA thread
/// publishes one averaged reading per `SAMPLES_PER_UPDATE × SAMPLE_INTERVAL`
/// (1 s, in `src/ina.rs`); `data::STALE_WINDOW` (5 s) then has to expire
/// before `battery_reading()` starts returning `None`; and only then does
/// this window start. Worst case from the last good conversion to a latched
/// buck-off is their sum, 16 s — all three charged in wall time, so the
/// figure holds even when a loop runs slow.
const BATTERY_MISSING_TIMEOUT: Duration = Duration::from_secs(10);
/// Pins the two windows this crate owns. The INA publish period is the
/// firmware's, so 1 s of the documented 16 s is out of reach from here.
const _: () = assert!(
    STALE_WINDOW.as_secs() + BATTERY_MISSING_TIMEOUT.as_secs() == 15,
    "sensor-loss budget changed — re-derive the total documented on \
     BATTERY_MISSING_TIMEOUT"
);
/// How far over the profile's charge rate the pack may actually draw before
/// the supervisor calls it an overcurrent.
///
/// The buck's CC loop bounds *total* output current, which on a UPS is the
/// charge current plus the load — so I_SET is sized for both and cannot by
/// itself hold the pack to `REGULATION_C`. With an idle load the buck will
/// happily put the whole setpoint into the pack. Only the INA228 sees what
/// the pack is actually taking, which makes this the one thing that
/// enforces the pack's own rate.
///
/// 1.25 sits clear of INA noise and CC-loop overshoot, and well under the
/// 0.5C manufacturer maximum that `REGULATION_C` is already conservative
/// against.
const OVERCURRENT_TOL: f32 = 1.25;
/// How long charging current must hold over the tolerance before tripping.
/// Debounced because the buck pulses near a full pack and a load stepping
/// off is a genuine transient, not a fault.
const OVERCURRENT_DURATION: Duration = Duration::from_secs(5);

/// How long the supervisor must go without a new self-clearing hold before
/// it forgets the ones before it. Five minutes is far longer than the
/// second-scale loop a sagging rail produces, and far shorter than the gap
/// between two unrelated supply events.
const FLAP_WINDOW: Duration = Duration::from_secs(5 * 60);
/// Self-clearing holds tolerated inside one run before the supervisor stops
/// waiting them out. The next one latches `ProtectionFlapping`.
///
/// Four leaves room for a supply that hiccups on a cold start or a
/// compressor kicking in nearby, while still ending a genuine flap inside a
/// handful of seconds rather than never.
const MAX_HOLDS: u8 = 4;

/// Ambient range within which the pack may be charged, in °C.
///
/// Below freezing, charging lithium plates metal onto the anode instead of
/// intercalating it: capacity is lost permanently, the plating is
/// invisible from outside, and enough of it shorts the cell. Discharging
/// cold is fine, which is why this bounds charging only. The ceiling is the
/// usual cell-spec figure, above which ageing accelerates sharply.
///
/// One pair rather than a per-chemistry curve because every chemistry this
/// crate supports shares it — the plating mechanism is not specific to LFP
/// or NMC. If one ever needs its own, this moves onto `Chemistry` beside
/// `charge_voltages`.
const CHARGE_TEMP_MIN_C: f32 = 0.0;
const CHARGE_TEMP_MAX_C: f32 = 45.0;
const _: () = assert!(CHARGE_TEMP_MIN_C < CHARGE_TEMP_MAX_C);
/// How long a fitted pack-temperature sensor may go unread before the
/// supervisor fails closed. Mirrors `BATTERY_MISSING_TIMEOUT`: the same
/// argument applies, since neither reading can be substituted for.
const PACK_TEMP_STALE_TIMEOUT: Duration = Duration::from_secs(10);

/// How long Modbus reads to the XY can keep failing before we fail closed.
const MODBUS_UNHEALTHY_TIMEOUT: Duration = Duration::from_secs(5);

/// How many latch transitions the supervisor buffers between drains.
/// Transitions are rare — a healthy unit produces one at bring-up and
/// nothing else for hours — and the caller drains every tick, so this
/// only has to cover a caller that stops draining briefly.
const TRANSITION_BUFFER: usize = 8;

/// Allowed drift between commanded and observed setpoint. One register
/// quantum is 0.01; two-quantum slack absorbs IEEE-float round-trip
/// quirks on values like 14.4 V whose binary repr isn't exact.
///
/// Public because the firmware's boot-time readback verification is the same
/// commanded-vs-reported comparison, made once instead of every tick, and
/// must not disagree about how close counts as equal.
pub const SETPOINT_DRIFT_TOL: f32 = 0.02;

#[cfg(test)]
mod tests;
