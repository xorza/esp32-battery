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
//! A fault latches only while the buck is actually sourcing. The same
//! conditions detected during `Pending` bring-up *inhibit* instead: the
//! output is already off, so latching would disable nothing while still
//! costing a reboot to clear. Inhibits are reported via `inhibit()` and
//! clear on their own when the condition does.
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
pub(crate) mod charge_supervisor;
pub(crate) mod debounce;
pub(crate) mod fault_reason;
pub(crate) mod inhibit_reason;
pub(crate) mod phase;
pub(crate) mod poll_result;
pub(crate) mod profile;
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
const SETPOINT_DRIFT_TOL: f32 = 0.02;

#[cfg(test)]
mod tests;
