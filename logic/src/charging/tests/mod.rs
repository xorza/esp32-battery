//! Charging supervisor tests. Each submodule covers one area of the
//! machine; the fixtures they all share live here.

use super::*;

use crate::battery::Chemistry;
use crate::charging::action::{Action, DisableTicket, EnableTicket, VoltageTicket};
use crate::charging::charge_state::ChargeState;
use crate::charging::charge_supervisor::{ChargeSupervisor, internals::SupervisorInternals};
use crate::charging::fault_reason::FaultReason;
use crate::charging::inhibit_reason::InhibitReason;
use crate::charging::pack_temp::PackTemp;
use crate::charging::phase::Phase;
use crate::charging::poll_result::{BatterySample, BuckOutput, PollResult};
use crate::charging::profile::{Profile, SupplyBudget};
use crate::charging::voltage_writer::{VoltageWriteOutcome, VoltageWriter, apply_update_voltage};
use crate::error_log::{ChargeTransition, XyError};
use xy_modbus::{ProtectionStatus, RtuError, Setpoints, XyError as BusError};

mod absorb_timeout;
mod bring_up;
mod faults;
mod pack_temp;
mod parked;
mod phase_machine;
mod profile;
mod protection_recovery;
mod sweep;
mod transition_log;
mod voltage_sequencing;

/// 4S 50 Ah LFP — the board's actual pack. With the module's C-rate
/// constants this gives reg = 10 A, enter = 3 A, exit = 2.5 A. Tests that
/// exercise threshold edges expect those numbers.
fn lfp_4s() -> Profile {
    Profile::for_pack(Chemistry::LiFePo4, 4, 50.0)
}

/// The board budget these tests assume: a 24 V rail and no load, so
/// `i_set_a` is the pack's own charge rate and every threshold figure the
/// tests quote still reads as written. `ChargeOvercurrent` is measured
/// against `regulation_a` either way, so it is unaffected by the choice.
const TEST_SUPPLY: SupplyBudget = SupplyBudget {
    input_nominal_v: 24.0,
    load_a: 0.0,
};

/// A fresh supervisor for `profile`, programmed the way the firmware
/// programs one.
fn supervisor(profile: Profile) -> ChargeSupervisor {
    ChargeSupervisor::new(
        profile,
        profile.buck_setup(TEST_SUPPLY).i_set_a,
        PackTemp::Absent,
    )
}

/// As `supervisor`, for a board that can see the pack's temperature. Only
/// the temperature tests want this; everything else mirrors the shipped
/// board, which has no sensor.
fn supervisor_with_temp_sensor(profile: Profile) -> ChargeSupervisor {
    ChargeSupervisor::new(
        profile,
        profile.buck_setup(TEST_SUPPLY).i_set_a,
        PackTemp::Fitted,
    )
}

fn approx(a: f32, b: f32) -> bool {
    (a - b).abs() < 1e-4
}

/// Assert two floats match, naming both in the failure. `assert!(approx(..))`
/// reports only "assertion failed", which says nothing about how far off a
/// derived value actually was. `#[track_caller]` keeps the reported line at
/// the call site.
#[track_caller]
fn assert_approx(actual: f32, expected: f32) {
    assert!(
        approx(actual, expected),
        "{actual} != {expected}"
    );
}

/// Sub-OV-threshold voltage used by tests that only care about phase logic.
/// Below the LFP fixture's CV plateau (14.4), so it does NOT arm the absorb
/// timeout — represents the CC ramp.
const OK_V: f32 = 13.5;

/// Pack voltage sitting at the LFP fixture's CV plateau (`absorb_v` = 14.4).
/// Arms the `MAX_ABSORB` clock; stays under the OV trip (14.6).
const CV_V: f32 = 14.4;

/// Resting voltage of a genuinely part-charged 4S LFP pack: 3.25 V/cell,
/// which the chemistry's OCV curve puts at exactly 40 % — far under
/// `RESUME_ABSORB_SOC`. Distinct from `OK_V`, which is `float_v` and reads
/// as *full* at rest (3.375 V/cell ⇒ 97.5 %). That two voltages 0.5 V apart
/// sit on opposite sides of "full" is the whole reason the resume gate asks
/// the OCV curve instead of measuring distance to the CV plateau.
const LOW_V: f32 = 13.0;

/// A pack temperature comfortably inside the charge window, so the fixtures
/// stay silent on temperature unless a test says otherwise. Ignored
/// entirely by the `PackTemp::Absent` supervisors most tests build.
const TEST_PACK_TEMP_C: f32 = 20.0;

/// Wall time elapsed per simulated tick. Tests choose 1 s so iteration
/// counts read as seconds when comparing against duration budgets.
const TICK: Duration = Duration::from_secs(1);

fn b(voltage: f32, current: f32) -> Option<BatterySample> {
    Some(BatterySample { voltage, current })
}

fn matches_disable(a: &Action, expected: FaultReason) -> bool {
    matches!(a, Action::DisableOutput(t) if t.reason() == expected)
}

/// Commit an `EnableOutput` the way the firmware does, and hand back the
/// `resume_absorb` the supervisor asked for. Panics on any other action,
/// so a test that expected bring-up fails where it stops being true.
fn accept_enable(s: &mut ChargeSupervisor, a: Action) -> bool {
    let Action::EnableOutput(ticket) = a else {
        panic!("expected EnableOutput, got {a:?}");
    };
    let resume_absorb = ticket.resume_absorb();
    s.commit_enable(ticket);
    resume_absorb
}

/// Drift-free PollResult matching the supervisor's expected state.
/// Sourcing → output ON; bringing up or latched → output OFF (no
/// protection cause). Tests that need to perturb one field use spread
/// syntax: `PollResult { output: Some(BuckOutput::On), ..expected_poll(&s, ...) }`.
fn expected_poll(s: &ChargeSupervisor, battery: Option<BatterySample>) -> PollResult {
    PollResult {
        setpoints: Some(s.readback_setpoints()),
        output: Some(expected_output(s)),
        battery,
        pack_temp_c: Some(TEST_PACK_TEMP_C),
    }
}

/// The `OUTPUT_EN` a healthy buck reports for the supervisor's current
/// state: on while sourcing, off otherwise.
fn expected_output(s: &ChargeSupervisor) -> BuckOutput {
    if s.state().sourcing() {
        BuckOutput::On
    } else {
        BuckOutput::Off {
            cause: ProtectionStatus::Normal,
        }
    }
}

/// Tick with a successful, drift-free Modbus readback where the buck
/// reports the output state the supervisor currently expects — the
/// common case. Phase transitions don't fire spurious `SettingsDrift`,
/// and Active ticks don't fire spurious `OutputUnexpectedlyOff`.
fn ok_tick(s: &mut ChargeSupervisor, battery: Option<BatterySample>, elapsed: Duration) -> Action {
    let a = s.tick(expected_poll(s, battery), elapsed);
    // Auto-commit voltage updates: this helper simulates the happy path
    // (every Modbus write succeeds), and a successful set_voltage write
    // is part of that. Tests of the retry-on-failure path use
    // `s.tick(...)` directly so they can drop the ticket and verify the
    // re-emit on the next tick.
    //
    // Committing consumes the ticket, so an equivalent one is handed back
    // for the caller to assert on. Only this module can mint one — that is
    // the whole point of the type outside the crate.
    if let Action::UpdateVoltage(ticket) = a {
        let echo = VoltageTicket {
            phase: ticket.phase,
            target_v: ticket.target_v,
            cycle_output: ticket.cycle_output,
        };
        s.commit_voltage(ticket);
        return Action::UpdateVoltage(echo);
    }
    a
}

/// Tick where the Modbus read failed — both `setpoints` and
/// `output_on` are None, exercising the modbus-unhealthy debounce path.
fn fail_tick(
    s: &mut ChargeSupervisor,
    battery: Option<BatterySample>,
    elapsed: Duration,
) -> Action {
    s.tick(
        PollResult {
            setpoints: None,
            output: None,
            battery,
            pack_temp_c: Some(TEST_PACK_TEMP_C),
        },
        elapsed,
    )
}

/// Build a supervisor and drive it from `Boot` into `Float`. Tests that
/// don't care about the bring-up dance use this; tests that exercise
/// bring-up call `ChargeSupervisor::new` directly.
fn active(profile: Profile) -> ChargeSupervisor {
    // Bring up at the CV plateau (`absorb_v`, still under the OV trip) so
    // the pack reads as full and the supervisor lands in Float — the
    // precondition these tests assume. A resting voltage below the plateau
    // would (correctly) resume Absorb; that path has its own tests.
    bring_up(supervisor(profile), profile.absorb_v)
}

/// Drive `s` from `Boot` into `Float`, energising at a resting `rest_v` that
/// must read as full — anything else resumes Absorb instead. Split out from
/// `active` so a supervisor built with a non-default `i_set_a` can reach the
/// same starting point.
fn bring_up(mut s: ChargeSupervisor, rest_v: f32) -> ChargeSupervisor {
    let a = ok_tick(&mut s, b(rest_v, -0.1), TICK);
    assert!(!accept_enable(&mut s, a), "full pack must not resume Absorb");
    assert_eq!(s.state(), ChargeState::Float);
    s
}

/// Drive the supervisor into Absorb. After this, exactly one Absorb tick
/// has elapsed (the transition itself).
fn enter_absorb(s: &mut ChargeSupervisor) {
    assert!(matches!(
        ok_tick(s, b(OK_V, -4.0), TICK),
        Action::UpdateVoltage { .. }
    ));
    assert_eq!(s.state(), ChargeState::Absorb);
}

/// Drain the buffered transitions the way the firmware's poll loop does.
/// Tests that only care about a later stretch drain first and discard.
fn drain_transitions(s: &mut ChargeSupervisor) -> Vec<ChargeTransition> {
    let mut out = Vec::new();
    while let Some(t) = s.pop_transition() {
        out.push(t);
    }
    out
}

/// Confirm the tick that raised `expected` parked on it rather than
/// latching: charging stopped, buck still up, load still fed.
///
/// Takes the action `ok_tick` returned, which has already committed the
/// write — so the supervisor has reached `Parked`, not `ToParked`.
fn accept_park(s: &ChargeSupervisor, a: Action, expected: FaultReason) {
    match a {
        // Parking out of absorb owes the same off→write→on step-down that
        // an ordinary Absorb→Float taper does.
        Action::UpdateVoltage(t) => {
            assert!(t.cycle_output, "a step down to float must cycle the output");
        }
        // Parking from a state already holding float has nothing to write.
        Action::None => {}
        other => panic!("expected a park, got {other:?}"),
    }
    assert_eq!(s.fault(), Some(expected));
    assert!(s.parked(), "charging must stop, but the load stays fed");
    assert_eq!(s.state(), ChargeState::Parked);
}

/// The buck drops its output on input UVLO — one turn of a sagging rail.
fn sag(s: &mut ChargeSupervisor) -> Action {
    let p = poll_with_output(
        s,
        BuckOutput::Off {
            cause: ProtectionStatus::Lvp,
        },
    );
    s.tick(p, TICK)
}

/// …and brings it back once the rail recovers unloaded.
fn recover(s: &mut ChargeSupervisor) -> Action {
    let p = poll_with_output(s, BuckOutput::On);
    s.tick(p, TICK)
}

/// Hold steady and healthy for `span`. One big-elapsed tick, which is all a
/// run of holds needs to age out of `FLAP_WINDOW`.
fn quiet(s: &mut ChargeSupervisor, span: Duration) {
    assert!(matches!(ok_tick(s, b(OK_V, -0.1), span), Action::None));
}

/// A poll where the buck reports `output` and everything else is drift-free.
/// Takes the output as an argument rather than deriving it like
/// `expected_poll` does, so it stays valid across the state changes these
/// tests drive — including into a latched state, which keeps no setpoint
/// expectation of its own.
fn poll_with_output(s: &ChargeSupervisor, output: BuckOutput) -> PollResult {
    PollResult {
        output: Some(output),
        setpoints: Some(s.readback_setpoints()),
        battery: b(OK_V, -0.1),
        pack_temp_c: Some(TEST_PACK_TEMP_C),
    }
}
