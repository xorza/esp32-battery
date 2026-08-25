//! Charging supervisor tests. Each submodule covers one area of the
//! machine; the fixtures they all share live here.

use super::*;

use crate::battery::Chemistry;
use crate::charging::action::{Action, DisableTicket, EnableTicket, VoltageTicket};
use crate::charging::charge_state::ChargeState;
use crate::charging::charge_supervisor::{ChargeSupervisor, internals::SupervisorInternals};
use crate::charging::fault_reason::FaultReason;
use crate::charging::inhibit_reason::InhibitReason;
use crate::charging::phase::Phase;
use crate::charging::poll_result::{BatterySample, BuckOutput, PollResult};
use crate::charging::profile::Profile;
use crate::charging::voltage_writer::{VoltageWriteOutcome, VoltageWriter, apply_update_voltage};
use crate::error_log::{ChargeTransition, XyError};
use xy_modbus::{ProtectionStatus, RtuError, Setpoints, XyError as BusError};

mod absorb_timeout;
mod bring_up;
mod faults;
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
    let bring_up_v = profile.absorb_v;
    let mut s = ChargeSupervisor::new(profile);
    let a = ok_tick(&mut s, b(bring_up_v, -0.1), TICK);
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
    }
}
