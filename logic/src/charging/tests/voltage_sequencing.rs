//! V_SET writes: the ticket retry protocol, and the safe step-down
//! sequence with every partial failure it has to survive.

use super::*;

/// Drive `s` to the Absorb→Float transition and hand back its ticket,
/// uncommitted, so the caller owns the apply step.
fn drive_to_absorb_to_float_pending(s: &mut ChargeSupervisor) -> VoltageTicket {
    enter_absorb(s);
    let tapered = b(CV_V, -(lfp_4s().exit_absorb_a - 0.1));
    let p = expected_poll(s, tapered);
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(s.tick(p, TICK), Action::None));
    }
    let Action::UpdateVoltage(ticket) = s.tick(p, TICK) else {
        panic!("expected Absorb→Float UpdateVoltage after EXIT_DEBOUNCE");
    };
    assert!(ticket.cycle_output, "Absorb→Float is a step DOWN");
    ticket
}

/// Drive `s` to the Float→Absorb transition and hand back its ticket.
fn drive_to_float_to_absorb_pending(s: &mut ChargeSupervisor) -> VoltageTicket {
    let p = expected_poll(s, b(OK_V, -4.0));
    let Action::UpdateVoltage(ticket) = s.tick(p, TICK) else {
        panic!("expected Float→Absorb UpdateVoltage");
    };
    assert!(!ticket.cycle_output, "Float→Absorb is a step UP");
    ticket
}

/// Run one `UpdateVoltage` exactly as the firmware does: execute the
/// writes, then commit the ticket only if V_SET actually landed. Returns
/// the errors the sequence reported. `settle` is a no-op here — the
/// quiet window is a real delay only on hardware.
fn apply_ticket(
    s: &mut ChargeSupervisor,
    xy: &mut MockWriter,
    ticket: VoltageTicket,
) -> Vec<XyError> {
    let mut errs = Vec::new();
    let outcome = apply_update_voltage(xy, &ticket, || {}, |e| errs.push(e));
    if outcome == VoltageWriteOutcome::Committed {
        s.commit_voltage(ticket);
    }
    errs
}

#[test]
fn update_voltage_retries_until_acked() {
    // Phase machine wants Float→Absorb (heavy charging current). The
    // first tick emits UpdateVoltage. If the caller doesn't ack (write
    // failed), the next tick must re-emit UpdateVoltage with the same
    // target — and the drift check must NOT latch SettingsDrift, since
    // V_SET on the buck is still the old (Float) value matching the
    // supervisor's still-Float `target_voltage`.
    let profile = lfp_4s();
    let mut s = active(profile);
    // expected_poll uses s.expected_setpoints(), which reflects the
    // *current* phase (Float) — exactly the still-on-the-buck values
    // the failed write would leave behind.
    let p = expected_poll(&s, b(OK_V, -4.0));

    let Action::UpdateVoltage(t1) = s.tick(p, TICK) else {
        panic!("expected UpdateVoltage");
    };
    assert_approx(t1.target_v, profile.absorb_v);
    assert_eq!(s.phase(), Phase::Float); // not yet committed
    assert_eq!(s.fault(), None);

    // No ack — second tick re-emits UpdateVoltage, same target. No
    // SettingsDrift even though setpoints (Float) lag the pending phase
    // (Absorb), because expected_setpoints still uses the old phase.
    let Action::UpdateVoltage(t2) = s.tick(p, TICK) else {
        panic!("expected UpdateVoltage retry");
    };
    assert_approx(t2.target_v, profile.absorb_v);
    assert_eq!(s.phase(), Phase::Float);
    assert_eq!(s.fault(), None);

    // Now commit — phase commits, debouncers reset, normal operation.
    s.commit_voltage(t2);
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn float_to_absorb_emits_step_up_no_output_cycle() {
    // V_SET goes up; safe to write live, no output cycling needed.
    // The caller can keep regulating through the transition.
    let profile = lfp_4s();
    let mut s = active(profile);
    let p = expected_poll(&s, b(OK_V, -4.0));
    let Action::UpdateVoltage(ticket) = s.tick(p, TICK) else {
        panic!("expected UpdateVoltage");
    };
    assert_approx(ticket.target_v, profile.absorb_v);
    assert!(
        !ticket.cycle_output,
        "Float→Absorb is a step UP — must not cycle"
    );
}

#[test]
fn absorb_to_float_emits_step_down_with_output_cycle() {
    // V_SET goes down. The caller MUST disable output around the write
    // — stepping V_SET below V_OUT with output enabled drives reverse
    // current through the buck's synchronous low-side FET (the battery
    // sources back in), which can blow the FET and propagate upstream.
    // The XY7025 has no anti-backup protection on either port.
    let profile = lfp_4s();
    let mut s = active(profile);
    enter_absorb(&mut s); // now in Absorb at absorb_v

    // Hold at CV plateau with tapered current long enough to trip the
    // exit debouncer. Drive the supervisor manually (not via ok_tick)
    // so we can inspect the transition tick before it auto-acks.
    let tapered = b(CV_V, -(profile.exit_absorb_a - 0.1));
    let p = expected_poll(&s, tapered);
    for _ in 0..(EXIT_DEBOUNCE.as_secs() - 1) {
        assert!(matches!(s.tick(p, TICK), Action::None));
    }
    let Action::UpdateVoltage(ticket) = s.tick(p, TICK) else {
        panic!("expected Absorb→Float UpdateVoltage after EXIT_DEBOUNCE");
    };
    assert_approx(ticket.target_v, profile.float_v);
    assert!(
        ticket.cycle_output,
        "Absorb→Float is a step DOWN — caller must cycle output"
    );
}

#[test]
#[should_panic]
fn commit_voltage_without_pending_phase_panics() {
    // A VoltageTicket only exists after an UpdateVoltage was emitted.
    // Committing one from steady-state Active would move `self.phase`
    // with nothing pending.
    let mut s = active(lfp_4s());
    s.commit_voltage(VoltageTicket {
        phase: Phase::Absorb,
        target_v: lfp_4s().absorb_v,
        cycle_output: false,
    });
}

/// Programmable mock for `VoltageWriter`. Records every call in order and
/// can be primed to fail at a specific call index per method, exercising
/// the partial-failure paths in `apply_update_voltage`.
#[derive(Default)]
struct MockWriter {
    set_output_calls: Vec<bool>,
    set_voltage_calls: Vec<f32>,
    fail_set_output_at: Vec<usize>,
    fail_set_voltage_at: Vec<usize>,
}

impl VoltageWriter for MockWriter {
    fn set_voltage(&mut self, volts: f32) -> Result<(), BusError> {
        let idx = self.set_voltage_calls.len();
        self.set_voltage_calls.push(volts);
        if self.fail_set_voltage_at.contains(&idx) {
            Err(BusError::Rtu(RtuError::Timeout))
        } else {
            Ok(())
        }
    }
    fn set_output(&mut self, on: bool) -> Result<(), BusError> {
        let idx = self.set_output_calls.len();
        self.set_output_calls.push(on);
        if self.fail_set_output_at.contains(&idx) {
            Err(BusError::Rtu(RtuError::Timeout))
        } else {
            Ok(())
        }
    }
}

/// Drive `s` from Active+Absorb to Active+pending_voltage=Some(Float) by
#[test]
fn apply_step_up_happy_path() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_float_to_absorb_pending(&mut s);
    let mut xy = MockWriter::default();
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().absorb_v]);
    assert!(
        xy.set_output_calls.is_empty(),
        "step-up must not touch output"
    );
    assert!(errs.is_empty());
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn apply_step_down_happy_path() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_absorb_to_float_pending(&mut s);
    let mut xy = MockWriter::default();
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert_eq!(
        xy.set_output_calls,
        vec![false, true],
        "must disable then re-enable around the write"
    );
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().float_v]);
    assert!(errs.is_empty());
    assert_eq!(s.phase(), Phase::Float);
}

#[test]
fn apply_step_down_step1_failure_does_not_write_voltage() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_absorb_to_float_pending(&mut s);
    let mut xy = MockWriter {
        fail_set_output_at: vec![0],
        ..Default::default()
    };
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert_eq!(xy.set_output_calls, vec![false]);
    assert!(xy.set_voltage_calls.is_empty());
    assert_eq!(errs, vec![XyError::SetOutput]);
    // Outcome was Retry, so the ticket went uncommitted.
    assert_eq!(s.phase(), Phase::Absorb);
    // Supervisor re-emits UpdateVoltage on next tick for retry.
    let p = expected_poll(&s, b(CV_V, -(lfp_4s().exit_absorb_a - 0.1)));
    assert!(
        matches!(s.tick(p, TICK), Action::UpdateVoltage(ref t) if t.cycle_output)
    );
}

#[test]
fn apply_step_down_step2_failure_restores_output() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_absorb_to_float_pending(&mut s);
    let mut xy = MockWriter {
        fail_set_voltage_at: vec![0],
        ..Default::default()
    };
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert_eq!(
        xy.set_output_calls,
        vec![false, true],
        "must restore output after set_voltage failure"
    );
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().float_v]);
    assert_eq!(errs, vec![XyError::SetVoltage]);
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn apply_step_down_step2_then_restore_both_fail_records_both() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_absorb_to_float_pending(&mut s);
    let mut xy = MockWriter {
        fail_set_voltage_at: vec![0],
        // call 0 = initial disable (success), call 1 = restore (fail).
        fail_set_output_at: vec![1],
        ..Default::default()
    };
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert_eq!(xy.set_output_calls, vec![false, true]);
    assert_eq!(errs, vec![XyError::SetVoltage, XyError::SetOutput]);
    assert_eq!(s.phase(), Phase::Absorb);
}

#[test]
fn apply_step_down_step3_failure_retries_once_then_records() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_absorb_to_float_pending(&mut s);
    let mut xy = MockWriter {
        // call 0 = initial disable (ok), 1 = re-enable attempt 1 (fail),
        // 2 = re-enable attempt 2 (fail).
        fail_set_output_at: vec![1, 2],
        ..Default::default()
    };
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert_eq!(xy.set_output_calls, vec![false, true, true]);
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().float_v]);
    assert_eq!(errs, vec![XyError::SetOutput]);
    // V_SET landed, so the outcome is Committed and the phase moves even
    // though the buck is now dark — the next tick latches for that.
    assert_eq!(s.phase(), Phase::Float);
}

#[test]
fn apply_step_down_step3_first_attempt_recovers_on_retry() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_absorb_to_float_pending(&mut s);
    let mut xy = MockWriter {
        // Re-enable attempt 1 fails, attempt 2 succeeds — no error recorded.
        fail_set_output_at: vec![1],
        ..Default::default()
    };
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert_eq!(xy.set_output_calls, vec![false, true, true]);
    assert!(errs.is_empty(), "transient single failure must not record");
    assert_eq!(s.phase(), Phase::Float);
}

#[test]
fn apply_step_up_failure_does_not_touch_output() {
    let mut s = active(lfp_4s());
    let ticket = drive_to_float_to_absorb_pending(&mut s);
    let mut xy = MockWriter {
        fail_set_voltage_at: vec![0],
        ..Default::default()
    };
    let errs = apply_ticket(&mut s, &mut xy, ticket);
    assert!(
        xy.set_output_calls.is_empty(),
        "step-up failure must NOT cycle output"
    );
    assert_eq!(xy.set_voltage_calls, vec![lfp_4s().absorb_v]);
    assert_eq!(errs, vec![XyError::SetVoltage]);
    assert_eq!(s.phase(), Phase::Float);
}
