# Open issues

- `ChargeState::Latched` returns `Action::None` on every subsequent tick. If
  the buck's output comes back on after a latch (front-panel toggle, a
  device-side re-enable), the supervisor never notices and never re-disables.

- `MAX_ABSORB` is clocked with `Debounce::step`, so any tick where the pack
  leaves the `ABSORB_CV_BAND_V` window zeroes the accumulator. A load that
  periodically pulls the buck out of CV keeps the absorb cap from firing.
  There is no cap on total time in Absorb, and none at all on the CC ramp.

- Protection holds are unbounded: `Verdict::SelfDisabled` and
  `Verdict::SelfEnabled` can alternate indefinitely. An input rail that
  sags under charge current gives a repeating LVP hold/resume loop with no
  flap limit and no back-off on `regulation_a`.

- `at_cv_plateau(battery.voltage)` decides `resume_absorb` at bring-up from the
  pack's resting voltage, but a full LFP pack rests near `float_v`, so the test
  is false for every real bring-up. Every reboot forces an Absorb cycle, and
  the Absorb→Float step-down that ends it cycles the buck output under load.

- `ocp_a` is derived as `regulation_a * 1.5`, but the buck's output current is
  charge current plus the UPS load, and the load does not appear in the
  derivation. A load surge trips device OCP, which latches
  `OutputUnexpectedlyOff` and drops the load onto the pack until a reboot.

- `Verdict::SelfEnabled` is returned from check 2 of `safety_verdict`,
  ahead of the modbus-health, battery-freshness and overvoltage checks, so the
  supervisor resumes sourcing for one tick without those having run.

- No pack temperature reaches the supervisor. Charging below 0 °C is not
  inhibited for any chemistry. The buck's OTP covers its own die only.

- Nothing bounds pack discharge. The buck's LVP register is input-side UVLO,
  so after a latch or during a protection hold the pack supplies the load with
  no firmware-side cutoff.
