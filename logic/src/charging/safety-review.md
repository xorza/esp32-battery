# Charging Path Safety Review

Findings from a review of `src/xy.rs` + `logic/src/charging/` looking for
states where the supervisor could leave the buck in an incorrect or
unsafe configuration.

Date: 2026-04-27. Re-run when the boot sequence, fault paths, or
debounce timeouts change materially.

---

## High

### NaN battery samples bypass missing-battery and OV checks — FIXED

**Status: fixed.** `tick` now coerces `Some(BatterySample {..})` with
non-finite voltage *or* current to `None` at intake, routing through
the existing `BatterySensorStale` debounce (10 s).

Original issue: `battery_missing.step(p.battery.is_none(), ...)` treated
NaN/Inf samples as present, and the inner `is_finite()` guards on the
OV and phase paths silently ignored them. A sensor stuck on NaN would
leave the buck regulating to its last commanded V_SET indefinitely.

---

## Medium

### Cold boot enables output without checking pack voltage

`src/xy.rs:342` — `boot_sequence` programs setpoints, verifies via
readback, then calls `set_output(true)`. The pack voltage is never
read before energizing.

**Effect**: if the pack is already above the absorb target at boot (hot
pack from a prior interrupted charge, sensor offset, after-fault
restart), we energize and rely entirely on the hardware OVP register
(15.0 V for 4S LFP). The supervisor's debounced OV trip starts only
after `boot_sequence` returns and the loop spins.

**Fix**: take one battery sample before `set_output(true)`. Refuse to
enable if `b.voltage > absorb_v + OV_MARGIN_V` (or `!is_finite()`).
Costs at most one POLL_INTERVAL of latency.

### Boot-failure path discards the disable write

`src/xy.rs:389` — after `BOOT_RETRY_COUNT` failed boot attempts:

```rust
let _ = xy.set_output(false);
return;
```

If `boot_sequence` failed because the buck is unreachable, this final
disable likely fails too — and the result is discarded. The thread then
exits, leaving no further supervision.

**Effect**: if the buck's `S-INI` register is 1 (factory default, or
written by a previous firmware), the buck sources into the pack with no
firmware-side cutoff. Only hardware OVP/OCP/LVP protect — and those
registers may carry stale values from a prior firmware that used a
different chemistry/cell-count profile.

**Fix options**:
- Log the disable result (cheap, observable).
- Replace `return;` with a degraded loop that retries `set_output(false)`
  every few seconds — keeps trying to fail-closed instead of giving up.
- Document explicitly that the only safety net here is the hardware
  registers + S-INI=0 from a *previous* successful boot.

---

## Low

### Phase-transition write failure relies on next-tick drift catch

`src/xy.rs:451` — `apply_action` for `Action::SetVoltage(v)` logs and
continues if `set_voltage` returns Err. The next tick's drift check
sees V_SET still at the old value (e.g., float_v) while the supervisor
expects absorb_v, and latches `SettingsDrift`.

**Effect**: one extra tick (1 s) at the wrong setpoint before the
supervisor disables. Probably fine — buck is briefly held at float_v
when supervisor wants absorb_v, never the other way around. Flagging
for awareness; immediate retry would be tighter but adds complexity.

### SETPOINT_DRIFT_TOL is 0.02 (two register quanta)

`logic/src/charging/mod.rs:132` — drift check tolerates ±0.02 V/A.
Single-bit corruption in V_SET (e.g., 1440 → 1441 = 14.41 V) doesn't
trigger drift. Trade-off is against IEEE-float round-trip noise on
values like 14.4 V whose binary repr isn't exact. Acceptable; logged so
it's intentional, not accidental.

### sensor_data Mutex held across Modbus I/O

`src/xy.rs:411` — `poll` holds `sensor_data.lock()` for the duration of
`read_status()` (~500 ms response timeout on failure). HTTP handlers
reading sensor_data block during that window. Responsiveness, not
safety.

---

## What's right

- Drift check runs *before* modbus-unhealthy debounce, so a successful
  read with bad values latches faster than transport-noise debounce.
- Latch is sticky until `ack_disable()`, which is only called after a
  successful `set_output(false)`. Failed disable writes are retried on
  every subsequent tick.
- Supervisor always boots in `Phase::Float` — no NVS-backed phase
  resume, even after a crash mid-Absorb.
- `set_output(false)` is retried indefinitely when the disable write
  fails (no give-up budget on disable, unlike boot).
- Hardware OVP/OCP/LVP are programmed and verified at boot; mismatches
  bail before output is enabled.
- First-fault-wins ordering matches the latched-state path: subsequent
  faults can't overwrite the existing latch.
- `Debounce::step` resets on `cond = false` — a single bad tick doesn't
  accumulate against the timeout.
- Compile-time invariants: `REGULATION_C > ENTER_ABSORB_C >
  EXIT_ABSORB_C`, `HARDWARE_OVP_MARGIN_V > OV_MARGIN_V`, and a
  runtime `assert!(absorb_v > float_v)` in `ChargeSupervisor::new`.
