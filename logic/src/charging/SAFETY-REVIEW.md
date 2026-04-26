# Charging safety review

Review of `logic/src/charging/mod.rs` (ChargeSupervisor) and `src/xy.rs`
(XY7025 driver + thread loop). Goal: confirm the battery is never charged
at the wrong voltage or current, and that all fault cases fail closed.

The design is genuinely defense-in-depth — debounced supervisor faults,
hardware OVP/OCP/LVP as backstops, drift detection, fail-closed Modbus
timeout, NaN filtering, boot verification, S_INI=OFF for crash safety.
Findings below are ordered by severity.

## Critical / safety-affecting

### 1. No watchdog if the supervisor thread dies

`run()` in `xy.rs` is `loop { … }` with `sensor_data.lock().unwrap()` and
other panic paths. If the xy thread panics (poisoned lock, OOM, anything),
the buck **keeps running indefinitely at its last setpoint**. The XY7025
doesn't have a host-deadman feature, so the only backstops are its own
hardware OVP/OCP/LVP.

Mitigations to consider:
- Wrap the thread body in `catch_unwind` and force `set_output(false)` on
  panic before re-spawning.
- Use the ESP IDF task watchdog and feed it each tick — if the loop hangs,
  reboot brings the buck up with S_INI=OFF.
- Periodic write-and-readback "heartbeat" so even a missed schedule slot
  eventually fails closed via SettingsDrift on resume.

### 2. `OutputUnexpectedlyOff` auto-recovery doesn't re-run boot_sequence

`try_recover` (mod.rs:577) flips `Tripped → Pending` after 60 s healthy,
but Pending only re-emits `EnableOutput`. It never re-verifies that the
XY's OVP/OCP/LVP and V_SET/I_SET registers are still what we programmed at
boot. Those registers persist in EEPROM, but a partial brown-out / ESD
event that scrambles them is exactly the kind of thing that *would* cause
the original `OutputUnexpectedlyOff`. Re-enabling without re-verification
means the next charge could be at the wrong voltage.

Recommend: on recovery, re-run `boot_sequence` (or at least
`read_protection` + `read_status` verification) before letting the
supervisor enable output.

### 3. Recovery health check doesn't account for OCP / over-temp causes

The "healthy" predicate (mod.rs:584) is voltage-only. If the buck
self-disabled from OCP (sticky FET, momentary short downstream) or
over-temp, looking at v_pack 60 s later under no load tells us nothing
about whether the underlying condition cleared. The 3-attempt cap bounds
blast radius, but each attempt does mean re-energizing into a possibly
shorted load. If the XY exposes a last-fault flag, gating recovery on it
would be a real improvement.

## Moderate

### 4. Failed `UpdateVoltage` write latches instead of retrying

`apply_action` for `UpdateVoltage` (xy.rs:485) just logs on error. Next
tick, readback shows old V_SET, supervisor's expected is new V_SET →
`SettingsDrift` → permanent latch requiring reboot. The fail-closed
direction is correct (Absorb→Float fail = buck stuck high, drift catches),
but the operational cost is high for a transient Modbus glitch on a single
write. Consider mirroring `EnableOutput`'s pattern: stash a
"voltage-not-yet-acked" flag and retry on the next tick before drift
fires.

### 5. Boot fall-through window

If `boot_sequence` fails all 10 retries (xy.rs:394) and the eager
`set_output(false)` *also* fails, the supervisor doesn't latch
ModbusUnhealthy until ~5 s later. During that window, if S_INI in EEPROM
happened to be ON (e.g., never provisioned, or a stale value), the buck
could be sourcing at whatever V_SET / I_SET / OVP it had. S_INI=OFF is
supposed to be set inside boot_sequence — meaning a freshly-shipped
board's *first* boot is the one risk.

Either (a) panic / reboot instead of falling through when boot fails, or
(b) document that the board must be hand-provisioned with S_INI=0 before
first deploy.

### 6. `expected_setpoints()` assumes I_SET never changes

True today (regulation_a is constant), so this is safe — but the drift
check at mod.rs:480 silently depends on it. Worth a short comment noting
the invariant. If anyone ever adds a CC-tapering feature, the drift check
needs updating in lockstep.

## Minor

### 7. Logging bug at xy.rs:403

`error!("XY post-boot-fail set_output(false) succeeded")` — should be
`warn!` or `info!`. Cosmetic but noisy.

### 8. NaN handling in `try_recover`

Uses `b.voltage <= absorb_v + OV_MARGIN_V`. NaN comparisons return false,
so the `unwrap_or(false)` already covers it correctly. Just calling out —
it's a subtle correctness hinge worth a short comment.

### 9. `read_protection` is performed at boot but never again

If the XY's protection registers somehow drift mid-run (EEPROM corruption,
undocumented external write), the supervisor never notices. Periodic
re-verification (every N ticks) would close the loop. Low likelihood, low
cost — judgment call.

## Things checked and confirmed correct

- Sign convention (`charging_a = -b.current`) — wrong-polarity sensor
  leaves supervisor in Float, which is safe long-term.
- `HARDWARE_OVP_MARGIN_V > OV_MARGIN_V` enforced at compile time →
  supervisor's debounced trip always fires before hardware OVP.
- OV check is **undebounced** in Pending (mod.rs:522) — a pack already
  over threshold at boot can never see EnableOutput.
- NaN/Inf battery samples routed through `BatterySensorStale` debounce,
  not silently ignored.
- `set_power_on_default_off()` ensures crash + power-cycle leaves output
  OFF (only on already-provisioned boards — see #5).
- `Profile::for_pack` const-asserts cells>0 and capacity_ah>0;
  `ChargeSupervisor::new` asserts absorb_v > float_v.
- Float→Absorb only on observed charging current, never on time/voltage
  alone — won't push absorb voltage on a nearly-full pack.
- All Modbus failures route through one of: ModbusUnhealthy debounce,
  BatterySensorStale debounce, SettingsDrift (immediate), or — for write
  failures — supervisor retry on next tick.

## Top priority

The single highest-leverage thing to fix is **#1 (thread death =
unsupervised buck)**. Everything else is incremental.
