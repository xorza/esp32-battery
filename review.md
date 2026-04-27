# Code Review — 2026-04-27

Branch: `main` @ `6c68f65`

## High severity

### 1. `tick_and_persist` first-save delay
`src/main.rs:80-112`. After NTP sync the `(Some(t), None)` arm initializes
`last_save_s` to "now" and skips the save, so the first persist happens
`SAVE_INTERVAL_S` (10 min) after boot. A brownout in that window loses
in-RAM history that was supposed to be persisted at boot. Trigger an
immediate save on first sync (or initialize `last_save_s` so the first
sync triggers a save).

### 2. INA infinite retry without backoff
`src/ina.rs:163-189`. If `device.read()` errors permanently (bus stuck),
the inner `while count < SAMPLES_PER_UPDATE` loop spins forever calling
`recorder.record(...)` at 100 ms cadence. Effects: floods `EventLog`,
pegs CPU on a thread without WDT subscription, never publishes a stale
reading so `BatterySensorStale` (`logic/src/data/mod.rs:88-105`) never
fires. Bound the retry budget; on overflow break out so the supervisor's
staleness counter increments and the fault latches.

### 3. OTA upload: 4 KiB buffer on the HTTP server stack
`src/ota.rs:62`. `let mut buf = [0u8; 4096];` lives on the HTTP server
task stack alongside TLS state — stack-overflow footgun. Move to
`Box<[u8; 4096]>` (heap) or a static scratch.

## Medium severity

### 4. `PROTECTION_RECOVERY` trait is one-impl over-abstraction
`logic/src/charging/mod.rs:159-189`. Single impl on a foreign enum, used
in one match. Inline as `fn is_recoverable(s: ProtectionStatus) -> bool`.

### 5. Setpoint drift tolerance uses `>=`
`logic/src/charging/mod.rs:584-585`. Comment says "two-quantum slack
absorbs round-trip quirks" but `>=` latches at exact tolerance. Use `>`.

### 6. `History::compact_if_needed` drops earlier `time_s`
`logic/src/data/history.rs:147-153`. Averages voltages/currents but
takes `b.time_s` for the merged sample, so the visible series shifts
forward by `interval/2` per compaction. Use the midpoint.

### 7. `logic` framed as "pure host-testable" but uses `std`
`logic/src/charging/mod.rs`, NOTES-AI. Tests use `Vec`/`Box` and `std`.
Either declare `no_std` honestly or update the framing.

### 8. `power_online` can latch on capacitor sag
`logic/src/data/mod.rs:54,109-113`. With
`POWER_ONLINE_VOLTAGE_THRESHOLD = 2.0`, briefly-held `v_out` after buck
disable can report online. Add hysteresis or document.

## Low severity

- **`XyError::SetCurrent` is dead** — `logic/src/error_log.rs:41`. Remove (CLAUDE.md forbids dead code).
- **`#[allow(dead_code)]` on `XyDevice::clear_protection_status` / `set_power_on_default_off`** — `src/xy.rs:51`. Used in boot_sequence; drop the allow.
- **`sta_ip` `#[allow(dead_code)]`** — `src/wifi.rs:64`. Gate `#[cfg_attr(not(feature = "lcd"), allow(dead_code))]` so real dead code surfaces in `lcd` builds.
- **Missing `Debug` on `Chemistry`, `Profile`, `BatterySample`** — `logic/src/charging/mod.rs:102,116,322`. CLAUDE.md says derive Debug for panic/assert formatting.
- **Tests using `is_some()` / loose epsilons** — `logic/src/data/mod.rs:545,556,577`; `logic/src/data/history.rs:335` uses `< 0.01` for an integer-mean. Hand-compute exact values.
- **`serde_json_core::to_slice` Err silently dropped** — `src/api.rs:115`. Verify outer `mount_json_get` distinguishes Err vs partial buffer.
