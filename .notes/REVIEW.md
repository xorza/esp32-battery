# Crate review

Findings only — no fixes, no designs. **Delete an item once it is addressed**;
this file lists open findings and nothing else. Groups are ordered by severity
and benefit; items within a group share one root cause.

## The network state is re-encoded three times along one path

- [ ] `NetPhase` (5 variants, `logic/src/net/net_phase.rs`) → `NetStatus`
      (4 variants, `#[repr(u8)]`) → `LowerKey` (3 variants, `src/lcd.rs`).
      Each hop is a lossy hand-written match, and the middle one round-trips
      through an `AtomicU8` and `from_repr(...).expect(...)`. The atomic hop
      earns its place — it is how the LCD thread reads the phase without
      locking the supervisor — but nothing checks that the two matches stay
      consistent with each other.
- [ ] `NetStatusHandle` and `SubmissionStatusHandle` (`src/net.rs`) are the
      same `Arc<AtomicU8>` newtype with identical `new`/`store`/`load`, written
      twice. *(Left alone in Phase 3: strum's `FromRepr` is an inherent method
      rather than a trait, so the generic that removes the duplication needs a
      local trait, two impls, `PhantomData`, and manual `Clone`/`Debug` to
      avoid spurious `T:` bounds — more code and more indirection than the ~15
      lines it deletes. Worth revisiting only if a third one appears.)*
- [ ] `NetStatusHandle::load` carries `#[allow(dead_code)] // consumed by the
      lcd thread` — it is dead only when the `lcd` feature is off, expressed as
      an unconditional allow rather than a cfg.

## `SensorData` is a pass-through wrapper carrying an unrelated mailbox

- [ ] `update_battery`, `update_ps`, `battery_reading`, `ps_reading`,
      `power_online`, `history`, `interval` are each a one-line delegate to a
      private `LiveReadings` or `History` (`logic/src/data/mod.rs:178-210`).
      The type adds no behaviour of its own except `tick`.
- [ ] Its five public fields — `model_code`, `ps_offline`, `charge_phase`,
      `charge_fault`, `charge_inhibit` (`:131-153`) — are a supervisor →
      dashboard mailbox with nothing to do with sensor readings, and they share
      the sensor mutex, so the LCD's 2 Hz read and the `/api` handler contend
      with the XY poll thread's writes.
- [ ] `power_online` is a boolean carried as `f32` because history compaction
      averages it (`logic/src/data/mod.rs:110`). That representation reaches the
      wire: `/api` ships `1.0`/`0.0` for both the live field and every history
      row (`src/api.rs:86`, `:55`).
- [ ] `src/xy.rs` takes the `SensorData` mutex three times per tick — once in
      `poll` to publish `PsReading` + `ps_offline` (`:505`), once in `poll` to
      read the battery back out (`:545`), once in `run` to publish
      phase/fault/inhibit (`:471`).
- [ ] `SensorData::interval()` and `History::interval()` have no caller outside
      the logic crate's own tests; the compaction interval is never surfaced to
      the dashboard that would need it to read the x-axis.

## One number, two literals

- [ ] `SETPOINT_DRIFT_TOL = 0.02` (`logic/src/charging/mod.rs:111`) and the bare
      `0.02` in `xy::verify` (`src/xy.rs:383`) are the same tolerance, with the
      same "one register quantum is 0.01, allow two quanta for IEEE round-trip
      slack on values like 14.4 V" rationale copied into both doc comments.
- [ ] `EXPECTED_MODEL_CODE` (`src/xy.rs:46`) is never compared against anything.
      The real gate is `ModelCheck::scales_match`, decided inside the driver
      (`:335`); the constant only fills in an error message and the fake's
      canned response, so it can silently disagree with what the driver
      actually expects.
- [ ] `api::PsReading` (`src/api.rs:32`) is field-for-field
      `esp32_battery_logic::PsReading`, redeclared only to attach `Serialize`;
      `api::BatteryReading` (`:25`) is `Ina228Reading` with `soc` substituted
      for `voltage`.
- [ ] `Sample` averaging is written twice inside `logic/src/data/history/mod.rs`
      — once in `SampleAccum::average` (`:39`), once inline in
      `compact_if_needed`'s pairwise loop (`:122-128`) — and a third
      structurally identical accumulator exists for `Ina228Reading`
      (`ina::ReadingAccum`, `src/ina.rs:22`).
- [ ] `AP_GATEWAY` is a `[u8; 4]` reformatted into a dotted quad for the LCD
      (`src/lcd.rs:415-417`), while `/generate_204` hard-codes the same address
      as the literal string `"http://192.168.71.1/"` (`src/captive_api.rs:111`).

## The history pipeline pays for adaptive resolution it never reaches

- [ ] `History` commits one raw sample per `SensorData::tick` regardless of
      how long that tick took, so `interval` ("how many raw samples per stored
      entry") no longer implies a fixed span once the main loop jitters. The
      `time_s` stamps stay honest, but sample *density* along the x-axis does
      not, and nothing records the span an averaged sample covers.
- [ ] `History` carries a `SampleAccum`, an `acc_count`, a doubling `interval`
      and `compact_if_needed`, but `MAX_INTERVAL` is `4`
      (`logic/src/data/history/mod.rs:16`) — two doublings, capping coverage at
      ~13.6 minutes. `MAX_ABSORB` alone permits a 2 h absorb phase
      (`logic/src/charging/mod.rs:80`), so the dashboard cannot show a charge
      cycle, and the machinery that grows coverage exponentially stops almost
      immediately.
- [ ] Once the interval is capped, the steady-state drop path is
      `self.samples.remove(0)` on a `heapless::Vec<Sample, 204>`
      (`logic/src/data/history/mod.rs:113`) — a ~4 KB memmove on every push,
      forever, where the buffer is otherwise a ring.
- [ ] `/api` re-serializes all 204 history rows into the shared 16 KB buffer on
      every request (`src/api.rs:164`), at the dashboard's poll rate.

## `xy::poll` does five jobs, and "LVP is benign" is decided in four places

- [ ] `poll` (`src/xy.rs:488`) reads status, publishes `PsReading`, sets
      `ps_offline`, de-duplicates PROTECT events through a
      `&mut ProtectionStatus` out-parameter threaded down from `run`, converts
      to `BuckOutput`, and re-locks the mutex to fetch the battery.
      `last_protection` is per-episode state owned by the caller rather than by
      a type.
- [ ] The rule "input UVLO is benign and self-clearing" is stated independently
      in four places: `src/xy.rs:503` (`ps_offline`), `src/xy.rs:524` (skip the
      event-log record), `charge_supervisor.rs:447` (`Lvp | Otp` →
      `EnterProtectRecovery`), and `InhibitReason::BuckProtection`
      (`logic/src/charging/inhibit_reason.rs:33`). Firmware and logic each
      decide it for themselves.

## The HTTP helper stack is deeper and more parameterised than its callers

- [ ] `create_server(10240, false, 3, true)` and
      `create_server(8192, true, 4, false)` (`src/http/main_server.rs:24`,
      `src/http/captive.rs:19`) — four positional parameters, two of them bare
      bools, at exactly two call sites.
- [ ] `read_exact` (`src/http/mod.rs:105`) takes two separate `&'static str`
      error messages for its one caller, the OTA HMAC prefix (`src/ota.rs:43`).
- [ ] `mount_uri` → `mount_get`/`mount_post` → `mount_json_get` →
      `with_json_buf` → `json_response` → `json_reply` is six layers to write a
      JSON body, and `mount_json_get` wraps its own parameter in a pass-through
      closure (`|inner| build(inner)`, `src/http/mod.rs:270`).
- [ ] `serve_static` (`:60`) rebuilds a 4-entry header vector with four
      `unwrap()`s on every request, though the header set is fully determined
      at mount time.

## A failed credential save reboots the device

- [ ] `nvs_creds::save` `.unwrap()`s both `set_str` calls (`src/nvs_creds.rs`).
      It runs from `promote_to_sta` right after an association succeeds, so a
      full or worn flash panics — the hook reboots, the credentials were never
      persisted, and the unit comes back to the captive portal for the user to
      try again, which fails the same way. Unlike `load`, this is an I/O
      failure rather than untrusted data, so it wants the project's I/O rule
      rather than the validation one. *(Noticed while doing Phase 2; the
      `load` half of that path is fixed.)*

## Two canonical paths for the xy-modbus types

- [ ] The firmware depends on `xy-modbus` directly (`Cargo.toml`) and imports
      `ModelCheck`, `ProtectionStatus`, `RegMode`, `SafetyLimits`, `Setpoints`
      and `Status` straight from it (`src/xy.rs:24`, `:209`, `src/main.rs:35`),
      while `logic/src/lib.rs` also re-exports `ProtectionStatus`,
      `SafetyLimits` and `Setpoints`. Each of those is reachable by two paths,
      against the "one canonical path per item" rule `lib.rs`'s own module doc
      states. Only `BusError` — the rename of `xy_modbus::XyError`, which would
      otherwise collide with `error_log::XyError` — has a reason to be
      re-exported. *(Found while doing Phase 0; the group this replaces is
      addressed.)*

## `lcd.rs` layout is half-derived, half-baked-in

- [ ] `LOWER_ROW1_TOP` 96 / `LOWER_ROW2_TOP` 120 / `LOWER_ROW3_TOP` 144
      (`src/lcd.rs:72-74`) write out a 24 px stride three times, while
      `SCRATCH_H` — the band height those rows must not overlap — is an
      independent `22` (`:107`) that has to be kept ≤ the stride by hand.
      `LOWER_VALUE_X` (`:76`) is the only derived coordinate.
- [ ] `draw_lower` has change detection via `last_lower` (`:393`);
      `draw_upper` (`:343`) repaints all four value cells plus the uptime
      corner every 500 ms whether or not anything moved, through the same blit
      path two functions away.
- [ ] `title_row` positions its badge from `b.len() * 10` and then subtracts a
      hand-tuned `14` for "the panel's column-offset quirk" (`:299-303`) — a
      magic margin sitting inside a font-metric calculation.
