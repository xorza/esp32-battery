# Plan for working through `.notes/REVIEW.md`

Ordered so that contract changes land before the shapes that depend on them, and
so no phase re-does work a later phase would undo. Every phase ends with
`./run_tests.sh`. Nothing here commits — the diff is yours to inspect at each
phase boundary.

Three items need your call before they can be done; they are marked **[needs
your call]** and are listed again at the bottom.

---

## Phase 0 — Guardrails and pure deletion — **DONE**

No behaviour change; everything here is compiler-checked, so it clears noise out
of the way of the phases that follow.

Landed with two deviations from what was written below:

- `TimedEvent` and `LinkState` were **kept**. They are not dead: `TimedEvent`
  is the item type of `EventLog::recent()` (used by `src/errors.rs`) and
  `LinkState` is a public field type of the exported `NetPhase`. Dropping the
  re-exports would have left live public signatures unnameable. What is
  actually wrong there is a different finding, now recorded in `REVIEW.md`
  under "Two canonical paths for the xy-modbus types".
- The test-only accessors are exposed as **extension traits** in the gated
  `internals` mods, not free functions, so the call sites keep method syntax
  (`s.target_voltage()`, `sd.history.interval()`). A same-named inherent method
  in a gated `impl` block is a hard duplicate-definition error; a trait method
  resolves cleanly at call sites where the private inherent one is out of scope.

1. **[needs your call] Pin dependency versions.** Read the resolved versions out
   of `Cargo.lock` and write them into `Cargo.toml` and `logic/Cargo.toml` in
   place of every `"*"`. Rebuild for both MCU targets to confirm the lock did
   not move. *(Manifest edit — I wait for your go-ahead.)*
2. **Delete unreferenced public surface.** `RtuError`, `TimedEvent`,
   `LinkState`, `HISTORY_CAPACITY`, `OV_MARGIN_V`, `CAPTIVE_AFTER_DISCONNECT`,
   `CAPTIVE_TRYING_TIMEOUT` from `lib.rs`; `EventLog::CAPACITY`;
   `Ring::capacity()`; `SensorData::interval()` / `History::interval()`.
   Compiler proves each removal.
3. **Fix the stale doc reference** to `api::RESPONSE_BUF_SIZE` in
   `src/http/mod.rs`.
4. **Move the supervisor's test-only accessors behind the `internals` gate** —
   `target_voltage`, `expected_setpoints`, the `latch` field, `LatchState`,
   `PendingReason` — and repoint `logic/src/charging/tests/` at them. Mechanical,
   but do it now so Phase 3's supervisor rework isn't fighting `pub(super)`
   visibility it doesn't need.

**Verify:** `./run_tests.sh`. Expect zero behaviour delta.

---

## Phase 1 — Make the safety timers actually time-based

Highest severity. This changes the supervisor's input contract, so it goes
before any reshaping of the supervisor itself.

5. **Feed measured elapsed into `ChargeSupervisor::tick`.** The xy loop already
   has `clock::uptime()`; take it at the top of each iteration and pass the real
   delta instead of the `POLL_INTERVAL` constant. Watch for the first tick
   (no previous timestamp) and for a tick so long it should be treated as a gap
   rather than as accumulated evidence.
6. **Add tests that pin the new behaviour.** The existing charging tests drive
   `tick` with a constant, so they pass either way and prove nothing here. Needed:
   a debounce that receives one 3 s tick fires exactly as it does after three 1 s
   ticks, and a tick longer than a window fires that window once.
7. **Convert `data::STALE_TICKS` to a `Duration`** and drive
   `SensorData::tick` / `LiveReadings::age` from measured elapsed in the main
   loop, so a main loop blocked inside `try_connect` no longer silently stretches
   the staleness window and the history cadence.
8. **State the end-to-end sensor-loss latency once**, derived from the INA
   averaging window, the staleness window and `BATTERY_MISSING_TIMEOUT`, rather
   than as prose adding a tick count to a duration.
9. **Settle the NaN policy.** `ocv_soc` reporting 0 % and the supervisor
   reporting "missing" for the same reading is one decision made twice; pick one
   and apply it in both places.

**Verify:** `./run_tests.sh`, plus a bench flash to watch a real charge tick —
this phase changes live timing behaviour, not just its description.

---

## Phase 2 — Stop untrusted input from bricking the unit

10. **Give `WifiCredentials` a fallible constructor** and route the two
    untrusted producers (`nvs_creds::load`, `captive_api`'s `/save`) through it.
    The asserting constructor can stay for the internal/test path or go entirely.
11. **Delete the duplicated checks in `captive_api`** once the constructor
    returns errors — the `8..=63` rule and the `SSID_MAX` bound then live in one
    file, which is what `wifi_credentials.rs` already claims.
12. **Make `nvs_creds::load` tolerate a corrupt or oversized blob** instead of
    `.unwrap()`ing `get_str` — the current path is a boot loop with no captive
    portal to recover through.
13. **Make `json_err` truncate rather than `.expect`** so no HTTP error path can
    reboot the device.
14. **Test the recovery path:** an over-long stored SSID/password and an
    over-long `/save` body both produce a working captive portal, not a panic.

**Verify:** `./run_tests.sh`, then flash and confirm a deliberately corrupted NVS
key still boots into the portal.

---

## Phase 3 — Collapse the duplicated encodings

The bulk of the code reduction. Each item is independent of the others, so they
can land in any order within the phase.

15. **Event-kind trio.** One representation for kind + count, so `EventLog`
    stops carrying three parallel arrays and three accessor triples, and
    `errors.rs` stops needing `CountKind` to re-dispatch into them.
16. **Reason labels.** Derive `label()` and `Display` for `FaultReason` and
    `InhibitReason` the way `Phase` and the error kinds already do, retiring four
    hand-maintained variant lists. `FaultReason::OutputUnexpectedlyOff` and
    `InhibitReason::BuckProtection` carry payloads — check the derive still gives
    the stable snake_case identifier the dashboard matches on.
17. **`api::reason_message`** — decide null-vs-present from the `Option`, not
    from whether a formatted string came out empty.
18. **Supervisor state.** Fold `Mode` back into reads of `LatchState`, drop
    `transition_between`'s re-derivation in favour of the transition each
    `set_latch` caller already knows, and remove the `set_latch` /`commit_*`
    asserts that the call sites make unreachable.
19. **Network state hops.** Collapse `NetPhase → NetStatus → LowerKey` where the
    intermediate buys nothing, unify `NetStatusHandle` /
    `SubmissionStatusHandle` / `ResetSignal` into one atomic newtype, and delete
    `MixedWifi::sta_configured` (its short-circuit is unreachable behind
    `polls_association()`).
20. **`NetResources` invariant.** Three mechanisms guard one coupling — the
    per-tick `debug_assert`, the silently-no-op `if let` arms, the `warn!`
    else-branches. Keep one.
21. **`net_supervisor::step`.** Factor the `associated → PromoteToSta` block
    (twice) and the `submitted → CaptiveTrying` block (three times); the
    per-transition `WifiCredentials` clone goes with them.
22. **`wifi.rs` wrappers.** One `try_connect` body, one mixed-configuration
    builder shared by `into_mixed` and `apply_sta_config`.

**Verify:** `./run_tests.sh`. The net and charging test suites carry this phase —
if a collapse is unsafe, they are where it shows.

---

## Phase 4 — The data layer

23. **Split `SensorData`.** The supervisor → dashboard mailbox
    (`model_code`, `ps_offline`, `charge_phase`, `charge_fault`,
    `charge_inhibit`) is a different concern from the reading store and currently
    shares its mutex, so the LCD and `/api` contend with the XY poll thread.
24. **Flatten the delegating wrapper** once the mailbox is out — every remaining
    `SensorData` method is a one-liner onto `LiveReadings` or `History`.
25. **[needs your call] `power_online` on the wire.** It is a bool carried as
    `f32` because compaction averages it, and that reaches `/api` and the
    history rows. Changing it touches `assets/index.html`.
26. **Reduce the xy loop to one lock acquisition per tick** (currently three).
27. **[needs your call] History coverage.** `MAX_INTERVAL = 4` caps the chart at
    ~13.6 min while `MAX_ABSORB` permits a 2 h absorb — the dashboard cannot
    show a charge cycle. Either the cap moves or the compaction machinery is
    doing very little for its size.
28. **Replace the steady-state `samples.remove(0)`** — a ~4 KB memmove on every
    push once the interval is capped — and collapse the two hand-written
    averaging paths in `history/mod.rs` into one.

**Verify:** `./run_tests.sh`, plus a dashboard load against a live unit if 25 or
27 land.

---

## Phase 5 — Firmware ergonomics

Lowest risk, lowest urgency; safe to defer.

29. **`http` helper stack.** Named server configuration instead of
    `create_server(10240, false, 3, true)`; inline `read_exact` into its single
    OTA caller; drop `mount_json_get`'s pass-through closure; build
    `serve_static`'s header set at mount time rather than per request.
30. **`xy::poll`.** Split the five jobs; give the PROTECT rising-edge dedup a
    home instead of a `&mut ProtectionStatus` out-parameter threaded from `run`.
31. **Single LVP policy.** "Input UVLO is benign" is decided independently in
    firmware and in logic, in four places.
32. **Remaining duplicated constants and types.** `SETPOINT_DRIFT_TOL` vs
    `xy::verify`'s literal `0.02`; `EXPECTED_MODEL_CODE`, which is compared
    against nothing; `api::PsReading` / `api::BatteryReading` mirroring the logic
    types; `ina::ReadingAccum` vs `SampleAccum`; the `AP_GATEWAY` array vs the
    hard-coded `"http://192.168.71.1/"` in `/generate_204`.
33. **`lcd.rs` layout.** Derive the row stride instead of writing 96/120/144 and
    keeping `SCRATCH_H` compatible by hand; give `draw_upper` the change
    detection `draw_lower` already has; fold the `- 14` badge margin into the
    font-metric calculation or say what it is.

**Verify:** `./run_tests.sh`, then flash and look at the panel — 33 is the one
group tests cannot judge.

---

## Decisions I need from you

- **Pinning dependencies** (step 1) — a manifest change, so I wait on it.
- **`power_online` as a bool on the wire** (step 25) — breaks
  `assets/index.html` unless the frontend changes with it.
- **History coverage vs. the 2 h absorb window** (step 27) — a product call
  about what the chart is for, not something the code answers.

## Suggested stopping points

Phases 0–2 stand on their own and are worth having regardless: they remove dead
code, make the safety timers mean what they say, and close the boot-loop path.
Phases 3–5 are quality work that can be taken in any order, or dropped, without
leaving anything half-done.
