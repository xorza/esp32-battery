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

## Phase 1 — Make the safety timers actually time-based — **DONE**

Highest severity. This changes the supervisor's input contract, so it goes
before any reshaping of the supervisor itself.

Landed with two corrections to what was written below:

- **Step 6 was wrong about the tests.** The charging suite already had
  `ov_trip_accumulates_elapsed_time_not_tick_count`, which drives `tick` with
  500 ms / 1.5 s / 600 ms steps and pins exactly the property in question. The
  supervisor was never the defect — only its caller was. The new tests are on
  the data layer, which genuinely had no time-based coverage.
- **Step 9 was applied at ingest, not at `ocv_soc`.** `LiveReadings::update_*`
  now rejects non-finite readings outright, which settles the policy for the
  supervisor, `/api` and history in one place. That also fixes a latent bug the
  review missed: a single NaN reaching `SampleAccum` makes every average
  computed from that accumulator NaN, and pairwise compaction then spreads it
  across the buffer.

No clamp was added on the measured interval. The task watchdog reboots at 10 s
and is fed at the top of the loop, so `elapsed` is already bounded; an
arbitrary ceiling would be a tolerance with no stated reason behind it.

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

## Phase 2 — Stop untrusted input from bricking the unit — **DONE**

`CredentialsError` lives in a new `logic/src/error.rs` per the house rule that
error types go there. It carries no payload: `message()` returns `&'static str`,
so the HTTP error path needs no formatting buffer and `Display` is written in
terms of it rather than as a second copy of the same strings.

Step 14's device-side half is not covered by tests — `nvs_creds::load` and
`/save` both need esp-idf and cannot run on the host. What is tested is the
single validation point they now share, at every boundary. Confirming the
corrupt-NVS path on hardware still wants a flash.

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

## Phase 3 — Collapse the duplicated encodings — **DONE**

Step 18 landed last, after a first pass had set it aside. Re-reading it against
the code, the objection did not hold: the two predicates the gauntlet needs —
"is the buck sourcing?" and "are we still deciding whether to bring it up?" —
are *total* over `LatchState`, and `Tripped` answers both correctly (the output
is off, or on its way off). `Mode` was buying an unrepresentable-state guarantee
for a case that needed no guarantee, at the cost of a second encoding of the
same state and a mapping written out in `tick`. It is gone; `safety_verdict`
reads `self.latch`, and `tick` dispatches Pending-vs-regulating off the same
field.

`transition_between`'s from×to table is gone too. `commit_enable` was the one
caller that genuinely did not know which transition it was making — so it now
reads the `PendingReason` it is leaving (via the `let-else` that already had to
check the state) and names `Energised` or `ProtectCleared` itself. Every other
caller knew all along.

What was kept, deliberately: the `commit_*` asserts. Step 18 scoped the removal
to asserts "that the call sites make unreachable", and these are public API —
their call sites are in the firmware, outside this crate, so nothing in here can
make a stashed ticket unreachable. `commit_voltage` was reshaped to match
`commit_disable`'s `let-else` + `assert_eq!` shape, which also names *which*
half of the check failed. The three `set_latch` `debug_assert!`s did go: all
six call sites are in this file, each now names its own transition, and the
invariants they restated are visible at those sites.

For step 20 the mechanism kept is the one that works in the shipped build.
`debug_assert_matches_phase` compiled to nothing in release — the firmware
flashes `--release` — so it was deleted in favour of a single
`warn_out_of_step` that every mismatched arm now reports through.

Two further judgement calls, recorded in `REVIEW.md` rather than acted on: the
`NetStatusHandle` / `SubmissionStatusHandle` generic costs more than it saves,
and the `NetPhase → NetStatus → LowerKey` hops each earn their place.

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

## Phase 4 — The data layer — **DONE**

Both flagged decisions were taken: the dead `/api` field goes, and the chart
window becomes 3.6 h.

`power_online` turned out not to be the finding it was written as. Nothing reads
`/api`'s top-level `power_online` — the dashboard only touches the history
column, and there the fractional average is load-bearing: it *is* the offline
percentage. So the live field was deleted as dead, and the history column stayed
`f32` and got documented as what it actually is, a duty-cycle fraction over the
sample's span. No dashboard change.

`MAX_INTERVAL` 4 → 64. The buffer was already 204 × 20 B, so the window grows
from 13.6 min to 3.6 h for no RAM: it now covers a full `MAX_ABSORB` cycle with
margin, and the doubling ladder does five steps instead of two before it caps.

The mailbox is `ChargeStatus`, behind its own mutex, and it is `Copy` — every
reader takes the lock, copies the struct out and releases it, so the two locks
are never nested and cannot deadlock whichever order a future caller wants them.
`SensorData` absorbed `LiveReadings` and is no longer a delegating wrapper. The
XY loop now takes each mutex exactly once per tick (it took `SensorData` three
times), and publishes `ps_offline` in the same acquisition as the supervisor
state derived from it, so readers see a coherent pair rather than a half-updated
one.

**One deviation, in step 28.** The two averaging paths are collapsed —
compaction now goes through `SampleAccum::average`, so the two cannot drift on
which fields get averaged. `samples.remove(0)` was **kept**. Its premise moved
under the cap decision: it now runs once per 64 s, not once per 4 s, and it is a
~4 KB memmove costing single-digit microseconds. Replacing it means a
`heapless::Deque`, which costs every reader the contiguous `&[Sample]`
(`/api`'s `HistoryView` and ~35 slice-indexing sites in tests); the alternative,
dropping in chunks, makes the chart's left edge jump instead of slide. Under the
project's stated order — convenience and good looks both outrank performance —
paying microseconds a minute for a contiguous buffer and a smooth left edge is
the right trade. The finding stays open in `REVIEW.md` with corrected numbers.

**Verify:** `./run_tests.sh` passes. A dashboard load against a live unit is
still worth doing for the `power_online` removal, though nothing in
`assets/index.html` references that field.

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
