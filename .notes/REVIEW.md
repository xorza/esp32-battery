# Crate review

Findings only — no fixes, no designs. **Delete an item once it is addressed**;
this file lists open findings and nothing else. Groups are ordered by severity
and benefit; items within a group share one root cause.

## Safety timers are specified in wall time but driven by tick counts

- [ ] `src/xy.rs:460` passes the constant `POLL_INTERVAL` as `elapsed` to
      `ChargeSupervisor::tick`, never a measured interval. `Debounce` documents
      itself as "time-based so the debounce isn't sensitive to poll cadence"
      (`logic/src/charging/debounce.rs:5`), and every window — OV 3 s, absorb
      cap 2 h, exit taper 60 s, missing battery 10 s, modbus 5 s — inherits
      that claim. The xy loop's real period is `POLL_INTERVAL` plus the Modbus
      transactions in that iteration (up to a 500 ms response timeout each,
      plus `STEP_DOWN_SETTLE` and up to four writes on a step-down), so every
      window fires late by an amount nothing bounds.
- [ ] `logic/src/data/mod.rs:49` defines staleness as `STALE_TICKS: u32 = 5`
      counted in ticks, while its sibling `BATTERY_MISSING_TIMEOUT`
      (`logic/src/charging/mod.rs:98`) is a `Duration`.
      `logic/src/charging/mod.rs:95-97` documents the end-to-end latency as
      "`data::STALE_TICKS + BATTERY_MISSING_TIMEOUT`" — a tick count added to a
      duration in prose.
- [ ] `LiveReadings::age` is driven from the `main` loop
      (`src/main.rs:176`), which blocks inside `resources.try_connect` on
      `wifi.connect()` + `wait_netif_up()` (`src/wifi.rs:223`, `:297`). A slow
      or failing association stretches both the staleness window and the
      history sample cadence, while `History::interval` keeps reporting 1/2/4
      as though the samples were evenly spaced.
- [ ] The total time from a dead INA228 to a latched buck is the sum of three
      independently specified windows measured in two units and on two threads
      (INA averaging 10 × 100 ms in `src/ina.rs:18-19`, `STALE_TICKS` on the
      main loop, `BATTERY_MISSING_TIMEOUT` on the xy loop). No single place
      states the resulting number.

## Untrusted input is validated by assertions, and a failed assertion reboots

- [ ] `WifiCredentials::new` (`logic/src/net/wifi_credentials.rs:21`) validates
      with `assert!`/`expect`. It is called on NVS blob content
      (`src/nvs_creds.rs:26`) and on a decoded HTTP body
      (`src/captive_api.rs:86`). A corrupt or hand-written NVS value therefore
      panics, the `main.rs` panic hook reboots, and the captive portal never
      starts — a boot loop with no way in. `src/nvs_creds.rs:13-15` states this
      is intentional.
- [ ] `src/nvs_creds.rs:19-20` `.unwrap()`s `get_str`, which errors when the
      stored blob exceeds the buffer — the same boot-loop path, reached before
      any length check runs.
- [ ] Because the constructor asserts, `/save` re-implements every check before
      calling it (`src/captive_api.rs:78-84`: empty SSID, `SSID_MAX`, the
      `8..=63` password rule). The "centralised here so every site that
      produces credentials gets the same checks" claim in
      `logic/src/net/wifi_credentials.rs:16-20` holds only because the checks
      are duplicated; the `8..=63` rule and its explanatory comment appear
      verbatim in both files.
- [ ] `http::json_err` (`src/http/mod.rs:193`) `.expect`s when a message
      exceeds its 192-byte buffer — a panic, and therefore a device reboot, on
      an HTTP error path.
- [ ] `battery::ocv_soc` returns `0.0` for a non-finite pack voltage
      (`logic/src/battery/mod.rs:113`), while `ChargeSupervisor::tick` treats a
      non-finite sample as *missing* (`logic/src/charging/charge_supervisor.rs:481-484`).
      One bad reading is "0 % charged" on the dashboard and "no sample" to the
      supervisor.

## The network state is re-encoded five times along one path

- [ ] `NetPhase` (5 variants, `logic/src/net/net_phase.rs:20`) →
      `NetStatus` (4 variants, `#[repr(u8)]`, `:52`) → `LowerKey`
      (3 variants, `src/lcd.rs:197`). Each hop is a lossy hand-written match,
      and the middle hop round-trips through an `AtomicU8` and
      `from_repr(...).expect("invalid NetStatus discriminant")`
      (`src/net.rs:40`).
- [ ] `NetResources` (`src/net.rs:134`) is a fourth encoding — two variants the
      phase already determines — kept honest by three separate mechanisms:
      `debug_assert_matches_phase` once per tick (`:182`), `if let
      NetResources::Mixed {..}` arms in `apply_net_action` that silently no-op
      on mismatch (`src/main.rs:207`, `:216`, `:226`), and
      `warn!("… out of step")` else-branches in `promote_to_sta` /
      `sta_to_captive` (`src/main.rs:246`, `:267`). Its own doc comment says it
      is recovering a property "the old fused enum enforced by construction".
- [ ] `MixedWifi::sta_configured` (`src/wifi.rs:241`) is a fifth: it tracks
      "are credentials on the radio", which `NetPhase::polls_association()`
      already answers and `NetResources::try_connect` already gates on
      (`src/net.rs:152`). The only phase in which it can be `false` is
      `CaptiveIdle`, which is exactly the phase `polls_association()` rejects —
      so the short-circuit at `src/wifi.rs:290` is unreachable.
- [ ] `NetStatusHandle` (`src/net.rs:26`) and `SubmissionStatusHandle` (`:60`)
      are the same `Arc<AtomicU8>` newtype with identical `new`/`store`/`load`
      bodies, written out twice; `ResetSignal` (`:83`) is the `AtomicBool`
      sibling of the same shape.
- [ ] `NetStatusHandle::load` carries `#[allow(dead_code)] // consumed by the
      lcd thread` (`src/net.rs:37`) — it is dead only when the `lcd` feature is
      off, expressed as an unconditional allow rather than a cfg.

## The event-kind trio is written out three times at every layer

- [ ] `InaError`, `XyError` and `ChargeTransition` each carry a hand-written
      `index()` + `name()` pair with identical bodies
      (`logic/src/error_log/mod.rs:22-33`, `:77-88`, `:90-98`).
- [ ] `EventLog` holds three parallel count arrays, three `*_count` accessors,
      three `*_counts_iter` accessors, and a three-arm `record` match whose arms
      differ only in which array they index (`logic/src/error_log/mod.rs:143-222`).
- [ ] `src/errors.rs` adds a `CountKind` enum (`:27`) whose only job is to
      re-dispatch to one of those three iterators, plus a third three-arm match
      in `RecentView` (`:51-55`).
- [ ] `ina_count` / `xy_count` / `charge_count` are `pub` but have no caller
      outside the `*_counts_iter` methods on the same type and the tests.

## The charge supervisor's own state lives in five overlapping types

- [ ] `LatchState` (`logic/src/charging/charge_supervisor.rs:38`) is re-derived
      every tick into `Mode` (`:69`), a lossy copy that exists only because
      `safety_verdict` takes `&mut self` and cannot hold a borrow of
      `self.latch`. `PendingReason` is then carried in both.
- [ ] `Verdict` (`:97`) is a fifth enum encoding the transitions between the
      other two.
- [ ] `transition_between(&from, &to)` (`:126`) re-derives which transition just
      happened by comparing two states, though each of the four `set_latch`
      call sites already knows which one it is making.
- [ ] `set_latch` carries three `debug_assert!`s (`:593-613`) re-checking
      legality that those same four callers establish by construction.
- [ ] Each `commit_*` method asserts that the ticket matches the current state
      (`:269-276`, `:291-295`, `:318-327`); `commit_disable`'s own doc says the
      assert "cannot fire through the public API".

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

## The net supervisor's transition table repeats itself

- [ ] The `p.associated → StaServing + PromoteToSta` block is byte-identical in
      the `CaptiveTrying` and `CaptiveFallbackRetrying` arms
      (`logic/src/net/net_supervisor.rs:88-96`, `:121-129`).
- [ ] The `p.submitted → CaptiveTrying + ApplyCreds` block appears three times
      (`:70-76`, `:97-106`, `:130-138`).
- [ ] Every arm that stays put reconstructs its own phase by hand
      (`:113-116`, `:139-142`, `:166-172`, `:193-196`), and `tick` runs a
      `mem::replace` against a `CaptiveIdle` placeholder (`:46`) purely to move
      the credentials out by value.
- [ ] `WifiCredentials` (a 32- + 64-byte heapless pair) is cloned on each of
      those transitions because `Step` needs the same creds in both the phase
      and the action.

## The two `wifi.rs` mode wrappers duplicate each other

- [ ] `StaWifi::try_connect` (`src/wifi.rs:218`) and `MixedWifi::try_connect`
      (`:289`) share the same four-line body, `MixedWifi` prefixing the
      unreachable `sta_configured` check; `NetResources::try_connect` then
      matches on both variants to call the same-named method (`src/net.rs:155`).
- [ ] `WifiDriver::into_mixed` (`src/wifi.rs:190`) and
      `MixedWifi::apply_sta_config` (`:276`) both build
      `Configuration::Mixed(sta_config-or-default, ap_config())` — one via
      stop/start, one live.
- [ ] `StaWifi::into_mixed` (`:228`) and `MixedWifi::into_sta` (`:320`) are
      one-line forwards to the identically named `WifiDriver` methods.

## Three mechanisms produce one kind of label

- [ ] `Phase` derives its label from `strum::IntoStaticStr`
      (`logic/src/charging/phase.rs:5`), as do `InaError`, `XyError` and
      `ChargeTransition`. `FaultReason` and `InhibitReason` each hand-write both
      a `label()` match *and* a `Display` match — four variant lists maintained
      by hand next to enums that already derive the same string
      (`logic/src/charging/fault_reason.rs:47-74`,
      `logic/src/charging/inhibit_reason.rs:36-60`).
- [ ] `api::reason_message` (`src/api.rs:116`) builds a 64-byte
      `heapless::String` even when the reason is `None`, then tests
      `is_empty()` to decide whether the field serializes as null
      (`:161`, `:163`) — "no fault" and "a fault whose `Display` is empty" are
      the same value.

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
