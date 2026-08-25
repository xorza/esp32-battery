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

## One number, two literals

- [ ] `api::PsReading` (`src/api.rs:32`) is field-for-field
      `esp32_battery_logic::PsReading`, redeclared only to attach `Serialize`;
      `api::BatteryReading` (`:25`) is `Ina228Reading` with `soc` substituted
      for `voltage`.
- [ ] `ina::ReadingAccum` (`src/ina.rs:22`) is structurally identical to
      `data::history::SampleAccum` — the same sum-then-divide over a fixed
      field list, written a second time for `Ina228Reading`.
## The history pipeline pays for adaptive resolution it never reaches

- [ ] `History` commits one raw sample per `SensorData::tick` regardless of
      how long that tick took, so `interval` ("how many raw samples per stored
      entry") no longer implies a fixed span once the main loop jitters. The
      `time_s` stamps stay honest, but sample *density* along the x-axis does
      not, and nothing records the span an averaged sample covers.
- [ ] Once the interval is capped, the steady-state drop path is
      `self.samples.remove(0)` on a `heapless::Vec<Sample, 204>`
      (`logic/src/data/history/mod.rs`) — a ~4 KB memmove once per
      `MAX_INTERVAL` seconds, forever, where the buffer is otherwise a ring.
- [ ] `/api` re-serializes all 204 history rows into the shared 16 KB buffer on
      every request (`src/api.rs:164`), at the dashboard's poll rate.

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
