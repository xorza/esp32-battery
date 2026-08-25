# Open issues

- The root `Cargo.lock` pins `xy-modbus` at `fae334c7` while `logic/Cargo.lock` pins
  `b515e541`. Firmware builds and `-p esp32-battery-logic` host builds therefore compile
  against different revisions of the same dependency.

- After a `CaptiveTrying` timeout the failed credentials stay applied to the radio, but
  `CaptiveIdle` never attempts association, so a slow-but-successful association is never
  noticed until another `/save`.

- The `PendingReason::ProtectRecovery` transition in `ChargeSupervisor::tick` calls
  `reset_phase_timers` but leaves the `ov` debouncer accumulated.

- `src/main.rs` retains a commented-out error-simulation block labelled "TEMP" and
  "Remove before merging".

- `create_server` in `src/http/mod.rs` no longer sets TCP keep-alive or `SO_LINGER`:
  published `esp-idf-svc` 0.52.1 exposes neither on its server `Configuration`. Half-open
  sockets are reclaimed only by `lru_purge_enable` and the 2 s session timeout, and closed
  sockets pass through TIME_WAIT while `max_open_sockets` is 3 on the dashboard.

- `ESP_IDF_VERSION` is pinned to the `v5.5.4` tag rather than tracking `release/v5.5`,
  because `sdmmc_host_t` gained `unaligned_multi_block_rw_max_chunk_size` in v5.5.5 and
  published `esp-idf-hal` 0.46.2 does not initialize it.

- The boot-time MODEL gate in `src/xy.rs` now refuses an unrecognised MODEL code. The
  previous `ModelCheck::Inconclusive` path allowed undocumented codes through.

- `rust-toolchain.toml` pins `channel = "esp"` because nightly cannot build `std` for the
  espidf targets: `library/std/src/sys/fs/unix.rs` references `libc::AT_FDCWD` and `libc`
  defines no `AT_*` constants for the espidf platform.
