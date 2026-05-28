AI coding rules for Rust projects:

## Product context

This is a **UPS module**, not a standalone battery charger. The XY7025
buck simultaneously charges the battery and powers a continuous external
load on the output. The battery is the backup; the buck is the primary
source. Design implications:

- **Float is required**, not optional. After Absorb completes, the
  supervisor MUST stay in Float to keep the load powered. Do not
  propose "skip float for LFP" simplifications — terminating output
  would drop the load. The load is what makes the Absorb→Float
  transition a normal, repeatable event, not a one-shot.
- **The buck output is normally enabled.** Any `set_output(false)` is
  either a fault response (latched off, load drops to battery) or part
  of a deliberate transition sequence (e.g. the safe step-down around
  V_SET changes — `apply_update_voltage` in `src/xy.rs`).
- **Output disables are user-visible**: when the buck is off, the load
  is running on battery alone. Treat fault latches as urgent — the pack
  will drain on the load's timescale until manual reboot.
- **V_SET step-downs need the off→write→on cycle** to avoid reverse
  current through the buck's low-side FET (XY7025 has no anti-backup
  protection on either port). See `Action::UpdateVoltage`'s
  `cycle_output` flag. A previous PS was likely killed by live
  step-down writes before this mitigation existed.


## Verification

- After changing code, run `./run_tests.sh` before confirming. It runs host-side
  logic tests + clippy + fmt, then firmware clippy on both esp32c6 and esp32c3
  with and without fake-hardware features. Fails fast on the first error.
- Check test run times are reasonable. Research and fix slow tests.
- Check online documentation for best practices and patterns.

## Flashing / Serial monitor

- Flash: `MCU=esp32c6 ./flash.sh` — uses `espflash flash --monitor --non-interactive`, so boot logs stream to stdout without needing a TTY. Always wrap in `timeout 30` — that's enough to flash + see initial boot output. Never use the default 2-minute Bash timeout, and never the 5–10 min upper bound. If the flash itself fails, it fails fast; long timeouts only burn wall time waiting on a successful boot's monitor stream.
- Monitor only (no flash): `espflash monitor --non-interactive --port /dev/ttyACM0`.

## Pre-allowed commands

The actual allow list lives in `.claude/settings.local.json` and accumulates
over sessions. If a command would prompt, first reshape it to match an
existing pattern (e.g. reuse `./run_tests.sh` instead of typing the inner
cargo invocations, wrap flash calls as `timeout N bash -c "MCU=… ./flash.sh"`).
If a shape is genuinely needed and recurring, invoke `/fewer-permission-prompts`
to extend the allow list rather than burning prompts each session.

For device HTTP probing, no fixed IP is canonical — units live on the LAN
under their assigned addresses (e.g. `https://<device>/api`, `/api/errors`,
`/api/log`). Use whichever device you're debugging.
