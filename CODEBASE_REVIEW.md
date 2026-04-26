# Codebase review


## Minor / opinionated

### 7. DNS startup panics are acceptable — `src/dns.rs:39, 42`

`UdpSocket::bind("0.0.0.0:53").expect(...)` and the
`set_read_timeout` unwrap are boot-time. Failure means either another
captive bundle is alive (impossible — FSM enforces single-owner) or the
kernel socket layer is broken. Panic → reboot is fine.

**No change recommended.** Listed for completeness.

### 8. `heapless::Vec::push(...).unwrap()` on a fixed-cap-2 header vec — `src/http/mod.rs:159-163`

Adding a 3rd header silently panics.

**Fix:** add `// cap: 2` comment, or accept that this is an internal
helper that any future refactor will surface.

### 9. Dead code with `#[allow(dead_code)]` — `src/xy.rs:41`, `src/ina.rs:56, 73`

`XyStatus` carries `v_set`, `i_set`, `v_in` with a TODO comment "will
be surfaced via HTTP panel" that never happened. Some `XyIoError`
variants are unread under `xy-fake`. Per CLAUDE.md "remove unused
code."

**Fix:** wire the fields through to `/api`, or delete them. For
xy-fake-only error variants, gate them behind `#[cfg(not(feature =
"xy-fake"))]` or split the enum.

### 10. `SubmissionStatus::Pending` is briefly observable but redundant

`/save` writes `Pending` → next supervisor tick → `apply_submission`
writes `Trying`. `Pending` is therefore observable for ≤1 s. The
captive page presumably treats `Pending` and `Trying` identically
(both = "show spinner").

**Fix (low priority):** if the page makes no UX distinction, drop to
`Idle | Trying | Failed`. Verify against the captive page logic before
removing.

## Skipped (not real issues)

- **"saturating_sub edge case if uptime drifts"** — `uptime()` is
  monotonic by construction.
- **"main loop missing watchdog kicks"** — ESP-IDF task WDT isn't
  enabled on the main task by default; not a regression.
- **"panic hook floods on tight panic loop"** — 500 ms sleep + restart
  is the standard ESP-IDF pattern; bounded by boot timing.
- **"NVS open panic at boot"** — unrecoverable; panic-reboot is
  correct.
- **"OTA HMAC key length `expect`"** — key length is compile-time;
  `expect` is appropriate.

## Suggested order

1. Items 1–3 (correctness).
2. Items 4–6 (robustness, observability).
3. Items 7–10 (taste, low value).
