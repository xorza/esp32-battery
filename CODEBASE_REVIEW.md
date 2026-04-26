# Codebase review

Whole-codebase pass after the WiFi FSM flatten. Items grouped by severity;
each entry has the file:line, the issue, and a concrete fix.

## Correctness

> Items 1–3 below were applied in commit following this review.

### 1. UTF-8 fallback masks bad input — `src/captive_api.rs:51-63`

`std::str::from_utf8(...).unwrap_or("")` on form fields means malformed
UTF-8 silently becomes an empty string. The length validation catches
the SSID-empty case, but "garbage bytes" should not be treated as "user
submitted empty."

**Fix:** reject with HTTP 400 on UTF-8 failure (use `?`-style propagation
into `text_response(req, 400, b"Invalid UTF-8")`).

### 2. Inconsistent panic style on atomic-discriminant load — `src/net.rs:74`

`SubmissionStatusHandle::load` uses `.unwrap()`; `NetStatusHandle::load`
(line 43) uses `.expect("invalid NetStatus discriminant")`. Both
indicate a logic error if they fire (we always store via `s as u8` from
a valid variant), so the panic is appropriate — but pick one form.

**Fix:** use `expect("invalid SubmissionStatus discriminant")` for
parity.

### 3. Fragile SSID/password length contract — `src/wifi.rs:84-85`

`creds.ssid.as_str().try_into().unwrap()` panics inside `wifi.rs` if
the SSID is longer than the wifi crate's fixed buffer. Currently safe —
`captive_api` validates ≤32 chars and `nvs_creds::load` reads into
`[u8; 33]` — but any future caller constructing `WifiCredentials`
directly with a long string panics inside `wifi`.

**Fix:** centralise validation in a `WifiCredentials::new(ssid, pw)`
constructor (asserts on length), make the public field private, and
have `nvs_creds::load` / `captive_api` go through it.

## Should fix

### 4. History-blob corruption is silent — `src/main.rs:103-107`

When the NVS history blob is corrupt, we log a warning and start fresh.
The user has no way to know they lost history; the dashboard
`/api/errors` page won't show it.

**Fix:** push an event into `event_log` so it surfaces on
`/api/errors`.

### 5. Inconsistent HTTP error response shapes

`/api`, `/api/errors`, `/save`, `/wifi-reset` return errors in
different shapes: plain text, JSON-encoded `{"error":"..."}`, raw 400
body.

**Fix:** standardise on JSON envelopes for API routes (`/api/*`,
`/save`) and plain text for human-targeted routes (`/wifi-reset`),
or document why each is what it is. Frontend probably already
accommodates this implicitly; pick what matches what the page expects.

### 6. `board.rs` has no pin-mapping doc

The `Board` struct hands out peripherals without documenting which
pins, why, or which feature flag changes the mapping.

**Fix:** add a comment block listing LCD / I²C / UART / XY pins, or a
`src/NOTES-AI.md` if it gets long.

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
