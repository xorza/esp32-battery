# WiFi FSM

A single flat `enum NetPhase` in `logic/src/net/mod.rs`. The variant
alone determines:

- radio mode (STA-only vs Mixed AP+STA)
- which servers/threads are alive (dashboard vs captive bundle)
- what the LCD shows
- which transitions are legal

Each variant carries only the data meaningful to that variant. No
`Option`s used as state flags. Illegal combinations are not
representable.

## States

| # | State | Radio | Servers up | Creds on radio | LCD |
|---|---|---|---|---|---|
| 1 | `CaptiveIdle` | Mixed | captive | none, or last attempt failed | `Captive` (with error if `status==Failed`) |
| 2 | `CaptiveTrying` | Mixed | captive | applied, association in flight (≤ 20 s) | `CaptiveTrying` |
| 3 | `CaptiveFallbackRetrying` | Mixed | captive | known-good creds, STA half retrying in background | `Captive` |
| 4 | `StaConnecting` | STA-only | dashboard | known, never associated this session | `Connecting` |
| 5 | `StaServing` | STA-only | dashboard + mDNS | known | `Host` if `link == Up`, else `Connecting` |

`CaptiveIdle` covers both cold boot (`status == Idle`) and post-failure
retry (`status == Failed`); the captive page reads the
`SubmissionStatus` atomic to decide whether to render a "wrong
credentials" error. `StaServing` carries a `link: LinkState` enum
(`Up | Down { since }`) recomputed each tick from `is_connected()`;
the dashboard server stays up across `Down` windows so re-associations
are silent.

## Transitions

Triggers:
- **boot**
- **/save** — user submits creds in captive page
- **tick:assoc** — supervisor sees `is_connected() == true`
- **tick:!assoc** — supervisor sees `is_connected() == false`
- **timeout:20s** — `CaptiveTrying` association budget expired
- **timeout:2h** — STA-side fallback grace expired

Drain semantics: when the supervisor sees a fresh `/save` payload in
the mailbox during a captive-arm tick, it applies the creds to the
radio (`set_sta_creds`), flips the submission status to `Trying`, and
transitions directly to `CaptiveTrying`. Within `CaptiveTrying` and
`CaptiveFallbackRetrying`, the **assoc-success check runs before the
mailbox drain** so a /save that arrives in the same tick as a
late-but-successful association can't disconnect us.

| From | Trigger | To | Action |
|---|---|---|---|
| (start) | boot, no NVS creds | `CaptiveIdle` | start Mixed, mount captive |
| (start) | boot, NVS creds present | `StaConnecting` | start STA-only with creds, start dashboard |
| `CaptiveIdle` | /save (drain) | `CaptiveTrying` | apply creds, status → Trying, 20 s deadline starts |
| `CaptiveFallbackRetrying` | /save (drain) | `CaptiveTrying` | apply creds, status → Trying (overrides carry-over) |
| `CaptiveTrying` | /save (drain) | `CaptiveTrying` | apply new creds, restart 20 s window (overrides in-flight; only fires if assoc didn't already succeed this tick) |
| `CaptiveTrying` | tick:assoc | `StaServing { link: Up }` | persist creds to NVS, drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `CaptiveTrying` | timeout:20s | `CaptiveIdle` | status → Failed, captive page shows error |
| `CaptiveFallbackRetrying` | tick:assoc | `StaServing { link: Up }` | drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `StaConnecting` | tick:assoc | `StaServing { link: Up }` | take mDNS |
| `StaConnecting` | timeout:2h | `CaptiveFallbackRetrying` | drop dashboard, switch radio to Mixed (carry creds), mount captive |
| `StaServing` | tick (any) | `StaServing` | refresh `link` from `is_connected()`: `Up` while associated, `Down { since: now }` on first miss, `Down { since }` carried while still missing |
| `StaServing { link: Down }` | timeout:2h since `since` | `CaptiveFallbackRetrying` | drop dashboard + mDNS, switch radio to Mixed (carry creds), mount captive |

States 1, 3, 4 with no listed trigger simply stay put.

## Credential ownership

NVS is the durable store. Every variant that has creds at runtime
carries them in the variant — the supervisor never reads NVS per tick.
NVS is written exactly once per successful association
(`CaptiveTrying → StaServing`, `CaptiveFallbackRetrying → StaServing`)
so bad creds can't overwrite a known-good pair.

| Variant | Creds in variant? |
|---|---|
| `CaptiveIdle` | no (NVS empty pre-first-/save, or last attempt's creds intentionally dropped on retry) |
| `CaptiveTrying`, `CaptiveFallbackRetrying` | **yes** |
| `StaConnecting`, `StaServing` | **yes** |

## Submission status (captive-page UX only)

A `SubmissionStatusHandle` (atomic, shared with the HTTP handler)
reports `Idle | Pending | Trying | Failed | Connected` on `/status` so
the captive page's spinner / success / error UI can poll it.

- `/save` handler sets `Pending` immediately on mailbox-write.
- Supervisor's `apply_submission` (drain step) sets `Trying`.
- Supervisor sets `Failed` on the 20 s timeout.
- On successful association the supervisor sets `Connected` and lingers
  ~1.5 s before tearing down the captive bundle — long enough for the
  page's 1 Hz `/status` poll to pick up the explicit `Connected` state
  before the AP disappears. After the linger the bundle drops, the
  radio switches to STA-only, and the dashboard comes up.

## Sketch

The phase is pure timing and credential state. Resources live in the
firmware, keyed off the phase, because five phases collapse to two
resource shapes.

```rust
// logic/src/net/mod.rs — pure, host-tested
enum NetPhase {
    CaptiveIdle,
    CaptiveTrying            { creds: WifiCredentials, since: Duration },
    CaptiveFallbackRetrying  { creds: WifiCredentials },
    StaConnecting            { creds: WifiCredentials, session_start: Duration },
    StaServing               { creds: WifiCredentials, link: LinkState },
}

enum LinkState {
    Up,
    Down { since: Duration },
}

// src/net.rs — what cannot be pure
enum NetResources {
    Mixed { wifi: MixedWifi, bundle: CaptiveBundle },
    Sta   { wifi: StaWifi, server: EspHttpServer, mdns: Option<EspMdns> },
}
```

Each tick the firmware gathers a `NetPoll` (uptime, this tick's
association result, any drained `/save` payload, the reset flag), calls
`NetSupervisor::tick`, and performs the returned `NetAction` against the
resources it owns. The phase *is* the state machine — no separate
`Submission` enum, no shared mutex with HTTP for control state, no
`Option`-as-flag fields. `/save` writes to a single-slot mailbox
(`Arc<Mutex<Option<WifiCredentials>>>`) that the firmware drains during
the relevant captive-arm ticks; latest submission wins.

The 5 → 2 mapping is total and checked by
`NetResources::debug_assert_matches_phase` once per tick, which is what
recovers "illegal combinations are not representable" now that the two
halves are separate types.

## Constants

- 20 s — `CaptiveTrying` association budget.
- 2 h — STA-side fallback grace (`StaConnecting` from boot, or
  `StaServing { link: Down }` since the moment we went Down).
- 10 s — captive scan-cache TTL (refreshed by the supervisor on every
  captive-arm tick where STA is not mid-association —
  `CaptiveIdle`, `CaptiveFallbackRetrying`).
