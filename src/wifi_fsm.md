# WiFi FSM

A single flat `enum NetState`. The variant alone determines:

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
| 1 | `BootNoCreds` | Mixed | captive | none | `Captive` |
| 2 | `CaptiveTrying` | Mixed | captive | applied, association in flight (≤ 20 s) | `CaptiveTrying` |
| 3 | `CaptiveFailed` | Mixed | captive | last attempt failed | `Captive` (with error) |
| 4 | `CaptiveFallbackRetrying` | Mixed | captive | known-good creds, STA half retrying in background | `Captive` |
| 5 | `StaConnecting` | STA-only | dashboard | known, never associated this session | `Connecting` |
| 6 | `StaHost` | STA-only | dashboard + mDNS | known, associated | `Host` |
| 7 | `StaReassociating` | STA-only | dashboard + mDNS | known, link briefly dropped | `Connecting` |

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
| (start) | boot, no NVS creds | `BootNoCreds` | start Mixed, mount captive |
| (start) | boot, NVS creds present | `StaConnecting` | start STA-only with creds, start dashboard |
| `BootNoCreds` | /save (drain) | `CaptiveTrying` | apply creds, status → Trying, 20 s deadline starts |
| `CaptiveFailed` | /save (drain) | `CaptiveTrying` | apply creds, status → Trying, 20 s deadline starts |
| `CaptiveFallbackRetrying` | /save (drain) | `CaptiveTrying` | apply creds, status → Trying (overrides carry-over) |
| `CaptiveTrying` | /save (drain) | `CaptiveTrying` | apply new creds, restart 20 s window (overrides in-flight attempt; only fires if assoc didn't already succeed this tick) |
| `CaptiveTrying` | tick:assoc | `StaHost` | persist creds to NVS, drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `CaptiveTrying` | timeout:20s | `CaptiveFailed` | status → Failed, captive page shows error |
| `CaptiveFallbackRetrying` | tick:assoc | `StaHost` | drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `StaConnecting` | tick:assoc | `StaHost` | take mDNS |
| `StaConnecting` | timeout:2h | `CaptiveFallbackRetrying` | drop dashboard, switch radio to Mixed (carry creds), mount captive |
| `StaHost` | tick:!assoc | `StaReassociating` | servers stay up |
| `StaReassociating` | tick:assoc | `StaHost` | bump `last_assoc` |
| `StaReassociating` | timeout:2h since last assoc | `CaptiveFallbackRetrying` | drop dashboard + mDNS, switch radio to Mixed (carry creds), mount captive |

States 1/3/4 and 5/7 with no listed trigger simply stay put.

## Credential ownership

NVS is the durable store. Every variant that has creds at runtime
carries them in the variant — the supervisor never reads NVS per tick.
NVS is written exactly once per successful association
(`CaptiveTrying → StaHost`, `CaptiveFallbackRetrying → StaHost`) so
bad creds can't overwrite a known-good pair.

| Variant | Creds in variant? |
|---|---|
| `BootNoCreds` | no (NVS is empty by definition) |
| `CaptiveFailed` | no (last attempt's creds intentionally dropped — captive page is source of truth on retry) |
| `CaptiveTrying`, `CaptiveFallbackRetrying` | **yes** |
| `StaConnecting`, `StaHost`, `StaReassociating` | **yes** |

## Submission status (captive-page UX only)

A separate `SubmissionStatusHandle` (atomic, shared with the HTTP
handler) reports `Idle | Pending | Trying | Failed` on `/status` so the
captive page's spinner / error UI can poll it. The supervisor writes
this atomic at the same moments it transitions: `Pending` on
mailbox-write (from `/save`), `Trying` on `CaptiveSubmitted →
CaptiveTrying`, `Failed` on the 20 s timeout. Successful association
just drops the captive bundle — the page's `/status` poll then errors,
which it treats as success.

## Sketch

```rust
enum NetState {
    BootNoCreds              { wifi: MixedWifi, bundle: CaptiveBundle },
    CaptiveTrying            { wifi: MixedWifi, bundle: CaptiveBundle, creds: Creds, since: Duration },
    CaptiveFailed            { wifi: MixedWifi, bundle: CaptiveBundle },
    CaptiveFallbackRetrying  { wifi: MixedWifi, bundle: CaptiveBundle, creds: Creds },
    StaConnecting            { wifi: StaWifi,   server: HttpServer, creds: Creds, session_start: Duration },
    StaHost                  { wifi: StaWifi,   server: HttpServer, mdns: EspMdns, creds: Creds, last_assoc: Duration },
    StaReassociating         { wifi: StaWifi,   server: HttpServer, mdns: EspMdns, creds: Creds, last_assoc: Duration },
}
```

Each tick: `match` on `state`, run that arm's logic, return the next
`NetState`. The variant *is* the state machine — no separate
`Submission` enum, no shared mutex with HTTP for control state, no
`Option`-as-flag fields. `/save` writes to a single-slot mailbox
(`Arc<Mutex<Option<Creds>>>`) that the supervisor drains during the
relevant captive-arm ticks; latest submission wins.

## Constants

- 20 s — `CaptiveTrying` association budget.
- 2 h — STA-side fallback grace (`StaConnecting` from boot, or
  `StaReassociating` since last associated tick).
- 10 s — captive scan-cache TTL (refreshed by the supervisor on every
  captive-arm tick where STA is not mid-association — states
  `BootNoCreds`, `CaptiveFailed`, `CaptiveFallbackRetrying`).
