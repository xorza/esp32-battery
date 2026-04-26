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
| 2 | `CaptiveSubmitted` | Mixed | captive | freshly submitted, not yet applied | `CaptiveTrying` |
| 3 | `CaptiveTrying` | Mixed | captive | applied, association in flight (≤ 20 s) | `CaptiveTrying` |
| 4 | `CaptiveFailed` | Mixed | captive | last attempt failed | `Captive` (with error) |
| 5 | `CaptiveFallbackRetrying` | Mixed | captive | known-good creds, STA half retrying in background | `Captive` |
| 6 | `StaConnecting` | STA-only | dashboard | known, never associated this session | `Connecting` |
| 7 | `StaHost` | STA-only | dashboard + mDNS | known, associated | `Host` |
| 8 | `StaReassociating` | STA-only | dashboard + mDNS | known, link briefly dropped | `Connecting` |

## Transitions

Triggers:
- **boot**
- **/save** — user submits creds in captive page
- **tick:assoc** — supervisor sees `is_connected() == true`
- **tick:!assoc** — supervisor sees `is_connected() == false`
- **timeout:20s** — `CaptiveTrying` association budget expired
- **timeout:2h** — STA-side fallback grace expired

| From | Trigger | To | Action |
|---|---|---|---|
| (start) | boot, no NVS creds | `BootNoCreds` | start Mixed, mount captive |
| (start) | boot, NVS creds present | `StaConnecting` | start STA-only with creds, start dashboard |
| `BootNoCreds` | /save | `CaptiveSubmitted` | park creds |
| `CaptiveFailed` | /save | `CaptiveSubmitted` | park creds |
| `CaptiveFallbackRetrying` | /save | `CaptiveSubmitted` | park creds (overrides carry-over) |
| `CaptiveTrying` | /save | `CaptiveSubmitted` | park new creds (overrides in-flight attempt) |
| `CaptiveSubmitted` | tick | `CaptiveTrying` | apply creds to radio, status → Trying, 20 s deadline runs from /save |
| `CaptiveTrying` | tick:assoc | `StaHost` | persist creds to NVS, drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `CaptiveTrying` | timeout:20s | `CaptiveFailed` | status → Failed, captive page shows error |
| `CaptiveFallbackRetrying` | tick:assoc | `StaHost` | drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `StaConnecting` | tick:assoc | `StaHost` | take mDNS |
| `StaConnecting` | timeout:2h | `CaptiveFallbackRetrying` | drop dashboard, switch radio to Mixed (carry creds), mount captive |
| `StaHost` | tick:!assoc | `StaReassociating` | servers stay up |
| `StaReassociating` | tick:assoc | `StaHost` | bump `last_assoc` |
| `StaReassociating` | timeout:2h since last assoc | `CaptiveFallbackRetrying` | drop dashboard + mDNS, switch radio to Mixed (carry creds), mount captive |

States 1/4/5 and 6/8 with no listed trigger simply stay put.

## Credential ownership

Credentials live in **NVS** (`nvs_creds::{load, save, clear}`) and on
the **radio config** (set via `MixedWifi::set_sta_creds` /
`into_sta`). The FSM only carries an in-memory copy during the
*un-persisted window* — between `/save` and the success that writes
them to NVS. Outside that window, the FSM reads from NVS or the radio,
not from a variant field.

| Variant | Creds in variant? | Source of truth |
|---|---|---|
| `BootNoCreds`, `CaptiveFailed` | no | — |
| `CaptiveSubmitted`, `CaptiveTrying` | **yes** | variant (not yet in NVS) |
| `CaptiveFallbackRetrying` | no | NVS + radio |
| `StaConnecting`, `StaHost`, `StaReassociating` | no | NVS + radio |

On `CaptiveTrying → StaHost`: take the creds out of the variant,
`nvs_creds::save(...)`, then construct `StaHost` without them.

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
    CaptiveSubmitted         { wifi: MixedWifi, bundle: CaptiveBundle, creds: Creds, since: Duration },
    CaptiveTrying            { wifi: MixedWifi, bundle: CaptiveBundle, creds: Creds, since: Duration },
    CaptiveFailed            { wifi: MixedWifi, bundle: CaptiveBundle },
    CaptiveFallbackRetrying  { wifi: MixedWifi, bundle: CaptiveBundle },
    StaConnecting            { wifi: StaWifi,   server: HttpServer, session_start: Duration },
    StaHost                  { wifi: StaWifi,   server: HttpServer, mdns: EspMdns, last_assoc: Duration },
    StaReassociating         { wifi: StaWifi,   server: HttpServer, mdns: EspMdns, last_assoc: Duration },
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
  captive-arm tick where STA is not mid-association — states 1, 4, 5).
