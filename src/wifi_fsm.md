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
| 2 | `CaptiveAwaitingUser` | Mixed | captive | none | `Captive` |
| 3 | `CaptiveSubmitted` | Mixed | captive | freshly submitted, not yet applied | `CaptiveTrying` |
| 4 | `CaptiveTrying` | Mixed | captive | applied, association in flight (≤ 20 s) | `CaptiveTrying` |
| 5 | `CaptiveFailed` | Mixed | captive | last attempt failed | `Captive` (with error) |
| 6 | `CaptiveFallbackRetrying` | Mixed | captive | known-good creds, STA half retrying in background | `Captive` |
| 7 | `StaConnecting` | STA-only | none yet | known, never associated this session | `Connecting` |
| 8 | `StaHost` | STA-only | dashboard + mDNS | known, associated | `Host` |
| 9 | `StaReassociating` | STA-only | dashboard + mDNS | known, link briefly dropped | `Host` (≤ 3 s hysteresis) → `Connecting` |

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
| (start) | boot, NVS creds present | `StaConnecting` | start STA-only with creds |
| `BootNoCreds` | /save | `CaptiveSubmitted` | park creds |
| `CaptiveAwaitingUser` | /save | `CaptiveSubmitted` | park creds |
| `CaptiveFailed` | /save | `CaptiveSubmitted` | park creds |
| `CaptiveFallbackRetrying` | /save | `CaptiveSubmitted` | park creds (overrides carry-over) |
| `CaptiveSubmitted` | tick (drain) | `CaptiveTrying` | apply creds to radio, start 20 s deadline |
| `CaptiveTrying` | tick:assoc | `StaHost` | persist creds to NVS, drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `CaptiveTrying` | timeout:20s | `CaptiveFailed` | show error on captive page |
| `CaptiveFallbackRetrying` | tick:assoc | `StaHost` | drop captive bundle, switch radio to STA-only, start dashboard + mDNS |
| `StaConnecting` | tick:assoc | `StaHost` | take mDNS, expose dashboard |
| `StaConnecting` | timeout:2h | `CaptiveFallbackRetrying` | drop dashboard, switch radio to Mixed (carry creds), mount captive |
| `StaHost` | tick:!assoc | `StaReassociating` | servers stay up, start hysteresis timer |
| `StaReassociating` | tick:assoc | `StaHost` | — |
| `StaReassociating` | timeout:2h since last assoc | `CaptiveFallbackRetrying` | drop dashboard, switch radio to Mixed (carry creds), mount captive |

States 1/2/5/6 and 7/9 with no listed trigger simply stay put.

## Credential ownership

Credentials live in **NVS** (`nvs_creds::{load, save, clear}`) and on
the **radio config** (set via `MixedWifi::set_sta_creds` /
`into_sta`). The FSM only carries an in-memory copy during the
*un-persisted window* — between `/save` and the success that writes
them to NVS. Outside that window, the FSM reads from NVS or the radio,
not from a variant field.

| Variant | Creds in variant? | Source of truth |
|---|---|---|
| `BootNoCreds`, `CaptiveAwaitingUser`, `CaptiveFailed` | no | — |
| `CaptiveSubmitted`, `CaptiveTrying` | **yes** | variant (not yet in NVS) |
| `CaptiveFallbackRetrying` | no | NVS + radio |
| `StaConnecting`, `StaHost`, `StaReassociating` | no | NVS + radio |

On `CaptiveTrying → StaHost`: take the creds out of the variant,
`nvs_creds::save(...)`, then construct `StaHost` without them.

## Sketch

```rust
enum NetState {
    BootNoCreds              { wifi: MixedWifi, bundle: CaptiveBundle },
    CaptiveAwaitingUser      { wifi: MixedWifi, bundle: CaptiveBundle },
    CaptiveSubmitted         { wifi: MixedWifi, bundle: CaptiveBundle, creds: Creds, since: Duration },
    CaptiveTrying            { wifi: MixedWifi, bundle: CaptiveBundle, creds: Creds, since: Duration },
    CaptiveFailed            { wifi: MixedWifi, bundle: CaptiveBundle },
    CaptiveFallbackRetrying  { wifi: MixedWifi, bundle: CaptiveBundle, since: Duration },
    StaConnecting            { wifi: StaWifi,   session_start: Duration },
    StaHost                  { wifi: StaWifi,   server: HttpServer, mdns: EspMdns, last_assoc: Duration },
    StaReassociating         { wifi: StaWifi,   server: HttpServer, mdns: EspMdns, last_assoc: Duration },
}
```

Each tick: `match` on `state`, run that arm's logic, return the next
`NetState`. The variant *is* the state machine — no separate
`Submission` enum, no shared mutex with HTTP, no `Option`-as-flag
fields. `/save` writes to an MPSC channel; the supervisor drains it
during the relevant captive-arm ticks.

## Constants

- 20 s — `CaptiveTrying` association budget.
- 2 h — STA-side fallback grace (`StaConnecting` from boot, or
  `StaReassociating` since last associated tick).
- 3 s — LCD hysteresis: `StaReassociating` still reads as `Host` for
  this long before flipping to `Connecting`.
- 10 s — captive scan-cache TTL (refreshed by supervisor while in any
  captive state where STA is not mid-association — i.e. states 1, 2,
  5, 6).
