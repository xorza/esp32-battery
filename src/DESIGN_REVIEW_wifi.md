# Design review: WiFi stack (`src/wifi.rs`, `src/net.rs`, `src/main.rs` supervisor, `src/captive_api.rs`, `src/http/captive.rs`, `src/dns.rs`, `src/nvs_creds.rs`, `src/wifi_reset.rs`)  (2026-04-25)

## Current design

The stack has two layers. **`Wifi`** (`wifi.rs`) wraps `BlockingWifi<EspWifi>`, owns the radio, and exposes imperative verbs: `start_sta`, `start_ap_mixed`, `set_sta_creds_live`, `tick`, `scan`, plus a free-function `sta_rssi()` that bypasses the struct via raw `esp_wifi_sta_get_ap_info`. mDNS is owned by `Wifi` and lazy-initialised inside `tick` on the first associated tick.

The **supervisor** in `main.rs` is the actual state machine. `Net` (`net.rs`) is a 2-variant enum (`Sta { _server, link_seen }` vs `Captive { bundle }`) where the variant doubles as RAII for the dashboard server / captive bundle. A separate `creds: Option<WifiCredentials>` lives in `main` as a free local; it's read by `Wifi::tick` (passed as `has_sta_creds: bool`), mutated by `drain_submission` when a `/save` lands, and persisted to NVS only on the `Captive→Sta` transition. `LinkSeen { Never { session_start } | At(Duration) }` lives inside `Net::Sta` and is the timestamp the 2h captive-fallback grace and 3s LCD hysteresis both read off. `Submission { Idle | Pending{creds, since} | Trying{since} | Failed }` lives inside `CaptiveBundle.state` (an `Arc<Mutex>` shared with the `/save` handler) and is the captive sub-state machine.

A typical tick locks the wifi mutex once (`Wifi::tick`), and in the captive arm locks the submission mutex once (`drain_submission`). Transitions are driven by the supervisor: `Captive→Sta` happens when `Wifi::tick` reports `is_connected()`; `Sta→Captive` when `now - link_seen.timestamp() ≥ 2h`. The two `start_*_session` helpers in `main.rs` re-build the radio config and either the dashboard server or the captive bundle (HTTP server + DNS thread + state Mutex).

## Canonical trace

User submits creds via the captive portal, STA associates, dashboard comes up:

1. `/save` handler (`captive_api.rs:47`) parses + validates the form, builds `WifiCredentials`, locks `state`, writes `Submission::Pending { creds, since: uptime() }`, returns 200. (one mutex acquire)
2. Next supervisor tick (`main.rs:141`): matches `Net::Captive`, calls `drain_submission` which:
   - locks `state`, `mem::replace(&mut *s, Idle)`, matches taken: `Pending` → locks `wifi`, calls `set_sta_creds_live(&creds)`, writes back `Trying { since }`, sets `*creds = Some(creds)` (two mutex acquires; nested wifi-inside-state)
3. Same tick: `wifi.lock().tick(creds.is_some())` (`main.rs:144`) — `Wifi::tick` calls `is_connected()`, which is false, so it calls `wifi.connect()`, which may succeed, then `wait_netif_up()` (blocking, may take seconds), then lazily inits mDNS. Returns true. (third wifi acquire this tick)
4. Supervisor sees `connected == true`, returns `Step::Promote`.
5. `Step::Promote`: `nvs_creds::save`, then `start_sta_session` reassigns `net = Net::Sta { _server, link_seen: At(now) }`. Reassigning drops the old `Net::Captive`, which drops `CaptiveBundle._server` (HTTP/80 stops), `_dns` (signals stop, joins DNS thread), and `state` (Mutex destroyed).
6. `start_sta_session` locks wifi (4th acquire), calls `wifi.start_sta(creds)` — this internally `stop()`s + `set_configuration(Client)` + `start()`s, tearing down the AP; then mounts a fresh HTTPS dashboard server.
7. `net_status.store(Host)`.

## Overall take

The core shape is right and the recent commits (`net: split tick into decide-then-apply`, `Sta timing fields into LinkSeen`, `defer NVS save until association`) have already pushed it in good directions. The 2-variant `Net` with RAII-by-variant is clean; `Submission` as a typed sub-state machine with `creds` parked only in `Pending` avoids the `Option<WifiCredentials>` stringly-state anti-pattern; `LinkSeen::Never{session_start}` correctly distinguishes "no association yet this session" from "associated then dropped."

The remaining issues are about **responsibility leaks at the `Wifi` boundary** and **one real concurrency hazard** (scan blocks the supervisor). The `Wifi` struct is a thin imperative façade with several supervisor-shaped hooks baked in; pulling those out would let `main`'s state machine express more of the truth and let `Wifi` be a dumb radio-control object.

## Findings

### [F1] `Wifi::scan()` blocks the supervisor tick (and `/save`) for seconds

- **Pillar**: Control flow (cross-module ping-pong via shared lock) + Responsibility (scan is a long-running IO operation, not the radio's API surface)
- **Impact**: 4/5 — a user opening the captive page issues `/scan`, which holds `wifi.lock()` for the duration of `scan_n` (≈2s on c6, longer on c3). During that window, the supervisor's per-second `wifi.lock().tick(...)` blocks, so association attempts pause; the next `/save` is also stuck behind it. A user who refreshes the page once or twice can wedge the connect path for ≥6s.
- **Effort**: 3/5 — supervisor-owned scan worker, single call site change in `captive_api.rs`
- **Current**: `Wifi::scan(&mut self)` (`wifi.rs:174`) called as `wifi.lock().unwrap().scan()` from the `/scan` handler thread (`captive_api.rs:39`). Same `Mutex<Wifi>` is locked every supervisor tick.
- **Problem**: Scan is the only `Wifi` method whose duration is unbounded by the radio state — it competes with the connect path on the same mutex. The mutex is the wrong primitive here: the supervisor wants exclusive control of the radio for control-plane operations; scan is a request that should queue behind them, not race for the same lock.
- **Alternative**: Cache scan results in a `Mutex<(Instant, ScanResult)>` shared between `Wifi` and the captive API. `Wifi::tick` opportunistically refreshes the cache when stale (e.g. >10s old) **only when not associated** — i.e. when scan can't disrupt anything. The `/scan` handler reads the cache without touching the wifi mutex. Trade-off: a freshly-opened captive page sees up-to-10s-stale scan data, which is fine for SSID picking.
  - Alt sketch: `pub struct ScanCache(Arc<Mutex<(Duration, ScanResult)>>);` constructed in `start_captive_session`, passed to both `captive_api::mount` and `Wifi::tick` via a new arg.
- **Recommendation**: Do it. The current design is a latent foot-gun and an active UX problem on slow scans.

### [F2] `creds: Option<WifiCredentials>` is supervisor state but lives outside `Net`

- **Pillar**: Data (placement) + Control flow (ordering trap)
- **Impact**: 3/5 — removes a dangling local that has to be kept in sync with `Net`, and removes the `has_sta_creds: bool` parameter that leaks supervisor state into `Wifi::tick`.
- **Effort**: 2/5 — touches `main.rs` only.
- **Current**: `let mut creds = nvs_creds::load(&nvs);` in `main` (`main.rs:73`), then mutated inside `drain_submission` via `&mut Option<...>` (`main.rs:223`), read into `Step::Promote` via `creds.clone().expect(...)` (`main.rs:171`), and threaded into `Wifi::tick(creds.is_some())` (`main.rs:144,152`).
- **Problem**: The lifecycle of `creds` is completely determined by the `Net` variant: in `Captive` it can be `None` (boot-with-no-creds) or `Some` (Pending was drained, or fell back from STA); in `Sta` it must be `Some`. That invariant lives in the programmer's head. The `expect("captive→sta transition requires creds")` in `Step::Promote` is the visible scar — that path is reachable only because the type doesn't carry the proof. `creds.is_some()` to drive `Wifi::tick` is the same fact reflected back at the radio.
- **Alternative**: Move creds into the variants:
  ```rust
  enum Net {
      Sta {
          _server: EspHttpServer<'static>,
          creds: WifiCredentials,
          link_seen: LinkSeen,
      },
      Captive {
          bundle: CaptiveBundle,
          creds: Option<WifiCredentials>,  // None pre-first-save, Some after a drained Pending
      },
  }
  ```
  `drain_submission` mutates `Captive.creds`. `Step::Promote` carries the `WifiCredentials` it observed (or moves it out of `Captive`). `Wifi::tick`'s parameter goes away — instead the supervisor only calls a connect-path on `Wifi` when `creds.is_some()`, or `Wifi` exposes `try_reconnect(&self)` and the supervisor decides whether to call it.
- **Recommendation**: Do it. This and F3 compose — together they remove `Wifi::tick`'s mixed responsibility.

### [F3] `Wifi::tick` mixes reconnect attempt, mDNS init, and connection probe

- **Pillar**: Responsibility (god-method) + Control flow (hidden side-effect inside a per-tick poll)
- **Impact**: 3/5 — disentangles a function that does three unrelated things and is the only place mDNS gets set up (a fact invisible to readers of `start_sta`).
- **Effort**: 2/5 — pure `wifi.rs` rearrangement plus a one-line supervisor change.
- **Current**: `Wifi::tick(&mut self, has_sta_creds: bool) -> bool` (`wifi.rs:164`):
  ```rust
  if has_sta_creds && !self.is_connected() && self.wifi.connect().is_ok() {
      let _ = self.wifi.wait_netif_up();
      self.setup_mdns();
  }
  self.is_connected()
  ```
  - Probes `is_connected` twice per call.
  - Lazy `setup_mdns()` hidden inside the connect path means "the first thing readers of `start_sta` notice missing — mDNS — is quietly initialised on a successful tick." Load-bearing ordering not visible at construction.
  - The `wait_netif_up()` blocks the supervisor mutex for an unbounded time inside what is supposed to be a per-second poll.
- **Problem**: A reader scanning `start_sta` cannot tell that the device will ever resolve `battery-esp32.local`. Coupling the reconnect path to the mDNS setup means we cannot reconnect without re-trying mDNS init (cheap, but the responsibility leak is the point).
- **Alternative**: Split into:
  - `Wifi::is_connected(&self) -> bool` (already exists)
  - `Wifi::try_connect(&mut self) -> bool` — single attempt, returns post-attempt `is_connected`. No mDNS, no `wait_netif_up` (or move `wait_netif_up` into `start_sta` where it belongs alongside the initial bring-up).
  - mDNS owned by the supervisor: `start_sta_session` constructs and stores an `EspMdns` next to `_server`, drops it on transition. `mdns: Option<EspMdns>` field disappears entirely.
- **Recommendation**: Do it. mDNS-into-`Net::Sta` matches the existing "RAII by variant" pattern.

### [F4] Two paths build the AP config; `set_sta_creds_live` silently assumes Mixed mode

- **Pillar**: Data (single source of truth) + Responsibility (hidden mode invariant)
- **Impact**: 2/5 — fixes a duplicated literal and removes a foot-gun nobody is currently triggering.
- **Effort**: 1/5 — local refactor.
- **Current**: `AccessPointConfiguration { ssid: AP_SSID, password: AP_PASS, auth_method: WPA2Personal, channel: 1, max_connections: 4, .. }` is built in `set_sta_creds_live` (`wifi.rs:118`) and again in `start_ap_mixed` (`wifi.rs:141`). `set_sta_creds_live` writes `Configuration::Mixed(sta, ap)` unconditionally — if it were ever called from `Net::Sta` (STA-only) mode, the radio would silently switch to Mixed mode and bring the AP up, dropping the dashboard's TLS sessions.
- **Problem**: Two literal copies that must stay in sync. A type-system invariant ("only call `set_sta_creds_live` while AP is up") encoded only by call-site discipline.
- **Alternative**:
  - Extract `fn ap_config() -> AccessPointConfiguration` once.
  - Either narrow `set_sta_creds_live` to take only the new `ClientConfiguration` and `set_configuration(Mixed(sta, ap_config()))` from one place, or — stronger — split `Wifi` into `WifiSta` and `WifiCaptive` typestates so the supervisor can only call live-update from the captive type. (Probably overkill given `Net` already encodes this; the in-`Wifi` enforcement is belt-and-suspenders.)
- **Recommendation**: Do the dedupe. Skip the typestate split unless F2/F3 land first — at that point the supervisor is the single caller anyway.

### [F5] `NetStatusHandle` reinvents enum<->u8 round-trip

- **Pillar**: Data (representation)
- **Impact**: 1/5 — cosmetic; one fewer place to update when adding a status.
- **Effort**: 1/5
- **Current**: `NetStatusHandle::load` (`net.rs:85`) hand-writes a `match self.0.load(...) { 0 => ..., 1 => ..., 2 => ..., 3 => ..., v => unreachable!(...) }`. The `unreachable!` arm is genuinely unreachable today, but only because all writers go through `store(NetStatus)`.
- **Problem**: The duplication between `#[repr(u8)] enum NetStatus` and the manual match is mechanical. Adding a 5th status requires editing two places.
- **Alternative**: Add `#[derive(strum::FromRepr)]` (already pulling `strum` for `IntoStaticStr` on `Submission`) and write `NetStatus::from_repr(v).unwrap()`. Or just `unsafe { std::mem::transmute(v) }` since the repr is fixed and writers are constrained.
- **Recommendation**: Do it via `FromRepr` if the derive is already in scope from another use. Otherwise leave it — the cost is real but tiny.

## Considered and rejected

- **Free function `wifi::sta_rssi()` bypassing `Wifi`** — looked like a leaky abstraction at first, but `esp_wifi_sta_get_ap_info` is thread-safe IDF state and the alternative (locking `Mutex<Wifi>` from `/api`) would couple dashboard latency to the supervisor mutex unnecessarily. Keep as a free fn.
- **Splitting `Wifi` into `WifiCaptive` / `WifiSta` typestates** — would make F4's mode-mismatch impossible, but requires re-creating the wrapper across every transition and complicates the `Arc<Mutex<Wifi>>` shared with the captive API. Not worth it once F2+F3 land.
- **Promoting `Submission` `Idle/Failed` into separate top-level captive sub-states** — `Submission` already does the right thing; the four-variant shape with `creds` only in `Pending` is exactly "make illegal states unrepresentable" applied correctly.
- **Replacing `LinkSeen` with `Option<Duration>` + a separate `session_start: Duration`** — equivalent information but loses the encoded "have we ever associated this session" question that `for_sta`'s LCD hysteresis depends on. The current named enum is clearer.
