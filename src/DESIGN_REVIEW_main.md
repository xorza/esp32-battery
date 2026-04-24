# Design review: src/main.rs  (2026-04-24)

## Current design

`main()` does three things in order: (1) build the singleton resources (board peripherals, NVS, clock, history store, sensor data, wifi handle), (2) start the long-lived producer threads (xy, ina, optionally lcd) and SNTP, (3) drive a 1 Hz event loop that owns two responsibilities — flushing pending NVS history saves, and reconciling the HTTP server flavor against current WiFi state.

The reconciliation is expressed as a `match (&server, connected)` over a `Server` enum (`Main` | `Captive` | `None`). `Server::None` is both the initial value and a transient value used to bridge a Captive→Main transition across two loop iterations. The variants of `Server` exist to pin `EspHttpServer` and `DnsHandle` for `Drop`; `#[allow(dead_code)]` silences the unread-fields warning. Credentials loaded from NVS at startup are kept in a local `creds: Option<WifiCredentials>` and re-used when restarting STA after a captive→connected transition; the captive portal itself updates credentials by writing NVS and rebooting, so this local snapshot is never stale at runtime.

History persistence is split: producer threads (`update_ps`, `update_battery`) call `try_commit` internally, which arms a save flag. The main loop polls `take_save_payload()` once a second under the sensor lock, drops the lock, and writes to NVS — keeping the slow flash erase/write off the producers' critical path.

## Overall take

The shape is mostly right for a single-purpose embedded app: linear startup, one event loop, one shared `Arc<AppState>`. The split between fast producers and slow NVS writer is a real, load-bearing decision and it's correct. The weak spot is the `match (&server, connected)` reconciliation — it conflates "what's running" with "what should be running", uses `Server::None` as a transient state the type doesn't actually model, and depends on a non-local invariant (captive always reboots) for one branch's correctness. Tightening that area is high value, low risk.

## Findings

### [F1] `Server::None` doubles as initial state and as a one-tick transient

- **Category**: State / Control flow
- **Impact**: 3/5 — removes a non-obvious two-tick transition path and one always-dead branch
- **Effort**: 2/5 — local refactor inside `main()`
- **Current**: `Server::None` is set initially (main.rs:125) and again inside `(Captive, true)` (main.rs:157) so that the next loop tick's `(None, true)` arm starts the main server (main.rs:147). The Captive→Main transition therefore takes ≥2 ticks (~2 s) and routes through a state that the enum's variant name doesn't suggest is reachable at runtime.
- **Problem**: A reader has to simulate two iterations to see how Captive becomes Main. The state enum claims three states, but `None` post-init only ever lives between two consecutive `match` evaluations — it's a hidden coroutine yield. The branch in `(Captive, true)` also restarts STA on a wifi handle that's already connected (the very condition that took us into this arm), and immediately tears down the captive server while the user's browser may still be loading "OK" from `/save` (mitigated only because captive reboots, see F2).
- **Alternative**: Remove `Server::None` entirely. Use `Option<Server>` for the initial nil, then in `(Captive, true)` build the main server and replace in place: `server = Some(Server::Main(http::start_main(...)))` — drop of the old `Captive(_, _)` runs first, captive is gone within one tick, no `start_sta` re-call needed (wifi is already connected). The arm becomes symmetric with `(None|Main, false)`.
- **Recommendation**: Do it.

### [F2] Captive→Main arm depends on a non-local "captive always reboots" invariant

- **Category**: Contract
- **Impact**: 3/5 — removes a latent panic if captive flow ever changes
- **Effort**: 2/5 — delete the dead `start_sta` re-call once F1 lands, or pull credentials from `Wifi`
- **Current**: `(Captive, true)` does `let creds = creds.as_ref().expect("connected requires credentials"); wifi.lock().unwrap().start_sta(creds);` (main.rs:158–159). The `creds` binding is the boot-time NVS snapshot (main.rs:87) — never updated. Today this only fires if a previously-saved network drops and recovers without a reboot, which guarantees `creds` is `Some`. If the captive `/save` handler ever stopped rebooting (http/captive.rs:95) and instead asked the runtime to switch to STA, `creds` would still be `None` from the cold-boot load and the `expect` would panic.
- **Problem**: Load-bearing comment territory: the safety of the `expect` is established two files away by an unrelated design choice. `Wifi` already owns its STA configuration (`self.sta_configured`, wifi.rs:104) — main.rs is caching credentials it doesn't own.
- **Alternative**: Two options.
  1. **Pair with F1**: once captive→main no longer restarts STA, the `start_sta` call (and the `expect`) just go away.
  2. **Move the source of truth into `Wifi`**: expose `Wifi::reconnect_stored()` that re-uses whatever STA config is already programmed. Drop the `creds` parameter on `start_ap_mixed` too if it can read its own state.
- **Recommendation**: Do option 1 along with F1; option 2 is the right shape but a bigger refactor and only worth it if other callers grow.

### [F3] `Server` enum is really "currently held resources", not "what kind of server"

- **Category**: Types
- **Impact**: 2/5 — clarifies intent, no behavior change
- **Effort**: 1/5 — rename + comment
- **Current**: The enum carries `EspHttpServer` and `DnsHandle` purely so their `Drop` runs at the right time; nothing reads the fields. `#[allow(dead_code)]` silences the warning (main.rs:37). The variant names suggest a polymorphic server but no method dispatches on the variant.
- **Problem**: A reader looking for "what does `Main` do differently from `Captive`" finds nothing — the variants don't model behavior, they model a Drop guard with two shapes. The `#[allow(dead_code)]` is a flag that the type isn't carrying its weight.
- **Alternative**: Rename to `ActiveServer` (or `RunningHttp`) and add a one-line doc: "Held purely for `Drop`; the variant determines whether captive DNS is also torn down." Or, more aggressive: collapse to `struct ActiveServer { http: EspHttpServer<'static>, dns: Option<DnsHandle> }` and drop the enum entirely — the `Option<DnsHandle>` already encodes "is this captive-mode".
- **Recommendation**: Do the struct collapse. The enum's two variants don't earn their keep once F1 removes `None`.

### [F4] History buffer sizing leaks serialization details into `main`

- **Category**: Abstraction
- **Impact**: 2/5 — small, but the magic number is a footgun if the on-flash format grows
- **Effort**: 1/5 — add a const in `logic/src/data.rs`, import here
- **Current**: `vec![0u8; HISTORY_CAPACITY * 32 + 64]` (main.rs:101). The `* 32` is "bytes per sample including timestamp + ps + bat", `+ 64` is header overhead — both numbers live in `data.rs` but the buffer sizing lives here. If the sample format gains a field, this allocation is silently undersized and `history_store.load` truncates.
- **Problem**: The caller has to know the producer's serialization layout to size a buffer for it. Classic leaky abstraction.
- **Alternative**: Export `pub const SERIALIZED_MAX_BYTES: usize = HISTORY_CAPACITY * SAMPLE_BYTES + HEADER_BYTES;` from `logic::data`, and have `main` write `vec![0u8; SensorData::SERIALIZED_MAX_BYTES]`. Better still, give `HistoryStore::load` a signature that returns `Vec<u8>` and owns the sizing entirely.
- **Recommendation**: Do the const. The owned-`Vec` version is nicer but `HistoryStore` lives in `platform.rs` and would need to depend on `logic` — only worth it if you don't already.

### [F5] `reboot_after` lives in `main.rs` but is called from captive

- **Category**: Responsibility
- **Impact**: 1/5 — cosmetic
- **Effort**: 1/5 — move + update one import
- **Current**: `pub fn reboot_after` (main.rs:26) is called as `crate::reboot_after("Rebooting after WiFi setup")` (http/captive.rs:95). It's the only public symbol from `main.rs` other than `AppState` (re-exported) and `uptime_s`.
- **Problem**: `main.rs` is the entry point, not a utility module. Public helpers that aren't entry-related muddy that role.
- **Alternative**: Move it next to `uptime_s` in `app_state.rs`, or into `platform.rs` next to other ESP-IDF wrappers (`esp_idf_svc::hal::reset::restart` is the body).
- **Recommendation**: Do it; trivial and improves the read of `main.rs`.

## Considered and rejected

- **Replace the 1 Hz polled save with an event-driven channel** (producer signals "save due", main loop wakes). Would remove up-to-1 s save latency. Rejected because the existing 1 Hz tick is also driving WiFi reconciliation — adding a second wake source costs more complexity than the latency saves, and NVS write cost (50–100 ms) dwarfs the 1 s polling jitter on user-visible "did my history persist" timelines.
- **Move WiFi reconciliation into a dedicated thread.** The main loop's job is small enough that splitting it just adds a channel and a second lock-acquisition pattern without removing anything.
