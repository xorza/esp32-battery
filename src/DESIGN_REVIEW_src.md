## Current design

The `src/` tree is the ESP-IDF–facing half of the firmware: hardware adapters (`board`, `ina`, `xy`, `lcd`), network plumbing (`wifi`, `dns`, `http/`, `ota`, `nvs_creds`), and the main-thread supervisor (`main`, `app_state`). Pure protocol/algorithm code (history aggregation, charge controller, modbus framing, form parsing, OCV→SoC, save scheduler) is properly hived off into the `esp32_battery_logic` crate, which is the only host-testable piece. Worker threads exchange data through `Arc<Shared>`, which carries `Mutex<SensorData>`, `Mutex<Option<WifiCredentials>>`, and an `AtomicU8` net-status byte.

The main loop owns one `AppState` value with a `NetPhase` state machine (`Idle | Connecting{ ticks, host_server: Option } | Host{ server } | Captive{ server, dns }`) plus a flat `NetStatus` enum that's a derived projection written into the atomic on every transition. Net phase transitions are driven by 1 Hz polls of `wifi.is_connected()`: a disconnect from `Host` enters `Connecting` with the live `EspHttpServer` held in an `Option` so a brief WiFi blip doesn't tear down HTTPS state; after `CAPTIVE_AFTER_FAILURES` ticks the server drops and the captive AP + DNS responder come up. Sensor producers (ina, xy) push `Ina228Reading`/`PsReading` into `SensorData`; the main loop ticks `SensorData::tick(now)` once per second and asks `SaveScheduler` whether to flush the serialized blob to NVS — serialization happens under the data mutex, NVS I/O is released outside it.

The `register`-style pattern for HTTP routes (`api::register`, `log_ring::register`, `ota::register`, `nvs_creds::register_reset`) keeps each feature module in charge of its own endpoint(s); `http::main_server` and `http::captive` just wire them together and own server creation. Hardware-absent variants (`xy-fake`, `ina-fake`) are cfg-gated submodules that swap the entire `start()` body, not test seams.

## Overall take

The split between the pure logic crate and the ESP-side `src/` is the right cleavage and is mostly clean. Two things are genuinely worth fixing: (1) the `lcd` thread clones the whole history under the sensor mutex — exactly the pattern `api.rs` was just refactored away from in 96f08d1, so it's now an inconsistency rather than an oversight; and (2) the `NetPhase`/`NetStatus` pair carries a derived shadow updated by manual discipline in `set_phase`, and `Connecting{ host_server: Option }` encodes "Host that's about to die" awkwardly enough that the variant could be folded back into `Host`. Beyond that, several pure helpers (DNS framing, log ring, ina averaging) live in `src/` with no tests when they could trivially be in the logic crate or have an inline test module.

## Findings

### [F1] LCD clones the entire history under the sensor-data mutex

- **Category**: Data structures / Control flow
- **Impact**: 3/5 — undoes the lock-hold-time win from 96f08d1 and allocates ~4 KB on every 500 ms tick
- **Effort**: 2/5 — same shape as `api::HistoryView`; or snapshot into a pre-allocated thread-local
- **Current**: `lcd::start`'s render loop locks `sensor_data`, then collects the entire `history()` into a fresh `heapless::Vec<(u32,f32,f32,f32,f32), HISTORY_CAPACITY>` while still holding the lock (lcd.rs:481–498). The lock guard is then dropped and rendering happens against the cloned `Vec`.
- **Problem**: This is the exact pattern `api.rs` was refactored away from in commit 96f08d1 ("serialize history through a borrow, drop Vec<HistoryRow> alloc"). The producer threads (`ina`, `xy`) stall on the mutex for the duration of an N-element copy + heapless-Vec push loop on every LCD frame, twice per second. It also burns a copy of ~200 samples × 20 bytes per render. The two HTTP-side and LCD-side patterns now disagree on how to read history.
- **Alternative**: Either (a) render the graph under the lock by passing `&[Sample]` directly into `draw_graph` — the per-frame draw is a few ms and the producer threads update once per second/100 ms anyway; or (b) keep the lock-then-snapshot split but reuse a `Box<[Sample; HISTORY_CAPACITY]>` field on the LCD thread so no allocation happens. Option (a) matches `api.rs` and is simpler; option (b) keeps the mutex hold time minimal at the cost of one extra field.
- **Recommendation**: Do (a). Consistency with `api.rs` matters more than shaving sub-millisecond lock holds, and the LCD is the only consumer of this snapshot.

### [F2] `NetPhase::Connecting { host_server: Option<…> }` should fold back into `Host`

- **Category**: Data structures / State
- **Impact**: 3/5 — removes a redundant variant, an `Option` that's nearly always `Some`, and a derived enum
- **Effort**: 3/5 — touches `app_state` plus the two callers in `main`
- **Current**: `NetPhase` has four variants (`Idle`, `Connecting{ ticks, host_server: Option<EspHttpServer> }`, `Host{ server }`, `Captive{ … }`) plus a parallel `NetStatus` enum with three variants (`Captive | Connecting | Host`) projected through `NetPhase::status()` and stored in `Shared.status: AtomicU8` (app_state.rs:21–28, 73–80). Every `set_phase` call rewrites the atomic. `Connecting.host_server` is `Some` whenever we entered from `Host` (the grace-window case the design exists for) and `None` only on the `on_creds_applied` path that explicitly drops it.
- **Problem**: The `Option<EspHttpServer>` encodes "is there a server still mounted" inside what's already a state-machine variant — three states (Connecting-with-server / Connecting-no-server / Host) that could be two. `NetStatus` is a hand-maintained projection: any new `set_phase` site has to remember to also write the atomic, and the three-vs-four mapping (`Idle | Connecting → Connecting`) is implicit in `NetPhase::status()`. The grace window is the load-bearing reason the variant exists, and it'd be more honest to keep the server visibly in `Host` until it's actually torn down.
- **Alternative**: Collapse to three variants:
  ```
  enum NetPhase {
      Booting,                                                  // no server
      Host { server: EspHttpServer<'static>, grace: Option<u32> }, // grace=Some means "disconnected for N ticks, will tear down at threshold"
      Captive { server, dns },
  }
  ```
  `on_tick_disconnected` increments `grace` in place; once it hits the threshold, drop and transition to `Captive`. `on_tick_connected` just clears `grace` (reassociation reuses the existing server — no `mem::replace` dance needed). `NetStatus` derivation: `Booting | Host{grace: Some(_)} → Connecting`, `Host{grace: None} → Host`, `Captive → Captive`. The `Option<EspHttpServer>` goes away because the server is unconditionally present in `Host`. Same `AtomicU8` projection still fits, but the rule "every state transition must remember to write the atomic" can be enforced by routing all writes through one helper (already `set_phase` — but the new shape lets `grace` mutate without touching the atomic, since status doesn't change while grace is `Some(_)`).
- **Recommendation**: Do it. The current four-variants-with-Option shape required `#[allow(dead_code)]` on three fields and a comment explaining the optionality — both signs the model isn't quite the right shape.

### [F3] Pure helpers stranded in `src/` with no host tests

- **Category**: Testability / Responsibility
- **Impact**: 3/5 — three correctness-load-bearing pieces have zero tests because they live next to ESP-IDF imports
- **Effort**: 2/5 — extract or just add tests in the existing files (the project already runs `cargo nextest` for the logic crate)
- **Current**: Three pieces of subtle pure logic live in `src/` files that can't be compiled host-side because the surrounding module imports `esp_idf_svc` / `esp_idf_hal`:
  - `log_ring::Ring` (log_ring.rs:36–88): wrap-around ring buffer with an oversized-input branch and a snapshot that re-orders post-wrap. No tests.
  - `dns.rs` (dns.rs:33–105): DNS query parser + A-record response builder. The label walk, qtype extraction, and response assembly are all hand-rolled byte math, exercised only by live captive-portal traffic.
  - `ina::ReadingAccum` (ina.rs:27–50): tiny sample averager. Trivial but still untested, and adding `f64`-accumulator math without a test is exactly how rounding bugs creep in.
- **Problem**: All three are pure functions of `&[u8]` / `&Reading` and have nothing to do with ESP-IDF. They sit in modules that pull in HAL types because a thread loop or HTTP handler is registered alongside them. This is the same separation the project already applied successfully to `modbus`, `form`, `battery`, `charge_strategy`, etc. — the refactor was done for the harder cases and skipped on the easier ones.
- **Alternative**: Either move each into the logic crate (`logic/src/dns_packet.rs`, `logic/src/log_ring.rs`, drop `ReadingAccum` into `logic/src/data.rs` next to `SampleAccum`) or split each `src/` file in two — `src/dns.rs` keeps the UDP loop, calls into `esp32_battery_logic::dns_packet::{parse_query, build_a_response}`. Tests live in the logic crate and run on host. `ReadingAccum` is trivial enough that an inline `#[cfg(test)] mod tests` in the same file is fine if you don't want to move it (current rules forbid `#[cfg(test)]` on production *fns* but not on a tests module).
- **Recommendation**: Do `dns` and `log_ring` — both have non-trivial branching and are reachable from network input. `ReadingAccum` is borderline; either move it or delete it (a 10-line inline accumulator in `start()` would lose nothing).

### [F4] `Shared` is three unrelated mailboxes glued into one Arc

- **Category**: Data structures / Responsibility
- **Impact**: 2/5 — small ergonomic + clarity win; no current bug
- **Effort**: 2/5 — change worker `start(pins, shared)` signatures to take only what they need
- **Current**: `Shared` (app_state.rs:81–88) holds `sensor_data: Mutex<SensorData>`, `pending_creds: Mutex<Option<WifiCredentials>>`, and `status: AtomicU8`. Producers (`ina`, `xy`) only touch `sensor_data`. The captive `/save` handler only writes `pending_creds`. The LCD reads `sensor_data` and `status`. Nothing reads all three.
- **Problem**: Every worker carries an `Arc` to the union of state, exposing access to fields it has no business touching, and growing `Shared` grows the surface for every consumer. The atomic is also bolted into the cross-thread struct purely so the LCD can read it; conceptually it belongs to whatever publishes phase transitions (`AppState`).
- **Alternative**: Keep an `Arc<Mutex<SensorData>>` as its own field; pass *that* into ina/xy/lcd/api. Make `pending_creds` a `Arc<Mutex<Option<…>>>` (or a `mpsc::SyncSender<WifiCredentials>` with capacity 1) plumbed only into the captive handler. Make `status` an `Arc<AtomicU8>` plumbed only into the LCD. `AppState` holds clones of each. The `Shared` aggregate goes away; `start_captive` and `start_main` take exactly the fields they need.
- **Recommendation**: Do it when you next touch worker startup. Not worth a dedicated PR.

### [F5] `NetStatus` shadow couples every phase write to a manual atomic store

- **Category**: Contract
- **Impact**: 2/5 — invariant maintained by discipline in one helper today; new code paths are easy to forget
- **Effort**: 1/5 — already routed through `set_phase`; just delete the field and have LCD compute on demand
- **Current**: `Shared.status: AtomicU8` is rewritten in `AppState::set_phase` (app_state.rs:115–119); LCD reads it via `shared.status()` once per frame to decide which overlay to paint (lcd.rs:556).
- **Problem**: The atomic exists because `NetPhase` lives on the main thread (`EspHttpServer` is `!Send`) and the LCD can't read it directly. But if the only cross-thread question is "what should I draw?", a tiny enum-shaped cell can be the source of truth instead of a derived projection. Today it works because every phase write goes through one helper; one missed call site silently desyncs the LCD.
- **Alternative**: Make `Arc<AtomicU8>` (typed `NetStatusCell`) the *primary* state for the cross-thread question. `AppState` writes it directly when transitioning; the `NetPhase::status()` projection becomes redundant once F2 collapses the variants. Or, more aggressively: replace the AtomicU8 with `Arc<Mutex<NetStatus>>` and lose the `from_u8`/`unreachable!` round-trip — LCD reads it 2 Hz, contention is non-existent.
- **Recommendation**: Defer until F2 lands; the `NetStatus` shadow is the cleanest part of the current shape. After F2, drop the projection function and just write the atomic directly at the two transition sites.

### [F6] `nvs_creds` and `HistoryStore` use NVS through different shapes

- **Category**: Abstraction
- **Impact**: 2/5 — small consistency issue
- **Effort**: 1/5 — wrap one or unwrap the other
- **Current**: `HistoryStore` wraps `EspNvs` in a `Mutex` and exposes `save(&self, &[u8])` / `load(&self, &mut [u8])` (history_store.rs:11–43). `nvs_creds` exposes the raw `EspNvs<NvsDefault>` (nvs_creds.rs:18–20) and a free-function API (`load`, `save`, `clear`) the captive portal calls directly with `&nvs`. The HTTP `/wifi-reset` handler closure captures `Arc<EspNvs<NvsDefault>>` without a mutex.
- **Problem**: Two NVS namespaces, two access patterns. `EspNvs::set_str` happens to take `&self`, so the unwrapped shape works; but the precedent set by `HistoryStore` says NVS access should go through a small typed wrapper. New NVS-backed state will have to pick a side.
- **Alternative**: Move `nvs_creds` behind a `WifiCredsStore` struct mirroring `HistoryStore` (constructor takes the partition, methods are `load() -> Option<Creds>`, `save(&Creds)`, `clear()`, `register_reset_handler(&mut server)`). The captive portal stops poking NVS directly; the route becomes "captive writes pending_creds, main loop persists via the store." Or (smaller change): drop the `Mutex` from `HistoryStore` and make `save`/`load` take `&mut self` — only main calls them.
- **Recommendation**: Pick one direction; either is fine. The wrapper version is the stronger move because it eliminates the captive handler's direct NVS coupling.

### [F7] `uptime_s` lives in `app_state`

- **Category**: Responsibility
- **Impact**: 1/5 — pure organization
- **Effort**: 1/5 — move + update one re-export
- **Current**: `pub fn uptime_s() -> u32` is a free function at the bottom of `app_state.rs:208–210`, re-exported through `main.rs` as `pub use app_state::uptime_s` and called from `api.rs` and `lcd.rs`.
- **Problem**: It has nothing to do with `AppState`. It's a wrapper around `esp_timer_get_time`. The re-export hides the misplacement.
- **Alternative**: Move it into `clock.rs` (which is already the wall-clock module), or a new tiny `time.rs`. Drop the re-export.
- **Recommendation**: Do it next time you touch `clock.rs`.

## Considered and rejected

- **Unify the `*::register(server, …)` pattern behind a trait.** Each module's `register` has a different signature (some take `Shared`, some take `Arc<EspNvs>`, `log_ring` takes nothing). A trait would force one signature with `Box<dyn Any>` parameters or per-module wrapper types. The current pattern is just a convention; it reads fine and `main_server`/`captive` make the wiring obvious.
- **Replace `Mutex<Option<WifiCredentials>>` with a oneshot channel.** Functionally equivalent, more types, no win.
- **Move `EspClock` back into `main.rs` now that `SensorData` doesn't take a `Clock` generic.** Cloned into the SNTP callback, so the small struct earns its keep. Fine where it is.
- **Make `xy-fake` / `ina-fake` first-class test seams (trait + two impls) instead of cfg-swap.** The fakes exist for "develop on a board without the sensor," not for unit testing — the loops do real UART/I2C bring-up. A trait would force every method signature and erase the cfg ergonomics for ~no testability gain. The pure pieces (modbus framing, charge controller) are already extracted and host-tested; the loop body is the only thing that isn't, and F3's "extract a pure step function" is the right answer there if it ever gets hairy.
- **Add a `Drop` impl on `EspHttpServer`-bearing variants to log teardown.** Useful once, not a design issue.
