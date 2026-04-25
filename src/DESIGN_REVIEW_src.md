---
name: Design review — src/  (2026-04-25)
description: Read-only design review of the ESP-side Rust crate (src/) — focus on simplification, reusability, code structure.
---

# Design review: `src/`  (2026-04-25)

Scope: all 23 Rust files in `src/` (~2.8 kLOC). The pure-logic crate at `logic/` is intentionally out of scope — this review is about the ESP-bound shell.

## Current design

`main.rs` owns process bring-up: it constructs the `Board` (peripherals), shared handles (`SensorDataHandle`, `EventLogHandle`, `NetStatusHandle`), spawns long-running threads (`xy::start`, `ina::start`, `lcd::start`), and runs a 1 Hz tick that drives `Persister` (data + save scheduling) and `Supervisor` (network phase). Worker threads communicate with the main thread exclusively through `Arc<Mutex<…>>` handles for live data and an `mpsc` channel for new credentials submitted via the captive portal.

State is split deliberately: `SensorData` (in the logic crate) is the single source of truth for live readings + history; the `Persister` is the only writer at tick boundaries; HTTP handlers and the LCD are read-only consumers that hold the lock briefly. The network state machine lives entirely in `logic::net_supervisor::Phase`; `app_state::Supervisor` is the thin ESP-bound shell that pins the generic to real `EspHttpServer` / `DnsHandle` types and mirrors the discriminant to an `AtomicU8` for the LCD's lock-free read. The captive portal's `/save` handler hands credentials back via an `mpsc::Sender`, drained latest-wins on the main tick — same semantics as a mailbox without a mutex.

HTTP is split into a captive-mode plaintext server (`http/captive.rs`) and a TLS dashboard server (`http/main_server.rs`), with each feature module exposing a `register(&mut server, …)` fn that mounts its routes. JSON is serialized into a per-handler `Mutex<Box<[u8; N]>>` so handlers don't allocate per-request. Sensor errors are recorded to a bounded `EventLog` ring rather than panicking, with hard faults latched by the charge supervisor.

## Overall take

The architecture is sound and the load-bearing decisions are right: state placement (`SensorData` owns history, `Persister` is the sole writer), pulling the network state machine into the testable `logic` crate, the lock-free `NetStatusHandle` for the LCD, and the choice to record sensor errors instead of panicking. There is no module here that wants to be redesigned. The opportunities are reusability and structural — a handful of patterns are duplicated across HTTP handlers and across the two sensor-thread modules in ways that obscure intent and invite drift.

## Findings

### [F1] JSON HTTP handlers duplicate the same lock + serialize + write boilerplate

- **Category**: Abstraction
- **Impact**: 4/5 — three near-identical handlers; new endpoints will copy the pattern again.
- **Effort**: 2/5 — one helper, three call-site rewrites. Local to `src/http/`.
- **Current**: `api::register` (api.rs:103–166), `errors::register` (errors.rs:77–113), and the `/scan` handler in `captive::start` (http/captive.rs:34–57) all repeat:
  1. allocate a `Mutex<Box<[u8; N]>>` JSON buffer outside the closure,
  2. lock the buffer, call `serde_json_core::to_slice`, branch on error → `text_response(req, 500, …)`,
  3. build a response with the same `Content-Type: application/json` + `Connection: close` headers,
  4. `write_all(&buf[..len])`.

  ~30 lines × 3, plus subtle drift: `/scan` calls `.unwrap()` on `to_slice` while `/api` and `/api/errors` return 500 on overflow.
- **Problem**: The handlers want to express *"serve `T: Serialize` as JSON"* but they all hand-roll the same sequence. Each new endpoint starts with copy-paste, and errors-vs-`unwrap` divergence is exactly the kind of inconsistency this invites. The `text_response` helper in `http/mod.rs:109` already establishes that response-shape helpers live there — JSON is the missing twin.
- **Alternative**: Add to `http/mod.rs`:
  ```rust
  pub(crate) fn json_response<T: Serialize>(
      req: Request<&mut EspHttpConnection>,
      buf: &mut [u8],
      value: &T,
  ) -> Result<(), EspError> { … }
  ```
  Plus `pub(crate) fn json_handler<T, F>(server, path, buf_size, build) where F: Fn(&Store) -> T` for the common "lock, build response struct, serialize" sweep — three call sites collapse to ~10 lines each. Even just the lower-level `json_response` removes the 500-vs-unwrap drift.
- **Recommendation**: Do it. This is the highest-leverage cleanup in `src/`.

### [F2] `record(event_log, clock, kind)` is duplicated across `ina.rs` and `xy.rs`

- **Category**: Responsibility
- **Impact**: 3/5 — small but the duplication will compound with every new sensor.
- **Effort**: 1/5 — move the helper to `app_state` or extend `EventLogHandle`.
- **Current**: `ina.rs:78–81` and `xy.rs:191–194` both define:
  ```rust
  fn record(event_log: &EventLogHandle, clock: &EspClock, kind: …) {
      let ts = clock.epoch_s().unwrap_or(0);
      event_log.lock().unwrap().record(ts, Event::…(kind));
  }
  ```
  Both call sites pair an `EventLogHandle` with an `EspClock` and pass them around together (e.g. `ina::start`, `xy::start`). The pairing is the actual abstraction.
- **Problem**: Two threads, one concept ("record an event with the current epoch"). The helper lives at the wrong layer — it's per-source plumbing for a property that's actually shared. Future sensors (e.g. LCD diagnostics, OTA fault counters) will copy it again.
- **Alternative**: Either
  (a) wrap the pair in `EventRecorder { log: EventLogHandle, clock: EspClock }` with a `record(&self, event: Event)` method — passed to threads instead of the two handles separately; or
  (b) add a free fn `record(event_log: &EventLogHandle, clock: &EspClock, event: Event)` in `app_state` and let callers construct the `Event` variant inline — one-liner each.

  (a) is the cleaner shape because the two handles always travel together; (b) is the lower-effort version of the same idea.
- **Recommendation**: Do (a). Cheap, removes a class of "I forgot to wrap in `Event::Xy(…)`" drift.

### [F3] `xy.rs` and `ina.rs` use inconsistent fake-mode strategies

- **Category**: Abstraction
- **Impact**: 3/5 — affects testability and onboarding more than runtime correctness.
- **Effort**: 2/5 — pick one strategy and apply.
- **Current**:
  - `ina.rs:148–159` does a *leaf swap*: only `read_battery` is `cfg`-gated. The thread loop, accumulator, lock pattern are shared between fake and real builds.
  - `xy.rs:14–320` does a *whole-module swap*: `mod real { … pub fn start … }` and `mod fake { … pub fn start … }` are entirely separate, ~140 lines vs ~30 lines, with no shared scaffolding. The fake `start` reimplements thread spawn + sleep + lock from scratch.
- **Problem**: Two adjacent files with the same job (sensor thread, optional fake) use opposite fake strategies. A reader has to learn both. The `xy` fake also bypasses the supervisor's tick loop entirely, so `xy-fake` builds aren't exercising the charge state machine — that's a real test coverage gap masquerading as a stylistic difference.
- **Alternative**: Push the leaf seam into `xy.rs` too — define a `XyDevice` trait (`read_status`, `set_voltage`, `set_output`, `set_protection`, `set_power_on_default_off`) with a real impl over UART and a fake impl that returns canned values. The thread loop + `ChargeSupervisor` integration becomes shared code, and `xy-fake` builds actually exercise the supervisor against a fake device.

  If a trait feels heavy for two impls, the alternative is `cfg`-gating only the `Xy::new` / `read_status` / `write_holding` bodies. Either way the loop is shared.
- **Recommendation**: Do it. Aligns the two sensor modules and closes the test-coverage gap on the charge supervisor under fake builds.

### [F4] `get_rssi` lives in `api.rs` but is WiFi state

- **Category**: Responsibility
- **Impact**: 2/5 — small leak, but it's the only `unsafe esp_wifi_*` call outside `wifi.rs`.
- **Effort**: 1/5 — move the function.
- **Current**: `api.rs:94–101` defines `fn get_rssi() -> i32` with raw `esp_wifi_sta_get_ap_info` ffi, then `register` calls it from the `/api` handler. `wifi.rs` already owns every other esp-wifi interaction and is the natural home.
- **Problem**: Splits the "wifi access" responsibility across two modules, and puts `unsafe` in the API serialization file where readers don't expect it. Also: since the `/api` handler runs only when `Supervisor` is in `Host` phase (i.e. STA connected), the call always succeeds in practice — but `api.rs` returns 0 on failure, defining its own contract for "wifi unavailable" rather than asking the wifi module.
- **Alternative**: Add `Wifi::sta_rssi(&self) -> Option<i8>` to `wifi.rs`. The `/api` handler takes a `WifiHandle` (or just the rssi value, computed before locking sensor data) — and `0` becomes `None` serialized as `null`, which the frontend can render as "—" instead of treating 0 as a real signal.
- **Recommendation**: Do it. Threads through naturally if F1's `json_handler` helper takes context.

### [F5] `Framebuf<W, H, N>` exposes a redundant const-generic

- **Category**: Types
- **Impact**: 2/5 — footgun, not a bug today.
- **Effort**: 1/5 — internal to `lcd.rs`.
- **Current**: `lcd.rs:87–162` parameterizes `Framebuf<const W, const H, const N>` where `N` must equal `W * H`. Callers compute it at the use site: `const VALUE_PIXELS: usize = (VALUE_W * VALUE_H) as usize;` (lcd.rs:75) and `const GRAPH_PIXELS: usize = (GRAPH_W * GRAPH_H) as usize;` (lcd.rs:80). Pass the wrong `N` and you get a UB-shaped index out of `pixels`, no compile error.
- **Problem**: A type whose third parameter must always satisfy a relation to the first two is a leaky abstraction. The const-generic exists only because `[T; W*H]` isn't allowed in stable yet — but the workaround is invisible to readers and fragile for future changes.
- **Alternative(s)**:
  1. Use `Box<[Rgb565]>` (heap slice) of length `W*H` set in `new()`. The type drops to `Framebuf<const W, const H>`, callers stop computing `N`, no `unsafe`. Heap cost is unchanged — `Box<[T; N]>` already heap-allocates.
  2. Replace the generic with two concrete structs `FieldBuf` and `GraphBuf` — there are exactly two instances. Removes the generic entirely.
- **Recommendation**: Do (1). The generic is doing real work (reusing draw helpers across both buffers), so keeping it but dropping the third parameter is the right shape.

### [F6] Captive `/save` re-implements OTA's `read_exact`-on-Request loop

- **Category**: Abstraction
- **Impact**: 2/5 — small, but it's a third near-copy after F1 and F2.
- **Effort**: 1/5 — promote `read_exact` to `http/mod.rs`.
- **Current**: `ota.rs:61–77` defines a private `read_exact(req, buf)`. `http/captive.rs:62–73` does the same loop inline (with a slightly different stop condition: stop at `filled >= body_buf.len()` rather than treating short read as error). Both are reading a request body into a fixed-size buffer.
- **Problem**: The body-read loop is fiddly enough that everyone gets it slightly different. The captive variant silently accepts truncation; OTA treats it as error. That divergence is invisible.
- **Alternative**: Move `read_exact` to `http/mod.rs`, plus a sibling `read_to_buf(req, buf) -> usize` that returns bytes filled (for the captive case where short reads are OK). Two named helpers, intent-clear at call sites.
- **Recommendation**: Do it alongside F1 — same module, same theme.

## Considered and rejected

- **`Wifi::Mode` enum vs two bools**: already refactored per the comment at wifi.rs:38–45. Right shape — leaving alone.
- **Fold `Supervisor::on_tick_connected` / `on_tick_disconnected` into one `tick(connected)`**: the two paths take different closure types (build_host vs build_captive) and different parameters (grace ticks). Folding requires both closures up front, which forces unnecessary captures even on the connected path. Current shape is fine.
- **`HeapInfo::new()` with `unsafe`**: isolated, single call site, well-documented. Not worth touching.
- **`xy.rs` named setter methods (`set_voltage`, `set_current_limit`, `set_protection`)**: could be one `set_register(reg, scaled)` but the named methods document intent at call sites. Keep.
- **`NetStatusHandle::load` `unreachable!`**: AtomicU8 only ever stores values written by `store`, so the panic is provably unreachable. Strict invariant; keep.
