# Design review: sensor data ownership & update flow  (2026-04-24)

Files in scope: `main.rs`, `app_state.rs`, `ina.rs`, `xy.rs`, `platform.rs`, plus the `logic/src/data.rs` surface these callers touch.

## Current design

One `Arc<AppState>` is handed to every thread. It holds a `Mutex<SensorData<EspPlatform>>`, an `Arc<AtomicBool> ntp_synced`, and a captive-portal flag. Two producer threads (`ina`, `xy`) at ~1 Hz each call `sensor_data.lock().unwrap().update_battery(...)` and `update_ps(...)` respectively. Each `update_*` sets `Option<Reading> = Some(...)` plus a `*_updated: bool` flag and internally triggers `try_commit()`, which gates history pushes on (both flags set) ∧ (both Options `Some`) ∧ (`platform.epoch_s()` returns `Some`) ∧ (strictly-increasing `time_s`). `try_commit` also handles NVS load-on-first-commit and periodic save — still under the caller's mutex lock (a consciously-accepted trade-off flagged in the file itself at `data.rs:269–273`).

`SensorData` owns `EspPlatform`, so the logic crate is platform-free and testable via a mock `Platform`. `EspPlatform::epoch_s()` double-checks time plausibility via `VALID_EPOCH_S`; the SNTP callback in `main.rs` validates the same range before flipping `ntp_synced` (belt-and-suspenders against poisoned NTP replies that poisoned history in an earlier revision).

The main-loop state machine (`Server::{None, Main, Captive}`) couples SNTP lifetime to the HTTPS server — `Main` owns both, so one enum transition tears both down in lockstep and ensures at most one SNTP client ever exists.

## Overall take

The shape is right for what this firmware is — single-core ESP32, 1 Hz data rates, ~4 KB of history. The logic-crate/platform-trait split is a real asset (63 tests, offline-runnable). The mutex-per-shared-store pattern is appropriately simple for these contention levels. The questions worth asking are about where responsibilities sit on the producer and persistence boundaries.

## Findings

### [F1] `SensorData` owns `Platform`, so NVS I/O runs under the data mutex

- **Category**: Responsibility / Control flow
- **Impact**: 4/5 — every other thread stalls 50–100 ms every 10 min (XY Modbus timing, INA read pacing, HTTP latency, LCD refresh), and the stall grows if NVS is ever slower.
- **Effort**: 3/5 — refactors `data.rs`, but the logic crate is well-tested so changes are verifiable.
- **Current**: `SensorData::try_commit` calls `self.platform.save_blob(...)` at `logic/src/data.rs:282`, reached while the caller holds `Mutex<SensorData>` (e.g. `ina.rs:112–116`, `xy.rs:319`). The comment at `data.rs:269–273` acknowledges this.
- **Problem**: The data store conflates "what samples exist" with "how they are persisted." The logic crate was supposed to be platform-free and is, but by *owning* the platform it forces persistence to happen wherever commits do, which is inside the hot path's lock. With the log_ring ring buffer now being snapshotted through `/api/log` under its own mutex, there are more concurrent consumers that feel this stall.
- **Alternative**: flip the responsibility. `SensorData` exposes `take_save_payload() -> Option<&[u8]>` (returns Some when an interval has elapsed or on an explicit request) and `fn load_from(&mut self, &[u8])` for the reverse. Main.rs spins a small persistence helper (or the existing ina thread) that *after* releasing the lock, copies the bytes out and hands them to `EspPlatform::save_blob`. `Platform` becomes a clock-only trait (`fn epoch_s`). NVS access lives entirely in `EspPlatform`, which no longer needs to be owned by `SensorData` — it becomes plain module-level code in the firmware.
- **Cost**: the "write" buffer copy adds ~4 KB/10 min of memmove. Free.
- **Recommendation**: Do it. Also cleanly separates two concerns that are always changing independently.

---

### [F2] INA thread loops forever on a dead sensor, blocking history forever

- **Category**: Control flow / Error contract
- **Impact**: 3/5 — a real I²C failure silently freezes the data pipeline (no new history samples, `battery_reading` never updates, HTTP responds with stale values). No watchdog catches it because the thread keeps ticking `thread::sleep`.
- **Effort**: 1/5 — a counter and a `panic!`.
- **Current**: `ina.rs:102–110`:
  ```rust
  while count < SAMPLES_PER_UPDATE {
      thread::sleep(SAMPLE_INTERVAL);
      if let Some(bat_r) = read_battery(&mut battery_ina) {
          bat_acc.add(&bat_r);
          count += 1;
      }
  }
  ```
  No progress ⇒ the thread stays in the `while` forever. `read_battery`'s 3-retry wrapper doesn't help because `retry()` is per-call, not per-cycle.
- **Problem**: silent sensor death presents as "everything seems OK, data just isn't updating." The panic hook in `main.rs:81` only fires on actual panics; this loop doesn't panic.
- **Alternative**: track consecutive failures in the outer loop and `panic!` after N (say 20 ≈ 2 s of dead I²C), letting the panic hook reboot. Or mark a health flag in `AppState` that HTTP exposes; LCD surfaces it too.
- **Recommendation**: Do it. One counter + one panic. Preserves the "reboot on fatal" policy this codebase already uses.

---

### [F3] SNTP lifetime coupled to the HTTPS server enum

- **Category**: Control flow / Responsibility
- **Impact**: 3/5 — SNTP restarts on every WiFi drop-reconnect cycle, which can delay time sync after each flap and means any NVS load in `try_commit` is deferred until the next successful sync. Also, the "one SNTP at a time" invariant is enforced by lifetime coincidence rather than by a dedicated owner.
- **Effort**: 3/5 — restructures the `Server` enum and main loop.
- **Current**: `main.rs:39–50` — `Server::Main(EspHttpServer, EspSntp)`. Starting SNTP only happens when WiFi transitions from disconnected to connected *and* the main server is starting.
- **Problem**: SNTP depends on IP connectivity, not on an HTTPS listener. Bundling them means any server teardown (e.g. a future "restart HTTPS on cert swap") drops SNTP too, and reconnect flaps cause repeated resyncs from a cold cache. The invariant "at most one SNTP" is structurally enforced but the cost is that SNTP can't outlive the server.
- **Alternative**: make the state machine two-dimensional — `wifi_state: Sta|AP|Off` governs SNTP, `server_mode: Main|Captive|None` governs HTTP. Store `sntp: Option<EspSntp>` on `AppState` directly, (re)start on STA-up, tear down on STA-down. HTTPS server lives independently.
- **Considered alternative**: start SNTP once ever at boot and let it idle through WiFi flaps. Espressif's SNTP client survives network loss, so this is plausible and dramatically simpler — `AppState { sntp: EspSntp }` created once after the first STA-up.
- **Recommendation**: Consider the "start SNTP once" variant — near-zero cost, keeps epoch synced across flaps, removes the lifetime-binding cleverness. Verify with the esp-idf-svc API that the client handles reconnects internally.

---

### [F4] XY boot sequence has no retry or failure surfacing

- **Category**: Error contract
- **Impact**: 3/5 — if the XY misses a single boot write (UART noise, power-up race), `set_voltage`/`set_current_limit`/`set_protection` force the output off via their infallible-fallback contract, and the thread drops into its 1 Hz poll loop. Output stays off; user has no indication except the LCD showing no PS voltage.
- **Effort**: 2/5 — a retry loop around the boot block, maybe a health flag.
- **Current**: `xy.rs:305–309`:
  ```rust
  xy.set_output(false);
  xy.set_protection(BOOT_OVP, BOOT_OCP, BOOT_LVP);
  xy.set_voltage(BOOT_V_SET);
  xy.set_current_limit(BOOT_I_SET);
  xy.set_output(true);
  ```
  On any Modbus failure the individual setter force-disables output and returns silently. Entering the poll loop means we'll read_status forever without reconfiguring.
- **Problem**: fire-and-forget boot with no observable failure. In the edge case where the XY needed `POST_WRITE_GAP` to be larger (we recently shrank it from 150 ms → 10 ms — bold move), a single transient failure here becomes permanent.
- **Alternative**:
  - Wrap the boot block in a loop: try, read back `v_set`/`i_set` via `read_status`, retry up to N times with a growing backoff if setpoints don't match, panic if exhausted.
  - Add a `xy_health: AtomicU8` or similar on `AppState` and surface "XY not configured" on LCD / `/api`.
- **Recommendation**: Do the retry. Health flag only if retries are exhausted and we decide not to panic-reboot.

---

### [F5] `AppState::ntp_synced` is public, used by exactly one caller

- **Category**: Abstraction / Types
- **Impact**: 2/5 — low-risk code smell, no wrong behavior, but removes a free-floating mutable across module boundaries.
- **Effort**: 2/5 — a method on `EspPlatform` plus a closure passed to `start_sntp`.
- **Current**: `app_state.rs:16` (`pub ntp_synced: Arc<AtomicBool>`), set by `start_sntp` callback in `main.rs:64`, read by `EspPlatform::epoch_s` (`platform.rs:33`). Main.rs clones the Arc to hand to SNTP.
- **Problem**: three modules share one mutable atomic, and `AppState` publishes it without a reason — no HTTP/LCD code reads it. The original motivation was probably "SNTP needs a cross-module way to signal NTP is good"; once `EspPlatform` grew its own clock-validation logic, the Arc became an internal implementation detail leaked into shared state.
- **Alternative**: move the `Arc<AtomicBool>` inside `EspPlatform`. Add `fn mark_synced(&self)` that stores `true`. `start_sntp` takes a closure; main.rs passes `{ let p = platform.clone(); move |t| if valid(t) { p.mark_synced() } }`. Drop `AppState::ntp_synced`.
- **Blocker**: `EspPlatform` is moved into `SensorData` at `main.rs:103`. Either give it internal sharing (`Arc<Inner>` with atomic) or keep the Arc external. Former is cleaner.
- **Recommendation**: Do it together with F1 — if the platform gets extracted out of `SensorData`, it becomes trivial to share via `Arc`.

---

### [F6] `update_*` methods hide `try_commit` as a side effect

- **Category**: Contract
- **Impact**: 2/5 — callers are unaware an NVS save can fire inside their "quick reading publish"; also makes it harder to build a coordinator that wants to batch before committing.
- **Effort**: 2/5 — change two call sites (`ina.rs:116`, `xy.rs:319`).
- **Current**: `data.rs:184–196`:
  ```rust
  pub fn update_battery(&mut self, bat: Ina228Reading) {
      self.battery_reading = Some(bat);
      self.battery_updated = true;
      self.try_commit();
  }
  ```
  `try_commit` touches NVS. The caller wrote "publish my reading" and got "publish + maybe block 100 ms for flash erase."
- **Problem**: hidden work under the lock, hidden I/O on what looks like an in-memory field write. Also makes the new XY command-channel plan harder: a coordinator thread that wants to drain a channel and apply updates efficiently can't separate "publish" from "commit."
- **Alternative**: split into `publish_battery(reading)` / `publish_ps(reading)` (pure field + flag) and `try_commit(&mut self)` (history + save). Producers still call both back-to-back today; a future coordinator can call `try_commit` once per cycle regardless of which side updated.
- **Considered rejected**: keep as-is — current design is concise and there is no working coordinator that would benefit yet.
- **Recommendation**: Defer until either F1 lands (which makes the save step cheap to defer out of the lock) or the XY command-channel coordinator arrives. Not worth it in isolation.

---

## Rethink

Not substantially wrong. The overall architecture — producer threads → shared `Mutex<SensorData>` → consumer threads (HTTP/LCD) — is the right default for this workload. The targeted changes above are local surgery, not a redesign.

## Considered and rejected

- **Replace `Mutex<SensorData>` with a seqlock-style atomic snapshot for live readings + separate lock for history.** Would help if HTTP polled at tens of Hz. Today `/api` is ad-hoc and LCD is 10 Hz; contention isn't measurable. Keep the mutex.
- **Move to a coordinator-thread model (producers → channel → one owner of `SensorData`).** On a single-core MCU this adds ceremony without reducing serialization. The OS scheduler already single-files access through the mutex.
- **Bump `SensorData` readings from `Option<T>` back to `T` with default values.** Would shave the `unwrap_or_default()` at read sites but loses the "no reading yet" distinction HTTP/LCD currently get for free.
