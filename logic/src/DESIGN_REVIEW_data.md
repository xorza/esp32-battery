# Design review: logic/src/data.rs  (2026-04-25)

## Current design

`SensorData<C: Clock>` is the central store for both live readings (battery + PS) and the rolling history blob. Live `Option<Reading>` fields stay populated for HTTP/LCD to snapshot; two `*_updated: bool` flags track which side has produced a fresh value since the last history commit. Commits fire only when *both* flags are set — this is the load-bearing invariant the module is built around (data.rs:97–102). After a commit, the flags clear but the readings persist, so HTTP/LCD see the latest known values between history rows.

History is a fixed-capacity `heapless::Vec<Sample, HISTORY_CAPACITY>` with exponential compaction: when full, adjacent pairs are averaged and the sampling `interval` doubles. At `MAX_INTERVAL` (1024 s, ≈58 h of history), the oldest sample is dropped instead of compacting further. A separate accumulator (`acc`, `acc_count`) handles down-sampling within the current interval — the *committed* sample is the average of `interval` raw readings.

Persistence is two-phase to keep NVS I/O out of the mutex: `try_commit` sets `save_pending` when `SAVE_INTERVAL_S` has elapsed; the main loop polls `take_save_payload()`, drains the flag, and drives the flash write outside the lock. A one-shot `anchored` flag prevents re-saving a just-loaded blob. A pre-allocated `Box<[u8; 4096]>` scratch buffer lives inside the struct so serialization doesn't allocate on the hot path.

## Overall take

The core model — adaptive-resolution history in fixed memory, deferred NVS I/O, pre-allocated scratch — is well-thought-through and the tests are comprehensive. One real problem: the "commit only when both producers are fresh" rule silently loses all history if either sensor dies permanently, which is a realistic failure mode (I2C glitch, XY Modbus wedged, pins disconnected). The other findings are small.

## Findings

### [F1] Commit-on-both-fresh means a dead producer silently blocks all history

- **Category**: Contract / Control flow
- **Impact**: 4/5 — realistic failure turns the device from "battery monitor with degraded data" into "battery monitor with no data and no indication why"
- **Effort**: 2/5 — add a time-based watchdog to `try_commit`, or a driver-call on the 1 Hz main loop
- **Current**: `try_commit` early-returns if `!(battery_updated && ps_updated)` (data.rs:250–252). Both flags only clear inside `try_commit`, and only set inside `update_battery`/`update_ps`. If one producer permanently stops calling its `update_*`, the other side's flag saturates `true` forever and `try_commit` never commits. The history `Vec` stays empty (or frozen at its pre-failure state). `save_pending` never fires either, so NVS stops updating. From the outside, the dashboard shows "fresh" live data from the side that still works — nothing signals the commit pipeline is dead.
- **Problem**: The invariant "every row is a synchronized snapshot" is protecting an edge case (double-commit with partially-stale data) at the cost of a much worse failure (total history loss). The comment at data.rs:99–102 justifies it as "without this we'd commit twice per cycle" — true, but partial-data rows are diagnosable, while silent stop isn't.
- **Alternative**: Time-driven watchdog commit. Add a `last_commit_s` field; in `try_commit`, if both flags aren't set *but* `time_s - last_commit_s >= STALE_COMMIT_S` (say, 2× the normal commit cadence), force a commit using whatever `Option<Reading>` values exist, logging a warning. The `power_online` projection already handles a stale PS side — downstream consumers just see `0.0` for that column, which is the honest answer. Alternative framing: drive commits from `main`'s 1 Hz tick via a `SensorData::tick()` method; the flags become a hint (prefer-both-fresh) rather than a gate. Cost: 1–2 fields, ~20 lines, one new test for the stale path.
- **Recommendation**: Do it. The double-commit the current design prevents is a minor cosmetic issue; the silent-stop case is a real monitoring hole.

### [F2] `anchored` + `last_save_s` is a two-field encoding of one state

- **Category**: Types
- **Impact**: 2/5 — cosmetic, but the two fields can get out of sync if `load_from_bytes` is ever called post-init
- **Effort**: 1/5 — rename + three call sites
- **Current**: `anchored: bool` (data.rs:112–114) gates the first-commit "anchor `last_save_s` to now" behavior. `last_save_s: u32` holds the epoch of the last drained save. Together they encode three states: (a) not yet anchored (`anchored=false`, `last_save_s=0`), (b) anchored, not due (`anchored=true`, `last_save_s=t`), (c) anchored, due (`anchored=true`, `time - last_save_s >= SAVE_INTERVAL_S`).
- **Problem**: State (a) is `last_save_s: Option<u32> = None`. States (b) and (c) are both `Some(t)`. The `anchored` bool is redundant with `last_save_s.is_some()`. Nothing prevents setting `anchored=false` while `last_save_s != 0` (or vice versa), though in practice nothing does.
- **Alternative**: `last_save_s: Option<u32>`. The anchoring check becomes `if self.last_save_s.is_none() { self.last_save_s = Some(time_s); }`. The save-due check becomes `self.last_save_s.is_some_and(|last| time_s.saturating_sub(last) >= SAVE_INTERVAL_S)`. Drop the `anchored` field.
- **Recommendation**: Do it. One less field, one less invariant.

### [F3] Pre-allocated `buf` buys no speed and costs 4 KB of steady-state RAM

- **Category**: State
- **Impact**: 2/5 — 4 KB of ESP32-C6's ~400 KB free heap, freed up for ~1% headroom
- **Effort**: 2/5 — `take_save_payload` allocates a fresh `Vec`, `load_from_bytes` parses from the caller's slice directly
- **Current**: `buf: Box<[u8; SERIALIZED_MAX_BYTES]>` (data.rs:116) is used by `take_save_payload` (writes into `buf`, then `buf[..len].to_vec()`, data.rs:235–240) and by `load_from_bytes` (copies caller's slice into `buf`, parses from there, data.rs:215–224). Serialization already double-copies: into `buf`, then into a fresh `Vec`. Load also double-copies: caller's `load_buf` → `self.buf` → parsed samples.
- **Problem**: The pre-allocation was probably intended to avoid alloc on the hot path, but `take_save_payload` already allocates the returned `Vec` every time. The scratch buffer is 4 KB that sits idle 99% of the time (one save per 10 min).
- **Alternative**: `take_save_payload` builds a `Vec<u8>` with `with_capacity(size_estimate)` and serializes into it directly — one alloc instead of two, 4 KB less steady-state RAM. `load_from_bytes` parses from its input slice directly — no copy. The `BufWriter`/`BufReader` helpers already take `&mut [u8]` / `&[u8]`, so the change is plumbing only.
- **Recommendation**: Do it if RAM pressure materializes; otherwise low priority. The module works fine as-is.

## Considered and rejected

- **Pair `(reading, fresh_flag)` into a `FreshReading<T>` type.** Would make the relationship explicit (`Option<FreshReading<Ina228Reading>>`). Rejected because it adds a type for one place that's already readable — and F1 makes the flags partially obsolete anyway.
- **Drive compaction on a separate tick instead of inside `try_commit`.** Compaction is O(N) once per saturation event and rare (happens when `interval` doubles). Current placement is fine; pulling it out complicates the path for no win.
- **Replace `heapless::Vec` with `VecDeque` to make `remove(0)` O(1).** MAX_INTERVAL drop runs once per ~17 min; the O(N) shift is ~200 samples × 20 bytes = 4 KB memmove. Irrelevant cost.
- **Extract the serialization format into its own module with explicit versioning.** `FORMAT_VERSION = 6` suggests several past migrations. The current flat write/read pair is 70 lines and works. Splitting would add ceremony without changing outcomes.
- **`Clock::epoch_s` returning `u32` will overflow in 2106.** Every consumer in-tree handles it as-is. Not worth pre-emptive migration to `u64`.
