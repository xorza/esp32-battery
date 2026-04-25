AI coding rules for Rust projects:

## Error Handling

- Use `Result<>` only for expected failures (network, I/O, external services, user input).
- Avoid `Option<>` and `Result<>` for cases that cannot fail.
- For required values, use `.unwrap()`. For non-obvious cases, use `.expect("clear message")`.
- Crash on logic errors. Do not silently swallow them.
- Add asserts for function inputs and outputs to catch logic errors. Do not assert on user input or network failures.

## Code Style

- Do not add `#[derive(Debug)]` to structs.
- No backward compatibility. Remove old/deprecated code, rename freely, change APIs. Rewrite callers to use new APIs. No compatibility shims, re-exports, or wrappers.
- Remove unused code. If kept intentionally, add a comment explaining why and silence linter warnings.
- Keep public API clean and consistent.
- Never use `#[cfg(test)]` on functions in production code. If tests need convenience helpers, define them in the test module itself.
- Comments only when needed to explain *why* (non-obvious constraint, surprising choice, workaround). Never restate what the code does. Keep them short — one line where possible. Don't write multi-paragraph doc comments.

## Verification

- After changing code, run before confirming:
  ```
  cargo nextest run && cargo fmt && cargo check && cargo clippy --all-targets -- -D warnings
  ```
  Skip doc-tests.
- Check test run times are reasonable. Research and fix slow tests.
- Check online documentation for best practices and patterns.

## Testing

- Write tests for ALL new and modified non-GUI code. No exceptions.
- Tests must verify **correctness**, not just "it runs without panicking":
  - Use hand-computed expected values. Show the math in comments.
  - Assert exact outputs (survivor counts, indices, computed values), not vague ranges.
  - Verify edge cases: empty input, minimal input, boundary conditions.
- For algorithms with parameters, test that parameters actually change behavior:
  - Test with parameter A: expect result X. Test with parameter B: expect result Y. Assert X != Y.
- For SIMD implementations, test against scalar reference for identical results.
- For rejection/filtering: verify exactly which elements survive and which are rejected.
- For numerical code: validate against known-good reference values or analytical solutions.
- Do NOT write tests that only check `result < 10` or `remaining > 0`. These catch nothing.

## Documentation

- Read `NOTES-AI.md` files for summarized project knowledge. Check current directory and relevant subdirectories.
- `NOTES-AI.md` files are AI-generated notes on implementation details and structure. Place in any directory where context is needed. Store only current state, not change history. Split files >300 lines into subdirectory files with a brief parent overview.
- Avoid editing root `README.md` unless asked; update `NOTES-AI.md` instead.
- Add `README.md` to folders that benefit from human-readable docs (crates, examples, benchmarks, complex modules).

## Flashing / Serial monitor

- Flash: `MCU=esp32c6 ./flash.sh` — uses `espflash flash --monitor --non-interactive`, so boot logs stream to stdout without needing a TTY. Always wrap in `timeout 30` — that's enough to flash + see initial boot output. Never use the default 2-minute Bash timeout, and never the 5–10 min upper bound. If the flash itself fails, it fails fast; long timeouts only burn wall time waiting on a successful boot's monitor stream.
- Monitor only (no flash): `espflash monitor --non-interactive --port /dev/ttyACM0`.

## Pre-allowed commands (`.claude/settings.local.json`)

These run without a permission prompt — prefer them over alternatives that would prompt:

- Cargo: `cargo build|check|clippy|fmt|test|nextest|run|search|tree …`, `cargo +stable nextest …`, `cargo c6 …`, `cargo c3 …` (also accepted with a leading `MCU=…` env).
- Flash/deploy (any chip via `MCU=…`, optional `INA_FAKE=1 XY_FAKE=1`): `MCU=… ./flash.sh …`, `MCU=… ./deploy.sh …`. Wrap with `timeout * bash -c '…'` for the documented `flash.sh` / `deploy.sh` invocations — any timeout duration is fine.
- espflash: `espflash flash|monitor|erase-flash …`; `timeout * espflash monitor …` is also pre-allowed.
- Misc: `WebFetch`, `WebSearch`, `curl -ks --max-time 8 https://192.168.0.116/api/log`.

Anything outside these patterns (other shell tools, ad-hoc `bash -c …` wrappers, different env layouts) will prompt — adjust the command to match a pre-allowed shape when possible.

## Optimization Workflow

- Before optimizing, always run or create a relevant benchmark and save the baseline results.
- After optimizing, run the same benchmark again and compare against the baseline to verify the optimization actually improved performance.
- If the optimization is a regression or no improvement, revert it.

## Benchmarks and Profiling

- Run benchmarks: `cargo test -p <crate> --release <bench_name> -- --ignored --nocapture`
- Save benchmark results to a txt file in the bench directory. Maintain a `bench-analysis.md` with interpretations. Update on re-runs.
- Add readme files to benchmark folders explaining which optimizations were tried.
- Use nextest for running tests and measuring execution time.
- Perf profiling: use 3000 samples per second.
- If `addr2line` errors appear in `perf report`/`perf script`, use `perf script --no-inline`.
