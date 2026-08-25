# State machines: readability, stability, safety

## Verdict

The architecture is already right. Both machines use the techniques that
matter — a flat enum where the variant *is* the state, typestate wrappers
so the radio's mode is enforced by the compiler, a pure supervisor with
no I/O, and an ack protocol that makes every effect retryable. Nothing
here needs rewriting.

What makes the code feel complicated is four specific things, and they
are fixable in isolation:

1. The network machine is untestable, so it has **zero** tests. The
   charging machine has 97.
2. The ack protocol is enforced by `panic!` at runtime instead of by the
   type system at compile time — and on this board a panic is a reboot.
3. `ChargeSupervisor::tick` is a 100-line straight line in which the
   *order of the `if` blocks is the specification*, and nothing names or
   tests that order.
4. Faults latch permanently from `Pending`, where the buck output is
   already off. Latching there buys no safety and costs availability —
   on a UPS, that means the load runs on battery until someone reboots it.

Ranked by value: **P1 > P3 > P4 > P2 > P5 > P6**.

---

## What is already good (do not touch)

- **`WifiDriver` → `StaWifi` / `MixedWifi`** — mode transitions consume
  the wrapper and produce the other one. "The radio is in mode X" is a
  compile-time fact at every call site. Textbook typestate.
- **`NetState` carries its resources.** Each variant owns exactly the
  servers alive in that state, so "captive portal running while in STA
  mode" is not representable. `net_fsm.md` is a genuinely good spec.
- **`ChargeSupervisor` is sans-IO.** `tick(PollResult, Duration) ->
  Action` touches no hardware. This is the single highest-value technique
  for making a state machine testable, and it is already applied.
- **The ack protocol makes effects idempotent.** A failed Modbus write
  means the caller doesn't ack, the state doesn't advance, and the next
  tick re-emits the same action. That is the right shape.

The proposals below extend these patterns rather than replacing them.

---

## P1 — Make the network FSM testable (sans-IO, again)

**Problem.** `step()` lives in `main.rs` and takes `MixedWifi`,
`EspHttpServer`, `EspMdns`. None of that compiles for the host, so the
FSM with the 2-hour timers, the 20-second association budget, and the
subtle "assoc-success must be checked before the mailbox drain" ordering
has no tests at all. That ordering rule is currently guaranteed by a
comment.

**Technique.** The same split that already worked for charging: separate
*deciding* from *owning*.

```rust
// logic/src/net/mod.rs — pure, host-testable
pub enum NetPhase {
    CaptiveIdle,
    CaptiveTrying { since: Duration },
    CaptiveFallbackRetrying,
    StaConnecting { session_start: Duration },
    StaServing { link: LinkState },
}

pub struct NetPoll {
    pub now: Duration,
    pub associated: bool,
    pub submitted: Option<WifiCredentials>,
    pub reset_requested: bool,
}

pub enum NetAction {
    Nothing,
    RefreshScan,
    ApplyCreds(WifiCredentials),
    PromoteToSta { persist: WifiCredentials },
    FallbackToCaptive { carry: WifiCredentials },
    ForceCaptive,
    MarkSubmissionFailed,
}

impl NetSupervisor {
    pub fn tick(&mut self, p: NetPoll) -> NetAction;
}
```

Firmware keeps the resources in a shell:

```rust
enum NetResources {
    Mixed { wifi: MixedWifi<'static>, bundle: CaptiveBundle },
    Sta   { wifi: StaWifi<'static>, server: EspHttpServer<'static>, mdns: Option<EspMdns> },
}
```

**On the objection that this splits the single source of truth.** It
does, and that is worth naming. But look at what the 5-variant fusion is
actually buying: the resource distinctions are only ever *two* — Mixed
radio + captive bundle, or STA radio + dashboard — and those two covary
perfectly. The other three distinctions are pure timing state. So the
shell honestly has two variants, the phase has five, and the mapping
5 → 2 is total. Assert it once per tick:

```rust
debug_assert!(matches!(
    (&phase, &resources),
    (NetPhase::CaptiveIdle | NetPhase::CaptiveTrying { .. }
        | NetPhase::CaptiveFallbackRetrying, NetResources::Mixed { .. })
      | (NetPhase::StaConnecting { .. } | NetPhase::StaServing { .. },
         NetResources::Sta { .. })
));
```

"Illegal combinations are not representable" becomes "illegal
combinations are one assert away", and in exchange the trickiest timing
logic in the project becomes testable on the host.

**Also fixes:** `force_captive_idle`'s `unreachable!()` — which today
reboots the MCU if the "wifi-reset is only mounted on the dashboard"
assumption is ever broken by a routing change — becomes a `NetAction`
the supervisor simply does not emit from captive phases.

---

## P2 — Replace the panic-enforced ack protocol with linear tickets

**Problem.** Three `panic!`s guard the ack protocol
(`charging/mod.rs:545, 567, 584`). On this board the panic hook reboots.
So a caller-sequencing bug becomes a reboot loop. Worse, `ack_enable`
takes `resume_absorb: bool` that the *caller* echoes back from the
action it just matched — nothing enforces it's the same bool.

**Technique.** Effect tokens. Make the action carry a value that is the
only key which opens the ack, and make it neither `Copy` nor `Clone`.

```rust
#[derive(Debug)]
pub struct EnableTicket {
    resume_absorb: bool,
    _seal: (),                    // only `tick` can construct one
}

impl EnableTicket {
    pub fn resume_absorb(&self) -> bool { self.resume_absorb }
}

#[derive(Debug)]
pub enum Action {
    None,
    EnableOutput(EnableTicket),
    UpdateVoltage(VoltageTicket),
    DisableOutput(DisableTicket),
}

impl ChargeSupervisor {
    /// Consumes the ticket — one ack per action, checked at compile time.
    pub fn commit_enable(&mut self, t: EnableTicket) { … }
}
```

Acking an action you weren't handed, acking twice, and passing the wrong
`resume_absorb` all stop compiling. Three runtime reboots become three
type errors. Call sites in `xy.rs` barely change.

---

## P3 — Separate the safety gauntlet from the mode machine

**Problem.** `tick` runs eight checks in a fixed order, and that order is
load-bearing: drift → output-mismatch → modbus-health → battery-stale →
OV → bring-up → pending-write → phase machine. Swap any two `if`s and the
machine changes behaviour silently. Nothing names the order, and no test
asserts it *as* an order.

Worse, the state is smeared: `let pending_reason = match self.latch {…}`
produces an `Option<PendingReason>` and then `pending_reason.is_some()`
is used twice as a bare "am I in Pending?" flag. That is exactly the
Option-as-flag pattern `net_fsm.md` proudly says the project doesn't use.
The two machines in this codebase are being held to different standards.

**Technique.** Make the gauntlet an explicit, named, ordered function
returning a verdict, and let `tick` become a five-line dispatcher.

```rust
/// Ordered highest-authority first. This order IS the safety spec.
enum Verdict {
    Clear,
    Inhibit(InhibitReason),   // not safe to energise; output already off
    Latch(FaultReason),       // disable now, reboot-only recovery
    Recover(PendingReason),   // buck self-cleared; step back
}

fn safety_verdict(&mut self, p: &PollResult, dt: Duration, mode: Mode) -> Verdict;

pub fn tick(&mut self, p: PollResult, dt: Duration) -> Action {
    match self.safety_verdict(&p, dt, self.mode()) {
        Verdict::Latch(r)   => self.latch(r),
        Verdict::Inhibit(_) => Action::None,
        Verdict::Recover(r) => self.enter_pending(r),
        Verdict::Clear      => self.run_mode(&p, dt),
    }
}
```

The order now lives in one function whose only job is order, and it can
be tested as an order: construct a `PollResult` that trips two conditions
at once and assert which verdict wins.

Also fold the six scattered `self.latch = …` assignments
(`565, 587, 651, 661, 769, 786`) into a single `set_latch()` setter. One
choke point gives you somewhere to put P4's invariants and transition
log, for free.

---

## P4 — Latch only when there is something to disable

**Problem, and this is the safety one.** `Pending` means *the buck output
is off and we haven't decided it's safe to turn on*. From that state, the
current code can latch `ModbusUnhealthy`, `BatterySensorStale`, and
`Overvoltage`. All three are permanent — reboot-only recovery. But the
output was already off, so the latch disabled nothing. It only converted
a transient condition into an outage that needs a human.

On a UPS this is backwards. Per `CLAUDE.md`: *"Output disables are
user-visible: when the buck is off, the load is running on battery
alone."* A latch from `Pending` is a guaranteed load drop bought for no
safety.

The overvoltage case is the sharpest:

```rust
let ov = b.voltage > self.profile.absorb_v + OV_MARGIN_V;
let ov_debounced = self.ov.step(ov, elapsed, OV_DURATION);
if ov_debounced || (pending_reason.is_some() && ov) {
    return self.latch(FaultReason::Overvoltage);
}
```

In `Active`, OV needs 3 seconds. In `Pending`, **one** sample latches the
unit off forever. A single noisy INA228 reading — or a pack still holding
surface charge right after an OTP self-disable, which is precisely when
we are in `Pending { ProtectRecovery }` — permanently drops the load.

**Proposed policy, stated once and enforced by P3's `Verdict` type:**

> A fault latches only when the buck is actually sourcing. In `Pending`,
> the same conditions **inhibit** — they block `EnableOutput` and are
> reported, but they clear on their own when the condition does.

| Condition | In `Active` | In `Pending` (proposed) |
|---|---|---|
| `SettingsDrift` | Latch | Inhibit |
| `ModbusUnhealthy` | Latch | Inhibit |
| `BatterySensorStale` | Latch | Inhibit |
| `Overvoltage` (debounced) | Latch | Inhibit |
| `Overvoltage` (single sample) | — | Inhibit |
| `AbsorbTimeout` | Latch | n/a |
| `OutputUnexpectedlyOff` | Latch | n/a |
| `OutputOnInPending` | n/a | **Latch** — output really is on |

Note that no safety property is weakened. "Never energise into an
over-volt pack" is preserved exactly: OV inhibits `EnableOutput` from the
first sample, as it does today. What changes is only that the inhibit is
not *permanent*.

**Observability that falls out of this.** `PendingReason` and the new
`InhibitReason` should reach `/api`. Today the dashboard shows an absent
phase and nothing else, so "waiting for input UVLO to clear" and "the
INA228 has been dead for 8 seconds" look identical. On a UPS whose load
is on battery right now, that distinction is the whole story.

---

## P5 — Invariants and a transition log

Two cheap additions that turn field debugging from guesswork into
reading.

**Invariants**, in the `set_latch()` choke point from P3:

```rust
debug_assert!(!matches!(
    (&self.latch, phase_write_pending),
    (LatchState::Pending { .. } | LatchState::Tripped { .. }, true)
), "pending_voltage outside Active");
debug_assert!(self.exit.elapsed <= EXIT_DEBOUNCE + dt, "leaky debounce overran");
```

Per the house rules these are `debug_assert!` — this is a per-tick path,
and release must not pay for them.

**Transition log.** Today only faults reach `EventLog`, so a unit that
ends up `Tripped` shows you the destination and not the route. Record
every latch transition as `(from, to, cause)` from the single setter. It
costs one enum and one call, and it is the difference between "it tripped
on ModbusUnhealthy" and "it flapped ProtectRecovery four times in ninety
seconds and then tripped on ModbusUnhealthy."

---

## P6 — Property tests over random input sequences

**Needs your go-ahead: this adds `proptest` as a dev-dependency of
`esp32-battery-logic` (host-only, never in firmware).**

97 scenario tests are a lot, but scenario tests only cover paths someone
thought of. A state machine earns reliability from properties that must
hold for *every* input sequence. Now that `tick` is pure, this is nearly
free:

- Never emit `EnableOutput` on a tick whose sample has
  `voltage > absorb_v + OV_MARGIN_V`.
- `Tripped` is absorbing: after the first `DisableOutput`, no later tick
  ever returns `EnableOutput` or `UpdateVoltage`.
- `expected_setpoints().v_set` always equals the last *committed*
  `UpdateVoltage.target_v`. (This is the invariant `SettingsDrift` exists
  to police — worth proving directly rather than trusting the ack dance.)
- Any `cycle_output: true` action has `target_v` strictly below the
  current V_SET. Get this wrong and you back-feed the low-side FET.
- Feeding drift-free polls at a resting voltage forever never leaves
  `Float`.

With P1 done, the same treatment applies to `NetSupervisor`: no sequence
of `NetPoll`s should ever reach `StaServing` without a `PromoteToSta`
having persisted creds first.

---

## P7 — The step-down sequencer

`apply_update_voltage` is the one impurity left in the logic crate: it
calls `thread::sleep`. It also encodes a five-step effect sequence with
two distinct recovery branches as straight-line code with early returns,
and its most important guarantee — *the buck is never left off without
the supervisor finding out* — lives in a comment spanning two files.

Two options:

**(a) Keep it inline, make the outcome explicit.** Take `settle: impl
Fn()` instead of `Duration` (the logic crate stops importing
`std::thread` entirely), and *return* a `StepDownOutcome` instead of
reaching back into `supervisor.ack_voltage_update()` from inside.
Combined with P2, a ticket goes in and an outcome comes out — control
flows one direction.

**(b) Make it a state.** `Active { output: Live | SteppingDown { … } }`,
one stage per tick. Fully pure, fully testable, and "the buck is off and
here's why" becomes representable and visible on the dashboard.

**Recommend (a).** (b) is architecturally cleaner but stretches the
output-off window from ~100 ms to 2–3 s on *every* Float↔Absorb
transition, with the load on battery for all of it. On a UPS that trade
is not worth the purity.

---

## Sequencing

| Step | Work | Why here |
|---|---|---|
| P3 + P4 | One file, mechanical | Biggest readability and safety win per line changed; P4 needs P3's `Verdict` |
| P2 + P7(a) | One file, mechanical | Same call sites as P3; do them in the same pass |
| P5 | Small | Needs P3's single setter to exist |
| P6 | Small, needs dep approval | Needs the above to be worth proving |
| P1 | Largest | Independent of the rest; schedule on its own |

P3, P4, P2 and P7 all land in `logic/src/charging/mod.rs` and its call
site in `src/xy.rs`. They are one coherent pass, and the existing 97
tests are the safety net for it.
