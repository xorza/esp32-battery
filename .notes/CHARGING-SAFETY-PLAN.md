# Charging supervisor — safety remediation plan

Addresses the eight open items in `ISSUES.md`. Every fix below is
structural: it changes what the machine *is*, not what it patches around.
Where a fix is a patch and can't be anything else, that is said out loud.

Two pieces of groundwork come first because most of the individual issues
are one or two lines once they exist. Doing them in the stated order keeps
every phase independently shippable and testable.

---

## Groundwork A — split `tick` into reconcile → evaluate

**DONE.** Closes issue 6. Prerequisite for issues 1 and 3.

### Why

`safety_verdict` currently does two unrelated jobs in one ordered list.
Checks 1, 3, 4, 5 ask *"given where we are, is it safe?"* — that is
evaluation. Check 2 asks *"does the machine still agree with what the buck
reports?"* — that is reconciliation, and it has to happen **before**
evaluation means anything.

Because check 2 lives inside the list and returns early, a buck that
re-enables itself out of a hold resumes sourcing without checks 3–5 having
run for that tick. That is issue 6. The same tangle is why
`on_self_disabled` has to set `self.inhibit` by hand — the tick has already
returned before check 6 could derive it.

### Shape

```rust
fn tick(&mut self, p: PollResult, elapsed: Duration) -> Action {
    if let Some(a) = self.reconcile(&p) {
        return a;                  // latched, or nothing left to decide
    }
    match self.gauntlet(&p, elapsed) { ... }
}
```

`reconcile` owns exactly one question — OUTPUT_EN versus state — and is the
only thing that may move the machine before the gauntlet runs:

| state | buck reports | outcome |
|---|---|---|
| sourcing | `Off{self-clearing}` | `step(SelfDisabled)`, fall through to gauntlet |
| sourcing | `Off{other}` | latch `OutputUnexpectedlyOff` |
| holding | `On` | `step(SelfEnabled)`, fall through to gauntlet |
| `Boot` | `On` | latch `OutputOnInPending` |
| `Tripping` | any | re-emit `DisableOutput` |
| `Latched` | `On` | `step(SelfEnabled)` → `Tripping`, emit `DisableOutput` (issue 1) |
| `Latched` | otherwise | `Action::None` |

Falling *through* rather than returning is the whole point: the gauntlet
then evaluates in the reconciled state, so a resumed buck gets its modbus,
battery-freshness and overvoltage checks on the same tick it resumed.

### Steps

1. Move the `Tripping` / `Latched` early returns out of `tick` and into
   `reconcile`, which is where they now belong — they are reconciliation
   answers, not a special case ahead of it.
2. Move check 2 out of `safety_verdict` into `reconcile`, returning
   `Option<Action>`.
3. Delete `Verdict::SelfDisabled` and `Verdict::SelfEnabled`. `Verdict`
   drops to three variants: `Latch`, `Inhibit`, `Clear`. Delete
   `on_self_disabled` and `on_self_enabled` — their `self.inhibit` writes
   are now derived by check 6 (`BuckProtection`) and by `Verdict::Clear`.
4. Renumber the gauntlet's remaining checks and restate the precedence in
   its doc comment.

### Precedence change to be aware of

Drift (old check 1) currently outranks the output check. After the split,
output reconciliation runs first. **This is the more correct order**: if
the buck's output is off, what it would have been regulating to is moot.
Re-derive the precedence tests in `tests/faults.rs`
(`setpoint_drift_does_not_overwrite_existing_latch`,
`drift_outranks_overvoltage_while_regulating`) against the new order and
state the new order in the doc comment as the specification.

### Tests

- A hold that resumes with a stale battery sensor latches on the **same**
  tick, not the next (the issue-6 regression).
- A hold that resumes over the OV line latches on the same tick.
- `on_self_disabled`'s inhibit is still reported, now via check 6.

---

## Groundwork B — separate pack profile from board supply

**Prerequisite for issue 5.**

### Why

`Profile` currently mixes two kinds of fact. Chemistry, cell count and
capacity are properties of the *pack*. The DC input rail and the UPS load
are properties of the *board and its wiring* — `INPUT_NOMINAL_V` already
lives in `main.rs` and gets passed into `safety_limits` as a loose
argument. The load budget has nowhere to live at all, which is why it is
missing from the OCP derivation (issue 5).

### Shape

```rust
/// What the board puts around the pack: the rail feeding the buck, and the
/// continuous load hanging off its output. Board wiring, not pack identity.
pub struct SupplyBudget {
    pub input_nominal_v: f32,
    /// Worst-case continuous load on the buck output. The buck's CC loop
    /// limits *total* output current, so the pack only receives
    /// `i_set_a - load`; sizing I_SET without this silently derates the
    /// charge rate, and sizing OCP without it trips on load surges.
    pub load_a: f32,
}

/// What gets programmed into the buck, derived from pack × board.
pub struct BuckSetup {
    pub i_set_a: f32,
    pub limits: SafetyLimits,
}

impl Profile {
    pub const fn buck_setup(&self, supply: SupplyBudget) -> BuckSetup { ... }
}
```

`i_set_a = regulation_a + supply.load_a`, `ocp_a = i_set_a * 1.5`. The
device-ceiling asserts added earlier move from `safety_limits` into
`buck_setup`, since they now bound the combined figure.

`Profile::regulation_a` keeps its current meaning — the *pack's* charge
rate — and stops being what is written to I_SET.

### Steps

1. `logic/src/charging/supply_budget.rs` — `SupplyBudget`, plus
   `BuckSetup` as its satellite (it is what `buck_setup` hands back and
   stands for nothing on its own).
2. Move the ceiling asserts into `buck_setup`; `safety_limits` folds into
   it and goes away.
3. `main.rs`: `const SUPPLY: SupplyBudget = ...`, `const BUCK: BuckSetup =
   PACK_PROFILE.buck_setup(SUPPLY);` replacing `SAFETY`.
4. `ChargeSupervisor::new(profile, i_set_a)` — `setpoints_for` uses the
   stored `i_set_a` instead of `profile.regulation_a`. Update the
   `expected_setpoints` doc, which already warns that the drift check
   assumes a constant I_SET; that assumption is unchanged, only its source
   moves.

---

## Issue 1 — `Latched` never re-disables

**DONE.** Severity: high. Someone pressing the front panel re-energises a pack
the supervisor latched off, and the firmware never notices.

Fully covered by Groundwork A's `Latched` row, plus **one table cell**:

```
/* Latchd */ [   X,    X,  TRIP,    X,   X,   X,   X,   X,   X],
                            ^^^^ SelfEnabled
```

`ChargeEvent::SelfEnabled`'s doc currently says "once the cause cleared",
which no longer covers both uses. Reword to *"the buck's output came back
on without us asking"* — true from a hold and from a latch alike.

`self.fault` is still `Some`, so `Tripping` re-emits the original
`DisableOutput` and `commit_disable` returns it to `Latched`.
`logged_as(Latched, Tripping)` already yields `ChargeTransition::Latched`,
so each re-disable episode is recorded — which is what you want to see in
the log when a panel button is being pressed repeatedly.

**Tests:** latch, ack, then report `On`; expect `DisableOutput` with the
original reason, ack it, expect a return to `Action::None`. Repeat twice to
prove the cycle is stable and each episode logs.

---

## Issue 2 — absorb cap defeatable, no total charge cap

**DONE**, with `MAX_CHARGE = 8 h` and the accumulator named `charge_total`.
Severity: high for a large pack. Two distinct holes, two fixes.

### 2a. Make the CV clock leaky

`MAX_ABSORB` uses `Debounce::step`, so one tick outside
`ABSORB_CV_BAND_V` zeroes two hours of accumulation. A UPS load that
periodically pulls the buck out of CV keeps the cap from ever firing.

`Debounce::step_leaky` already exists and already solves exactly this
shape for the exit taper — the burst-pulse problem it was written for is
the same problem. Switch the absorb clock to it. Dips then *shave* the
window instead of erasing it, and a genuine sustained return to CC (real
charging) still drains it to zero and blocks the trip.

### 2b. Add an absolute charge-time budget

The CC ramp is deliberately uncapped, and correctly so — a deeply
discharged pack at 0.2C legitimately runs for hours. But that means there
is currently **no upper bound at all** on time spent charging. A pack that
never reaches CV charges forever.

Add a fifth window: `charge_elapsed`, stepped unconditionally
(`step(true, ...)`) while in `Absorb`, firing `FaultReason::ChargeTimeout`
at `MAX_CHARGE`. It rides `ChargeSupervisor::step`'s existing reset block,
so it restarts on any state change — entering Float (the cycle finished),
entering a hold (output was off, nothing was charging), or re-entering
Absorb (a new cycle). Nothing else has to be written.

Sizing: empty→full at 0.2C is ~5 h of CC plus ~1 h of CV. `MAX_CHARGE = 8 h`
is generous headroom. Pin the relation:

```rust
const _: () = assert!(MAX_CHARGE.as_secs() > MAX_ABSORB.as_secs());
```

Response class: **park, not disable** — see issue 8.

**Tests:** a CV dip every N ticks no longer prevents `AbsorbTimeout`
(exact leaky arithmetic, hand-computed); `ChargeTimeout` fires at exactly
`MAX_CHARGE` of continuous Absorb regardless of voltage; a taper to Float
and back restarts it; a protection hold restarts it.

---

## Issue 3 — protection holds flap without limit

**Severity: medium-high.** An input rail that sags under charge current
gives `SelfDisabled` → `SelfEnabled` once a second, forever, each cycle
blipping the UPS load. Nothing counts, nothing gives up.

### Shape

A small owned type, `logic/src/charging/hold_budget.rs`:

```rust
/// How often the buck has dropped into a self-clearing hold lately.
///
/// A hold is normal — the supply was unplugged, the case got warm. A
/// *stream* of them is a supply that cannot carry the charge current, and
/// waiting it out forever means an output that blips once a second for as
/// long as the condition lasts.
pub(super) struct HoldBudget {
    holds: u8,
    since_last: Duration,
}
```

`record()` on entering a hold; `step(dt)` each tick, clearing `holds` to
zero once `since_last` passes `FLAP_WINDOW`. Fires
`FaultReason::ProtectionFlapping` at `MAX_HOLDS`. Wired into
`ChargeSupervisor::step` where `to.holding()` already triggers the OV
reset.

Integer counter and a duration, not a float leaky bucket — the quantity
being counted is discrete, and the decay is "has it been quiet for a
while", which is a comparison, not an exponential.

### Explicitly rejected: backing off `regulation_a`

Reducing the charge current on each flap is the tempting answer and it is
wrong here. `expected_setpoints` documents that the drift check depends on
I_SET never changing at runtime; a varying I_SET needs the same
arm-then-commit treatment the `To*` states give V_SET, which is a feature,
not a fix. Latching is the correct conservative answer, and the log
(`ProtectHold` × N then `Latched`) tells the operator exactly what
happened.

**Tests:** `MAX_HOLDS` holds inside `FLAP_WINDOW` latch; the same count
spread past the window does not; the counter clears after a quiet stretch.

---

## Issue 4 — `resume_absorb` is `true` for every real bring-up

**Severity: medium.** Not dangerous, but it means every reboot forces an
Absorb cycle plus the output-cycling step-down that ends it, and the
comment claiming a full pack parks in Float describes something that never
happens.

### Why it is broken

`at_cv_plateau(v)` tests `v >= absorb_v - 0.1` — 14.3 V for the 4S LFP
pack. It is doing two unrelated jobs: *"is the pack at the CV plateau right
now"* (correct for clocking the absorb cap, where the buck is holding the
pack there) and *"is the pack full"* (wrong at bring-up, where the output
has been off and the reading is resting OCV). A full LFP pack rests at
~13.5 V, so the second use is false every time.

### Fix

Split the two jobs. `at_cv_plateau` keeps its name and its CV-clock use.
Bring-up gets a separate predicate built on `Profile::soc()`, which already
exists, is chemistry-aware, and is exactly the resting-OCV→SoC lookup this
needs:

```rust
/// Whether the pack's resting voltage says it is full. Only meaningful
/// while the output is off — OCV is a rested-pack measurement, so a
/// sample taken under meaningful current is not trusted and the answer
/// falls back to "not full", which resumes Absorb.
fn rested_full(&self, b: BatterySample) -> bool {
    b.current.abs() < self.profile.exit_absorb_a
        && self.profile.soc(b.voltage) >= RESUME_ABSORB_SOC
}
```

`RESUME_ABSORB_SOC = 95.0` separates cleanly on the LFP curve: 13.5 V
resting → 97.5 %, 13.2 V → 70 %, 13.0 V → 40 %.

**Known limitation, worth a comment rather than a workaround:** right after
a reboot the pack has not relaxed, so the reading is still pulled toward
whatever the buck was holding. The current gate above narrows this, and the
failure direction is toward charging, which is what happens today anyway.
A real fix is a rest timer, which is not worth the complexity here.

**Tests:** a 4S pack resting at 13.5 V does **not** resume Absorb (the
regression this fixes); at 13.2 V it does; at 13.5 V *while drawing 5 A* it
does, because the OCV is not trusted. Hand-compute each SoC from the curve.

---

## Issue 5 — OCP ignores the load; charge current is never supervised

**Severity: high for a UPS with a real load.** Two halves, and the second
is what makes the first safe.

### 5a. Size I_SET and OCP for charge + load

On Groundwork B. Today `ocp_a = regulation_a * 1.5` is derived from charge
current alone while the buck's output current is charge **plus** load, so a
load surge trips device OCP → `OutputUnexpectedlyOff` → hard latch → the
load drops onto the pack until someone reboots it.

### 5b. Supervise the charge current closed-loop

Raising I_SET to cover the load creates a new hazard on its own: when the
load is idle, the buck's CC loop will happily push the whole
`regulation_a + load_a` into the pack. For a 50 Ah pack with a 5 A load
budget that is 0.3C against a 0.2C design target.

The supervisor already has the measurement it needs — the INA228 reads
*battery* current, not output current — and has never checked it. Add:

```rust
/// Charging current above the pack's rate for `OVERCURRENT_DURATION`.
/// The buck's CC loop limits total output current, which includes the
/// load; only this sees what the pack is actually taking.
ChargeOvercurrent,
```

Debounced (the buck pulses, and a load stepping off is a real transient),
tripping at `regulation_a * OVERCURRENT_TOL` with `OVERCURRENT_TOL ≈ 1.25`
— comfortably above INA noise and CC-loop overshoot, comfortably below the
0.5C manufacturer maximum.

This closes a gap that predates the load question: nothing has ever
verified that the pack receives the rate the profile asks for.

Response class: **park, not disable** — control is intact, the problem is
that the pack is taking too much. See issue 8.

**Tests:** `buck_setup` derives `i_set_a = regulation_a + load_a` and
`ocp_a` from that, hand-computed; the ceiling asserts move with it (the
90 Ah edge shifts once a load budget is added — re-derive it);
`ChargeOvercurrent` fires after the full window at `1.25 × regulation_a`
and not at `1.2 ×`; a single spike does not trip it.

---

## Issue 6 — `SelfEnabled` bypasses checks 3–5

**DONE.** Dissolved by Groundwork A. No separate work.

---

## Issue 7 — no pack temperature

**Severity: high for a large pack, and the largest genuine gap.** Charging
LFP below 0 °C plates lithium; the damage is cumulative and invisible until
the cell fails. Nothing in the current design can see it.

### Hardware first

The XY7025 reports only `read_temperature_internal` — its own die. The
driver exposes `read_temp_offset_external`, which hints some models take a
probe, but there is no external-temperature read, so **the buck cannot tell
you anything about the pack.**

A pack sensor is required. The board already runs I2C for the INA228, so an
I2C temperature sensor on the same bus is the no-new-pins option and slots
into the existing sensor thread rather than needing one of its own. Confirm
the free address range on that bus before choosing a part.

### Software

1. `Chemistry::charge_temp_range() -> RangeInclusive<f32>` beside
   `charge_voltages()` — the same "chemistry knowledge lives in one place"
   pattern the module is built on. LFP and Li-ion both charge 0–45 °C;
   discharge windows are wider and are the BMS's business.
2. `PollResult.pack_temp_c: Option<f32>`, fed like `battery` — with its own
   staleness window mirroring `BATTERY_MISSING_TIMEOUT`.
3. `InhibitReason::PackTooCold` / `PackTooHot` (out of range at bring-up —
   wait, do not latch) and `FaultReason::PackTemperature` (out of range
   while sourcing).
4. Slots into the gauntlet next to the battery-freshness check, and gets
   latch-versus-inhibit for free from `fault_or_inhibit`.

### The absent-sensor decision

Whether a missing reading refuses to charge is a **board** fact, not a
runtime one. Make it a const, so a board built without the sensor compiles
the check out with the risk documented at the declaration, and a board
built with one treats absence as a stale-sensor fault exactly like the
INA228. Silently charging on a `None` is the one option that is not
acceptable, because it is indistinguishable from a working sensor.

---

## Issue 8 — nothing bounds pack discharge

**Severity: high, and only partly fixable in firmware.**

### What cannot be fixed here

The buck cannot disconnect the pack from the load — output off *is* the
load running on the battery. No firmware change alters that. **A BMS with a
low-voltage disconnect is a required component of this system**, and its
LVD setpoint belongs in the README next to the pack profile. Say so
explicitly rather than leaving it implied.

### What can be fixed here: classify the fault response

Today every fault does the same thing — output off — and that is wrong for
half of them. The hazard behind `AbsorbTimeout`, `ChargeTimeout` and
`ChargeOvercurrent` is *overcharge*, and the correct response to overcharge
on a UPS is to stop charging, not to kill the load and start draining the
pack. Only faults where we have lost **control** genuinely need a dark
buck.

```rust
impl FaultReason {
    /// What the supervisor does about this fault. Losing control of the
    /// buck means the only safe output is no output. A pack taking too
    /// much charge is a different problem: control is intact, so dropping
    /// to the float target stops the overcharge while the load stays fed.
    fn response(self) -> FaultResponse { ... }
}
```

| fault | response | why |
|---|---|---|
| `ModbusUnhealthy` | Disable | no closed-loop control |
| `SettingsDrift` | Disable | unknown setpoints |
| `BatterySensorStale` | Disable | cannot supervise blind |
| `OutputUnexpectedlyOff` | Disable | already off |
| `OutputOnInPending` | Disable | sourcing under unknown setpoints |
| `ProtectionFlapping` | Disable | supply cannot carry it |
| `Overvoltage` | Disable | the buck's regulation is itself suspect |
| `AbsorbTimeout` | Park | not tapering; control fine |
| `ChargeTimeout` | Park | as above |
| `ChargeOvercurrent` | Park | taking too much; control fine |

`Overvoltage` staying on Disable is the deliberate call: parking means
trusting a buck we just caught regulating above target.

### Machine changes

Two states, `ToParked` and `Parked`. `ToParked` is the step-down to
`float_v` (it cycles the output like any step-down); `Parked` is sourcing
at float with the phase machine frozen. Because `regulate`'s match only
handles `Float` and `Absorb`, `Parked` falls through its `_ => {}` arm and
emits nothing — the freeze needs no new code, only the absent table rows.

A self-disable out of `Parked` **latches** rather than holding: we are
already in a degraded fault-parked mode, and a protection event on top of
that is not something to wait out. This keeps the addition to two states
rather than dragging a `HoldParked` in behind them.

`Parked` needs its own `/api` value so an operator can tell "charging
stopped, load fed" from both "charging" and "latched off" — that is the
whole point of the state existing.

---

## Sequencing

| phase | work | issues | behaviour change | status |
|---|---|---|---|---|
| 0 | Groundwork A: reconcile → evaluate | 6 | one-tick window closes; check precedence changes | **done** |
| 1 | `Latched` table cell | 1 | re-disables a resurfaced output | **done** |
| 2 | leaky absorb clock + `MAX_CHARGE` | 2 | new `ChargeTimeout` fault | **done** |
| 3 | `rested_full` on SoC | 4 | full packs stop forcing an Absorb cycle | next |
| 4 | Groundwork B + I_SET/OCP + `ChargeOvercurrent` | 5 | I_SET rises by the load budget | |
| 5 | `HoldBudget` | 3 | new `ProtectionFlapping` fault | |
| 6 | `FaultResponse` + `ToParked`/`Parked` | 8 (partial) | three faults stop killing the load | |
| 7 | pack temperature | 7 | hardware first | |

Phases 0–3 are small and touch nothing outside `charging/`. Phase 4 reaches
into `main.rs` and the `/api` ceiling asserts. Phase 6 adds `/api` surface.
Phase 7 needs a part chosen and fitted.

Adding fault variants is safe for the event log: `ChargeTransition` is the
last `EventKind`, and every offset derives from `EnumCount`, so the ring's
index space grows on its own. New `FaultReason` / `InhibitReason` variants
do add `/api` label strings, which
`tests/faults.rs::labels_are_the_snake_case_wire_identifiers` pins — extend
that table in the same commit as each variant.

---

## Decisions needed before implementing

1. **Load budget (phase 4).** Worst-case continuous current on the buck
   output. Everything in 5a/5b is sized from it, and it moves the maximum
   supportable pack capacity.
2. ~~**`MAX_CHARGE` (phase 2).**~~ Shipped at 8 h. Revisit once the load
   budget lands in phase 4 — the slowest legitimate empty→full runs at the
   *delivered* current, `i_set_a - load_a`, not `regulation_a`, so a large
   load budget stretches the legitimate case toward the cap.
3. **`FLAP_WINDOW` / `MAX_HOLDS` (phase 5).** Proposed: 4 holds in 5 min.
   Depends on how often your supply legitimately sags.
4. **Park-versus-disable on `Overvoltage` (phase 6).** Recommended Disable;
   the table above is a starting position, not a settled one.
5. **Temperature sensor and the absent-sensor policy (phase 7).** Part
   choice, plus whether a board without one refuses to charge.
6. **BMS low-voltage-disconnect setpoint (issue 8).** Needs writing down
   even though no code reads it.
