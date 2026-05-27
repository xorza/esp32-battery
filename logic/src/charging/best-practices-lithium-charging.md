# Best Practices for Charging LFP and Li-ion Batteries

Reference document describing standard CC/CV charging for Lithium Iron Phosphate (LFP) and Lithium-ion (NCM/NCA) batteries, and how this firmware implements them.

## 1. The CC/CV Charging Profile

Both chemistries use a Constant Current / Constant Voltage (CC/CV) algorithm:

- **Phase 1 — Constant Current (CC):** charger sources a fixed current ("Bulk"). Battery voltage rises as it accepts charge. Lasts until ~80–90% SoC.
- **Phase 2 — Constant Voltage (CV):** target absorption voltage is held constant. Charging current naturally tapers as the pack saturates.

**This implementation:** the CC/CV transition is enforced **in hardware** by the XY7025 buck (`REG_I_SET` = `regulation_a` = capacity × `REGULATION_C`). The firmware-level supervisor doesn't run a separate CC stage — it operates two CV setpoints and switches between them based on observed current:

- **Float CV** (`float_v`, e.g. 13.5 V for 4S LFP) when charging current < `enter_absorb_a`.
- **Absorb CV** (`absorb_v`, e.g. 14.4 V for 4S LFP) once charging current crosses `enter_absorb_a`.

The buck's internal CC limit caps current during bulk; the supervisor's job is choosing **which** CV setpoint the buck regulates to.

---

## 2. Voltage Thresholds

| Chemistry variant | Per-cell absorb | Per-cell float | 4S pack absorb |
| :--- | :--- | :--- | :--- |
| `Chemistry::LiFePo4` (daily) | 3.60 V | 3.375 V | **14.4 V** |
| `Chemistry::LiFePo4TopBalance` | 3.65 V | 3.375 V | 14.6 V |
| `Chemistry::LiIon` (longevity-tuned) | 4.10 V | 4.00 V | — (3S = 12.3 V) |

**Notes:**
- Daily-cycling LFP at 14.4 V matches Victron / Battle Born consensus — gentler on cells than 14.6 V, reaches ~99% SoC either way.
- `LiFePo4TopBalance` (14.6 V) is the manufacturer max — use sparingly when the BMS needs the headroom to balance cells.
- Standard NCM Li-ion charges to 4.20 V/cell (12.6 V on 3S). This implementation uses 4.10 V instead: trades ~15% capacity for dramatically more cycles. If you need maximum capacity, add a `LiIonStandard` variant.

---

## 3. Termination Current (Tail Current)

The CV stage ends once charging current taper drops below a fraction of pack capacity:

| Mode | C-rate | Use case | Implemented? |
| :--- | :--- | :--- | :--- |
| Standard | **0.05C** | Daily use, manufacturer-spec | ✓ `EXIT_ABSORB_C = 0.05` |
| Balancing | 0.02C | Top-balancing — keeps pack at CV longer for BMS bleed | ✗ Not exposed |
| Precision | 0.01C | Initial commissioning / out-of-balance recovery | ✗ Not exposed |

The shipped firmware implements only the standard (0.05C) tail. Adding a balance mode would require a new `Chemistry` variant or a per-profile override.

A second threshold, `ENTER_ABSORB_C = 0.06`, sits just above the tail and provides hysteresis — entering Absorb only when charging current exceeds 0.06C, exiting when it drops below 0.05C, so the pack doesn't flap at the boundary.

---

## 4. Implementation Details

### 4.1 Noise rejection

Sensor noise from the switching regulator and transient loads can briefly push current under the tail threshold. The firmware rejects this with a **time-based debounce**: charging current must stay below `exit_absorb_a` for `EXIT_DEBOUNCE = 60 s` continuously before the supervisor accepts the taper as real.

A moving average on the ADC would also work; the time-debounce is simpler and lets the same pattern apply uniformly to OV detection (`OV_DURATION = 3 s`), Modbus-unhealthy (`MODBUS_UNHEALTHY_TIMEOUT = 5 s`), and battery-stale (`BATTERY_MISSING_TIMEOUT = 10 s`).

### 4.2 Maximum Absorption Timer

`MAX_ABSORB = 2 h`. The timer clocks **time at the CV plateau only** — it arms once the pack reaches `absorb_v` (within `ABSORB_CV_BAND_V = 0.1 V`) and resets on any dip back into CC. If the current never tapers below `exit_absorb_a` while held at CV for this window, the supervisor latches `FaultReason::AbsorbTimeout` and disables the buck. Catches stuck-current scenarios (parasitic load pinning current above the tail, BMS balancer drawing continuously, etc.) before the pack sits at CV indefinitely.

The CC ramp is deliberately excluded: a deeply discharged pack enters Absorb immediately (current > `enter_absorb_a`) and can spend several hours in CC at 0.2C before reaching `absorb_v` — clocking that against a 2 h cap would fault a healthy charge-from-empty. A healthy pack at 0.05C tail taper-finishes in well under 30 min *once at CV*, so the 2 h cap is generous headroom, not a typical operating point.

### 4.3 BMS Handshaking

**Not implemented.** If the BMS opens its charge FET (high-voltage disconnect / cell fault), the supervisor sees current drop to ~0 while the buck holds the CV setpoint. There's no dedicated detection — the eventual `MAX_ABSORB` timeout is the backstop (latches `AbsorbTimeout` after 2 h).

A dedicated detector would fire faster and produce better diagnostics: "current < 0.01C for > 30 s while in Absorb at setpoint" → fault as `BmsTripped`. Worth adding only if BMS HVD events become a real operational concern.

### 4.4 Hardware-side OV / OCP / LVP backstops

The XY7025 has its own protection registers (OVP / OCP / input-LVP). The supervisor programs these via `Profile::safety_limits`:

- **OVP** = `absorb_v + 0.6 V` — sits 3× the supervisor's debounced OV margin (0.2 V) above absorb, so the supervisor's faster trip catches the issue first.
- **OCP** = `regulation_a × 1.5` — last-ditch over-current.
- **LVP** = `INPUT_NOMINAL_V − 2 V` = 22 V — input UVLO (tied to the XY7025's 24 V supply rail, **not** a pack-side cutoff).

### 4.5 Backflow protection

Hardware concern; firmware doesn't address it. Use a blocking diode or an ideal-diode MOSFET on the pack side so the battery can't backfeed a powered-down charger.

---

## 5. Faults & Latch Behavior

The supervisor latches the buck off on any of these conditions:

| `FaultReason` | Trigger | Time budget |
| :--- | :--- | :--- |
| `BatterySensorStale` | No fresh INA228 reading | `BATTERY_MISSING_TIMEOUT` = 10 s |
| `ModbusUnhealthy` | Continuous Modbus failures to the XY7025 | `MODBUS_UNHEALTHY_TIMEOUT` = 5 s |
| `Overvoltage` | `v_batt > absorb_v + OV_MARGIN_V` | `OV_DURATION` = 3 s |
| `AbsorbTimeout` | Held at CV plateau without tapering (CC ramp excluded) | `MAX_ABSORB` = 2 h |

After latching, the supervisor emits `Action::DisableOutput` on every `tick()` until the caller successfully writes `set_output(false)` to the buck and calls `ack_disable()`. Once acked, the supervisor goes silent — only a reboot clears the latch.

After a reboot, the supervisor always boots in **Float**, regardless of the previous phase. Conservative by design: re-derive phase from observed current rather than persist it across resets.

---

## 6. Temperature & Operational Safety

These are **not** enforced by firmware; they're operating constraints on the physical pack:

- **Cold charging:** never charge a lithium pack below 0 °C (32 °F) — causes permanent lithium plating and fire risk. This firmware has no temperature sensor wired in. If freezing temps are possible, add a thermistor + a `BatteryTooCold` fault path.
- **Storage:** if unused > 30 days, store at 40–60% SoC in a cool environment.
- **Periodic full charge:** for LFP, charge to 100% (let the supervisor reach Absorb and complete the taper) at least once every few cycles so the BMS can re-balance and recalibrate its SoC estimate.
