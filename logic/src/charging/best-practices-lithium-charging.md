# Best Practices for Charging LFP and Li-ion Batteries

This document outlines the standard procedures and technical requirements for safely and efficiently charging Lithium Iron Phosphate (LFP) and Lithium-ion (NCM/NCA) batteries.

## 1. The CC/CV Charging Profile
Both LFP and Li-ion batteries utilize a Constant Current / Constant Voltage (CC/CV) algorithm.

### Phase 1: Constant Current (CC)
The charger provides a fixed current (the "Bulk" stage). The battery voltage rises as it accepts the charge. This phase usually lasts until the battery reaches roughly 80–90% State of Charge (SOC).

### Phase 2: Constant Voltage (CV)
Once the target absorption voltage is reached, the charger holds that voltage constant. The current naturally tapers down as the battery reaches full saturation.

---

## 2. Recommended Voltage Thresholds

| Battery Type | Nominal Cell Voltage | Max Charge Voltage (Cell) | Typical 4S/3S Pack Target |
| :--- | :--- | :--- | :--- |
| **LiFePO4 (LFP)** | 3.2V | 3.65V | 14.4V - 14.6V (4S) |
| **Li-ion (NCM)** | 3.6V / 3.7V | 4.20V | 12.6V (3S) |

**Note:** For LFP, charging to 3.60V per cell (14.4V for a 12V pack) is often preferred to 3.65V to maximize cycle life with negligible capacity loss.

---

## 3. Termination Current (Cut-off)
The termination current (or "tail current") determines when the charger stops the CV phase. This is critical for cell balancing.

* **Standard Cut-off (0.05C):** Recommended for daily use where speed is preferred. For a 100Ah battery, this is 5A.
* **Balancing Cut-off (0.02C):** Recommended for "Top Balancing." A lower current keeps the battery in the CV stage longer, allowing the BMS's passive balancers more time to bleed off high-voltage cells. For a 100Ah battery, this is 2A.
* **Precision Cut-off (0.01C):** Used for initial commissioning or recovering out-of-balance packs.

---

## 4. Implementation for Programmable Chargers
When developing firmware for a custom charger, the following logic should be implemented:

1.  **Filtering:** Apply a moving average filter to ADC current readings to prevent noise from triggering a premature termination.
2.  **Safety Timeout:** Implement a "Maximum Absorption Timer" (e.g., 2 hours). If the termination current isn't reached within this window, stop charging to prevent damage.
3.  **BMS Handshaking:** The logic must account for the BMS disconnecting the charge circuit. If current drops to zero instantly while voltage is at the setpoint, the charger should transition to a standby state rather than cycling the power stage.
4.  **Backflow Protection:** Use a blocking diode or an ideal diode controller (MOSFET) to prevent the battery from discharging back into the charger when it is powered down.

---

## 5. Temperature and Safety
* **Cold Charging:** **Never** charge lithium batteries below 0°C (32°F). Doing so causes permanent lithium plating on the anode, which is a fire hazard.
* **Storage:** If the battery will not be used for more than 30 days, store it at 40–60% SOC in a cool environment.
* **Cell Balancing:** LFP batteries should be charged to 100% (reaching the CV stage) at least once every few cycles to allow the BMS to calibrate its SOC estimation and balance the cells.
