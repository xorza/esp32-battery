//! Two-phase CV charging strategy with hysteresis.
//!
//! Sits in Float (low CV) by default. When the battery draws more than
//! `enter_absorb_a` of charging current, switches to Absorb (high CV) to
//! finish the pack. Once current tapers below `exit_absorb_a`, drops back to
//! Float. Profiles are per-chemistry constants.
//!
//! Sign convention: battery current is **negative when charging** (matches
//! the INA228 wiring on this board). The controller takes signed amps and
//! negates internally, so profile thresholds stay positive and read
//! naturally.
//!
//! Pure logic: no I/O. The firmware calls `update()` each poll and writes
//! the returned setpoint to the buck converter on transitions.

#[derive(Copy, Clone)]
pub enum Chemistry {
    /// Daily-cycling LFP: 3.60 V/cell absorb, 3.375 V/cell float.
    /// Matches Victron / Battle Born defaults — gentler on cells than 3.65 V,
    /// reaches ~99% SoC either way (Battery University BU-808b, Off-Grid Garage tests).
    LiFePo4,
    /// Top-balance variant for LFP: 3.65 V/cell absorb (manufacturer max).
    /// Use sparingly when the BMS needs the high voltage to balance cells.
    LiFePo4TopBalance,
    /// Longevity-tuned Li-ion (NMC/LCO): 4.10 V/cell absorb, 4.00 V/cell float.
    /// 4.10 V trades ~15% capacity for dramatically more cycles vs. 4.20 V.
    LiIon,
}

impl Chemistry {
    /// Per-cell (absorb_v, float_v). Scaled by cell count in `Profile::for_pack`.
    const fn per_cell(self) -> (f32, f32) {
        match self {
            Chemistry::LiFePo4 => (3.60, 3.375),
            Chemistry::LiFePo4TopBalance => (3.65, 3.375),
            Chemistry::LiIon => (4.10, 4.00),
        }
    }
}

#[derive(Copy, Clone)]
pub struct Profile {
    pub absorb_v: f32,
    pub float_v: f32,
    pub enter_absorb_a: f32,
    pub exit_absorb_a: f32,
}

impl Profile {
    /// Build a pack-level profile from chemistry + series cell count. Voltages
    /// scale with `cells`; currents are pack-level (not per-cell) and so stay
    /// as configured by the caller.
    pub const fn for_pack(
        chemistry: Chemistry,
        cells: u8,
        enter_absorb_a: f32,
        exit_absorb_a: f32,
    ) -> Self {
        assert!(cells > 0);
        let (av, fv) = chemistry.per_cell();
        let s = cells as f32;
        Self {
            absorb_v: av * s,
            float_v: fv * s,
            enter_absorb_a,
            exit_absorb_a,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum Phase {
    Float,
    Absorb,
}

pub struct ChargeController {
    profile: Profile,
    phase: Phase,
}

impl ChargeController {
    pub fn new(profile: Profile) -> Self {
        assert!(profile.enter_absorb_a > profile.exit_absorb_a);
        assert!(profile.absorb_v > profile.float_v);
        Self {
            profile,
            phase: Phase::Float,
        }
    }

    pub fn phase(&self) -> Phase {
        self.phase
    }

    pub fn target_voltage(&self) -> f32 {
        match self.phase {
            Phase::Float => self.profile.float_v,
            Phase::Absorb => self.profile.absorb_v,
        }
    }

    /// Feed the latest signed battery current. Returns `Some(new_setpoint)`
    /// on a phase transition, `None` if the phase is unchanged.
    pub fn update(&mut self, battery_current_a: f32) -> Option<f32> {
        if !battery_current_a.is_finite() {
            return None;
        }
        // Charging current as a positive number.
        let charging_a = -battery_current_a;
        let next = match self.phase {
            Phase::Float if charging_a > self.profile.enter_absorb_a => Phase::Absorb,
            Phase::Absorb if charging_a < self.profile.exit_absorb_a => Phase::Float,
            p => p,
        };
        if next != self.phase {
            self.phase = next;
            Some(self.target_voltage())
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lfp_4s() -> Profile {
        // Pack-level defaults: 1 A enter, 0.5 A exit. Real CC-CV chargers
        // terminate absorb at 0.05C (5 A on 100 Ah), but they enter absorb on
        // VOLTAGE — we enter on current, which forces enter > exit so we don't
        // flap. 0.5 A keeps a usable hysteresis band without sitting at CV
        // forever. Absorb voltage (14.4 V) is the bigger longevity win.
        Profile::for_pack(Chemistry::LiFePo4, 4, 1.0, 0.5)
    }

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    // --- Profile construction ---

    #[test]
    fn lfp_4s_voltages_match_known_setpoints() {
        let p = lfp_4s();
        // 3.60 V × 4 = 14.4 V CV (daily-cycle); 3.375 V × 4 = 13.5 V float.
        assert!(approx(p.absorb_v, 14.4));
        assert!(approx(p.float_v, 13.5));
    }

    #[test]
    fn lfp_top_balance_uses_manufacturer_max() {
        // 3.65 V/cell — used only when BMS needs the headroom to balance.
        let p = Profile::for_pack(Chemistry::LiFePo4TopBalance, 4, 1.0, 2.0);
        assert!(approx(p.absorb_v, 14.6));
        assert!(approx(p.float_v, 13.5));
    }

    #[test]
    fn liion_3s_voltages_match_known_setpoints() {
        let p = Profile::for_pack(Chemistry::LiIon, 3, 1.0, 0.1);
        // Longevity-tuned: 4.10 × 3 = 12.3, 4.00 × 3 = 12.0.
        assert!(approx(p.absorb_v, 12.3));
        assert!(approx(p.float_v, 12.0));
    }

    #[test]
    fn voltages_scale_with_cell_count() {
        let p1 = Profile::for_pack(Chemistry::LiFePo4, 1, 1.0, 0.1);
        let p4 = Profile::for_pack(Chemistry::LiFePo4, 4, 1.0, 0.1);
        let p16 = Profile::for_pack(Chemistry::LiFePo4, 16, 1.0, 0.1);
        assert!(approx(p1.absorb_v, 3.60));
        assert!(approx(p4.absorb_v, 3.60 * 4.0));
        assert!(approx(p16.absorb_v, 3.60 * 16.0));
        assert!(approx(p1.float_v, 3.375));
        assert!(approx(p4.float_v, 3.375 * 4.0));
        assert!(approx(p16.float_v, 3.375 * 16.0));
    }

    #[test]
    fn currents_do_not_scale_with_cell_count() {
        // Pack-level current — independent of S.
        let p4 = Profile::for_pack(Chemistry::LiFePo4, 4, 2.5, 0.25);
        let p16 = Profile::for_pack(Chemistry::LiFePo4, 16, 2.5, 0.25);
        assert_eq!(p4.enter_absorb_a, 2.5);
        assert_eq!(p16.enter_absorb_a, 2.5);
        assert_eq!(p4.exit_absorb_a, 0.25);
        assert_eq!(p16.exit_absorb_a, 0.25);
    }

    #[test]
    #[should_panic]
    fn zero_cells_panics() {
        let _ = Profile::for_pack(Chemistry::LiFePo4, 0, 1.0, 0.1);
    }

    // --- Controller behavior ---

    #[test]
    fn starts_in_float_at_float_voltage() {
        let c = ChargeController::new(lfp_4s());
        assert!(matches!(c.phase(), Phase::Float));
        assert!(approx(c.target_voltage(), 13.5));
    }

    #[test]
    fn enters_absorb_when_charging_current_exceeds_threshold() {
        let mut c = ChargeController::new(lfp_4s());
        // charging at 1.5 A → -1.5 A on the bus; threshold is 1.0 A.
        let v = c.update(-1.5).unwrap();
        assert!(approx(v, 14.4));
        assert!(matches!(c.phase(), Phase::Absorb));
    }

    #[test]
    fn does_not_enter_absorb_at_exact_threshold() {
        // Strictly greater: 1.0 A must NOT trigger; 1.001 A must.
        let mut c = ChargeController::new(lfp_4s());
        assert_eq!(c.update(-1.0), None);
        assert!(matches!(c.phase(), Phase::Float));
        assert!(c.update(-1.001).is_some());
    }

    #[test]
    fn discharge_current_does_not_enter_absorb() {
        // 5 A discharge (positive). |I| > 1 A but it's NOT charging.
        let mut c = ChargeController::new(lfp_4s());
        assert_eq!(c.update(5.0), None);
        assert!(matches!(c.phase(), Phase::Float));
    }

    #[test]
    fn stays_in_absorb_above_exit_threshold() {
        let mut c = ChargeController::new(lfp_4s());
        c.update(-2.0); // → Absorb
        // Exit threshold is 0.5 A — anything above keeps us in absorb.
        assert_eq!(c.update(-1.0), None);
        assert_eq!(c.update(-0.6), None);
        assert_eq!(c.update(-0.5), None); // strictly less-than, so 0.5 stays
        assert!(matches!(c.phase(), Phase::Absorb));
    }

    #[test]
    fn exits_absorb_when_taper_drops_below_threshold() {
        let mut c = ChargeController::new(lfp_4s());
        c.update(-2.0); // → Absorb
        let v = c.update(-0.4).unwrap(); // 0.4 A charging — below 0.5 A exit.
        assert!(approx(v, 13.5));
        assert!(matches!(c.phase(), Phase::Float));
    }

    #[test]
    fn exits_absorb_when_load_pulls_current() {
        // Battery starts discharging mid-absorb (charger off / heavy load).
        // charging_a is negative → certainly < 0.1 A → drop to float.
        let mut c = ChargeController::new(lfp_4s());
        c.update(-2.0);
        let v = c.update(3.0).unwrap();
        assert!(approx(v, 13.5));
    }

    #[test]
    fn hysteresis_no_flap_between_thresholds() {
        let mut c = ChargeController::new(lfp_4s());
        for _ in 0..10 {
            assert_eq!(c.update(-0.5), None);
        }
        assert!(matches!(c.phase(), Phase::Float));
        c.update(-2.0);
        for _ in 0..10 {
            assert_eq!(c.update(-0.5), None);
        }
        assert!(matches!(c.phase(), Phase::Absorb));
    }

    #[test]
    fn returns_none_on_steady_state() {
        let mut c = ChargeController::new(lfp_4s());
        for _ in 0..100 {
            assert_eq!(c.update(-0.05), None);
        }
    }

    #[test]
    fn transition_only_emits_setpoint_once() {
        let mut c = ChargeController::new(lfp_4s());
        assert!(c.update(-2.0).is_some()); // first crossing → write
        assert_eq!(c.update(-2.0), None); // already absorb → silent
        assert_eq!(c.update(-3.0), None);
    }

    #[test]
    fn nan_and_inf_are_ignored() {
        let mut c = ChargeController::new(lfp_4s());
        assert_eq!(c.update(f32::NAN), None);
        assert_eq!(c.update(f32::INFINITY), None);
        assert_eq!(c.update(f32::NEG_INFINITY), None);
        assert!(matches!(c.phase(), Phase::Float));
    }

    #[test]
    fn different_chemistries_yield_different_setpoints() {
        let mut lfp = ChargeController::new(Profile::for_pack(Chemistry::LiFePo4, 4, 1.0, 0.1));
        let mut liion = ChargeController::new(Profile::for_pack(Chemistry::LiIon, 3, 1.0, 0.1));
        let v_lfp = lfp.update(-2.0).unwrap();
        let v_liion = liion.update(-2.0).unwrap();
        assert!(approx(v_lfp, 14.4));
        assert!(approx(v_liion, 12.3));
    }

    #[test]
    fn single_cell_lfp_works() {
        // 1S LFP charger — float 3.375 V, absorb 3.60 V (daily).
        let mut c = ChargeController::new(Profile::for_pack(Chemistry::LiFePo4, 1, 1.0, 0.1));
        assert!(approx(c.target_voltage(), 3.375));
        let v = c.update(-1.5).unwrap();
        assert!(approx(v, 3.60));
    }

    #[test]
    fn full_charge_cycle() {
        let mut c = ChargeController::new(lfp_4s());
        // Exit threshold is 0.5 A — design taper around that.
        let v_absorb = c.update(-8.0).unwrap();
        assert!(approx(v_absorb, 14.4));
        for &i in &[-7.0, -5.0, -3.0, -1.0, -0.6] {
            assert_eq!(c.update(i), None);
        }
        let v_float = c.update(-0.4).unwrap();
        assert!(approx(v_float, 13.5));
        for &i in &[-0.05, -0.02, 0.0, -0.4] {
            assert_eq!(c.update(i), None);
        }
    }
}
