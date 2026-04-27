//! High-level device API. One method per logical operation; all reads
//! and writes go through the [`crate::ModbusTransport`].

use crate::regs::*;
use crate::transport::{ModbusTransport, RtuError};
use crate::types::{
    BaudRate, GroupParams, OnTime, ProtectionStatus, RegMode, SafetyLimits, Setpoints, Status,
    TempUnit, Totals,
};

// Fixed-point conversion. Inputs are clamped to u16 — caller is responsible
// for staying within the device's documented ranges (per-model V/A/W limits).
fn to_reg(v: f32, scale: f32) -> u16 {
    let r = (v * scale + 0.5) as i32;
    r.clamp(0, u16::MAX as i32) as u16
}

fn from_reg(raw: u16, scale: f32) -> f32 {
    raw as f32 / scale
}

/// Driver for the XY-series buck converter.
///
/// Construct with [`Xy::new`] (default slave `0x01`) or
/// [`Xy::with_slave`]. All methods take `&mut self` because they go
/// through the transport.
pub struct Xy<T: ModbusTransport> {
    transport: T,
    slave: u8,
}

impl<T: ModbusTransport> Xy<T> {
    /// Wrap a transport using the default slave address (`0x01`).
    pub fn new(transport: T) -> Self {
        Self::with_slave(transport, DEFAULT_SLAVE)
    }

    pub fn with_slave(transport: T, slave: u8) -> Self {
        Self { transport, slave }
    }

    pub fn slave(&self) -> u8 {
        self.slave
    }

    /// Borrow the underlying transport.
    pub fn transport(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Consume the device and return the inner transport.
    pub fn into_transport(self) -> T {
        self.transport
    }

    // ─── Status & live readings ──────────────────────────────────────────────

    /// Read setpoints (V-SET, I-SET) — registers 0x0000–0x0001.
    pub fn read_setpoints(&mut self) -> Result<Setpoints, RtuError> {
        let mut r = [0u16; 2];
        self.transport.read_holding(self.slave, REG_V_SET, &mut r)?;
        Ok(Setpoints {
            v_set: from_reg(r[0], 100.0),
            i_set: from_reg(r[1], 100.0),
        })
    }

    /// Read the live status block (registers 0x0000–0x0005). Single
    /// 6-register transaction; recommended hot-loop poll.
    pub fn read_status(&mut self) -> Result<Status, RtuError> {
        let mut r = [0u16; 6];
        self.transport.read_holding(self.slave, REG_V_SET, &mut r)?;
        Ok(Status {
            v_set: from_reg(r[0], 100.0),
            i_set: from_reg(r[1], 100.0),
            v_out: from_reg(r[2], 100.0),
            i_out: from_reg(r[3], 100.0),
            p_out: from_reg(r[4], 10.0),
            v_in: from_reg(r[5], 100.0),
        })
    }

    pub fn read_voltage_out(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg(self.read_one(REG_V_OUT)?, 100.0))
    }
    pub fn read_current_out(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg(self.read_one(REG_I_OUT)?, 100.0))
    }
    pub fn read_power_out(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg(self.read_one(REG_P_OUT)?, 10.0))
    }
    pub fn read_voltage_in(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg(self.read_one(REG_V_IN)?, 100.0))
    }

    // ─── Cumulative totals ───────────────────────────────────────────────────

    /// Read cumulative output charge, energy, and on-time (registers
    /// 0x0006–0x000C, one transaction).
    pub fn read_totals(&mut self) -> Result<Totals, RtuError> {
        let mut r = [0u16; 7];
        self.transport.read_holding(self.slave, REG_AH_LOW, &mut r)?;
        let ah_raw = ((r[1] as u32) << 16) | r[0] as u32;
        let wh_raw = ((r[3] as u32) << 16) | r[2] as u32;
        Ok(Totals {
            charge_ah: ah_raw as f32 / 1000.0,
            energy_wh: wh_raw as f32 / 1000.0,
            on_time: OnTime {
                hours: r[4],
                minutes: r[5],
                seconds: r[6],
            },
            ah_low_raw: r[0],
            ah_high_raw: r[1],
            wh_low_raw: r[2],
            wh_high_raw: r[3],
        })
    }

    // ─── Setpoint shortcuts ──────────────────────────────────────────────────

    /// Set output voltage (V-SET, register 0x0000). Note: writing a
    /// V-SET higher than the current S-OVP latches OVP immediately —
    /// program protection (see [`Self::set_protection`]) first.
    pub fn set_voltage(&mut self, volts: f32) -> Result<(), RtuError> {
        self.write_one(REG_V_SET, to_reg(volts, 100.0))
    }

    pub fn set_current_limit(&mut self, amps: f32) -> Result<(), RtuError> {
        self.write_one(REG_I_SET, to_reg(amps, 100.0))
    }

    /// Program LVP / OVP / OCP into the active group's protection
    /// registers (0x0052–0x0054) in one bulk write.
    pub fn set_protection(&mut self, l: SafetyLimits) -> Result<(), RtuError> {
        let values = [
            to_reg(l.lvp_v, 100.0),
            to_reg(l.ovp_v, 100.0),
            to_reg(l.ocp_a, 100.0),
        ];
        self.transport
            .write_multiple_holdings(self.slave, REG_S_LVP, &values)
    }

    /// Read LVP / OVP / OCP from the active group (0x0052–0x0054).
    pub fn read_protection(&mut self) -> Result<SafetyLimits, RtuError> {
        let mut r = [0u16; 3];
        self.transport.read_holding(self.slave, REG_S_LVP, &mut r)?;
        Ok(SafetyLimits {
            lvp_v: from_reg(r[0], 100.0),
            ovp_v: from_reg(r[1], 100.0),
            ocp_a: from_reg(r[2], 100.0),
        })
    }

    /// Power-on output state (S-INI, register 0x005D). `false` = OFF
    /// at boot, `true` = ON. Persists in EEPROM. `false` is the safe
    /// default after an unexpected power loss — the buck stays disabled
    /// until explicitly re-enabled.
    pub fn set_power_on_output(&mut self, on: bool) -> Result<(), RtuError> {
        self.write_one(REG_S_INI, on as u16)
    }

    pub fn read_power_on_output(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_S_INI)? != 0)
    }

    // ─── Output enable & protection status ───────────────────────────────────

    /// Read the output-enable register (ONOFF, 0x0012).
    pub fn read_output(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_OUTPUT_EN)? != 0)
    }

    pub fn set_output(&mut self, on: bool) -> Result<(), RtuError> {
        self.write_one(REG_OUTPUT_EN, on as u16)
    }

    /// Read the latched protection cause (PROTECT, 0x0010). While the
    /// output is on, this register is necessarily `Normal` — only worth
    /// reading after observing OUTPUT_EN go low.
    pub fn read_protection_status(&mut self) -> Result<ProtectionStatus, RtuError> {
        Ok(ProtectionStatus::from_register(self.read_one(REG_PROTECT)?))
    }

    /// Clear a latched protection cause (write 0 to PROTECT). This
    /// stops the front-panel blink but does **not** re-enable the
    /// output — call [`Self::set_output`] separately.
    pub fn clear_protection_status(&mut self) -> Result<(), RtuError> {
        self.write_one(REG_PROTECT, 0)
    }

    pub fn read_reg_mode(&mut self) -> Result<RegMode, RtuError> {
        Ok(if self.read_one(REG_CVCC)? == 0 {
            RegMode::ConstantVoltage
        } else {
            RegMode::ConstantCurrent
        })
    }

    // ─── Temperatures ────────────────────────────────────────────────────────

    /// Returns `(internal, external)` in the unit selected by
    /// [`Self::read_temp_unit`].
    pub fn read_temperatures(&mut self) -> Result<(f32, f32), RtuError> {
        let mut r = [0u16; 2];
        self.transport.read_holding(self.slave, REG_T_IN, &mut r)?;
        Ok((from_reg(r[0], 10.0), from_reg(r[1], 10.0)))
    }

    pub fn read_temp_unit(&mut self) -> Result<TempUnit, RtuError> {
        Ok(TempUnit::from_reg(self.read_one(REG_TEMP_UNIT)?))
    }
    pub fn set_temp_unit(&mut self, unit: TempUnit) -> Result<(), RtuError> {
        self.write_one(REG_TEMP_UNIT, unit.to_reg())
    }

    pub fn read_temp_offset_internal(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg(self.read_one(REG_T_IN_OFFSET)?, 10.0))
    }
    pub fn set_temp_offset_internal(&mut self, offset: f32) -> Result<(), RtuError> {
        self.write_one(REG_T_IN_OFFSET, to_reg(offset, 10.0))
    }
    pub fn read_temp_offset_external(&mut self) -> Result<f32, RtuError> {
        Ok(from_reg(self.read_one(REG_T_EX_OFFSET)?, 10.0))
    }
    pub fn set_temp_offset_external(&mut self, offset: f32) -> Result<(), RtuError> {
        self.write_one(REG_T_EX_OFFSET, to_reg(offset, 10.0))
    }

    // ─── Front panel & misc ──────────────────────────────────────────────────

    pub fn read_lock(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_LOCK)? != 0)
    }
    pub fn set_lock(&mut self, locked: bool) -> Result<(), RtuError> {
        self.write_one(REG_LOCK, locked as u16)
    }

    /// Backlight brightness (0–5).
    pub fn read_backlight(&mut self) -> Result<u8, RtuError> {
        Ok(self.read_one(REG_BACKLIGHT)? as u8)
    }
    pub fn set_backlight(&mut self, level: u8) -> Result<(), RtuError> {
        self.write_one(REG_BACKLIGHT, level as u16)
    }

    /// Off-screen timeout in minutes.
    pub fn read_sleep_minutes(&mut self) -> Result<u16, RtuError> {
        self.read_one(REG_SLEEP)
    }
    pub fn set_sleep_minutes(&mut self, minutes: u16) -> Result<(), RtuError> {
        self.write_one(REG_SLEEP, minutes)
    }

    /// Buzzer enable. Often unimplemented in firmware.
    pub fn read_buzzer(&mut self) -> Result<bool, RtuError> {
        Ok(self.read_one(REG_BUZZER)? != 0)
    }
    pub fn set_buzzer(&mut self, on: bool) -> Result<(), RtuError> {
        self.write_one(REG_BUZZER, on as u16)
    }

    // ─── Identity & comms config ─────────────────────────────────────────────

    /// Product number (e.g. `0x6100`).
    pub fn read_model(&mut self) -> Result<u16, RtuError> {
        self.read_one(REG_MODEL)
    }

    /// Firmware version (e.g. `0x0071`).
    pub fn read_version(&mut self) -> Result<u16, RtuError> {
        self.read_one(REG_VERSION)
    }

    /// Read the device's currently configured Modbus slave address.
    /// Note: [`Self::slave`] is the address the *driver* is talking to;
    /// they may differ briefly while reconfiguring.
    pub fn read_slave_address(&mut self) -> Result<u8, RtuError> {
        Ok(self.read_one(REG_SLAVE_ADDR)? as u8)
    }
    /// Write a new slave address. Takes effect after the device resets.
    pub fn set_slave_address(&mut self, addr: u8) -> Result<(), RtuError> {
        self.write_one(REG_SLAVE_ADDR, addr as u16)
    }

    pub fn read_baud_rate(&mut self) -> Result<BaudRate, RtuError> {
        Ok(BaudRate::from_code(self.read_one(REG_BAUD_CODE)?))
    }
    /// Write a new baud-rate code. Takes effect after the device resets.
    pub fn set_baud_rate(&mut self, baud: BaudRate) -> Result<(), RtuError> {
        self.write_one(REG_BAUD_CODE, baud.code())
    }

    /// Recall a stored memory group (M0–M9) into the live operating set.
    /// Writing 0 is a no-op (M0 is already current).
    pub fn recall_group(&mut self, n: u8) -> Result<(), RtuError> {
        self.write_one(REG_EXTRACT_M, n as u16)
    }

    // ─── Memory groups (M0–M9) ───────────────────────────────────────────────

    /// Read all 14 registers of memory group `n` (0–9).
    pub fn read_group(&mut self, n: u8) -> Result<GroupParams, RtuError> {
        debug_assert!(n < GROUP_COUNT);
        let mut r = [0u16; GROUP_LEN as usize];
        self.transport
            .read_holding(self.slave, group_addr(n), &mut r)?;
        Ok(decode_group(&r))
    }

    /// Write all 14 registers of memory group `n` (0–9) in one bulk
    /// transaction. For M0 this updates the live operating set;
    /// otherwise it programs EEPROM and takes effect on
    /// [`Self::recall_group`].
    pub fn write_group(&mut self, n: u8, p: &GroupParams) -> Result<(), RtuError> {
        debug_assert!(n < GROUP_COUNT);
        let regs = encode_group(p);
        self.transport
            .write_multiple_holdings(self.slave, group_addr(n), &regs)
    }

    // ─── Internals ───────────────────────────────────────────────────────────

    fn read_one(&mut self, addr: u16) -> Result<u16, RtuError> {
        let mut r = [0u16; 1];
        self.transport.read_holding(self.slave, addr, &mut r)?;
        Ok(r[0])
    }

    fn write_one(&mut self, addr: u16, value: u16) -> Result<(), RtuError> {
        self.transport
            .write_single_holding(self.slave, addr, value)
    }
}

// ─── Group encode / decode ──────────────────────────────────────────────────

fn decode_group(r: &[u16; GROUP_LEN as usize]) -> GroupParams {
    GroupParams {
        v_set: from_reg(r[GROUP_OFF_V_SET as usize], 100.0),
        i_set: from_reg(r[GROUP_OFF_I_SET as usize], 100.0),
        s_lvp_v: from_reg(r[GROUP_OFF_S_LVP as usize], 100.0),
        s_ovp_v: from_reg(r[GROUP_OFF_S_OVP as usize], 100.0),
        s_ocp_a: from_reg(r[GROUP_OFF_S_OCP as usize], 100.0),
        s_opp_w: r[GROUP_OFF_S_OPP as usize],
        s_ohp_h: r[GROUP_OFF_S_OHP_H as usize],
        s_ohp_m: r[GROUP_OFF_S_OHP_M as usize],
        s_oah_low: r[GROUP_OFF_S_OAH_L as usize],
        s_oah_high: r[GROUP_OFF_S_OAH_H as usize],
        s_owh_low: r[GROUP_OFF_S_OWH_L as usize],
        s_owh_high: r[GROUP_OFF_S_OWH_H as usize],
        s_otp: from_reg(r[GROUP_OFF_S_OTP as usize], 10.0),
        power_on_output: r[GROUP_OFF_S_INI as usize] != 0,
    }
}

fn encode_group(p: &GroupParams) -> [u16; GROUP_LEN as usize] {
    let mut r = [0u16; GROUP_LEN as usize];
    r[GROUP_OFF_V_SET as usize] = to_reg(p.v_set, 100.0);
    r[GROUP_OFF_I_SET as usize] = to_reg(p.i_set, 100.0);
    r[GROUP_OFF_S_LVP as usize] = to_reg(p.s_lvp_v, 100.0);
    r[GROUP_OFF_S_OVP as usize] = to_reg(p.s_ovp_v, 100.0);
    r[GROUP_OFF_S_OCP as usize] = to_reg(p.s_ocp_a, 100.0);
    r[GROUP_OFF_S_OPP as usize] = p.s_opp_w;
    r[GROUP_OFF_S_OHP_H as usize] = p.s_ohp_h;
    r[GROUP_OFF_S_OHP_M as usize] = p.s_ohp_m;
    r[GROUP_OFF_S_OAH_L as usize] = p.s_oah_low;
    r[GROUP_OFF_S_OAH_H as usize] = p.s_oah_high;
    r[GROUP_OFF_S_OWH_L as usize] = p.s_owh_low;
    r[GROUP_OFF_S_OWH_H as usize] = p.s_owh_high;
    r[GROUP_OFF_S_OTP as usize] = to_reg(p.s_otp, 10.0);
    r[GROUP_OFF_S_INI as usize] = p.power_on_output as u16;
    r
}

#[cfg(test)]
mod tests {
    extern crate std;
    use std::vec;
    use std::vec::Vec;

    use super::*;
    use crate::transport::ModbusError;

    /// Scriptable transport for tests. Each script entry pairs a
    /// register-or-value request with a canned response or error.
    enum Op {
        Read {
            addr: u16,
            values: Vec<u16>,
        },
        WriteOne {
            addr: u16,
            value: u16,
        },
        WriteMany {
            addr: u16,
            values: Vec<u16>,
        },
    }

    struct MockTransport {
        script: Vec<Op>,
    }

    impl MockTransport {
        fn new(script: Vec<Op>) -> Self {
            Self { script }
        }
    }

    impl Drop for MockTransport {
        fn drop(&mut self) {
            if !std::thread::panicking() {
                assert!(
                    self.script.is_empty(),
                    "{} unconsumed mock ops",
                    self.script.len()
                );
            }
        }
    }

    impl ModbusTransport for MockTransport {
        fn read_holding(
            &mut self,
            _slave: u8,
            addr: u16,
            dst: &mut [u16],
        ) -> Result<(), RtuError> {
            let op = self.script.remove(0);
            match op {
                Op::Read { addr: a, values } => {
                    assert_eq!(addr, a);
                    assert_eq!(dst.len(), values.len());
                    dst.copy_from_slice(&values);
                    Ok(())
                }
                _ => panic!("expected read"),
            }
        }
        fn write_single_holding(
            &mut self,
            _slave: u8,
            addr: u16,
            value: u16,
        ) -> Result<(), RtuError> {
            let op = self.script.remove(0);
            match op {
                Op::WriteOne { addr: a, value: v } => {
                    assert_eq!(addr, a);
                    assert_eq!(value, v);
                    Ok(())
                }
                _ => panic!("expected write_single"),
            }
        }
        fn write_multiple_holdings(
            &mut self,
            _slave: u8,
            addr: u16,
            values: &[u16],
        ) -> Result<(), RtuError> {
            let op = self.script.remove(0);
            match op {
                Op::WriteMany { addr: a, values: v } => {
                    assert_eq!(addr, a);
                    assert_eq!(values, v.as_slice());
                    Ok(())
                }
                _ => panic!("expected write_multiple"),
            }
        }
    }

    #[test]
    fn read_status_decodes_six_regs() {
        // 1440 → 14.40 V; 1000 → 10.00 A; 1350 → 13.50 V; 50 → 0.50 A;
        // P_OUT scale 10, so 675 → 67.5 W; 2400 → 24.00 V.
        let mock = MockTransport::new(vec![Op::Read {
            addr: REG_V_SET,
            values: vec![1440, 1000, 1350, 50, 675, 2400],
        }]);
        let mut xy = Xy::new(mock);
        let s = xy.read_status().unwrap();
        assert_eq!(s.v_set, 14.40);
        assert_eq!(s.i_set, 10.00);
        assert_eq!(s.v_out, 13.50);
        assert_eq!(s.i_out, 0.50);
        assert_eq!(s.p_out, 67.5);
        assert_eq!(s.v_in, 24.00);
    }

    #[test]
    fn set_voltage_scales_correctly() {
        // 14.40 V → 1440 raw.
        let mock = MockTransport::new(vec![Op::WriteOne {
            addr: REG_V_SET,
            value: 1440,
        }]);
        let mut xy = Xy::new(mock);
        xy.set_voltage(14.40).unwrap();
    }

    #[test]
    fn set_protection_uses_bulk_write() {
        // LVP=10.00, OVP=15.00, OCP=12.50 → raw 1000, 1500, 1250.
        let mock = MockTransport::new(vec![Op::WriteMany {
            addr: REG_S_LVP,
            values: vec![1000, 1500, 1250],
        }]);
        let mut xy = Xy::new(mock);
        xy.set_protection(SafetyLimits {
            lvp_v: 10.0,
            ovp_v: 15.0,
            ocp_a: 12.5,
        })
        .unwrap();
    }

    #[test]
    fn read_protection_decodes_three_regs() {
        let mock = MockTransport::new(vec![Op::Read {
            addr: REG_S_LVP,
            values: vec![1000, 1500, 1250],
        }]);
        let mut xy = Xy::new(mock);
        let l = xy.read_protection().unwrap();
        assert_eq!(l.lvp_v, 10.0);
        assert_eq!(l.ovp_v, 15.0);
        assert_eq!(l.ocp_a, 12.5);
    }

    #[test]
    fn protection_status_decodes_known_codes() {
        let mock = MockTransport::new(vec![
            Op::Read {
                addr: REG_PROTECT,
                values: vec![0],
            },
            Op::Read {
                addr: REG_PROTECT,
                values: vec![4],
            },
            Op::Read {
                addr: REG_PROTECT,
                values: vec![99],
            },
        ]);
        let mut xy = Xy::new(mock);
        assert_eq!(xy.read_protection_status().unwrap(), ProtectionStatus::Normal);
        assert_eq!(xy.read_protection_status().unwrap(), ProtectionStatus::Lvp);
        assert_eq!(
            xy.read_protection_status().unwrap(),
            ProtectionStatus::Unknown(99)
        );
    }

    #[test]
    fn read_totals_composes_high_low() {
        // ah = (high<<16 | low) / 1000
        // pick high=2, low=500 → raw=131_572 → 131.572 Ah.
        // wh: high=0, low=12345 → 12.345 Wh.
        // on_time h=1, m=23, s=45.
        let mock = MockTransport::new(vec![Op::Read {
            addr: REG_AH_LOW,
            values: vec![500, 2, 12345, 0, 1, 23, 45],
        }]);
        let mut xy = Xy::new(mock);
        let t = xy.read_totals().unwrap();
        assert_eq!(t.charge_ah, 131.572);
        assert_eq!(t.energy_wh, 12.345);
        assert_eq!(
            t.on_time,
            OnTime {
                hours: 1,
                minutes: 23,
                seconds: 45
            }
        );
        assert_eq!(t.on_time.total_seconds(), 5025);
    }

    #[test]
    fn read_group_decodes_14_regs() {
        let mock = MockTransport::new(vec![Op::Read {
            addr: group_addr(1),
            values: vec![
                1440, // v_set
                1000, // i_set
                1000, // s_lvp
                1500, // s_ovp
                1250, // s_ocp
                1800, // s_opp (W, scale 1)
                0,    // ohp_h
                0,    // ohp_m
                0,    // oah_l
                0,    // oah_h
                0,    // owh_l
                0,    // owh_h
                950,  // s_otp (scale 10 → 95.0)
                0,    // s_ini
            ],
        }]);
        let mut xy = Xy::new(mock);
        let g = xy.read_group(1).unwrap();
        assert_eq!(g.v_set, 14.40);
        assert_eq!(g.s_ovp_v, 15.00);
        assert_eq!(g.s_opp_w, 1800);
        assert_eq!(g.s_otp, 95.0);
        assert!(!g.power_on_output);
    }

    #[test]
    fn write_group_round_trips_through_encode() {
        let p = GroupParams {
            v_set: 14.40,
            i_set: 10.00,
            s_lvp_v: 10.00,
            s_ovp_v: 15.00,
            s_ocp_a: 12.50,
            s_opp_w: 1800,
            s_ohp_h: 0,
            s_ohp_m: 0,
            s_oah_low: 0,
            s_oah_high: 0,
            s_owh_low: 0,
            s_owh_high: 0,
            s_otp: 95.0,
            power_on_output: true,
        };
        let mock = MockTransport::new(vec![Op::WriteMany {
            addr: group_addr(2),
            values: vec![1440, 1000, 1000, 1500, 1250, 1800, 0, 0, 0, 0, 0, 0, 950, 1],
        }]);
        let mut xy = Xy::new(mock);
        xy.write_group(2, &p).unwrap();
    }

    #[test]
    fn baud_round_trip() {
        for baud in [
            BaudRate::B2400,
            BaudRate::B4800,
            BaudRate::B9600,
            BaudRate::B14400,
            BaudRate::B19200,
            BaudRate::B38400,
            BaudRate::B56000,
            BaudRate::B57600,
            BaudRate::B115200,
        ] {
            assert_eq!(BaudRate::from_code(baud.code()), baud);
        }
        assert_eq!(BaudRate::from_code(99), BaudRate::Unknown(99));
        // Unknown round-trips its raw code.
        assert_eq!(BaudRate::Unknown(99).code(), 99);
        assert_eq!(BaudRate::Unknown(99).baud(), None);
        assert_eq!(BaudRate::B9600.baud(), Some(9600));
    }

    #[test]
    fn rtu_error_propagates() {
        struct FailRead;
        impl ModbusTransport for FailRead {
            fn read_holding(
                &mut self,
                _: u8,
                _: u16,
                _: &mut [u16],
            ) -> Result<(), RtuError> {
                Err(RtuError::Modbus(ModbusError::BadCrc))
            }
            fn write_single_holding(&mut self, _: u8, _: u16, _: u16) -> Result<(), RtuError> {
                unreachable!()
            }
            fn write_multiple_holdings(
                &mut self,
                _: u8,
                _: u16,
                _: &[u16],
            ) -> Result<(), RtuError> {
                unreachable!()
            }
        }
        let mut xy = Xy::new(FailRead);
        assert!(matches!(
            xy.read_voltage_out(),
            Err(RtuError::Modbus(ModbusError::BadCrc))
        ));
    }
}
