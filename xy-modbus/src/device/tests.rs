extern crate std;
use std::vec;
use std::vec::Vec;

use super::*;
use crate::transport::ModbusError;

/// Scriptable transport for tests. Each script entry pairs a
/// register-or-value request with a canned response or error.
enum Op {
    Read { addr: u16, values: Vec<u16> },
    WriteOne { addr: u16, value: u16 },
    WriteMany { addr: u16, values: Vec<u16> },
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
    fn read_holding(&mut self, _slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError> {
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
    let mut xy = Xy::new(mock, Model::Xy7025);
    let s = xy.read_status().unwrap();
    assert_eq!(s.v_set, 14.40);
    assert_eq!(s.i_set, 10.00);
    assert_eq!(s.v_out, 13.50);
    assert_eq!(s.i_out, 0.50);
    assert_eq!(s.p_out, 67.5);
    assert_eq!(s.v_in, 24.00);
}

/// Same wire bytes decoded under XY7025 vs SK120 must yield 10× different
/// physical values for I-SET, IOUT, S-OCP and POWER (per DATASHEET §3
/// model scale table). Locks in that `Model` actually changes behavior —
/// a no-op `current_scale` would silently report the same numbers.
#[test]
fn model_scales_diverge_between_xy7025_and_sk120() {
    // 1000 raw with /100 → 10.00 A, with /1000 → 1.000 A.
    // 675 raw with /10 → 67.5 W, with /100 → 6.75 W.
    let xy7025_mock = MockTransport::new(vec![Op::Read {
        addr: REG_V_SET,
        values: vec![1440, 1000, 1350, 1000, 675, 2400],
    }]);
    let mut xy = Xy::new(xy7025_mock, Model::Xy7025);
    let s = xy.read_status().unwrap();
    assert_eq!(s.i_set, 10.00);
    assert_eq!(s.i_out, 10.00);
    assert_eq!(s.p_out, 67.5);

    let sk_mock = MockTransport::new(vec![Op::Read {
        addr: REG_V_SET,
        values: vec![1440, 1000, 1350, 1000, 675, 2400],
    }]);
    let mut xy = Xy::new(sk_mock, Model::Sk120);
    let s = xy.read_status().unwrap();
    assert_eq!(s.i_set, 1.000);
    assert_eq!(s.i_out, 1.000);
    assert_eq!(s.p_out, 6.75);

    // V_SET, V_OUT, V_IN scales are model-invariant (always /100).
    assert_eq!(s.v_set, 14.40);
    assert_eq!(s.v_out, 13.50);
    assert_eq!(s.v_in, 24.00);
}

/// `Model::Custom` lets users dial in scales for hardware not covered
/// by the preset variants. Verify the three scale getters route
/// through the supplied values verbatim.
#[test]
fn custom_model_routes_user_supplied_scales() {
    let m = Model::Custom {
        current_scale: 500.0,
        power_scale: 25.0,
        opp_scale: 4.0,
    };
    assert_eq!(m.current_scale(), 500.0);
    assert_eq!(m.power_scale(), 25.0);
    assert_eq!(m.opp_scale(), 4.0);
}

#[test]
fn set_voltage_scales_correctly() {
    // 14.40 V → 1440 raw.
    let mock = MockTransport::new(vec![Op::WriteOne {
        addr: REG_V_SET,
        value: 1440,
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    xy.set_voltage(14.40).unwrap();
}

#[test]
fn set_protection_uses_bulk_write() {
    // LVP=10.00, OVP=15.00, OCP=12.50 → raw 1000, 1500, 1250.
    let mock = MockTransport::new(vec![Op::WriteMany {
        addr: REG_S_LVP,
        values: vec![1000, 1500, 1250],
    }]);
    let mut xy = Xy::new(mock, Model::Xy7025);
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
    let mut xy = Xy::new(mock, Model::Xy7025);
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
    let mut xy = Xy::new(mock, Model::Xy7025);
    assert_eq!(
        xy.read_protection_status().unwrap(),
        ProtectionStatus::Normal
    );
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
    let mut xy = Xy::new(mock, Model::Xy7025);
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
    let mut xy = Xy::new(mock, Model::Xy7025);
    let g = xy.read_group(1).unwrap();
    assert_eq!(g.v_set, 14.40);
    assert_eq!(g.s_ovp_v, 15.00);
    assert_eq!(g.s_opp_w, 1800.0);
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
        s_opp_w: 1800.0,
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
    let mut xy = Xy::new(mock, Model::Xy7025);
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
fn group_encode_decode_round_trip() {
    // Pin all 14 register offsets in one go — an offset swap would
    // surface as a field mismatch here.
    let p = GroupParams {
        v_set: 14.40,
        i_set: 10.00,
        s_lvp_v: 10.00,
        s_ovp_v: 15.00,
        s_ocp_a: 12.50,
        s_opp_w: 1800.0,
        s_ohp_h: 7,
        s_ohp_m: 30,
        s_oah_low: 500,
        s_oah_high: 2,
        s_owh_low: 12_345,
        s_owh_high: 0,
        s_otp: 95.0,
        power_on_output: true,
    };
    let regs = encode_group(&p, Model::Xy7025);
    let decoded = decode_group(&regs, Model::Xy7025);
    assert_eq!(decoded.v_set, p.v_set);
    assert_eq!(decoded.i_set, p.i_set);
    assert_eq!(decoded.s_lvp_v, p.s_lvp_v);
    assert_eq!(decoded.s_ovp_v, p.s_ovp_v);
    assert_eq!(decoded.s_ocp_a, p.s_ocp_a);
    assert_eq!(decoded.s_opp_w, p.s_opp_w);
    assert_eq!(decoded.s_ohp_h, p.s_ohp_h);
    assert_eq!(decoded.s_ohp_m, p.s_ohp_m);
    assert_eq!(decoded.s_oah_low, p.s_oah_low);
    assert_eq!(decoded.s_oah_high, p.s_oah_high);
    assert_eq!(decoded.s_owh_low, p.s_owh_low);
    assert_eq!(decoded.s_owh_high, p.s_owh_high);
    assert_eq!(decoded.s_otp, p.s_otp);
    assert_eq!(decoded.power_on_output, p.power_on_output);
}

/// Pins (register address, raw value) for each one-shot setter. A wrong
/// REG_* constant or a wrong scale would surface here. Inlined per-call
/// rather than table-driven to keep `no_std` (no `Box`) and let each row
/// take its own typed argument.
#[test]
fn one_shot_setters_use_correct_addr_and_value() {
    macro_rules! check {
        ($addr:expr, $value:expr, $action:expr) => {{
            let mock = MockTransport::new(vec![Op::WriteOne {
                addr: $addr,
                value: $value,
            }]);
            let mut xy = Xy::new(mock, Model::Xy7025);
            $action(&mut xy).unwrap();
        }};
    }
    check!(REG_V_SET, 1440, |x: &mut Xy<_>| x.set_voltage(14.40));
    check!(REG_I_SET, 500, |x: &mut Xy<_>| x.set_current_limit(5.00));
    check!(REG_OUTPUT_EN, 1, |x: &mut Xy<_>| x.set_output(true));
    check!(REG_OUTPUT_EN, 0, |x: &mut Xy<_>| x.set_output(false));
    check!(REG_PROTECT, 0, |x: &mut Xy<_>| x.clear_protection_status());
    check!(REG_LOCK, 1, |x: &mut Xy<_>| x.set_lock(true));
    check!(REG_BACKLIGHT, 3, |x: &mut Xy<_>| x.set_backlight(3));
    check!(REG_SLEEP, 12, |x: &mut Xy<_>| x.set_sleep_minutes(12));
    check!(REG_BUZZER, 1, |x: &mut Xy<_>| x.set_buzzer(true));
    // -2.5 °C → -25 raw → clamped to 0 by `to_reg`.
    check!(REG_T_IN_OFFSET, 0, |x: &mut Xy<_>| x
        .set_temp_offset_internal(-2.5));
    check!(REG_T_IN_OFFSET, 15, |x: &mut Xy<_>| x
        .set_temp_offset_internal(1.5));
    check!(REG_T_EX_OFFSET, 20, |x: &mut Xy<_>| x
        .set_temp_offset_external(2.0));
    check!(REG_SLAVE_ADDR, 7, |x: &mut Xy<_>| x.set_slave_address(7));
    check!(REG_S_INI, 1, |x: &mut Xy<_>| x.set_power_on_output(true));
    check!(REG_EXTRACT_M, 3, |x: &mut Xy<_>| x.recall_group(3));
    check!(REG_BAUD_CODE, 6, |x: &mut Xy<_>| x.set_baud_rate(BaudRate::B115200));
    check!(REG_BAUD_CODE, 99, |x: &mut Xy<_>| x.set_baud_rate(BaudRate::Unknown(99)));
    check!(REG_TEMP_UNIT, 1, |x: &mut Xy<_>| x.set_temp_unit(TempUnit::Fahrenheit));
}

/// Pins (register address, returned raw, expected decoded value) for each
/// single-register getter. A wrong REG_* constant or scale would surface here.
#[test]
fn one_shot_getters_use_correct_addr_and_scale() {
    macro_rules! check {
        ($addr:expr, $raw:expr, $action:expr, $expected:expr) => {{
            let mock = MockTransport::new(vec![Op::Read {
                addr: $addr,
                values: vec![$raw],
            }]);
            let mut xy = Xy::new(mock, Model::Xy7025);
            assert_eq!($action(&mut xy).unwrap(), $expected);
        }};
    }
    check!(REG_V_OUT, 1234, |x: &mut Xy<_>| x.read_voltage_out(), 12.34);
    check!(REG_I_OUT, 500, |x: &mut Xy<_>| x.read_current_out(), 5.00);
    check!(REG_P_OUT, 675, |x: &mut Xy<_>| x.read_power_out(), 67.5);
    check!(REG_V_IN, 2400, |x: &mut Xy<_>| x.read_voltage_in(), 24.00);
    check!(REG_OUTPUT_EN, 1, |x: &mut Xy<_>| x.read_output(), true);
    check!(REG_LOCK, 1, |x: &mut Xy<_>| x.read_lock(), true);
    check!(REG_BACKLIGHT, 4, |x: &mut Xy<_>| x.read_backlight(), 4u8);
    check!(REG_SLEEP, 15, |x: &mut Xy<_>| x.read_sleep_minutes(), 15u16);
    check!(REG_BUZZER, 0, |x: &mut Xy<_>| x.read_buzzer(), false);
    check!(REG_SLAVE_ADDR, 7, |x: &mut Xy<_>| x.read_slave_address(), 7u8);
    check!(REG_S_INI, 1, |x: &mut Xy<_>| x.read_power_on_output(), true);
    check!(REG_MODEL, 0x6100, |x: &mut Xy<_>| x.read_model(), 0x6100u16);
    check!(REG_VERSION, 0x71, |x: &mut Xy<_>| x.read_version(), 0x71u16);
    check!(REG_T_IN_OFFSET, 15, |x: &mut Xy<_>| x.read_temp_offset_internal(), 1.5);
    check!(REG_T_EX_OFFSET, 20, |x: &mut Xy<_>| x.read_temp_offset_external(), 2.0);
    check!(REG_CVCC, 0, |x: &mut Xy<_>| x.read_reg_mode(), RegMode::ConstantVoltage);
    check!(REG_CVCC, 1, |x: &mut Xy<_>| x.read_reg_mode(), RegMode::ConstantCurrent);
    check!(REG_BAUD_CODE, 6, |x: &mut Xy<_>| x.read_baud_rate(), BaudRate::B115200);

    // 2-reg bulk reads.
    let mut xy = Xy::new(
        MockTransport::new(vec![Op::Read {
            addr: REG_V_SET,
            values: vec![1440, 1000],
        }]),
        Model::Xy7025,
    );
    let s = xy.read_setpoints().unwrap();
    assert_eq!((s.v_set, s.i_set), (14.40, 10.00));

    let mut xy = Xy::new(
        MockTransport::new(vec![Op::Read {
            addr: REG_T_IN,
            values: vec![345, 256],
        }]),
        Model::Xy7025,
    );
    assert_eq!(xy.read_temperatures().unwrap(), (34.5, 25.6));
}

#[test]
fn temp_unit_round_trip() {
    // Write Celsius, read back Celsius (0); same for Fahrenheit (1).
    let mock = MockTransport::new(vec![
        Op::WriteOne {
            addr: REG_TEMP_UNIT,
            value: TempUnit::Celsius.to_reg(),
        },
        Op::Read {
            addr: REG_TEMP_UNIT,
            values: vec![TempUnit::Celsius.to_reg()],
        },
        Op::WriteOne {
            addr: REG_TEMP_UNIT,
            value: TempUnit::Fahrenheit.to_reg(),
        },
        Op::Read {
            addr: REG_TEMP_UNIT,
            values: vec![TempUnit::Fahrenheit.to_reg()],
        },
    ]);
    let mut xy = Xy::new(mock, Model::Xy7025);
    xy.set_temp_unit(TempUnit::Celsius).unwrap();
    assert_eq!(xy.read_temp_unit().unwrap(), TempUnit::Celsius);
    xy.set_temp_unit(TempUnit::Fahrenheit).unwrap();
    assert_eq!(xy.read_temp_unit().unwrap(), TempUnit::Fahrenheit);
}

/// All five preset variants must report the documented scales (DATASHEET §3).
/// Custom is verified separately in `custom_model_routes_user_supplied_scales`.
#[test]
fn preset_models_match_datasheet_scales() {
    // (model, current, power, opp)
    let presets = [
        (Model::Xy6020L, 100.0, 10.0, 1.0),
        (Model::Xy7025, 100.0, 10.0, 1.0),
        (Model::Sk60, 1000.0, 100.0, 10.0),
        (Model::Sk120, 1000.0, 100.0, 10.0),
        (Model::Sk120x, 1000.0, 100.0, 10.0),
    ];
    for (m, i, p, o) in presets {
        assert_eq!(m.current_scale(), i);
        assert_eq!(m.power_scale(), p);
        assert_eq!(m.opp_scale(), o);
    }
}

/// Group encode under SK-family must use opp_scale=10 (S-OPP raw stores
/// 0.1 W units), distinct from XY7025/XY6020L which store whole watts.
#[test]
fn group_encode_under_sk120_uses_opp_scale_10() {
    // 18.0 W S-OPP → raw 180 on SK family, would be 18 on XY7025.
    let p = GroupParams {
        v_set: 5.0,
        i_set: 1.0,
        s_lvp_v: 0.0,
        s_ovp_v: 0.0,
        s_ocp_a: 0.0,
        s_opp_w: 18.0,
        s_ohp_h: 0,
        s_ohp_m: 0,
        s_oah_low: 0,
        s_oah_high: 0,
        s_owh_low: 0,
        s_owh_high: 0,
        s_otp: 0.0,
        power_on_output: false,
    };
    let mock = MockTransport::new(vec![Op::WriteMany {
        addr: group_addr(0),
        // v_set=500, i_set=1000 (×1000 SK), s_opp=180, s_otp=0.
        values: vec![500, 1000, 0, 0, 0, 180, 0, 0, 0, 0, 0, 0, 0, 0],
    }]);
    let mut xy = Xy::new(mock, Model::Sk120);
    xy.write_group(0, &p).unwrap();
}

#[test]
fn rtu_error_propagates() {
    struct FailRead;
    impl ModbusTransport for FailRead {
        fn read_holding(&mut self, _: u8, _: u16, _: &mut [u16]) -> Result<(), RtuError> {
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
    let mut xy = Xy::new(FailRead, Model::Xy7025);
    assert!(matches!(
        xy.read_voltage_out(),
        Err(RtuError::Modbus(ModbusError::BadCrc))
    ));
}
