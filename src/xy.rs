//! XY7025 programmable buck converter — Modbus-RTU over UART1 @ 115200 8N1.
//!
//! Slave addr 0x01 (default). Holding registers (fn 0x03), /100 scale:
//!   0x0000 V_set, 0x0001 I_set, 0x0002 V_out, 0x0003 I_out,
//!   0x0004 P_out, 0x0005 V_in.
//!
//! Wiring: ESP TX -> XY RX, ESP RX -> XY TX, common GND. No voltage divider
//! needed — both sides are 3.3 V TTL.
//!
//! The device is abstracted behind `XyDevice` so the thread loop +
//! charge-supervisor integration runs unchanged under the `xy-fake` feature
//! (which substitutes a canned in-memory device for the UART).

use std::thread;
use std::time::Duration;

use log::{error, warn};

use esp32_battery_logic::charging::{
    Action, BatterySample, ChargeSupervisor, Chemistry, FaultReason, Profile, SafetyLimits,
};
use esp32_battery_logic::data::PsReading;
use esp32_battery_logic::error_log::{Event, XyError};
use esp32_battery_logic::modbus::ModbusError;

use crate::clock::EventRecorder;
use crate::supervisor::SensorDataHandle;
use crate::board::XyPins;

const POLL_INTERVAL: Duration = Duration::from_millis(1000);

/// This board's pack: 4S LiFePO4, 50 Ah. Daily-cycle setpoints — 14.4 V
/// absorb / 13.5 V float. CC at 10 A; enter absorb at 1 A (C/50), drop
/// back to float at 0.5 A (C/100). enter > exit so we don't flap.
const PACK_PROFILE: Profile = Profile::for_pack(Chemistry::LiFePo4, 4, 10.0, 1.0, 0.5);
/// Hard trip thresholds programmed into the XY's protection registers.
/// Derived from the profile so a chemistry/cell-count change moves them
/// in lockstep — no chance the OVP ceiling drifts below the absorb target.
const SAFETY: SafetyLimits = PACK_PROFILE.safety_limits();

#[allow(dead_code)] // v_set/i_set/v_in will be surfaced via HTTP panel
struct XyStatus {
    v_set: f32,
    i_set: f32,
    v_out: f32,
    i_out: f32,
    p_out: f32,
    v_in: f32,
}

/// The set of operations the charging loop needs from the buck. Real
/// builds get the UART-backed implementation; `xy-fake` builds get an
/// in-memory canned device. The thread loop is identical for both.
trait XyDevice {
    fn read_status(&self) -> Result<XyStatus, ModbusError>;
    fn set_voltage(&self, volts: f32) -> Result<(), ModbusError>;
    fn set_current_limit(&self, amps: f32) -> Result<(), ModbusError>;
    fn set_protection(&self, ovp_v: f32, ocp_a: f32, lvp_v: f32) -> Result<(), ModbusError>;
    fn set_output(&self, on: bool) -> Result<(), ModbusError>;
    fn set_power_on_default_off(&self) -> Result<(), ModbusError>;
}

// --- Real device ------------------------------------------------------------

#[cfg(not(feature = "xy-fake"))]
mod real {
    use std::thread;
    use std::time::Duration;

    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;

    use esp32_battery_logic::modbus::{
        ModbusError, build_read_request, build_write_request, parse_read_response,
        parse_write_response,
    };

    use super::{XyDevice, XyStatus};
    use crate::board::XyPins;
    use crate::clock::uptime;

    const SLAVE: u8 = 0x01;
    const REG_V_SET: u16 = 0x0000;
    const REG_I_SET: u16 = 0x0001;
    const REG_OUTPUT_EN: u16 = 0x0012;
    const REG_S_LVP: u16 = 0x0052;
    const REG_S_OVP: u16 = 0x0053;
    const REG_S_OCP: u16 = 0x0054;
    /// S-INI: power-on default for the output switch. 0 = OFF on power-up,
    /// 1 = ON. Persists in EEPROM. Set OFF so a brown-out / MCU crash never
    /// leaves the buck silently sourcing into the pack while we're not
    /// supervising.
    const REG_S_INI: u16 = 0x005D;
    const BAUD: u32 = 115200;
    const RESPONSE_TIMEOUT_MS: u64 = 500;
    /// Silence enforced after every write so the XY has time to process the
    /// previous command and re-arm its Modbus listener. Spec floor at 115200
    /// baud is 1.75 ms, but cheap Chinese slaves like the XY7025 empirically
    /// want more.
    const POST_WRITE_GAP: Duration = Duration::from_millis(10);

    pub struct Xy<'d> {
        uart: UartDriver<'d>,
    }

    impl<'d> Xy<'d> {
        pub fn new(pins: XyPins) -> Self {
            let config = Config::new().baudrate(Hertz(BAUD));
            let uart = UartDriver::new(
                pins.uart,
                pins.tx,
                pins.rx,
                None::<esp_idf_hal::gpio::AnyIOPin>,
                None::<esp_idf_hal::gpio::AnyIOPin>,
                &config,
            )
            .expect("UART1 init");
            Self { uart }
        }

        fn read_holding(&self, addr: u16, count: u16) -> Result<Vec<u16>, ModbusError> {
            assert!(count > 0 && count <= 125);
            let req = build_read_request(SLAVE, addr, count);
            let mut resp = [0u8; 256];
            let n = self.transact(&req, &mut resp)?;
            parse_read_response(&resp[..n], SLAVE, count)
        }

        fn write_holding(&self, addr: u16, value: u16) -> Result<(), ModbusError> {
            let req = build_write_request(SLAVE, addr, value);
            let mut resp = [0u8; 8];
            let result = match self.transact(&req, &mut resp) {
                Ok(n) => parse_write_response(&resp[..n], &req),
                Err(e) => Err(e),
            };
            thread::sleep(POST_WRITE_GAP);
            result
        }

        /// UART transaction: write request, collect reply until quiet-deadline.
        /// Returns the number of response bytes received (>= 1).
        fn transact(&self, req: &[u8], resp: &mut [u8]) -> Result<usize, ModbusError> {
            self.uart.clear_rx().ok();
            self.uart.write(req).map_err(|_| ModbusError::WriteFailed)?;
            self.uart.wait_tx_done(100).ok();

            let mut n = 0usize;
            let deadline = uptime() + Duration::from_millis(RESPONSE_TIMEOUT_MS);
            while n < resp.len() && uptime() < deadline {
                match self.uart.read(&mut resp[n..], 2) {
                    Ok(k) if k > 0 => n += k,
                    _ if n > 0 => break,
                    _ => {}
                }
            }
            if n == 0 {
                return Err(ModbusError::ReadFailed);
            }
            Ok(n)
        }
    }

    impl XyDevice for Xy<'_> {
        fn read_status(&self) -> Result<XyStatus, ModbusError> {
            let r = self.read_holding(0x0000, 6)?;
            Ok(XyStatus {
                v_set: r[0] as f32 / 100.0,
                i_set: r[1] as f32 / 100.0,
                v_out: r[2] as f32 / 100.0,
                i_out: r[3] as f32 / 100.0,
                p_out: r[4] as f32 / 100.0,
                v_in: r[5] as f32 / 100.0,
            })
        }

        fn set_voltage(&self, volts: f32) -> Result<(), ModbusError> {
            self.write_holding(REG_V_SET, (volts * 100.0).round() as u16)
        }

        fn set_current_limit(&self, amps: f32) -> Result<(), ModbusError> {
            self.write_holding(REG_I_SET, (amps * 100.0).round() as u16)
        }

        fn set_protection(&self, ovp_v: f32, ocp_a: f32, lvp_v: f32) -> Result<(), ModbusError> {
            self.write_holding(REG_S_OVP, (ovp_v * 100.0).round() as u16)?;
            self.write_holding(REG_S_OCP, (ocp_a * 100.0).round() as u16)?;
            self.write_holding(REG_S_LVP, (lvp_v * 100.0).round() as u16)?;
            Ok(())
        }

        fn set_output(&self, on: bool) -> Result<(), ModbusError> {
            self.write_holding(REG_OUTPUT_EN, if on { 1 } else { 0 })
        }

        fn set_power_on_default_off(&self) -> Result<(), ModbusError> {
            self.write_holding(REG_S_INI, 0)
        }
    }

    // Modbus frame tests live in esp32_battery_logic::modbus (host-runnable).
}

// --- Fake device ------------------------------------------------------------

#[cfg(feature = "xy-fake")]
mod fake {
    use std::cell::Cell;

    use esp32_battery_logic::modbus::ModbusError;

    use super::{XyDevice, XyStatus};

    /// In-memory stand-in for the buck. Tracks the last voltage/output
    /// state set by the supervisor so reads reflect what the supervisor
    /// last commanded — exercises the same control flow as the real path.
    pub struct FakeXy {
        v_set: Cell<f32>,
        output_on: Cell<bool>,
    }

    impl FakeXy {
        pub fn new() -> Self {
            Self {
                v_set: Cell::new(13.5),
                output_on: Cell::new(false),
            }
        }
    }

    impl XyDevice for FakeXy {
        fn read_status(&self) -> Result<XyStatus, ModbusError> {
            let v = if self.output_on.get() {
                self.v_set.get()
            } else {
                0.0
            };
            Ok(XyStatus {
                v_set: self.v_set.get(),
                i_set: 10.0,
                v_out: v,
                i_out: 0.0,
                p_out: 0.0,
                v_in: 24.0,
            })
        }
        fn set_voltage(&self, volts: f32) -> Result<(), ModbusError> {
            self.v_set.set(volts);
            Ok(())
        }
        fn set_current_limit(&self, _amps: f32) -> Result<(), ModbusError> {
            Ok(())
        }
        fn set_protection(&self, _ovp: f32, _ocp: f32, _lvp: f32) -> Result<(), ModbusError> {
            Ok(())
        }
        fn set_output(&self, on: bool) -> Result<(), ModbusError> {
            self.output_on.set(on);
            Ok(())
        }
        fn set_power_on_default_off(&self) -> Result<(), ModbusError> {
            Ok(())
        }
    }
}

// --- Shared thread loop -----------------------------------------------------

/// Programs protection + setpoints, then enables output. Any failure
/// short-circuits — we never enable output with unprogrammed setpoints.
fn boot_sequence<D: XyDevice>(xy: &D, initial_v_set: f32) -> Result<(), ModbusError> {
    xy.set_output(false)?;
    xy.set_power_on_default_off()?;
    xy.set_protection(SAFETY.ovp_v, SAFETY.ocp_a, SAFETY.lvp_v)?;
    xy.set_voltage(initial_v_set)?;
    xy.set_current_limit(PACK_PROFILE.regulation_a)?;
    xy.set_output(true)?;
    Ok(())
}

fn run<D: XyDevice>(xy: D, sensor_data: SensorDataHandle, recorder: EventRecorder) {
    let mut supervisor = ChargeSupervisor::new(PACK_PROFILE);
    thread::sleep(Duration::from_millis(100));

    if let Err(e) = boot_sequence(&xy, supervisor.target_voltage()) {
        error!("XY boot failed: {e} — forcing output OFF, will keep polling");
        recorder.record(Event::Xy(XyError::BootSequence));
        let _ = xy.set_output(false);
    }

    loop {
        let (modbus_ok, battery) = {
            let mut sd = sensor_data.lock().unwrap();
            let modbus_ok = match xy.read_status() {
                Ok(s) => {
                    sd.update_ps(PsReading {
                        voltage: s.v_out,
                        current: s.i_out,
                        power: s.p_out,
                    });
                    true
                }
                Err(e) => {
                    warn!("XY read_status: {e}");
                    recorder.record(Event::Xy(XyError::ReadStatus));
                    false
                }
            };
            let battery = sd.battery_reading().map(|b| BatterySample {
                voltage: b.voltage,
                current: b.current,
            });
            (modbus_ok, battery)
        };

        match supervisor.tick(modbus_ok, battery, POLL_INTERVAL) {
            Action::None => {}
            Action::SetVoltage(v) => {
                let phase = match supervisor.phase() {
                    esp32_battery_logic::charging::Phase::Float => "float",
                    esp32_battery_logic::charging::Phase::Absorb => "absorb",
                };
                log::info!("charge phase → {phase}: setting V_set = {v:.2} V");
                if let Err(e) = xy.set_voltage(v) {
                    warn!("XY set_voltage({v}): {e}");
                    recorder.record(Event::Xy(XyError::SetVoltage));
                }
            }
            Action::DisableOutput(reason) => {
                let reason_str = match reason {
                    FaultReason::BatterySensorStale => "battery sensor stale",
                    FaultReason::ModbusUnhealthy => "modbus link unhealthy",
                    FaultReason::Overvoltage => "pack overvoltage",
                    FaultReason::AbsorbTimeout => "absorb time cap reached",
                };
                match xy.set_output(false) {
                    Ok(()) => {
                        error!("CHARGE FAULT ({reason_str}): PS output DISABLED");
                        supervisor.ack_disable();
                    }
                    Err(e) => {
                        error!(
                            "CHARGE FAULT ({reason_str}): set_output(false) failed: {e} — will retry"
                        );
                        recorder.record(Event::Xy(XyError::SetOutput));
                    }
                }
            }
        }
        thread::sleep(POLL_INTERVAL);
    }
}

// --- Public entry point -----------------------------------------------------

#[cfg(not(feature = "xy-fake"))]
fn make_device(pins: XyPins) -> real::Xy<'static> {
    real::Xy::new(pins)
}

#[cfg(feature = "xy-fake")]
fn make_device(pins: XyPins) -> fake::FakeXy {
    // Burn the peripherals through black_box so XyPins fields aren't
    // flagged dead — we still claim them at boot and just don't drive
    // the bus.
    let XyPins { uart, tx, rx } = pins;
    std::hint::black_box((uart, tx, rx));
    log::info!("XY: fake mode — no UART, in-memory device");
    fake::FakeXy::new()
}

pub fn start(pins: XyPins, sensor_data: SensorDataHandle, recorder: EventRecorder) {
    let device = make_device(pins);
    thread::Builder::new()
        .name("xy".into())
        .stack_size(4096)
        .spawn(move || run(device, sensor_data, recorder))
        .unwrap();
}
