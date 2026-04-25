//! XY7025 programmable buck converter — Modbus-RTU over UART1 @ 115200 8N1.
//!
//! Slave addr 0x01 (default). Holding registers (fn 0x03), /100 scale:
//!   0x0000 V_set, 0x0001 I_set, 0x0002 V_out, 0x0003 I_out,
//!   0x0004 P_out, 0x0005 V_in.
//!
//! Wiring: ESP TX -> XY RX, ESP RX -> XY TX, common GND. No voltage divider
//! needed — both sides are 3.3 V TTL.

use std::time::Duration;

const POLL_INTERVAL: Duration = Duration::from_millis(1000);

#[cfg(not(feature = "xy-fake"))]
pub use real::start;

#[cfg(feature = "xy-fake")]
pub use fake::start;

#[cfg(not(feature = "xy-fake"))]
mod real {
    use std::thread;
    use std::time::{Duration, Instant};

    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;
    use log::{error, warn};

    use esp32_battery_logic::charging::{
        Action, BatterySample, ChargeSupervisor, Chemistry, FaultReason, Profile,
    };

    /// This board's pack: 4S LiFePO4, 50 Ah. Daily-cycle setpoints — 14.4 V
    /// absorb / 13.5 V float. Hysteresis: enter absorb at 1 A (C/50), drop
    /// back to float at 0.5 A (C/100). enter > exit so we don't flap.
    const PACK_PROFILE: Profile = Profile::for_pack(Chemistry::LiFePo4, 4, 1.0, 0.5);
    use esp32_battery_logic::data::PsReading;
    use esp32_battery_logic::modbus::{
        ModbusError, build_read_request, build_write_request, parse_read_response,
        parse_write_response,
    };

    use super::POLL_INTERVAL;
    use crate::app_state::SensorDataHandle;
    use crate::board::XyPins;

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

    const BOOT_I_SET: f32 = 10.0;
    /// Hard trip thresholds (OVP / OCP / LVP). These only fire if CV/CC regulation
    /// fails or the pack is badly out of spec — headroom above normal setpoints.
    const BOOT_OVP: f32 = 15.0; // V — safe ceiling above the CV target
    const BOOT_OCP: f32 = 16.0; // A — well above any normal CC current
    const BOOT_LVP: f32 = 10.0; // V — refuses to charge a deeply-dead / shorted pack

    struct Xy<'d> {
        uart: UartDriver<'d>,
    }

    impl<'d> Xy<'d> {
        fn new(pins: XyPins) -> Self {
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
            let result = match self.write_holding_once(addr, value) {
                Ok(()) => Ok(()),
                Err(_) => {
                    warn!("write_holding: first attempt failed, retrying");
                    thread::sleep(Duration::from_millis(80));
                    self.write_holding_once(addr, value)
                }
            };
            thread::sleep(POST_WRITE_GAP);
            result
        }

        fn write_holding_once(&self, addr: u16, value: u16) -> Result<(), ModbusError> {
            let req = build_write_request(SLAVE, addr, value);
            let mut resp = [0u8; 8];
            let n = self.transact(&req, &mut resp)?;
            parse_write_response(&resp[..n], &req)
        }

        /// UART transaction: write request, collect reply until quiet-deadline.
        /// Returns the number of response bytes received (>= 1).
        fn transact(&self, req: &[u8], resp: &mut [u8]) -> Result<usize, ModbusError> {
            self.uart.clear_rx().ok();
            self.uart.write(req).map_err(|_| ModbusError::WriteFailed)?;
            self.uart.wait_tx_done(100).ok();

            let mut n = 0usize;
            let deadline = Instant::now() + Duration::from_millis(RESPONSE_TIMEOUT_MS);
            while n < resp.len() && Instant::now() < deadline {
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
    }

    #[allow(dead_code)] // v_set/i_set/v_in will be surfaced via HTTP panel
    struct XyStatus {
        v_set: f32,
        i_set: f32,
        v_out: f32,
        i_out: f32,
        p_out: f32,
        v_in: f32,
    }

    /// Programs protection + setpoints, then enables output. Any failure
    /// short-circuits — we never enable output with unprogrammed setpoints.
    fn boot_sequence(xy: &Xy, initial_v_set: f32) -> Result<(), ModbusError> {
        xy.set_output(false)?;
        xy.set_power_on_default_off()?;
        xy.set_protection(BOOT_OVP, BOOT_OCP, BOOT_LVP)?;
        xy.set_voltage(initial_v_set)?;
        xy.set_current_limit(BOOT_I_SET)?;
        xy.set_output(true)?;
        Ok(())
    }

    pub fn start(pins: XyPins, sensor_data: SensorDataHandle) {
        thread::Builder::new()
            .name("xy".into())
            .stack_size(4096)
            .spawn(move || {
                let xy = Xy::new(pins);
                let mut supervisor = ChargeSupervisor::new(PACK_PROFILE);
                thread::sleep(Duration::from_millis(100));

                if let Err(e) = boot_sequence(&xy, supervisor.target_voltage()) {
                    error!("XY boot failed: {e} — forcing output OFF, will keep polling");
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
                                false
                            }
                        };
                        let battery = sd.battery_reading().map(|b| BatterySample {
                            voltage: b.voltage,
                            current: b.current,
                        });
                        (modbus_ok, battery)
                    };

                    match supervisor.tick(modbus_ok, battery) {
                        Action::None => {}
                        Action::SetVoltage(v) => {
                            let phase = match supervisor.phase() {
                                esp32_battery_logic::charging::Phase::Float => "float",
                                esp32_battery_logic::charging::Phase::Absorb => "absorb",
                            };
                            log::info!("charge phase → {phase}: setting V_set = {v:.2} V");
                            if let Err(e) = xy.set_voltage(v) {
                                warn!("XY set_voltage({v}): {e}");
                            }
                        }
                        Action::DisableOutput(reason) => {
                            let reason_str = match reason {
                                FaultReason::BatterySensorStale => "battery sensor stale",
                                FaultReason::ModbusErrorBudget => "modbus error budget exceeded",
                                FaultReason::Overvoltage => "pack overvoltage",
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
                                }
                            }
                        }
                    }
                    thread::sleep(POLL_INTERVAL);
                }
            })
            .unwrap();
    }

    // Modbus frame tests live in esp32_battery_logic::modbus (host-runnable).
}

#[cfg(feature = "xy-fake")]
mod fake {
    use std::thread;

    use esp32_battery_logic::data::PsReading;

    use super::POLL_INTERVAL;
    use crate::app_state::SensorDataHandle;
    use crate::board::XyPins;

    const FAKE_READING: PsReading = PsReading {
        voltage: 13.5,
        current: 0.0,
        power: 0.0,
    };

    pub fn start(pins: XyPins, sensor_data: SensorDataHandle) {
        drop(pins);
        thread::Builder::new()
            .name("xy".into())
            .stack_size(4096)
            .spawn(move || {
                log::info!("XY: fake mode — no UART, canned readings");
                loop {
                    sensor_data.lock().unwrap().update_ps(FAKE_READING);
                    thread::sleep(POLL_INTERVAL);
                }
            })
            .unwrap();
    }
}
