//! XY7025 programmable buck converter — Modbus-RTU over UART1 @ 115200 8N1.
//!
//! Slave addr 0x01 (default). Holding registers (fn 0x03), /100 scale:
//!   0x0000 V_set, 0x0001 I_set, 0x0002 V_out, 0x0003 I_out,
//!   0x0004 P_out, 0x0005 V_in.
//!
//! Wiring: ESP TX -> XY RX, ESP RX -> XY TX, common GND. No voltage divider
//! needed — both sides are 3.3 V TTL.

use std::time::Duration;

/// Default voltage/current setpoint applied on boot (real) / used as the fake
/// reading's voltage (fake). Output is kept OFF until enabled manually.
const BOOT_V_SET: f32 = 13.6;
const POLL_INTERVAL: Duration = Duration::from_millis(1000);

#[cfg(not(feature = "xy-fake"))]
pub use real::start;

#[cfg(feature = "xy-fake")]
pub use fake::start;

#[cfg(not(feature = "xy-fake"))]
mod real {
    use std::sync::Arc;
    use std::thread;
    use std::time::{Duration, Instant};

    use esp_idf_hal::uart::{UartDriver, config::Config};
    use esp_idf_hal::units::Hertz;
    use log::{error, warn};

    use esp32_battery_logic::data::PsReading;

    use super::{BOOT_V_SET, POLL_INTERVAL};
    use crate::app_state::Shared;
    use crate::board::XyPins;

    const SLAVE: u8 = 0x01;
    const FN_READ_HOLDING: u8 = 0x03;
    const FN_WRITE_HOLDING: u8 = 0x06;
    const REG_V_SET: u16 = 0x0000;
    const REG_I_SET: u16 = 0x0001;
    const REG_OUTPUT_EN: u16 = 0x0012;
    const REG_S_LVP: u16 = 0x0052;
    const REG_S_OVP: u16 = 0x0053;
    const REG_S_OCP: u16 = 0x0054;
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

    pub fn crc16_modbus(data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        for &b in data {
            crc ^= b as u16;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        crc
    }

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

        pub fn read_holding(&self, addr: u16, count: u16) -> Result<Vec<u16>, XyError> {
            assert!(count > 0 && count <= 125);

            let mut req = [0u8; 8];
            req[0] = SLAVE;
            req[1] = FN_READ_HOLDING;
            req[2..4].copy_from_slice(&addr.to_be_bytes());
            req[4..6].copy_from_slice(&count.to_be_bytes());
            let crc = crc16_modbus(&req[..6]);
            req[6] = crc as u8;
            req[7] = (crc >> 8) as u8;

            self.uart.clear_rx().ok();
            self.uart.write(&req).map_err(|_| XyError::WriteFailed)?;
            self.uart.wait_tx_done(100).ok();

            let mut resp = [0u8; 64];
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
                return Err(XyError::ReadFailed);
            }

            if n < 5 {
                return Err(XyError::ShortResponse(n));
            }
            if resp[0] != SLAVE {
                return Err(XyError::BadSlave(resp[0]));
            }
            if resp[1] & 0x80 != 0 {
                return Err(XyError::ModbusException(resp[2]));
            }
            let expected_len = 5 + 2 * count as usize;
            if resp[1] != FN_READ_HOLDING || resp[2] as usize != 2 * count as usize {
                return Err(XyError::BadHeader);
            }
            if n < expected_len {
                return Err(XyError::ShortResponse(n));
            }

            let crc_got = u16::from_le_bytes([resp[expected_len - 2], resp[expected_len - 1]]);
            let crc_calc = crc16_modbus(&resp[..expected_len - 2]);
            if crc_got != crc_calc {
                return Err(XyError::BadCrc);
            }

            Ok((0..count as usize)
                .map(|i| u16::from_be_bytes([resp[3 + 2 * i], resp[4 + 2 * i]]))
                .collect())
        }

        pub fn write_holding(&self, addr: u16, value: u16) -> Result<(), XyError> {
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

        fn write_holding_once(&self, addr: u16, value: u16) -> Result<(), XyError> {
            let mut req = [0u8; 8];
            req[0] = SLAVE;
            req[1] = FN_WRITE_HOLDING;
            req[2..4].copy_from_slice(&addr.to_be_bytes());
            req[4..6].copy_from_slice(&value.to_be_bytes());
            let crc = crc16_modbus(&req[..6]);
            req[6] = crc as u8;
            req[7] = (crc >> 8) as u8;

            self.uart.clear_rx().ok();
            self.uart.write(&req).map_err(|_| XyError::WriteFailed)?;
            self.uart.wait_tx_done(100).ok();

            let mut resp = [0u8; 8];
            let mut n = 0usize;
            let deadline = Instant::now() + Duration::from_millis(RESPONSE_TIMEOUT_MS);
            while n < resp.len() && Instant::now() < deadline {
                match self.uart.read(&mut resp[n..], 2) {
                    Ok(k) if k > 0 => n += k,
                    _ if n > 0 => break,
                    _ => {}
                }
            }
            if n < 8 {
                return Err(XyError::ShortResponse(n));
            }
            if resp[0] != SLAVE {
                return Err(XyError::BadSlave(resp[0]));
            }
            if resp[1] & 0x80 != 0 {
                return Err(XyError::ModbusException(resp[2]));
            }
            if resp != req {
                return Err(XyError::BadHeader);
            }
            Ok(())
        }

        pub fn set_voltage(&self, volts: f32) {
            let v = (volts * 100.0).round() as u16;
            if let Err(e) = self.write_holding(REG_V_SET, v) {
                self.set_output(false);
                error!("XY set_voltage({volts:.2} V) failed: {e} — disabling output");
            }
        }

        pub fn set_current_limit(&self, amps: f32) {
            let i = (amps * 100.0).round() as u16;
            if let Err(e) = self.write_holding(REG_I_SET, i) {
                self.set_output(false);
                error!("XY set_current_limit({amps:.2} A) failed: {e} — disabling output");
            }
        }

        pub fn set_protection(&self, ovp_volts: f32, ocp_amps: f32, lvp_volts: f32) {
            let writes = [
                (REG_S_OVP, (ovp_volts * 100.0).round() as u16, "OVP"),
                (REG_S_OCP, (ocp_amps * 100.0).round() as u16, "OCP"),
                (REG_S_LVP, (lvp_volts * 100.0).round() as u16, "LVP"),
            ];
            for (reg, val, name) in writes {
                if let Err(e) = self.write_holding(reg, val) {
                    self.set_output(false);
                    error!("XY set {name} failed: {e} — disabling output");
                    return;
                }
            }
        }

        /// Output-enable is safety-critical. Panics on failure → watchdog reset.
        pub fn set_output(&self, on: bool) {
            if let Err(e) = self.write_holding(REG_OUTPUT_EN, if on { 1 } else { 0 }) {
                panic!("XY set_output({on}) failed: {e} — triggering reset");
            }
        }

        pub fn read_status(&self) -> Result<XyStatus, XyError> {
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
    pub struct XyStatus {
        pub v_set: f32,
        pub i_set: f32,
        pub v_out: f32,
        pub i_out: f32,
        pub p_out: f32,
        pub v_in: f32,
    }

    pub enum XyError {
        WriteFailed,
        ReadFailed,
        ShortResponse(usize),
        BadSlave(u8),
        BadHeader,
        BadCrc,
        ModbusException(u8),
    }

    impl std::fmt::Display for XyError {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            match self {
                XyError::WriteFailed => write!(f, "UART write failed"),
                XyError::ReadFailed => write!(f, "UART read failed"),
                XyError::ShortResponse(n) => write!(f, "short response ({n} bytes)"),
                XyError::BadSlave(a) => write!(f, "wrong slave id 0x{a:02X}"),
                XyError::BadHeader => write!(f, "malformed header"),
                XyError::BadCrc => write!(f, "CRC mismatch"),
                XyError::ModbusException(c) => write!(f, "modbus exception 0x{c:02X}"),
            }
        }
    }

    pub fn start(pins: XyPins, shared: Arc<Shared>) {
        thread::Builder::new()
            .name("xy".into())
            .stack_size(4096)
            .spawn(move || {
                let xy = Xy::new(pins);
                thread::sleep(Duration::from_millis(100));

                xy.set_output(false);
                xy.set_protection(BOOT_OVP, BOOT_OCP, BOOT_LVP);
                xy.set_voltage(BOOT_V_SET);
                xy.set_current_limit(BOOT_I_SET);
                xy.set_output(true);

                loop {
                    match xy.read_status() {
                        Ok(s) => {
                            let reading = PsReading {
                                voltage: s.v_out,
                                current: s.i_out,
                                power: s.p_out,
                            };
                            shared.sensor_data.lock().unwrap().update_ps(reading);
                        }
                        Err(e) => warn!("XY read_status: {e}"),
                    }
                    thread::sleep(POLL_INTERVAL);
                }
            })
            .unwrap();
    }

    #[cfg(test)]
    mod tests {
        #[test]
        fn crc_known_vector() {
            assert_eq!(super::crc16_modbus(&[0x01, 0x03, 0x00, 0x1F, 0x00, 0x01]), 0xC0B5);
        }

        #[test]
        fn crc_empty() {
            assert_eq!(super::crc16_modbus(&[]), 0xFFFF);
        }
    }
}

#[cfg(feature = "xy-fake")]
mod fake {
    use std::sync::Arc;
    use std::thread;

    use esp32_battery_logic::data::PsReading;

    use super::{BOOT_V_SET, POLL_INTERVAL};
    use crate::app_state::Shared;
    use crate::board::XyPins;

    const FAKE_READING: PsReading = PsReading {
        voltage: BOOT_V_SET,
        current: 0.0,
        power: 0.0,
    };

    pub fn start(pins: XyPins, shared: Arc<Shared>) {
        drop(pins);
        thread::Builder::new()
            .name("xy".into())
            .stack_size(4096)
            .spawn(move || {
                log::info!("XY: fake mode — no UART, canned readings");
                loop {
                    shared.sensor_data.lock().unwrap().update_ps(FAKE_READING);
                    thread::sleep(POLL_INTERVAL);
                }
            })
            .unwrap();
    }
}
