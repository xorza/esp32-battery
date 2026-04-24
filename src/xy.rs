//! XY7025 programmable buck converter — Modbus-RTU over UART1 @ 115200 8N1.
//!
//! Slave addr 0x01 (default). Holding registers (fn 0x03), /100 scale:
//!   0x0000 V_set, 0x0001 I_set, 0x0002 V_out, 0x0003 I_out,
//!   0x0004 P_out, 0x0005 V_in.
//!
//! Wiring: ESP TX -> XY RX, ESP RX -> XY TX, common GND. No voltage divider
//! needed — both sides are 3.3 V TTL.

use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use esp_idf_hal::uart::{UartDriver, config::Config};
use esp_idf_hal::units::Hertz;
use log::{info, warn};

use esp32_battery_logic::data::PsReading;

use crate::AppState;
use crate::board::XyPins;

const SLAVE: u8 = 0x01;
const FN_READ_HOLDING: u8 = 0x03;
const FN_WRITE_HOLDING: u8 = 0x06;
const REG_V_SET: u16 = 0x0000;
const REG_I_SET: u16 = 0x0001;
const REG_OUTPUT_EN: u16 = 0x0012;
const BAUD: u32 = 115200;
const RESPONSE_TIMEOUT_MS: u64 = 500;

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

        // esp-idf-hal read() returns Err on timeout. Loop until deadline,
        // stopping early once quiet follows received data (frame end).
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

    /// Modbus fn 0x06 write, with one retry on short/failed response.
    pub fn write_holding(&self, addr: u16, value: u16) -> Result<(), XyError> {
        match self.write_holding_once(addr, value) {
            Ok(()) => Ok(()),
            Err(_) => {
                thread::sleep(Duration::from_millis(80));
                self.write_holding_once(addr, value)
            }
        }
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

    pub fn set_voltage(&self, volts: f32) -> Result<(), XyError> {
        let v = (volts * 100.0).round() as u16;
        self.write_holding(REG_V_SET, v)
    }

    pub fn set_current_limit(&self, amps: f32) -> Result<(), XyError> {
        let i = (amps * 100.0).round() as u16;
        self.write_holding(REG_I_SET, i)
    }

    pub fn set_output(&self, on: bool) -> Result<(), XyError> {
        self.write_holding(REG_OUTPUT_EN, if on { 1 } else { 0 })
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

/// Default voltage/current setpoints applied on boot. Output is kept OFF until
/// enabled manually (charging strategy is not yet implemented).
const BOOT_V_SET: f32 = 13.6;
const BOOT_I_SET: f32 = 1.0;
const POLL_INTERVAL: Duration = Duration::from_millis(1000);
const COMMAND_GAP: Duration = Duration::from_millis(150);

pub fn start(pins: XyPins, state: Arc<AppState>) {
    thread::Builder::new()
        .name("xy".into())
        .stack_size(4096)
        .spawn(move || {
            let xy = Xy::new(pins);
            thread::sleep(Duration::from_millis(100));

            // Always start with output disabled for safety. Setpoints are applied so the
            // first manual enable immediately charges at the intended target.
            if let Err(e) = xy.set_output(false) {
                warn!("XY set_output(off): {e}");
            }
            thread::sleep(COMMAND_GAP);
            if let Err(e) = xy.set_voltage(BOOT_V_SET) {
                warn!("XY set_voltage: {e}");
            }
            thread::sleep(COMMAND_GAP);
            if let Err(e) = xy.set_current_limit(BOOT_I_SET) {
                warn!("XY set_current_limit: {e}");
            }
            thread::sleep(COMMAND_GAP);

            loop {
                match xy.read_status() {
                    Ok(s) => {
                        let reading = PsReading {
                            voltage: s.v_out,
                            current: s.i_out,
                            power: s.p_out,
                        };
                        state.sensor_data.lock().unwrap().update_ps(reading);
                        info!(
                            "XY: V_in={:.2} set V={:.2} I={:.2} | out V={:.2} I={:.2} P={:.2}",
                            s.v_in, s.v_set, s.i_set, s.v_out, s.i_out, s.p_out
                        );
                    }
                    Err(e) => warn!("XY read_status: {e}"),
                }
                thread::sleep(POLL_INTERVAL);
            }
        })
        .unwrap();
}

#[cfg(test)]
#[allow(unused_imports)]
mod tests {
    use super::crc16_modbus;

    #[test]
    fn crc_known_vector() {
        // Read 1 reg at 0x001F from slave 1: 01 03 00 1F 00 01 -> CRC 0xC0B5 (LE: B5 C0).
        assert_eq!(crc16_modbus(&[0x01, 0x03, 0x00, 0x1F, 0x00, 0x01]), 0xC0B5);
    }

    #[test]
    fn crc_empty() {
        assert_eq!(crc16_modbus(&[]), 0xFFFF);
    }
}
