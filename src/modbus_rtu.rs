//! Modbus-RTU over UART. Slave-agnostic transport — the device driver
//! supplies its own slave id and register addresses per call. Quirks
//! (response timeout, post-write quiet gap) belong to the device, not
//! the protocol, so they're set at construction via `RtuConfig`.

use std::thread;
use std::time::Duration;

use esp_idf_hal::uart::UartDriver;

use esp32_battery_logic::modbus::{
    RtuError, build_read_request, build_write_request, parse_read_response, parse_write_response,
};

use crate::clock::uptime;

pub struct RtuConfig {
    /// Quiet-deadline for collecting a response after writing the request.
    pub response_timeout: Duration,
    /// Silence enforced after every successful write so the slave has
    /// time to process and re-arm its listener. Modbus spec floor at
    /// 115200 baud is ~1.75 ms; cheap slaves (XY7025 et al) want more.
    pub post_write_gap: Duration,
}

pub struct ModbusRtu<'d> {
    uart: UartDriver<'d>,
    config: RtuConfig,
}

impl<'d> ModbusRtu<'d> {
    pub fn new(uart: UartDriver<'d>, config: RtuConfig) -> Self {
        Self { uart, config }
    }

    /// Read `out.len()` consecutive holding registers into `out`.
    pub fn read_holding(&self, slave: u8, addr: u16, out: &mut [u16]) -> Result<(), RtuError> {
        let req = build_read_request(slave, addr, out.len() as u16);
        let mut resp = [0u8; 256];
        let n = self.transact(&req, &mut resp)?;
        parse_read_response(&resp[..n], slave, out)?;
        Ok(())
    }

    pub fn write_holding(&self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError> {
        let req = build_write_request(slave, addr, value);
        let mut resp = [0u8; 8];
        let result = self
            .transact(&req, &mut resp)
            .and_then(|n| Ok(parse_write_response(&resp[..n], &req)?));
        thread::sleep(self.config.post_write_gap);
        result
    }

    /// Write request, collect reply until the quiet-deadline. Returns
    /// the number of response bytes received (>= 1).
    fn transact(&self, req: &[u8], resp: &mut [u8]) -> Result<usize, RtuError> {
        self.uart.clear_rx().ok();
        self.uart.write(req).map_err(|_| RtuError::UartWrite)?;
        self.uart.wait_tx_done(100).ok();

        let mut n = 0usize;
        let deadline = uptime() + self.config.response_timeout;
        while n < resp.len() && uptime() < deadline {
            match self.uart.read(&mut resp[n..], 2) {
                Ok(k) if k > 0 => n += k,
                _ if n > 0 => break,
                _ => {}
            }
        }
        if n == 0 {
            return Err(RtuError::UartRead);
        }
        Ok(n)
    }
}
