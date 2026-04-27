//! Default Modbus-RTU transport over an `embedded-io` UART.
//!
//! Wrap any blocking byte stream that implements
//! [`embedded_io::Read`] + [`embedded_io::Write`] +
//! [`embedded_io::ReadReady`] together with an
//! [`embedded_hal::delay::DelayNs`] timer, and you have a working
//! [`ModbusTransport`].
//!
//! ```ignore
//! use xy_modbus::{Xy, uart::UartTransport};
//!
//! let transport = UartTransport::new(uart, delay);
//! let mut xy = Xy::new(transport);
//! ```
//!
//! Timing defaults match the XY-series spec (~500 ms response window,
//! ~50 ms post-write quiet gap). Override with [`UartTransport::with_timing`].

use embedded_hal::delay::DelayNs;
use embedded_io::{Read, ReadReady, Write};

use crate::framing::{
    MAX_ADU, MAX_READ_REGS, MAX_WRITE_REGS, build_read_request, build_write_multiple_request,
    build_write_single_request, parse_read_response, parse_write_multiple_response,
    parse_write_single_response,
};
use crate::transport::{ModbusTransport, RtuError};

// ─── UartTransport ───────────────────────────────────────────────────────────

/// Generic Modbus-RTU transport over any `embedded-io` UART.
pub struct UartTransport<U, D> {
    uart: U,
    delay: D,
    response_timeout_ms: u32,
    inter_frame_ms: u32,
}

impl<U, D> UartTransport<U, D>
where
    U: Read + Write + ReadReady,
    D: DelayNs,
{
    /// Build a transport with default XY-series timing
    /// (500 ms response window, 50 ms post-write quiet gap).
    pub fn new(uart: U, delay: D) -> Self {
        Self {
            uart,
            delay,
            response_timeout_ms: 500,
            inter_frame_ms: 50,
        }
    }

    /// Override response timeout (max wait without any RX progress) and
    /// the post-write quiet gap.
    pub fn with_timing(mut self, response_timeout_ms: u32, inter_frame_ms: u32) -> Self {
        self.response_timeout_ms = response_timeout_ms;
        self.inter_frame_ms = inter_frame_ms;
        self
    }

    /// Recover the inner UART and delay.
    pub fn release(self) -> (U, D) {
        (self.uart, self.delay)
    }

    // ─── I/O helpers ─────────────────────────────────────────────────────

    fn drain_rx(&mut self) {
        let mut scratch = [0u8; 32];
        while matches!(self.uart.read_ready(), Ok(true)) {
            if self.uart.read(&mut scratch).is_err() {
                break;
            }
        }
    }

    /// Enforce ≥t3.5 bus silence before the next master frame, then
    /// flush any noise that arrived during the gap.
    fn pre_tx_silence(&mut self) {
        self.delay.delay_ms(self.inter_frame_ms);
        self.drain_rx();
    }

    fn write_all(&mut self, mut buf: &[u8]) -> Result<(), RtuError> {
        while !buf.is_empty() {
            match self.uart.write(buf) {
                Ok(0) => return Err(RtuError::Io),
                Ok(n) => buf = &buf[n..],
                Err(_) => return Err(RtuError::Io),
            }
        }
        self.uart.flush().map_err(|_| RtuError::Io)?;
        Ok(())
    }

    // Bounded by buf.len() forward progress: each iteration either consumes
    // RX bytes (capped by buf.len() total), or sleeps 1ms and increments
    // idle_ms (capped by response_timeout_ms). No separate wall-clock budget
    // needed — DelayNs gives no clock to measure one against anyway.
    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), RtuError> {
        let mut filled = 0;
        let mut idle_ms = 0u32;
        while filled < buf.len() {
            match self.uart.read_ready() {
                Ok(true) => match self.uart.read(&mut buf[filled..]) {
                    Ok(0) => return Err(RtuError::Io),
                    Ok(n) => {
                        filled += n;
                        idle_ms = 0;
                    }
                    Err(_) => return Err(RtuError::Io),
                },
                Ok(false) => {
                    if idle_ms >= self.response_timeout_ms {
                        return Err(RtuError::Timeout);
                    }
                    self.delay.delay_ms(1);
                    idle_ms += 1;
                }
                Err(_) => return Err(RtuError::Io),
            }
        }
        Ok(())
    }

    /// Read a response of expected length `full_len`, short-circuiting
    /// on a 5-byte Modbus exception frame.
    fn read_response<'b>(
        &mut self,
        buf: &'b mut [u8],
        full_len: usize,
    ) -> Result<&'b [u8], RtuError> {
        assert!(full_len >= 5 && full_len <= buf.len());
        self.read_exact(&mut buf[..3])?;
        if buf[1] & 0x80 != 0 {
            self.read_exact(&mut buf[3..5])?;
            return Ok(&buf[..5]);
        }
        if full_len > 3 {
            self.read_exact(&mut buf[3..full_len])?;
        }
        Ok(&buf[..full_len])
    }
}

// ─── ModbusTransport impl ────────────────────────────────────────────────────

impl<U, D> ModbusTransport for UartTransport<U, D>
where
    U: Read + Write + ReadReady,
    D: DelayNs,
{
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError> {
        assert!(slave != 0, "read does not support broadcast");
        assert!(!dst.is_empty() && dst.len() <= MAX_READ_REGS);
        let count = dst.len() as u16;
        let req = build_read_request(slave, addr, count);
        let expected_len = 5 + 2 * dst.len();

        self.pre_tx_silence();
        self.write_all(&req)?;

        let mut buf = [0u8; MAX_ADU];
        let resp = self.read_response(&mut buf, expected_len)?;
        parse_read_response(resp, slave, dst)?;
        Ok(())
    }

    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError> {
        assert!(
            slave != 0,
            "single-register write does not support broadcast"
        );
        let req = build_write_single_request(slave, addr, value);

        self.pre_tx_silence();
        self.write_all(&req)?;

        let mut buf = [0u8; 8];
        let resp = self.read_response(&mut buf, 8)?;
        parse_write_single_response(resp, &req)?;
        Ok(())
    }

    fn write_multiple_holdings(
        &mut self,
        slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError> {
        assert!(
            slave != 0,
            "multi-register write does not support broadcast"
        );
        assert!(!values.is_empty() && values.len() <= MAX_WRITE_REGS);
        let mut req = [0u8; MAX_ADU];
        let n = build_write_multiple_request(slave, addr, values, &mut req)
            .expect("inputs validated above");

        self.pre_tx_silence();
        self.write_all(&req[..n])?;

        let mut buf = [0u8; 8];
        let resp = self.read_response(&mut buf, 8)?;
        parse_write_multiple_response(resp, slave, addr, values.len() as u16)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
