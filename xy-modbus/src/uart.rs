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
    MAX_ADU, build_read_request, build_write_multiple_request, build_write_single_request,
    parse_read_response, parse_write_multiple_response, parse_write_single_response,
};
use crate::transport::{ModbusTransport, RtuError};

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

    fn drain_rx(&mut self) {
        let mut scratch = [0u8; 32];
        while matches!(self.uart.read_ready(), Ok(true)) {
            if self.uart.read(&mut scratch).is_err() {
                break;
            }
        }
    }

    fn write_all(&mut self, mut buf: &[u8]) -> Result<(), RtuError> {
        while !buf.is_empty() {
            match self.uart.write(buf) {
                Ok(0) => return Err(RtuError::UartWrite),
                Ok(n) => buf = &buf[n..],
                Err(_) => return Err(RtuError::UartWrite),
            }
        }
        self.uart.flush().map_err(|_| RtuError::UartWrite)?;
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<(), RtuError> {
        let mut filled = 0;
        let mut idle_ms = 0u32;
        while filled < buf.len() {
            match self.uart.read_ready() {
                Ok(true) => match self.uart.read(&mut buf[filled..]) {
                    Ok(0) => return Err(RtuError::UartRead),
                    Ok(n) => {
                        filled += n;
                        idle_ms = 0;
                    }
                    Err(_) => return Err(RtuError::UartRead),
                },
                Ok(false) => {
                    if idle_ms >= self.response_timeout_ms {
                        return Err(RtuError::UartRead);
                    }
                    self.delay.delay_ms(1);
                    idle_ms += 1;
                }
                Err(_) => return Err(RtuError::UartRead),
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
        debug_assert!(full_len >= 5 && full_len <= buf.len());
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

impl<U, D> ModbusTransport for UartTransport<U, D>
where
    U: Read + Write + ReadReady,
    D: DelayNs,
{
    fn read_holding(
        &mut self,
        slave: u8,
        addr: u16,
        dst: &mut [u16],
    ) -> Result<(), RtuError> {
        assert!(!dst.is_empty() && dst.len() <= 125);
        let count = dst.len() as u16;
        let req = build_read_request(slave, addr, count);
        let expected_len = 5 + 2 * dst.len();

        self.drain_rx();
        self.write_all(&req)?;

        let mut buf = [0u8; MAX_ADU];
        let resp = self.read_response(&mut buf, expected_len)?;
        parse_read_response(resp, slave, dst)?;
        self.delay.delay_ms(self.inter_frame_ms);
        Ok(())
    }

    fn write_single_holding(
        &mut self,
        slave: u8,
        addr: u16,
        value: u16,
    ) -> Result<(), RtuError> {
        let req = build_write_single_request(slave, addr, value);

        self.drain_rx();
        self.write_all(&req)?;

        let mut buf = [0u8; 8];
        let resp = self.read_response(&mut buf, 8)?;
        parse_write_single_response(resp, &req)?;
        self.delay.delay_ms(self.inter_frame_ms);
        Ok(())
    }

    fn write_multiple_holdings(
        &mut self,
        slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError> {
        let mut req = [0u8; MAX_ADU];
        let n = build_write_multiple_request(slave, addr, values, &mut req)
            .expect("values length validated by caller");

        self.drain_rx();
        self.write_all(&req[..n])?;

        let mut buf = [0u8; 8];
        let resp = self.read_response(&mut buf, 8)?;
        parse_write_multiple_response(resp, slave, addr, values.len() as u16)?;
        self.delay.delay_ms(self.inter_frame_ms);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec::Vec;

    use embedded_io::ErrorType;

    use super::*;
    use crate::framing::crc16_modbus;
    use crate::transport::ModbusError;

    /// Mock UART: `stale` bytes are visible immediately (simulating
    /// junk in the RX FIFO before the request); `response` bytes only
    /// become available after the first byte is written.
    struct MockUart {
        tx: Vec<u8>,
        stale: Vec<u8>,
        stale_pos: usize,
        response: Vec<u8>,
        resp_pos: usize,
        armed: bool,
    }

    impl MockUart {
        fn new(response: Vec<u8>) -> Self {
            Self {
                tx: Vec::new(),
                stale: Vec::new(),
                stale_pos: 0,
                response,
                resp_pos: 0,
                armed: false,
            }
        }

        fn with_stale(mut self, stale: Vec<u8>) -> Self {
            self.stale = stale;
            self
        }

        fn available(&self) -> usize {
            let s = self.stale.len() - self.stale_pos;
            let r = if self.armed {
                self.response.len() - self.resp_pos
            } else {
                0
            };
            s + r
        }
    }

    impl ErrorType for MockUart {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Read for MockUart {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let mut written = 0;
            while written < buf.len() && self.stale_pos < self.stale.len() {
                buf[written] = self.stale[self.stale_pos];
                self.stale_pos += 1;
                written += 1;
            }
            if self.armed {
                while written < buf.len() && self.resp_pos < self.response.len() {
                    buf[written] = self.response[self.resp_pos];
                    self.resp_pos += 1;
                    written += 1;
                }
            }
            Ok(written)
        }
    }

    impl embedded_io::ReadReady for MockUart {
        fn read_ready(&mut self) -> Result<bool, Self::Error> {
            Ok(self.available() > 0)
        }
    }

    impl embedded_io::Write for MockUart {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            if !buf.is_empty() {
                self.armed = true;
            }
            self.tx.extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct NoDelay;
    impl DelayNs for NoDelay {
        fn delay_ns(&mut self, _: u32) {}
    }

    fn frame_with_crc(mut bytes: Vec<u8>) -> Vec<u8> {
        let crc = crc16_modbus(&bytes);
        bytes.push(crc as u8);
        bytes.push((crc >> 8) as u8);
        bytes
    }

    #[test]
    fn read_holding_round_trip() {
        // Slave 1 read 3 regs at 0x0000 → returns [10, 20, 30].
        let resp = frame_with_crc(std::vec![0x01, 0x03, 0x06, 0, 10, 0, 20, 0, 30]);
        let uart = MockUart::new(resp);
        let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);

        let mut out = [0u16; 3];
        t.read_holding(0x01, 0x0000, &mut out).unwrap();
        assert_eq!(out, [10, 20, 30]);

        // Verify the request that went out matches the canonical encoding.
        let (uart, _) = t.release();
        let expected_req = build_read_request(0x01, 0x0000, 3);
        assert_eq!(uart.tx, expected_req);
    }

    #[test]
    fn write_single_round_trip() {
        // Echo response.
        let req = build_write_single_request(0x01, 0x0012, 0x0001);
        let uart = MockUart::new(req.to_vec());
        let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
        t.write_single_holding(0x01, 0x0012, 0x0001).unwrap();
    }

    #[test]
    fn write_multiple_round_trip() {
        let resp = frame_with_crc(std::vec![0x01, 0x10, 0x00, 0x52, 0x00, 0x03]);
        let uart = MockUart::new(resp);
        let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
        t.write_multiple_holdings(0x01, 0x0052, &[1000, 1500, 1250])
            .unwrap();
    }

    #[test]
    fn exception_response_propagates() {
        let frame = frame_with_crc(std::vec![0x01, 0x83, 0x02]);
        let uart = MockUart::new(frame);
        let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
        let mut out = [0u16; 1];
        let err = t.read_holding(0x01, 0x0000, &mut out).unwrap_err();
        assert_eq!(err, RtuError::Modbus(ModbusError::Exception(0x02)));
    }

    #[test]
    fn timeout_when_no_data() {
        let uart = MockUart::new(Vec::new());
        let mut t = UartTransport::new(uart, NoDelay).with_timing(3, 0);
        let mut out = [0u16; 1];
        assert_eq!(
            t.read_holding(0x01, 0x0000, &mut out).unwrap_err(),
            RtuError::UartRead
        );
    }

    #[test]
    fn pre_existing_rx_is_drained() {
        // Stale garbage byte that would otherwise corrupt the parse,
        // followed by the real response.
        let response = frame_with_crc(std::vec![0x01, 0x03, 0x02, 0x00, 0x05]);
        let uart = MockUart::new(response).with_stale(std::vec![0xAA, 0xBB, 0xCC]);
        let mut t = UartTransport::new(uart, NoDelay).with_timing(50, 0);
        let mut out = [0u16; 1];
        t.read_holding(0x01, 0x0000, &mut out).unwrap();
        assert_eq!(out, [5]);
    }
}
