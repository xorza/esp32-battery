//! Modbus-RTU transport trait and error types.
//!
//! Implement [`ModbusTransport`] over your platform's UART. The
//! [`crate::framing`] module gives you the on-wire codec; a typical
//! implementation is <100 lines of UART-specific timing on top.

use core::fmt;

/// Protocol-layer error: a frame was received but failed validation.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModbusError {
    /// Response was shorter than the smallest valid frame for the
    /// expected reply.
    ShortResponse(usize),
    /// Slave address byte didn't match the request.
    BadSlave(u8),
    /// Function-code, byte-count, address, or quantity field didn't
    /// match what was expected.
    BadHeader,
    /// CRC-16 mismatch.
    BadCrc,
    /// Slave returned a Modbus exception. The byte is the exception
    /// code (`0x01`–`0x0B` per the spec).
    Exception(u8),
}

impl fmt::Display for ModbusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ShortResponse(n) => write!(f, "short response ({n} bytes)"),
            Self::BadSlave(a) => write!(f, "wrong slave id 0x{a:02X}"),
            Self::BadHeader => write!(f, "malformed header"),
            Self::BadCrc => write!(f, "CRC mismatch"),
            Self::Exception(c) => write!(f, "modbus exception 0x{c:02X}"),
        }
    }
}

/// Unified error returned by the device API: either the transport
/// (UART layer) failed, or the response was a malformed / exception
/// Modbus frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RtuError {
    /// No (or insufficient) bytes received within the response window.
    Timeout,
    /// Underlying UART returned an I/O error on read or write.
    Io,
    /// Decoded response was invalid or the slave reported a Modbus
    /// exception.
    Modbus(ModbusError),
}

impl fmt::Display for RtuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout => f.write_str("UART response timed out"),
            Self::Io => f.write_str("UART I/O error"),
            Self::Modbus(e) => fmt::Display::fmt(e, f),
        }
    }
}

impl From<ModbusError> for RtuError {
    fn from(e: ModbusError) -> Self {
        Self::Modbus(e)
    }
}

impl core::error::Error for ModbusError {}
impl core::error::Error for RtuError {
    fn source(&self) -> Option<&(dyn core::error::Error + 'static)> {
        match self {
            Self::Modbus(e) => Some(e),
            _ => None,
        }
    }
}

/// Modbus-RTU transport: send a request, validate the response, hand
/// back the payload (for reads) or just `Ok(())` (for writes).
///
/// Implementers handle UART framing timing — the inter-frame gap, the
/// per-device response timeout, and the post-write quiet gap. The
/// XY-series wants ~50 ms between frames and ~500 ms response window.
///
/// All three function codes are required; the device API uses each
/// (`0x03` for reads, `0x06` for single setpoint writes, `0x10` for
/// bulk memory-group writes).
pub trait ModbusTransport {
    fn read_holding(&mut self, slave: u8, addr: u16, dst: &mut [u16]) -> Result<(), RtuError>;

    fn write_single_holding(&mut self, slave: u8, addr: u16, value: u16) -> Result<(), RtuError>;

    fn write_multiple_holdings(
        &mut self,
        slave: u8,
        addr: u16,
        values: &[u16],
    ) -> Result<(), RtuError>;
}
