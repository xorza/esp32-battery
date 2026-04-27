//! Convenience glue for `esp-idf-hal` UART drivers.
//!
//! `UartDriver` already implements `embedded_io::{Read, Write}` but not
//! `ReadReady`, which the bundled [`UartTransport`] needs. The wrapper
//! here adds it via `UartDriver::remaining_read()` and exposes a
//! one-call constructor:
//!
//! ```ignore
//! use xy_modbus::{Model, Xy};
//!
//! let mut xy = Xy::from_esp_uart(uart, Model::Xy7025);
//! xy.set_protection(safety)?;
//! xy.set_voltage(13.5)?;
//! xy.set_output(true)?;
//! ```

use embedded_io::{ErrorType, Read, ReadReady, Write};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::io::EspIOError;
use esp_idf_hal::uart::UartDriver;

use crate::device::Xy;
use crate::types::Model;
use crate::uart::UartTransport;

/// Newtype that adds `embedded_io::ReadReady` to
/// `esp_idf_hal::uart::UartDriver`. Forwards `Read`, `Write`, and
/// `flush` unchanged.
pub struct UartReadReady<'d>(UartDriver<'d>);

impl<'d> UartReadReady<'d> {
    pub fn new(uart: UartDriver<'d>) -> Self {
        Self(uart)
    }

    pub fn release(self) -> UartDriver<'d> {
        self.0
    }
}

impl ErrorType for UartReadReady<'_> {
    type Error = EspIOError;
}

impl Read for UartReadReady<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        Read::read(&mut self.0, buf)
    }
}

impl Write for UartReadReady<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        Write::write(&mut self.0, buf)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Write::flush(&mut self.0)
    }
}

impl ReadReady for UartReadReady<'_> {
    fn read_ready(&mut self) -> Result<bool, Self::Error> {
        Ok(self.0.remaining_read().map_err(EspIOError)? > 0)
    }
}

/// Concrete transport type produced by [`Xy::from_esp_uart`].
pub type EspIdfTransport<'d> = UartTransport<UartReadReady<'d>, FreeRtos>;

impl<'d> Xy<EspIdfTransport<'d>> {
    /// Wrap an `esp_idf_hal::uart::UartDriver` with the default XY-series
    /// timing (500 ms response window, 50 ms inter-frame gap). For
    /// non-default timing, build the transport manually:
    ///
    /// ```ignore
    /// let transport = UartTransport::new(UartReadReady::new(uart), FreeRtos)
    ///     .with_timing(750, 100);
    /// let xy = Xy::new(transport, Model::Xy7025);
    /// ```
    pub fn from_esp_uart(uart: UartDriver<'d>, model: Model) -> Self {
        Self::new(
            UartTransport::new(UartReadReady::new(uart), FreeRtos),
            model,
        )
    }
}

