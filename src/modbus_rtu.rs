//! `xy_modbus::UartTransport` glue for `esp-idf-hal`'s `UartDriver`.
//!
//! `UartDriver` already implements `embedded_io::{Read, Write}` but not
//! `ReadReady`, which `UartTransport` uses to gate its read loop. The
//! newtype below adds it via `UartDriver::remaining_read()`.

use embedded_io::{ErrorType, Read, ReadReady, Write};
use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::io::EspIOError;
use esp_idf_hal::uart::UartDriver;

use xy_modbus::UartTransport;

pub struct UartReadReady<'d>(UartDriver<'d>);

impl<'d> UartReadReady<'d> {
    pub fn new(uart: UartDriver<'d>) -> Self {
        Self(uart)
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

pub type EspUartTransport<'d> = UartTransport<UartReadReady<'d>, FreeRtos>;

pub fn new_transport<'d>(
    uart: UartDriver<'d>,
    response_timeout_ms: u32,
    inter_frame_ms: u32,
) -> EspUartTransport<'d> {
    UartTransport::new(UartReadReady::new(uart), FreeRtos)
        .with_timing(response_timeout_ms, inter_frame_ms)
}
