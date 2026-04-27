//! Driver for the XY-series programmable buck converters
//! (XY7025, XY6020L, XY6015, XY-SK60, XY-SK120, XY-SK120X).
//!
//! These modules share a common Modbus-RTU register layout — see the
//! crate's `README.md` for the full protocol reference.
//!
//! ```ignore
//! use xy7025::{Xy, SafetyLimits};
//!
//! let mut xy = Xy::new(my_transport);
//! xy.set_protection(SafetyLimits { lvp_v: 22.0, ovp_v: 15.0, ocp_a: 15.0 })?;
//! xy.set_voltage(13.5)?;
//! xy.set_current_limit(10.0)?;
//! xy.set_output(true)?;
//!
//! let s = xy.read_status()?;
//! println!("{:.2} V @ {:.2} A", s.v_out, s.i_out);
//! ```
//!
//! The crate is `no_std` and has no dependencies. Bring your own
//! Modbus-RTU transport by implementing [`ModbusTransport`]; the
//! [`framing`] module exposes the on-wire codec so a transport
//! implementation is typically <100 lines over a UART.

#![no_std]
#![deny(rust_2018_idioms)]

pub mod device;
pub mod framing;
pub mod regs;
pub mod transport;
pub mod types;

pub use device::Xy;
pub use transport::{ModbusError, ModbusTransport, RtuError};
pub use types::{
    BaudRate, GroupParams, OnTime, ProtectionStatus, RegMode, SafetyLimits, Setpoints, Status,
    TempUnit, Totals,
};
