//! Driver for the XY-series programmable buck converters
//! (XY7025, XY6020L, XY6015, XY-SK60, XY-SK120, XY-SK120X).
//!
//! These modules share a common Modbus-RTU register layout — see the
//! crate's `README.md` for the full protocol reference.
//!
//! ```ignore
//! use xy_modbus::{Model, Xy, SafetyLimits};
//!
//! let mut xy = Xy::new(my_transport, Model::Xy7025);
//! xy.set_protection(SafetyLimits { lvp_v: 22.0, ovp_v: 15.0, ocp_a: 15.0 })?;
//! xy.set_voltage(13.5)?;
//! xy.set_current_limit(10.0)?;
//! xy.set_output(true)?;
//!
//! let s = xy.read_status()?;
//! println!("{:.2} V @ {:.2} A", s.v_out, s.i_out);
//! ```
//!
//! The crate is `no_std`. With the default `embedded-io` feature, the
//! [`uart`] module provides a ready-to-use [`ModbusTransport`] over any
//! `embedded-io` UART. To use a different transport, disable default
//! features and implement [`ModbusTransport`] yourself; the [`framing`]
//! module exposes the on-wire codec.

#![no_std]

pub mod device;
pub mod framing;
pub mod regs;
pub mod transport;
pub mod types;

#[cfg(feature = "embedded-io")]
pub mod uart;

#[cfg(feature = "embedded-io")]
pub use uart::UartTransport;

pub use device::Xy;
pub use transport::{ModbusError, ModbusTransport, RtuError};
pub use types::{
    BaudRate, GroupParams, Model, OnTime, ProtectionStatus, RegMode, SafetyLimits, Setpoints,
    Status, TempUnit, Totals,
};
