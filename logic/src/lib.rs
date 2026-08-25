//! Pure logic shared by the firmware: charge supervision, the network
//! state machine, sensor history, and the event log. No I/O, no clock,
//! no hardware — everything here is host-testable.
//!
//! The `pub use`s below are the crate's published surface; modules
//! themselves are crate-internal so each item has one canonical path.

mod battery;
mod charging;
mod data;
mod dns_packet;
mod error_log;
mod form;
mod log_ring;
mod net;

pub use battery::Chemistry;
pub use charging::action::{Action, DisableTicket, EnableTicket, VoltageTicket};
pub use charging::charge_supervisor::ChargeSupervisor;
pub use charging::fault_reason::FaultReason;
pub use charging::inhibit_reason::InhibitReason;
pub use charging::phase::Phase;
pub use charging::poll_result::{BatterySample, BuckOutput, PollResult};
pub use charging::profile::Profile;
pub use charging::voltage_writer::{VoltageWriteOutcome, VoltageWriter, apply_update_voltage};
pub use charging::INPUT_LVP_MARGIN_V;

pub use data::{Ina228Reading, PsReading, Sample, SensorData};
pub use error_log::{ChargeTransition, Event, EventLog, InaError, TimedEvent, XyError};

pub use dns_packet::build_response;
pub use form::{parse_form, url_decode};
pub use log_ring::Ring;

pub use net::wifi_credentials::{PASSWORD_MAX, SSID_MAX, WifiCredentials};
pub use net::net_action::NetAction;
pub use net::net_phase::{LinkState, NetPhase, NetStatus};
pub use net::net_poll::NetPoll;
pub use net::net_supervisor::NetSupervisor;

/// xy-modbus types that appear in this crate's own signatures, re-exported
/// so those signatures are nameable. `XyError` is renamed: `error_log::XyError`
/// is this crate's event kind.
pub use xy_modbus::{ProtectionStatus, SafetyLimits, Setpoints, XyError as BusError};
