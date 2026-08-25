//! GET /api: typed JSON snapshot of current sensor state + history.
//!
//! History rows use a 5-tuple that serializes as `[t, v, c1, c2, online]`,
//! where `online` is the fraction of the row's span the supply was up.

use core::fmt::Write as _;

use esp_idf_svc::http::server::EspHttpServer;
use log::debug;
use serde::Serialize;
use serde::ser::SerializeSeq;

use std::sync::{Arc, Mutex};

use esp32_battery_logic::{ChargeStatus, Sample, SensorData};

use crate::PACK_PROFILE;
use crate::clock::uptime;
use crate::http::mount_json_get;
use crate::wifi::sta_rssi;

/// Longest supervisor reason `Display` the response will carry.
const REASON_DISPLAY_CAP: usize = 64;

#[derive(Serialize)]
pub struct BatteryReading {
    pub soc: f32,
    pub current: f32,
    pub power: f32,
}

#[derive(Serialize)]
pub struct PsReading {
    pub voltage: f32,
    pub current: f32,
    pub power: f32,
    pub v_set: f32,
    pub i_set: f32,
}

/// Serializes a borrowed slice of history samples as a JSON array of 5-tuples
/// `[time_s, voltage, bat_current, ps_current, power_online]`. The wrapper
/// lets us build the `ApiResponse` + serialize it without first cloning the
/// history into an owned `Vec<HistoryRow>`.
pub struct HistoryView<'a>(pub &'a [Sample]);

impl Serialize for HistoryView<'_> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut seq = s.serialize_seq(Some(self.0.len()))?;
        for sample in self.0 {
            seq.serialize_element(&(
                sample.time_s,
                sample.voltage,
                sample.battery_current,
                sample.ps_current,
                sample.power_online,
            ))?;
        }
        seq.end()
    }
}

#[derive(Serialize)]
pub struct HeapInfo {
    /// Bytes currently free in the default heap.
    pub free: u32,
    /// Low-water mark of `free` since boot — useful to spot leaks/growth.
    pub min_free: u32,
}

impl HeapInfo {
    pub fn new() -> Self {
        unsafe {
            Self {
                free: esp_idf_svc::sys::esp_get_free_heap_size(),
                min_free: esp_idf_svc::sys::esp_get_minimum_free_heap_size(),
            }
        }
    }
}

#[derive(Serialize)]
pub struct ApiResponse<'a> {
    pub uptime: u32,
    pub rssi: i32,
    pub voltage: f32,
    pub heap: HeapInfo,
    pub battery: BatteryReading,
    pub ps: PsReading,
    pub model_code: u16,
    /// Static pack identity, e.g. `LFP 4S 50Ah`.
    pub profile: &'a str,
    /// `true` while the DC supply is disconnected (buck input UVLO). Shown
    /// as a transient "PS offline" status; clears when the supply returns.
    pub ps_offline: bool,
    /// `"absorb"` / `"float"` while the supervisor is actively regulating,
    /// `null` while still bringing up, or once a fault has latched the buck off.
    pub phase: Option<&'static str>,
    /// `true` when the fault stopped the charge but left the output up and
    /// the load fed. `false` with a fault set means the buck is dark and
    /// the pack is carrying the load.
    pub parked: bool,
    /// Snake_case identifier of the latched fault, or `null` if none.
    /// Stable for dashboards to switch on; `fault_message` is the
    /// human-readable form (with cause for `OutputUnexpectedlyOff`).
    pub fault: Option<&'static str>,
    pub fault_message: Option<&'a str>,
    /// Snake_case identifier of the condition currently blocking bring-up,
    /// or `null` if none. Mutually exclusive with `fault`: a latched fault
    /// is terminal, an inhibit clears on its own.
    pub inhibit: Option<&'static str>,
    pub inhibit_message: Option<&'a str>,
    pub history: HistoryView<'a>,
}

/// Render a supervisor reason for display, or `None` when there is none.
/// `fault` and `inhibit` are both `Option<impl Display>` and differ only in
/// which response field they land in, so they share this.
fn reason_message<T: core::fmt::Display>(
    reason: Option<T>,
) -> Option<heapless::String<REASON_DISPLAY_CAP>> {
    let reason = reason?;
    let mut out = heapless::String::new();
    let _ = write!(out, "{reason}");
    Some(out)
}

pub fn mount(
    server: &mut EspHttpServer<'static>,
    sensor_data: Arc<Mutex<SensorData>>,
    charge_status: Arc<Mutex<ChargeStatus>>,
) {
    mount_json_get(server, "/api", move |buf| {
        // Copied out and released before the sensor-data lock is taken, so
        // the XY thread's per-tick publish never queues behind the history
        // serialization below and the two locks are never nested.
        let status = *charge_status.lock().unwrap();
        // Sensor-data lock held only through serialization — history
        // is borrowed, not cloned. Lock drops at closure end, before
        // the network write.
        let store = sensor_data.lock().unwrap();
        let bat = store.battery_reading().unwrap_or_default();
        let ps = store.ps_reading().unwrap_or_default();
        let mut profile = heapless::String::<32>::new();
        let _ = write!(profile, "{PACK_PROFILE}");
        let fault_message = reason_message(status.fault);
        let inhibit_message = reason_message(status.inhibit);
        let response = ApiResponse {
            uptime: uptime().as_secs() as u32,
            rssi: sta_rssi(),
            voltage: bat.voltage,
            heap: HeapInfo::new(),
            battery: BatteryReading {
                soc: PACK_PROFILE.soc(bat.voltage),
                current: bat.current,
                power: bat.power,
            },
            ps: PsReading {
                voltage: ps.voltage,
                current: ps.current,
                power: ps.power,
                v_set: ps.v_set,
                i_set: ps.i_set,
            },
            model_code: status.model_code,
            profile: &profile,
            ps_offline: status.ps_offline,
            phase: status.phase.map(|p| p.label()),
            fault: status.fault.map(|f| f.label()),
            parked: status.parked,
            fault_message: fault_message.as_deref(),
            inhibit: status.inhibit.map(|i| i.label()),
            inhibit_message: inhibit_message.as_deref(),
            history: HistoryView(store.history()),
        };
        let history_len = response.history.0.len();
        let cap = buf.len();
        let result = serde_json_core::to_slice(&response, buf);
        if let Ok(len) = result {
            debug!("API: history={} json={}/{}", history_len, len, cap);
        }
        result
    });
}
