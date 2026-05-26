//! GET /api: typed JSON snapshot of current sensor state + history.
//!
//! Wire format preserves the prior hand-rolled shape so the frontend is unchanged.
//! History rows use a 5-tuple that serializes as `[t, v, c1, c2, online]`.

use esp_idf_svc::http::server::EspHttpServer;
use log::debug;
use serde::Serialize;
use serde::ser::SerializeSeq;

use std::sync::{Arc, Mutex};

use esp32_battery_logic::data::{Sample, SensorData};

use crate::PACK_PROFILE;
use crate::clock::uptime;
use crate::http::mount_json_get;
use crate::wifi::sta_rssi;

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
    pub power_online: f32,
    pub heap: HeapInfo,
    pub battery: BatteryReading,
    pub ps: PsReading,
    pub model_code: u16,
    pub history: HistoryView<'a>,
}

pub fn mount(server: &mut EspHttpServer<'static>, sensor_data: Arc<Mutex<SensorData>>) {
    mount_json_get(server, "/api", move |buf| {
        // Sensor-data lock held only through serialization — history
        // is borrowed, not cloned. Lock drops at closure end, before
        // the network write.
        let store = sensor_data.lock().unwrap();
        let bat = store.battery_reading().unwrap_or_default();
        let ps = store.ps_reading().unwrap_or_default();
        let response = ApiResponse {
            uptime: uptime().as_secs() as u32,
            rssi: sta_rssi(),
            voltage: bat.voltage,
            power_online: store.power_online(),
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
            model_code: store.model_code,
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
