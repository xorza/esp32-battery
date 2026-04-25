//! GET /api: typed JSON snapshot of current sensor state + history.
//!
//! Wire format preserves the prior hand-rolled shape so the frontend is unchanged.
//! History rows use a 5-tuple that serializes as `[t, v, c1, c2, online]`.

use esp_idf_svc::http::server::EspHttpServer;
use log::debug;
use serde::Serialize;
use serde::ser::SerializeSeq;

use std::sync::{Arc, Mutex};

use esp32_battery_logic::battery;
use esp32_battery_logic::data::{Sample, SensorData};

use crate::clock::uptime_s;
use crate::http::{JsonBuf, json_response, mount_get};
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
    pub history: HistoryView<'a>,
}

/// Response buffer. Typical size is ~5 KiB (144 rows × ~30 chars). Bad sensor readings
/// (NaN, denormals) can push ryu up to ~17 chars per float → 144 × 85 = 12 KiB worst case,
/// so 16 KiB leaves margin. If serialization still overflows we return 500 instead of panicking.
pub const RESPONSE_BUF_SIZE: usize = 16_384;

pub fn mount(server: &mut EspHttpServer<'static>, sensor_data: Arc<Mutex<SensorData>>) {
    let json_buf: JsonBuf<RESPONSE_BUF_SIZE> = JsonBuf::new();

    mount_get(server, "/api", move |req| {
        json_buf.with(|buf| {
            json_response(req, buf, |buf| {
                // Sensor-data lock held only through serialization — history
                // is borrowed, not cloned. Lock drops at closure end, before
                // the network write.
                let store = sensor_data.lock().unwrap();
                let bat = store.battery_reading().unwrap_or_default();
                let ps = store.ps_reading().unwrap_or_default();
                let response = ApiResponse {
                    uptime: uptime_s(),
                    rssi: sta_rssi(),
                    voltage: bat.voltage,
                    power_online: store.power_online(),
                    heap: HeapInfo::new(),
                    battery: BatteryReading {
                        soc: battery::ocv_soc(bat.voltage),
                        current: bat.current,
                        power: bat.power,
                    },
                    ps: PsReading {
                        voltage: ps.voltage,
                        current: ps.current,
                        power: ps.power,
                    },
                    history: HistoryView(store.history()),
                };
                let history_len = response.history.0.len();
                let result = serde_json_core::to_slice(&response, buf);
                if let Ok(len) = result {
                    debug!(
                        "API: history={} json={}/{}",
                        history_len, len, RESPONSE_BUF_SIZE
                    );
                }
                result
            })
        })
    });
}
