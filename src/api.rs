//! GET /api: typed JSON snapshot of current sensor state + history.
//!
//! Wire format preserves the prior hand-rolled shape so the frontend is unchanged.
//! History rows use a 5-tuple that serializes as `[t, v, c1, c2, online]`.

use std::sync::{Arc, Mutex};

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::sys::EspError;
use log::{debug, warn};
use serde::Serialize;

use esp32_battery_logic::battery;

use crate::app_state::Shared;
use crate::http::text_response;

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

/// Serializes as a JSON array `[time_s, voltage, bat_current, ps_current, power_online]`.
#[derive(Serialize)]
pub struct HistoryRow(pub u32, pub f32, pub f32, pub f32, pub f32);

impl From<&esp32_battery_logic::data::Sample> for HistoryRow {
    fn from(s: &esp32_battery_logic::data::Sample) -> Self {
        Self(
            s.time_s,
            s.voltage,
            s.battery_current,
            s.ps_current,
            s.power_online,
        )
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
pub struct ApiResponse {
    pub uptime: u32,
    pub rssi: i32,
    pub voltage: f32,
    pub power_online: f32,
    pub heap: HeapInfo,
    pub battery: BatteryReading,
    pub ps: PsReading,
    pub history: Vec<HistoryRow>,
}

/// Response buffer. Typical size is ~5 KiB (144 rows × ~30 chars). Bad sensor readings
/// (NaN, denormals) can push ryu up to ~17 chars per float → 144 × 85 = 12 KiB worst case,
/// so 16 KiB leaves margin. If serialization still overflows we return 500 instead of panicking.
pub const RESPONSE_BUF_SIZE: usize = 16_384;

fn get_rssi() -> i32 {
    let mut ap_info: esp_idf_svc::sys::wifi_ap_record_t = unsafe { std::mem::zeroed() };
    if unsafe { esp_idf_svc::sys::esp_wifi_sta_get_ap_info(&mut ap_info) } == 0 {
        ap_info.rssi as i32
    } else {
        0
    }
}

pub fn register(server: &mut EspHttpServer<'static>, shared: Arc<Shared>) {
    let json_buf = Mutex::new(Box::new([0u8; RESPONSE_BUF_SIZE]));

    server
        .fn_handler("/api", esp_idf_svc::http::Method::Get, move |req| {
            // Snapshot sensor state, release the lock, then serialize.
            // Keeps the measurement thread unblocked during JSON serialization.
            let response = {
                let store = shared.sensor_data.lock().unwrap();
                let bat = store.battery_reading.unwrap_or_default();
                let ps = store.ps_reading.unwrap_or_default();
                let power_online = store.power_online();
                let history_rows: Vec<HistoryRow> =
                    store.history().iter().map(HistoryRow::from).collect();

                ApiResponse {
                    uptime: crate::uptime_s(),
                    rssi: get_rssi(),
                    voltage: bat.voltage,
                    power_online,
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
                    history: history_rows,
                }
            };

            let mut guard = json_buf.lock().unwrap();
            let buf: &mut [u8] = &mut **guard;
            let len = match serde_json_core::to_slice(&response, buf) {
                Ok(n) => n,
                Err(e) => {
                    warn!("API: JSON serialization failed ({:?}); returning 500", e);
                    return text_response(req, 500, b"serialization error");
                }
            };

            debug!(
                "API: history={} json={}/{}",
                response.history.len(),
                len,
                RESPONSE_BUF_SIZE,
            );

            let mut resp = req
                .into_response(
                    200,
                    None,
                    &[
                        ("Content-Type", "application/json"),
                        ("Connection", "close"),
                    ],
                )
                .map_err(|e| e.0)?;
            resp.write_all(&buf[..len]).map_err(|e| e.0)?;
            Ok::<(), EspError>(())
        })
        .unwrap();
}
