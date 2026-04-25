//! GET /api/errors: structured snapshot of the event log.
//!
//! Wire shape:
//! ```json
//! {
//!   "ina_counts": { "init": 0, "bus_voltage_read": 3, ... },
//!   "xy_counts":  { "read_status": 5, ... },
//!   "recent":     [ [ts, "ina"|"xy", "<kind>"], ... ]   // oldest-first
//! }
//! ```

use std::sync::{Arc, Mutex};

use esp_idf_svc::http::server::EspHttpServer;
use serde::Serialize;
use serde::ser::{SerializeMap, SerializeSeq};

use esp32_battery_logic::error_log::{Event, EventLog};

use crate::http::{JsonBuf, json_response, mount_get};

/// EventLog is bounded (32 entries × ~40 chars + ~30 small counters), well
/// under 4 KiB even with worst-case float-ish formatting.
const RESPONSE_BUF_SIZE: usize = 4096;

struct InaCountsView<'a>(&'a EventLog);

impl Serialize for InaCountsView<'_> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut map = s.serialize_map(None)?;
        for (name, count) in self.0.ina_counts_iter() {
            map.serialize_entry(name, &count)?;
        }
        map.end()
    }
}

struct XyCountsView<'a>(&'a EventLog);

impl Serialize for XyCountsView<'_> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut map = s.serialize_map(None)?;
        for (name, count) in self.0.xy_counts_iter() {
            map.serialize_entry(name, &count)?;
        }
        map.end()
    }
}

struct RecentView<'a>(&'a EventLog);

impl Serialize for RecentView<'_> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut seq = s.serialize_seq(Some(self.0.len()))?;
        for e in self.0.recent() {
            let (source, kind) = match e.event {
                Event::Ina(k) => ("ina", k.name()),
                Event::Xy(k) => ("xy", k.name()),
            };
            seq.serialize_element(&(e.ts, source, kind))?;
        }
        seq.end()
    }
}

#[derive(Serialize)]
struct ErrorsResponse<'a> {
    ina_counts: InaCountsView<'a>,
    xy_counts: XyCountsView<'a>,
    recent: RecentView<'a>,
}

pub fn mount(server: &mut EspHttpServer<'static>, event_log: Arc<Mutex<EventLog>>) {
    let json_buf: JsonBuf<RESPONSE_BUF_SIZE> = JsonBuf::new();

    mount_get(server, "/api/errors", move |req| {
        json_buf.with(|buf| {
            json_response(req, buf, |buf| {
                let log = event_log.lock().unwrap();
                let response = ErrorsResponse {
                    ina_counts: InaCountsView(&log),
                    xy_counts: XyCountsView(&log),
                    recent: RecentView(&log),
                };
                serde_json_core::to_slice(&response, buf)
            })
        })
    });
}
