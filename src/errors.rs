//! GET /api/errors: structured snapshot of the event log.
//!
//! Wire shape:
//! ```json
//! {
//!   "ina_counts": { "init": 0, "bus_voltage_read": 3, ... },
//!   "xy_counts":  { "read_status": 5, ... },
//!   "charge_counts": { "energised": 1, "latched": 0, ... },
//!   "recent":     [ [ts, "ina"|"xy"|"charge", "<kind>"], ... ]   // oldest-first
//! }
//! ```

use std::sync::{Arc, Mutex};

use esp_idf_svc::http::server::EspHttpServer;
use serde::Serialize;
use serde::ser::SerializeSeq;

use esp32_battery_logic::error_log::{Event, EventLog};

use crate::http::mount_json_get;

/// Which per-kind counter map to render. The three groups differ only in
/// which iterator they pull from, so they share one view rather than three
/// identical `Serialize` impls.
#[derive(Copy, Clone)]
enum CountKind {
    Ina,
    Xy,
    Charge,
}

struct CountsView<'a>(&'a EventLog, CountKind);

impl Serialize for CountsView<'_> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        match self.1 {
            CountKind::Ina => s.collect_map(self.0.ina_counts_iter()),
            CountKind::Xy => s.collect_map(self.0.xy_counts_iter()),
            CountKind::Charge => s.collect_map(self.0.charge_counts_iter()),
        }
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
                Event::Charge(k) => ("charge", k.name()),
            };
            seq.serialize_element(&(e.ts, source, kind))?;
        }
        seq.end()
    }
}

#[derive(Serialize)]
struct ErrorsResponse<'a> {
    ina_counts: CountsView<'a>,
    xy_counts: CountsView<'a>,
    charge_counts: CountsView<'a>,
    recent: RecentView<'a>,
}

pub fn mount(server: &mut EspHttpServer<'static>, event_log: Arc<Mutex<EventLog>>) {
    mount_json_get(server, "/api/errors", move |buf| {
        let log = event_log.lock().unwrap();
        let response = ErrorsResponse {
            ina_counts: CountsView(&log, CountKind::Ina),
            xy_counts: CountsView(&log, CountKind::Xy),
            charge_counts: CountsView(&log, CountKind::Charge),
            recent: RecentView(&log),
        };
        serde_json_core::to_slice(&response, buf)
    });
}
