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

use std::marker::PhantomData;
use std::sync::{Arc, Mutex};

use esp_idf_svc::http::server::EspHttpServer;
use serde::Serialize;
use serde::ser::SerializeSeq;

use esp32_battery_logic::{ChargeTransition, EventKind, EventLog, InaError, XyError};

use crate::http::mount_json_get;

/// The `{ kind: count }` map for one event source. Generic over the source so
/// the three groups share one `Serialize` impl.
/// `PhantomData<fn() -> K>`: the view names a source, it does not own one.
struct CountsView<'a, K>(&'a EventLog, PhantomData<fn() -> K>);

impl<'a, K> CountsView<'a, K> {
    fn new(log: &'a EventLog) -> Self {
        Self(log, PhantomData)
    }
}

impl<K: EventKind> Serialize for CountsView<'_, K> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.collect_map(self.0.counts_iter::<K>())
    }
}

struct RecentView<'a>(&'a EventLog);

impl Serialize for RecentView<'_> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut seq = s.serialize_seq(Some(self.0.len()))?;
        for e in self.0.recent() {
            let name = e.event.name();
            seq.serialize_element(&(e.ts, name.source, name.kind))?;
        }
        seq.end()
    }
}

#[derive(Serialize)]
struct ErrorsResponse<'a> {
    ina_counts: CountsView<'a, InaError>,
    xy_counts: CountsView<'a, XyError>,
    charge_counts: CountsView<'a, ChargeTransition>,
    recent: RecentView<'a>,
}

pub fn mount(server: &mut EspHttpServer<'static>, event_log: Arc<Mutex<EventLog>>) {
    mount_json_get(server, "/api/errors", move |buf| {
        let log = event_log.lock().unwrap();
        let response = ErrorsResponse {
            ina_counts: CountsView::new(&log),
            xy_counts: CountsView::new(&log),
            charge_counts: CountsView::new(&log),
            recent: RecentView(&log),
        };
        serde_json_core::to_slice(&response, buf)
    });
}
