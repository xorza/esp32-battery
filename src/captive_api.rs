//! Captive-portal API: WiFi scan, credential save, status polling, and
//! the Android captive-detection probe.
//!
//! `/save` parks parsed creds in the single-slot mailbox and flips the
//! status atomic to `Pending`. The supervisor drains the mailbox on its
//! next captive-arm tick, applies creds via `set_sta_creds`, and (on
//! association) persists to NVS — bad creds therefore never overwrite a
//! known-good pair on flash.

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::sys::EspError;
use log::info;
use serde::Serialize;
use serde::ser::SerializeSeq;

use esp32_battery_logic::{parse_form, url_decode};

use crate::http::{json_err, json_ok, mount_get, mount_json_get, mount_post, read_to_buf};
use crate::net::{CredsMailbox, SubmissionStatus, SubmissionStatusHandle};
use crate::wifi::{ScanCache, ScanResult};
use esp32_battery_logic::{PASSWORD_MAX, SSID_MAX, WifiCredentials};

struct ScanRowsView<'a>(&'a ScanResult);

impl Serialize for ScanRowsView<'_> {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut seq = s.serialize_seq(Some(self.0.len()))?;
        for (ssid, rssi) in self.0.iter() {
            seq.serialize_element(&(ssid.as_str(), *rssi))?;
        }
        seq.end()
    }
}

pub fn mount(
    server: &mut EspHttpServer<'static>,
    scan_cache: ScanCache,
    mailbox: CredsMailbox,
    status: SubmissionStatusHandle,
) {
    mount_json_get(server, "/scan", move |buf| {
        let cached = scan_cache.lock().unwrap();
        serde_json_core::to_slice(&ScanRowsView(&cached.entries), buf)
    });

    let save_status = status.clone();
    mount_post(server, "/save", move |mut req| {
        // Worst-case URL-encoded body: `ssid=` + 32*3 + `&pass=` + 63*3
        // = 296 bytes. 384 leaves headroom for stray `&` params without
        // letting an attacker stream unbounded input into the handler.
        let mut body_buf = [0u8; 384];
        let Some(filled) = read_to_buf(&mut req, &mut body_buf)? else {
            return json_err(req, 413, "Request body too large");
        };
        let Ok(body) = std::str::from_utf8(&body_buf[..filled]) else {
            return json_err(req, 400, "Body is not valid UTF-8");
        };

        let Some((ssid_raw, pass_raw)) = parse_form(body) else {
            return json_err(req, 400, "Missing SSID");
        };

        // Buffers sized one past the radio limit so an at-limit input fits
        // and an oversize one still decodes to something over the limit —
        // `WifiCredentials::new` then rejects it, where a buffer sized to the
        // limit exactly would have silently truncated it into a valid pair.
        let mut ssid_buf = [0u8; SSID_MAX + 1];
        let ssid_len = url_decode(ssid_raw, &mut ssid_buf);
        let Ok(ssid) = std::str::from_utf8(&ssid_buf[..ssid_len]) else {
            return json_err(req, 400, "SSID is not valid UTF-8");
        };

        let mut pass_buf = [0u8; PASSWORD_MAX + 1];
        let pass_len = url_decode(pass_raw, &mut pass_buf);
        let Ok(password) = std::str::from_utf8(&pass_buf[..pass_len]) else {
            return json_err(req, 400, "Password is not valid UTF-8");
        };

        // Length and shape rules live in `WifiCredentials::new`, the one
        // place every producer of credentials goes through.
        let creds = match WifiCredentials::new(ssid, password) {
            Ok(creds) => creds,
            Err(e) => return json_err(req, 400, e.message()),
        };

        // Latest-wins: a second /save before the supervisor drains
        // overwrites the first.
        *mailbox.lock().unwrap() = Some(creds);
        save_status.store(SubmissionStatus::Pending);

        info!("Captive: queued new credentials for live STA reconnect");
        json_ok(req)?;
        Ok::<(), EspError>(())
    });

    let status_state = status;
    mount_json_get(server, "/status", move |buf| {
        let name: &'static str = status_state.load().into();
        let response = StatusResponse { state: name };
        serde_json_core::to_slice(&response, buf)
    });

    // Android captive portal detection: expects 204, gets 302 → triggers popup.
    mount_get(server, "/generate_204", |req| {
        req.into_response(
            302,
            None,
            &[
                ("Location", "http://192.168.71.1/"),
                ("Connection", "close"),
            ],
        )
        .map_err(|e| e.0)?;
        Ok::<(), EspError>(())
    });
}

#[derive(serde::Serialize)]
struct StatusResponse {
    state: &'static str,
}
