//! Captive-portal API: WiFi scan, credential save, status polling, and
//! the Android captive-detection probe.
//!
//! `/save` parks parsed creds in the single-slot mailbox and flips the
//! status atomic to `Pending`. The supervisor drains the mailbox on its
//! next captive-arm tick, applies creds via `set_sta_creds`, and (on
//! association) persists to NVS — bad creds therefore never overwrite a
//! known-good pair on flash. On success the supervisor drops the captive
//! bundle; the page's `/status` poll then errors, which it treats as
//! success.

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::sys::EspError;
use log::info;

use esp32_battery_logic::form;

use crate::http::{JsonBuf, mount_get, mount_json_get, mount_post, read_to_buf, text_response};
use crate::net::{CredsMailbox, SubmissionStatus, SubmissionStatusHandle};
use crate::nvs_creds::WifiCredentials;
use crate::wifi::ScanCache;

const SCAN_BUF_SIZE: usize = 1024;
const STATUS_BUF_SIZE: usize = 64;

pub fn mount(
    server: &mut EspHttpServer<'static>,
    scan_cache: ScanCache,
    mailbox: CredsMailbox,
    status: SubmissionStatusHandle,
) {
    mount_json_get(
        server,
        "/scan",
        JsonBuf::<SCAN_BUF_SIZE>::new(),
        move |buf| {
            let cached = scan_cache.lock().unwrap();
            let rows: Vec<(&str, i8)> = cached
                .entries
                .iter()
                .map(|(s, r)| (s.as_str(), *r))
                .collect();
            serde_json_core::to_slice(&rows, buf)
        },
    );

    let save_status = status.clone();
    mount_post(server, "/save", move |mut req| {
        let mut body_buf = [0u8; 256];
        let filled = read_to_buf(&mut req, &mut body_buf)?;
        let Ok(body) = std::str::from_utf8(&body_buf[..filled]) else {
            return text_response(req, 400, b"Body is not valid UTF-8");
        };

        let Some((ssid_raw, pass_raw)) = form::parse_form(body) else {
            return text_response(req, 400, b"Missing SSID");
        };

        let mut ssid_buf = [0u8; 33];
        let ssid_len = form::url_decode(ssid_raw, &mut ssid_buf);
        let Ok(ssid) = std::str::from_utf8(&ssid_buf[..ssid_len]) else {
            return text_response(req, 400, b"SSID is not valid UTF-8");
        };

        let mut pass_buf = [0u8; 65];
        let pass_len = form::url_decode(pass_raw, &mut pass_buf);
        let Ok(password) = std::str::from_utf8(&pass_buf[..pass_len]) else {
            return text_response(req, 400, b"Password is not valid UTF-8");
        };

        if ssid.is_empty() || ssid.len() > 32 {
            return text_response(req, 400, b"Invalid SSID");
        }
        // WPA/WPA2: 8-63 chars, or empty for open networks.
        if !password.is_empty() && !(8..=63).contains(&password.len()) {
            return text_response(req, 400, b"Password must be 8-63 characters");
        }

        let creds = WifiCredentials::new(ssid.to_string(), password.to_string());

        // Latest-wins: a second /save before the supervisor drains
        // overwrites the first.
        *mailbox.lock().unwrap() = Some(creds);
        save_status.store(SubmissionStatus::Pending);

        info!("Captive: queued new credentials for live STA reconnect");
        text_response(req, 200, b"OK")?;
        Ok::<(), EspError>(())
    });

    let status_state = status;
    mount_json_get(
        server,
        "/status",
        JsonBuf::<STATUS_BUF_SIZE>::new(),
        move |buf| {
            let name: &'static str = status_state.load().into();
            let response = StatusResponse { state: name };
            serde_json_core::to_slice(&response, buf)
        },
    );

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
