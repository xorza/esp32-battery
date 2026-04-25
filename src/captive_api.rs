//! Captive-portal API: WiFi scan, credential save, status polling, and the
//! Android captive-detection probe.
//!
//! `SaveLifecycle` tracks the lifecycle of a credential submission so the
//! page can poll `/status` and show "Connecting...", "Failed", or trigger a
//! popup close on success. It lives inside `CaptiveBundle` (one per captive
//! phase) so its existence and the captive phase are coextensive. `/save`
//! is the sole writer of `Trying`; the supervisor drives `Connected` and
//! the timeout-driven `Failed` via methods on the shared handle.

use std::sync::Arc;
use std::sync::Mutex;
use std::sync::mpsc::Sender;
use std::time::{Duration, Instant};

use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;
use log::info;

use esp32_battery_logic::form;

use crate::http::{JsonBuf, json_response, mount_get, mount_post, read_to_buf, text_response};
use crate::nvs_creds::{self, WifiCredentials};
use crate::wifi::Wifi;

/// 10 APs × ~40 bytes (quoted 32-char SSID + rssi) + brackets.
const SCAN_BUF_SIZE: usize = 1024;
/// `/status` JSON is tiny — `{"state":"connected"}` is ~21 bytes.
const STATUS_BUF_SIZE: usize = 64;

pub enum SaveLifecycle {
    Idle,
    Trying { since: Instant },
    Connected,
    Failed,
}

impl SaveLifecycle {
    fn name(&self) -> &'static str {
        match self {
            Self::Idle => "idle",
            Self::Trying { .. } => "trying",
            Self::Connected => "connected",
            Self::Failed => "failed",
        }
    }

    pub fn begin_trying(&mut self, now: Instant) {
        *self = Self::Trying { since: now };
    }

    pub fn mark_connected(&mut self) {
        *self = Self::Connected;
    }

    /// If currently `Trying` and the `Trying` window has aged past `timeout`,
    /// transition to `Failed` and return true. Caller logs on the rising edge.
    pub fn tick_timeout(&mut self, now: Instant, timeout: Duration) -> bool {
        if let Self::Trying { since } = self
            && now.duration_since(*since) >= timeout
        {
            *self = Self::Failed;
            return true;
        }
        false
    }
}

pub type SaveStateHandle = Arc<Mutex<SaveLifecycle>>;

pub fn new_save_state() -> SaveStateHandle {
    Arc::new(Mutex::new(SaveLifecycle::Idle))
}

pub fn mount(
    server: &mut EspHttpServer<'static>,
    wifi: Arc<Mutex<Wifi<'static>>>,
    creds_tx: Sender<WifiCredentials>,
    nvs: Arc<EspNvs<NvsDefault>>,
    save_state: SaveStateHandle,
) {
    let scan_buf: JsonBuf<SCAN_BUF_SIZE> = JsonBuf::new();
    mount_get(server, "/scan", move |req| {
        scan_buf.with(|buf| {
            json_response(req, buf, |buf| {
                let entries = wifi.lock().unwrap().scan();
                let rows: Vec<(&str, i8)> = entries.iter().map(|(s, r)| (s.as_str(), *r)).collect();
                serde_json_core::to_slice(&rows, buf)
            })
        })
    });

    let state_save = save_state.clone();
    mount_post(server, "/save", move |mut req| {
        let mut body_buf = [0u8; 256];
        let filled = read_to_buf(&mut req, &mut body_buf)?;
        let body = std::str::from_utf8(&body_buf[..filled]).unwrap_or("");

        let Some((ssid_raw, pass_raw)) = form::parse_form(body) else {
            return text_response(req, 400, b"Missing SSID");
        };

        let mut ssid_buf = [0u8; 33];
        let ssid_len = form::url_decode(ssid_raw, &mut ssid_buf);
        let ssid = std::str::from_utf8(&ssid_buf[..ssid_len]).unwrap_or("");

        let mut pass_buf = [0u8; 65];
        let pass_len = form::url_decode(pass_raw, &mut pass_buf);
        let password = std::str::from_utf8(&pass_buf[..pass_len]).unwrap_or("");

        if ssid.is_empty() || ssid.len() > 32 {
            return text_response(req, 400, b"Invalid SSID");
        }
        // WPA/WPA2: 8-63 chars, or empty for open networks.
        if !password.is_empty() && !(8..=63).contains(&password.len()) {
            return text_response(req, 400, b"Password must be 8-63 characters");
        }

        nvs_creds::save(&nvs, ssid, password);
        let _ = creds_tx.send(WifiCredentials {
            ssid: ssid.to_string(),
            password: password.to_string(),
        });
        // Flip to Trying immediately, stamping the window's start. The
        // supervisor sees the queued creds on its next tick and starts the
        // live STA-credential update; once the STA actually associates, the
        // supervisor will call `mark_connected`. If the window ages past
        // CAPTIVE_TRYING_TIMEOUT, the supervisor flips us to `Failed`.
        state_save.lock().unwrap().begin_trying(Instant::now());
        info!("Captive: queued new credentials for live STA reconnect");
        text_response(req, 200, b"OK")?;
        Ok::<(), EspError>(())
    });

    let status_buf: JsonBuf<STATUS_BUF_SIZE> = JsonBuf::new();
    let state_status = save_state;
    mount_get(server, "/status", move |req| {
        status_buf.with(|buf| {
            json_response(req, buf, |buf| {
                let name = state_status.lock().unwrap().name();
                let response = StatusResponse { state: name };
                serde_json_core::to_slice(&response, buf)
            })
        })
    });

    // Android captive portal detection: expects 204, gets 302 → triggers popup.
    // The redirect stays in place across the whole captive lifetime — once
    // STA associates the supervisor drops the AP, which is what actually
    // clears the OS-level captive flag.
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
