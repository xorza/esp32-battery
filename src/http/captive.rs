//! Captive portal HTTP server: WiFi scan, credential save, portal page.

use std::fmt::Write as FmtWrite;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;
use log::info;

use esp32_battery_logic::form;

use crate::dns::DnsHandle;
use crate::wifi::Wifi;

use super::{create_server, serve_common_assets, serve_static};

pub fn start(
    nvs: Arc<EspNvs<NvsDefault>>,
    wifi: Arc<Mutex<Wifi<'static>>>,
) -> (EspHttpServer<'static>, DnsHandle) {
    let dns_handle = DnsHandle::start();

    let mut server = create_server(8192, true, 4, Some(Duration::from_secs(2)), false);

    // GET /scan — scan for visible WiFi networks
    let scan_buf = Mutex::new(Box::new(heapless::String::<1024>::new()));
    server
        .fn_handler("/scan", esp_idf_svc::http::Method::Get, move |req| {
            let entries = wifi.lock().unwrap().scan();

            let mut guard = scan_buf.lock().unwrap();
            let buf = &mut **guard;
            buf.clear();
            write!(buf, "[").unwrap();
            for (i, (ssid, rssi)) in entries.iter().enumerate() {
                if i > 0 {
                    write!(buf, ",").unwrap();
                }
                write!(buf, r#"["{}",{}]"#, ssid, rssi).unwrap();
            }
            write!(buf, "]").unwrap();

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
            resp.write_all(buf.as_bytes()).map_err(|e| e.0)?;
            Ok::<(), EspError>(())
        })
        .unwrap();

    // POST /save — save credentials and reboot
    server
        .fn_handler("/save", esp_idf_svc::http::Method::Post, move |mut req| {
            let mut body_buf = [0u8; 256];
            let mut filled = 0;
            loop {
                let n = req.read(&mut body_buf[filled..]).map_err(|e| e.0)?;
                if n == 0 {
                    break;
                }
                filled += n;
                if filled >= body_buf.len() {
                    break;
                }
            }
            let body = std::str::from_utf8(&body_buf[..filled]).unwrap_or("");

            let (ssid_raw, pass_raw) = match form::parse_form(body) {
                Some(pair) => pair,
                None => {
                    let mut resp = req
                        .into_response(400, None, &[("Connection", "close")])
                        .map_err(|e| e.0)?;
                    resp.write_all(b"Missing SSID").map_err(|e| e.0)?;
                    return Ok::<(), EspError>(());
                }
            };

            let mut ssid_buf = [0u8; 33];
            let ssid_len = form::url_decode(ssid_raw, &mut ssid_buf);
            let ssid = std::str::from_utf8(&ssid_buf[..ssid_len]).unwrap_or("");

            let mut pass_buf = [0u8; 65];
            let pass_len = form::url_decode(pass_raw, &mut pass_buf);
            let password = std::str::from_utf8(&pass_buf[..pass_len]).unwrap_or("");

            if ssid.is_empty() || ssid.len() > 32 {
                let mut resp = req
                    .into_response(400, None, &[("Connection", "close")])
                    .map_err(|e| e.0)?;
                resp.write_all(b"Invalid SSID").map_err(|e| e.0)?;
                return Ok::<(), EspError>(());
            }

            // WPA/WPA2: 8-63 chars, or empty for open networks
            if !password.is_empty() && !(8..=63).contains(&password.len()) {
                let mut resp = req
                    .into_response(400, None, &[("Connection", "close")])
                    .map_err(|e| e.0)?;
                resp.write_all(b"Password must be 8-63 characters")
                    .map_err(|e| e.0)?;
                return Ok::<(), EspError>(());
            }

            crate::nvs_creds::save(&nvs, ssid, password);

            let mut resp = req
                .into_response(200, None, &[("Connection", "close")])
                .map_err(|e| e.0)?;
            resp.write_all(b"OK").map_err(|e| e.0)?;

            crate::reboot_after("Rebooting after WiFi setup");
            Ok::<(), EspError>(())
        })
        .unwrap();

    // Android captive portal detection: expects 204, gets 302 → triggers popup
    server
        .fn_handler("/generate_204", esp_idf_svc::http::Method::Get, |req| {
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
        })
        .unwrap();

    serve_common_assets(&mut server);

    // GET /* — serve portal page (wildcard catches captive portal detection URLs)
    serve_static(
        &mut server,
        "/*",
        "text/html",
        "no-cache",
        include_bytes!(concat!(env!("OUT_DIR"), "/captive_portal.html")),
        true,
    );

    info!("Captive portal started");

    (server, dns_handle)
}
