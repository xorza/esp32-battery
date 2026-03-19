use std::fmt::Write as FmtWrite;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{Configuration as HttpConfig, EspHttpServer};
use esp_idf_svc::nvs::{EspNvs, NvsDefault};
use esp_idf_svc::sys::EspError;
use esp_idf_svc::tls::X509;
use log::info;

use esp32_battery_logic::battery;
use esp32_battery_logic::data::{HISTORY_CAPACITY, Platform, Sample, SensorData};
use esp32_battery_logic::form;

// Worst case per history entry: "[4294967295,-99.999,-99.999,-99.999,1.00]," ≈ 45 chars
// × HISTORY_CAPACITY + ~600 bytes for headers, stats, and metadata
const JSON_BUF_SIZE: usize = 1000 + HISTORY_CAPACITY * 50;
type JsonBuf = heapless::String<JSON_BUF_SIZE>;

const SERVER_CERT: X509<'static> = X509::pem_until_nul(include_bytes!("../certs/selfsigned.crt"));
const SERVER_KEY: X509<'static> = X509::pem_until_nul(include_bytes!("../certs/selfsigned.key"));

fn create_server(
    stack_size: usize,
    wildcard: bool,
    max_sockets: usize,
    so_linger: Option<std::time::Duration>,
    https: bool,
) -> EspHttpServer<'static> {
    use esp_idf_svc::http::server::KeepAlive;

    let (cert, key) = if https {
        (Some(SERVER_CERT), Some(SERVER_KEY))
    } else {
        (None, None)
    };

    EspHttpServer::new(&HttpConfig {
        stack_size,
        max_open_sockets: max_sockets,
        uri_match_wildcard: wildcard,
        session_timeout: std::time::Duration::from_secs(2),
        lru_purge_enable: true,
        keep_alive: Some(KeepAlive {
            idle_secs: 3,
            interval_secs: 3,
            probe_count: 2,
        }),
        so_linger,
        server_certificate: cert,
        private_key: key,
        ..Default::default()
    })
    .unwrap()
}

pub fn serve_static(
    server: &mut EspHttpServer<'static>,
    path: &str,
    content_type: &'static str,
    cache: &'static str,
    body: &'static [u8],
    gzipped: bool,
) {
    server
        .fn_handler(path, esp_idf_svc::http::Method::Get, move |req| {
            let headers: &[(&str, &str)] = if gzipped {
                &[
                    ("Content-Type", content_type),
                    ("Content-Encoding", "gzip"),
                    ("Cache-Control", cache),
                    ("Connection", "close"),
                ]
            } else {
                &[
                    ("Content-Type", content_type),
                    ("Cache-Control", cache),
                    ("Connection", "close"),
                ]
            };
            let mut resp = req.into_response(200, None, headers).map_err(|e| e.0)?;
            resp.write_all(body).map_err(|e| e.0)?;
            Ok::<(), EspError>(())
        })
        .unwrap();
}

fn serve_common_assets(server: &mut EspHttpServer<'static>) {
    serve_static(
        server,
        "/style.css",
        "text/css",
        "max-age=3600",
        include_bytes!(concat!(env!("OUT_DIR"), "/style.css")),
        true,
    );
    serve_static(
        server,
        "/favicon.ico",
        "image/x-icon",
        "max-age=86400",
        include_bytes!("favicon.ico"),
        false,
    );
}

fn get_rssi() -> i32 {
    let mut ap_info: esp_idf_svc::sys::wifi_ap_record_t = unsafe { std::mem::zeroed() };
    if unsafe { esp_idf_svc::sys::esp_wifi_sta_get_ap_info(&mut ap_info) } == 0 {
        ap_info.rssi as i32
    } else {
        0
    }
}

// --- Main battery monitor server ---

fn write_history(json: &mut impl FmtWrite, key: &str, points: &[Sample]) {
    write!(json, r#","{}":["#, key).unwrap();
    for (i, p) in points.iter().enumerate() {
        if i > 0 {
            write!(json, ",").unwrap();
        }
        write!(
            json,
            "[{},{:.3},{:.3},{:.3},{:.2}]",
            p.time_s, p.voltage, p.current_1, p.current_2, p.power_online
        )
        .unwrap();
    }
    write!(json, "]").unwrap();
}

/// Parse a query parameter value from a URI query string.
fn query_param<'a>(uri: &'a str, key: &str) -> Option<&'a str> {
    let q = uri.split_once('?')?.1;
    for pair in q.split('&') {
        if let Some(val) = pair.strip_prefix(key).and_then(|s| s.strip_prefix('=')) {
            return Some(val);
        }
    }
    None
}

pub fn start_main<P: Platform + Send + 'static>(
    sensor_data: Arc<Mutex<SensorData<P>>>,
    nvs: Arc<EspNvs<NvsDefault>>,
) -> EspHttpServer<'static> {
    let mut server = create_server(10240, false, 4, Some(Duration::from_secs(0)), true);

    serve_common_assets(&mut server);
    serve_static(
        &mut server,
        "/",
        "text/html",
        "max-age=3600",
        include_bytes!(concat!(env!("OUT_DIR"), "/index.html")),
        true,
    );

    let json_buf = Mutex::new(Box::new(JsonBuf::new()));

    server
        .fn_handler("/api", esp_idf_svc::http::Method::Get, move |req| {
            let uri = req.uri().to_owned();
            let client_since: Option<u32> = query_param(&uri, "since").and_then(|s| s.parse().ok());
            let client_interval: Option<u32> =
                query_param(&uri, "interval").and_then(|s| s.parse().ok());

            let mut guard = json_buf.lock().unwrap();
            let json = &mut **guard;
            json.clear();

            let store = sensor_data.lock().unwrap();
            let s1 = store.last_reading_1;
            let s2 = store.last_reading_2;
            let history = store.history();
            let interval = store.interval();

            let voltage = (s1.voltage + s2.voltage) / 2.0;
            let max_charge = history
                .iter()
                .map(|s| s.max_charge)
                .fold(0.0_f64, f64::max);
            write!(
                json,
                r#"{{"uptime":{},"rssi":{},"voltage":{:.3},"interval":{},"read_err":[{},{}],"charge":{:.6},"max_charge":{:.6},"power_online":{:.2}"#,
                crate::uptime_s(),
                get_rssi(),
                voltage,
                interval,
                store.read_failures,
                store.read_total,
                s1.charge,
                max_charge,
                store.power_online,
            )
            .unwrap();

            // s1, s2
            write!(
                json,
                r#","s1":{{"soc":{:.1},"current":{:.3},"power":{:.3}}}"#,
                battery::ocv_soc(s1.voltage),
                s1.current,
                s1.power,
            )
            .unwrap();
            write!(
                json,
                r#","s2":{{"current":{:.3},"power":{:.3}}}"#,
                s2.current,
                s2.power,
            )
            .unwrap();

            // Send incremental history if client has matching interval and a valid `since` timestamp,
            // otherwise send full history.
            let incremental = client_interval == Some(interval)
                && client_since.is_some_and(|since| {
                    history.first().is_some_and(|first| since >= first.time_s)
                });

            if incremental {
                let since = client_since.unwrap();
                let start = history
                    .iter()
                    .position(|s| s.time_s > since)
                    .unwrap_or(history.len());
                write_history(json, "history_append", &history[start..]);
            } else {
                write_history(json, "history", history);
            }
            write!(json, "}}").unwrap(); // close root

            let heap = unsafe { esp_idf_svc::sys::esp_get_free_heap_size() };
            let peak = unsafe { esp_idf_svc::sys::esp_get_minimum_free_heap_size() };
            info!(
                "API: history={} json={}/{} heap={heap} peak={peak}",
                history.len(),
                json.len(),
                JSON_BUF_SIZE,
            );
            drop(store);

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
            resp.write_all(json.as_bytes()).map_err(|e| e.0)?;
            Ok::<(), EspError>(())
        })
        .unwrap();

    server
        .fn_handler("/wifi-reset", esp_idf_svc::http::Method::Post, move |req| {
            crate::nvs_creds::clear(&nvs);

            let mut resp = req
                .into_response(
                    200,
                    None,
                    &[("Content-Type", "text/plain"), ("Connection", "close")],
                )
                .map_err(|e| e.0)?;
            resp.write_all(b"WiFi credentials cleared. Rebooting...")
                .map_err(|e| e.0)?;

            crate::reboot_after("Rebooting after WiFi reset");
            Ok::<(), EspError>(())
        })
        .unwrap();

    serve_static(
        &mut server,
        "/ota",
        "text/html",
        "no-cache",
        include_bytes!(concat!(env!("OUT_DIR"), "/ota.html")),
        true,
    );

    crate::ota::register(&mut server);

    server
}

// --- Captive portal server ---

pub fn start_captive(
    nvs: Arc<EspNvs<NvsDefault>>,
    wifi: Arc<Mutex<crate::wifi::Wifi<'static>>>,
) -> (EspHttpServer<'static>, crate::dns::DnsHandle) {
    let dns_handle = crate::dns::DnsHandle::start();

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
