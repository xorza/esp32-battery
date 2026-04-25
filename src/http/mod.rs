//! HTTP servers. Two flavors:
//! - `main_server`: HTTPS on 443, serves the dashboard once WiFi is connected.
//! - `captive`: plaintext HTTP on 80, serves WiFi setup when WiFi is down.

mod captive;
mod main_server;

use std::time::Duration;

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{
    Configuration as HttpConfig, EspHttpConnection, EspHttpServer, Request,
};
use esp_idf_svc::sys::EspError;
use esp_idf_svc::tls::X509;
use log::warn;

pub use captive::start as start_captive;
pub use main_server::start as start_main;

const SERVER_CERT: X509<'static> =
    X509::pem_until_nul(include_bytes!("../../certs/selfsigned.crt"));
const SERVER_KEY: X509<'static> = X509::pem_until_nul(include_bytes!("../../certs/selfsigned.key"));

pub(crate) fn create_server(
    stack_size: usize,
    wildcard: bool,
    max_sockets: usize,
    so_linger: Option<Duration>,
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
        session_timeout: Duration::from_secs(2),
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

pub(crate) fn serve_static(
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

pub(crate) fn serve_common_assets(server: &mut EspHttpServer<'static>) {
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
        include_bytes!("../favicon.ico"),
        false,
    );
}

/// Send a `Connection: close` plaintext response with the given status and body.
pub(crate) fn text_response(
    req: Request<&mut EspHttpConnection>,
    status: u16,
    body: &[u8],
) -> Result<(), EspError> {
    let mut resp = req
        .into_response(status, None, &[("Connection", "close")])
        .map_err(|e| e.0)?;
    resp.write_all(body).map_err(|e| e.0)?;
    Ok(())
}

/// Serialize a value into `buf` and write it as `application/json`. The
/// serialize step runs inside `build`, so the caller can hold a data lock
/// across only the serialization (the lock drops when `build` returns) and
/// not across the network write. On serialization failure (typically buffer
/// too small) we return 500 with a plain-text body — matches what
/// hand-rolled handlers were doing.
pub(crate) fn json_response<F, E>(
    req: Request<&mut EspHttpConnection>,
    buf: &mut [u8],
    build: F,
) -> Result<(), EspError>
where
    F: FnOnce(&mut [u8]) -> Result<usize, E>,
    E: core::fmt::Debug,
{
    let len = match build(buf) {
        Ok(n) => n,
        Err(e) => {
            warn!("JSON serialization failed: {:?}", e);
            return text_response(req, 500, b"serialization error");
        }
    };
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
    Ok(())
}
