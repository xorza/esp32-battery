//! HTTP servers. Two flavors:
//! - `main_server`: HTTPS on 443, serves the dashboard once WiFi is connected.
//! - `captive`: plaintext HTTP on 80, serves WiFi setup when WiFi is down.

mod captive;
mod main_server;

use std::sync::Mutex;
use std::time::Duration;

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{
    Configuration as HttpConfig, EspHttpConnection, EspHttpServer, Method, Request,
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
        max_uri_handlers: 16,
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
    path: &'static str,
    content_type: &'static str,
    cache: &'static str,
    body: &'static [u8],
    gzipped: bool,
) {
    mount_get(server, path, move |req| {
        let mut headers: heapless::Vec<(&str, &str), 4> = heapless::Vec::new();
        headers.push(("Content-Type", content_type)).unwrap();
        if gzipped {
            headers.push(("Content-Encoding", "gzip")).unwrap();
        }
        headers.push(("Cache-Control", cache)).unwrap();
        headers.push(("Connection", "close")).unwrap();
        let mut resp = req.into_response(200, None, &headers).map_err(|e| e.0)?;
        resp.write_all(body).map_err(|e| e.0)?;
        Ok::<(), EspError>(())
    });
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
        include_bytes!("../../assets/favicon.ico"),
        false,
    );
}

/// Read exactly `buf.len()` bytes from the request body. Errors on EOF
/// before the buffer fills (`short`) or on any underlying read error
/// (`err_msg`). Used by handlers that expect a fixed-size prefix (OTA
/// HMAC tag, etc.).
pub(crate) fn read_exact(
    req: &mut Request<&mut EspHttpConnection>,
    buf: &mut [u8],
    err_msg: &'static str,
    short: &'static str,
) -> Result<(), &'static str> {
    let mut filled = 0;
    while filled < buf.len() {
        let n = req.read(&mut buf[filled..]).map_err(|e| {
            log::warn!("read_exact: {:?}", e);
            err_msg
        })?;
        if n == 0 {
            return Err(short);
        }
        filled += n;
    }
    Ok(())
}

/// Read up to `buf.len()` bytes from the request body, returning the
/// number actually read. Treats short reads (EOF) as success — for
/// handlers parsing variable-size form bodies into a bounded buffer.
pub(crate) fn read_to_buf(
    req: &mut Request<&mut EspHttpConnection>,
    buf: &mut [u8],
) -> Result<usize, EspError> {
    let mut filled = 0;
    while filled < buf.len() {
        let n = req.read(&mut buf[filled..]).map_err(|e| e.0)?;
        if n == 0 {
            break;
        }
        filled += n;
    }
    Ok(filled)
}

/// JSON response with a caller-chosen status and pre-serialized body.
/// For the common 200 path with streaming serialization use
/// `json_response`; for the canonical `{"ok":true}` / `{"error":...}`
/// envelopes use `json_ok` / `json_err`.
pub(crate) fn json_reply(
    req: Request<&mut EspHttpConnection>,
    status: u16,
    body: &[u8],
) -> Result<(), EspError> {
    let mut resp = req
        .into_response(
            status,
            None,
            &[
                ("Content-Type", "application/json"),
                ("Connection", "close"),
            ],
        )
        .map_err(|e| e.0)?;
    resp.write_all(body).map_err(|e| e.0)?;
    Ok(())
}

/// Canonical 200 success envelope: `{"ok":true}`. All action endpoints
/// (POSTs that do something rather than return data) use this so the
/// frontend can read a single shape.
pub(crate) fn json_ok(req: Request<&mut EspHttpConnection>) -> Result<(), EspError> {
    json_reply(req, 200, br#"{"ok":true}"#)
}

/// Canonical error envelope: `{"error":"<msg>"}`. `msg` is JSON-escaped
/// by the serializer; sized at 192 B so any reasonable status string
/// fits — overrun panics, which is correct since these are
/// developer-controlled static strings.
pub(crate) fn json_err(
    req: Request<&mut EspHttpConnection>,
    status: u16,
    msg: &str,
) -> Result<(), EspError> {
    #[derive(serde::Serialize)]
    struct E<'a> {
        error: &'a str,
    }
    let mut buf = [0u8; 192];
    let len = serde_json_core::to_slice(&E { error: msg }, &mut buf)
        .expect("json_err: msg too long for 192-byte buffer");
    json_reply(req, status, &buf[..len])
}

/// Serialize a value into `buf` and write it as `application/json`. The
/// serialize step runs inside `build` so the caller can hold a data lock
/// across only the serialization (the lock drops when `build` returns) and
/// not across the network write. On serialization failure (typically buffer
/// too small) the canonical error envelope goes back with status 500.
pub(crate) fn json_response<F, E>(
    req: Request<&mut EspHttpConnection>,
    buf: &mut [u8],
    build: F,
) -> Result<(), EspError>
where
    F: FnOnce(&mut [u8]) -> Result<usize, E>,
    E: core::fmt::Debug,
{
    match build(buf) {
        Ok(len) => json_reply(req, 200, &buf[..len]),
        Err(e) => {
            warn!("JSON serialization failed: {:?}", e);
            json_err(req, 500, "serialization error")
        }
    }
}

/// Single 16 KiB BSS-resident scratch buffer shared by all JSON handlers.
/// Sized to the largest consumer (`api::RESPONSE_BUF_SIZE`); smaller handlers
/// just use a prefix. Each `EspHttpServer` runs on a single httpd task so
/// handlers within one server are serial, but the main HTTPS server and the
/// captive HTTP server are separate tasks — the mutex makes the sharing safe
/// either way and is uncontended on the hot path.
const JSON_BUF_SIZE: usize = 16_384;
static JSON_BUF: Mutex<[u8; JSON_BUF_SIZE]> = Mutex::new([0u8; JSON_BUF_SIZE]);

pub(crate) fn with_json_buf<R>(f: impl FnOnce(&mut [u8]) -> R) -> R {
    let mut guard = JSON_BUF.lock().unwrap();
    f(guard.as_mut_slice())
}

/// Register a URI handler, panicking with the URI in the message on failure.
/// Replaces the bare `.unwrap()` on `fn_handler` so a startup failure (URI
/// table full, bad URI string) points at the offending route.
pub(crate) fn mount_uri<E, F>(
    server: &mut EspHttpServer<'static>,
    uri: &'static str,
    method: Method,
    f: F,
) where
    F: for<'r> Fn(Request<&mut EspHttpConnection<'r>>) -> Result<(), E> + Send + 'static,
    E: core::fmt::Debug,
{
    server
        .fn_handler(uri, method, f)
        .unwrap_or_else(|e| panic!("failed to mount handler for {method:?} {uri}: {e:?}"));
}

pub(crate) fn mount_get<E, F>(server: &mut EspHttpServer<'static>, uri: &'static str, f: F)
where
    F: for<'r> Fn(Request<&mut EspHttpConnection<'r>>) -> Result<(), E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_uri(server, uri, Method::Get, f);
}

/// Mount a `GET` route that serializes a JSON response into the shared
/// scratch buffer. `build` runs inside the buffer-mutex critical section,
/// so it can hold the data lock across `to_slice` without leaking it into
/// the network write.
pub(crate) fn mount_json_get<F, E>(server: &mut EspHttpServer<'static>, uri: &'static str, build: F)
where
    F: Fn(&mut [u8]) -> Result<usize, E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_get(server, uri, move |req| {
        with_json_buf(|b| json_response(req, b, |inner| build(inner)))
    });
}

pub(crate) fn mount_post<E, F>(server: &mut EspHttpServer<'static>, uri: &'static str, f: F)
where
    F: for<'r> Fn(Request<&mut EspHttpConnection<'r>>) -> Result<(), E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_uri(server, uri, Method::Post, f);
}
