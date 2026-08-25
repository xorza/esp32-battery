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

/// How one of the two servers is set up. Named because these are otherwise
/// four positional arguments, two of them bare bools, at call sites that read
/// `create_server(10240, false, 3, true)`.
#[derive(Debug)]
pub(crate) struct ServerConfig {
    /// httpd task stack. The HTTPS server needs the larger one — mbedtls
    /// handshakes run on it.
    pub stack_size: usize,
    /// Connections held open before `lru_purge_enable` starts evicting.
    pub max_sockets: usize,
    /// Accept `/*`-style wildcard URIs, which the captive portal's catch-all
    /// page needs and the dashboard does not.
    pub wildcard: bool,
    /// Serve TLS using the built-in self-signed certificate.
    pub https: bool,
}

/// Published `esp-idf-svc` 0.52.1 exposes neither TCP keep-alive nor
/// `SO_LINGER` on the server config, so neither is set here. What covers
/// the gap: `lru_purge_enable` evicts the least-recently-used connection
/// when the pool is full, so a new client always gets in, and the 5 s
/// `recv_wait_timeout` / `send_wait_timeout` the crate hard-codes tear
/// down a stalled socket. The pool counts *open* sockets, so sockets in
/// TIME_WAIT do not consume `max_open_sockets`. The residual loss is
/// slower reclamation of a peer that vanished silently mid-connection.
pub(crate) fn create_server(cfg: ServerConfig) -> EspHttpServer<'static> {
    let (cert, key) = if cfg.https {
        (Some(SERVER_CERT), Some(SERVER_KEY))
    } else {
        (None, None)
    };

    EspHttpServer::new(&HttpConfig {
        stack_size: cfg.stack_size,
        max_open_sockets: cfg.max_sockets,
        max_uri_handlers: 16,
        uri_match_wildcard: cfg.wildcard,
        session_timeout: Duration::from_secs(2),
        lru_purge_enable: true,
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
    // Built once here, not per request: nothing in it depends on the request,
    // and the four `unwrap()`s are startup failures rather than serving ones.
    let mut headers: heapless::Vec<(&'static str, &'static str), 4> = heapless::Vec::new();
    headers.push(("Content-Type", content_type)).unwrap();
    if gzipped {
        headers.push(("Content-Encoding", "gzip")).unwrap();
    }
    headers.push(("Cache-Control", cache)).unwrap();
    headers.push(("Connection", "close")).unwrap();

    mount_get(server, path, move |req| {
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

/// Read the request body into `buf`. Returns `Ok(Some(n))` with the
/// number of bytes actually read, or `Ok(None)` if the body did not fit
/// — the caller should reject with 413. One extra probe read past the
/// buffer end is what distinguishes "exact fit" from "truncated".
pub(crate) fn read_to_buf(
    req: &mut Request<&mut EspHttpConnection>,
    buf: &mut [u8],
) -> Result<Option<usize>, EspError> {
    let mut filled = 0;
    while filled < buf.len() {
        let n = req.read(&mut buf[filled..]).map_err(|e| e.0)?;
        if n == 0 {
            return Ok(Some(filled));
        }
        filled += n;
    }
    let mut probe = [0u8; 1];
    match req.read(&mut probe).map_err(|e| e.0)? {
        0 => Ok(Some(filled)),
        _ => Ok(None),
    }
}

/// JSON response with a caller-chosen status and pre-serialized body. For a
/// 200 built by serializing into the shared buffer use `mount_json_get`; for
/// the canonical `{"ok":true}` / `{"error":...}` envelopes use `json_ok` /
/// `json_err`.
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

/// Canonical error envelope: `{"error":"<msg>"}`. `msg` is JSON-escaped by
/// the serializer; 192 B fits any message this firmware sends. An overrun
/// degrades to a generic envelope rather than panicking — the status code is
/// what the client acts on, and a panic here would reboot the device (see the
/// hook in `main.rs`) while merely reporting some earlier error.
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
    match serde_json_core::to_slice(&E { error: msg }, &mut buf) {
        Ok(len) => json_reply(req, status, &buf[..len]),
        Err(_) => {
            warn!("json_err: {status} message did not fit its buffer: {msg}");
            json_reply(req, status, br#"{"error":"request failed"}"#)
        }
    }
}

/// Single 16 KiB BSS-resident scratch buffer shared by all JSON handlers.
/// Sized to the largest consumer — `/api`, whose history array dominates the
/// payload; smaller handlers just use a prefix. Each `EspHttpServer` runs on a
/// single httpd task so handlers within one server are serial, but the main
/// HTTPS server and the captive HTTP server are separate tasks — the mutex
/// makes the sharing safe either way and is uncontended on the hot path.
const JSON_BUF_SIZE: usize = 16_384;
static JSON_BUF: Mutex<[u8; JSON_BUF_SIZE]> = Mutex::new([0u8; JSON_BUF_SIZE]);

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
/// scratch buffer. `build` runs inside the buffer-mutex critical section, so
/// it can hold a data lock across `to_slice` — that lock drops when `build`
/// returns, before the network write. A serialization failure (typically the
/// buffer being too small) answers with the canonical error envelope and 500.
pub(crate) fn mount_json_get<F, E>(server: &mut EspHttpServer<'static>, uri: &'static str, build: F)
where
    F: Fn(&mut [u8]) -> Result<usize, E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_get(server, uri, move |req| {
        let mut guard = JSON_BUF.lock().unwrap();
        match build(guard.as_mut_slice()) {
            Ok(len) => json_reply(req, 200, &guard[..len]),
            Err(e) => {
                warn!("{uri}: JSON serialization failed: {e:?}");
                json_err(req, 500, "serialization error")
            }
        }
    });
}

pub(crate) fn mount_post<E, F>(server: &mut EspHttpServer<'static>, uri: &'static str, f: F)
where
    F: for<'r> Fn(Request<&mut EspHttpConnection<'r>>) -> Result<(), E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_uri(server, uri, Method::Post, f);
}
