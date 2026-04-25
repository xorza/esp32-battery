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
        include_bytes!("../favicon.ico"),
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

/// Send a `Connection: close` response with the given status, content-type,
/// and body.
fn body_response(
    req: Request<&mut EspHttpConnection>,
    status: u16,
    content_type: Option<&'static str>,
    body: &[u8],
) -> Result<(), EspError> {
    let mut headers: heapless::Vec<(&'static str, &'static str), 2> = heapless::Vec::new();
    if let Some(ct) = content_type {
        headers.push(("Content-Type", ct)).unwrap();
    }
    headers.push(("Connection", "close")).unwrap();
    let mut resp = req.into_response(status, None, &headers).map_err(|e| e.0)?;
    resp.write_all(body).map_err(|e| e.0)?;
    Ok(())
}

/// Plain-text response (no explicit Content-Type — httpd defaults to text/html
/// which browsers render fine for short status messages).
pub(crate) fn text_response(
    req: Request<&mut EspHttpConnection>,
    status: u16,
    body: &[u8],
) -> Result<(), EspError> {
    body_response(req, status, None, body)
}

/// JSON response with a caller-chosen status — used by handlers that need
/// non-200 outcomes (OTA error replies, etc). For the common 200 path with
/// streaming serialization, use `json_response`.
pub(crate) fn json_reply(
    req: Request<&mut EspHttpConnection>,
    status: u16,
    body: &[u8],
) -> Result<(), EspError> {
    body_response(req, status, Some("application/json"), body)
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
            return json_reply(req, 500, br#"{"error":"serialization error"}"#);
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

/// Heap-allocated, mutex-guarded scratch buffer for JSON handlers. Sized at
/// the type level so each handler picks its own response budget. The buffer
/// is allocated once at handler-mount time and reused across requests, so
/// concurrent calls serialize on the lock — fine because esp-idf's httpd
/// already serializes work to a single task by default.
pub(crate) struct JsonBuf<const N: usize>(Mutex<Box<[u8; N]>>);

impl<const N: usize> JsonBuf<N> {
    pub fn new() -> Self {
        Self(Mutex::new(Box::new([0u8; N])))
    }

    pub fn with<R>(&self, f: impl FnOnce(&mut [u8]) -> R) -> R {
        let mut guard = self.0.lock().unwrap();
        let buf: &mut [u8] = &mut **guard;
        f(buf)
    }
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
        .unwrap_or_else(|e| panic!("failed to mount {} {:?}: {:?}", uri, method, e));
}

pub(crate) fn mount_get<E, F>(server: &mut EspHttpServer<'static>, uri: &'static str, f: F)
where
    F: for<'r> Fn(Request<&mut EspHttpConnection<'r>>) -> Result<(), E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_uri(server, uri, Method::Get, f);
}

/// Mount a `GET` route that owns a reusable JSON scratch buffer and
/// serializes whatever `build` writes into it. Collapses the
/// `mount_get → JsonBuf::with → json_response` triple-closure that
/// every JSON handler used to spell out by hand. `build` runs inside
/// the buffer-mutex critical section, so it can hold the data lock
/// across `to_slice` without leaking it into the network write.
pub(crate) fn mount_json_get<F, E, const N: usize>(
    server: &mut EspHttpServer<'static>,
    uri: &'static str,
    buf: JsonBuf<N>,
    build: F,
) where
    F: Fn(&mut [u8]) -> Result<usize, E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_get(server, uri, move |req| {
        buf.with(|b| json_response(req, b, |inner| build(inner)))
    });
}

pub(crate) fn mount_post<E, F>(server: &mut EspHttpServer<'static>, uri: &'static str, f: F)
where
    F: for<'r> Fn(Request<&mut EspHttpConnection<'r>>) -> Result<(), E> + Send + 'static,
    E: core::fmt::Debug,
{
    mount_uri(server, uri, Method::Post, f);
}
