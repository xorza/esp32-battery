use std::time::{Duration, Instant};

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, EspHttpServer};
use esp_idf_svc::ota::EspOta;
use esp_idf_svc::sys::EspError;
use hmac::{Hmac, Mac, digest::KeyInit};
use log::{info, warn};
use serde::Serialize;
use sha2::Sha256;

/// Wall-clock timeout for the entire OTA upload. A legitimate 1.5 MB firmware
/// over the local network takes a few seconds; anything approaching this is
/// either a dead link or a slowloris-style DoS.
const UPLOAD_TIMEOUT: Duration = Duration::from_secs(60);

#[derive(Serialize)]
struct OkResponse {
    ok: bool,
}

#[derive(Serialize)]
struct ErrorResponse<'a> {
    error: &'a str,
}

const OTA_KEY: &[u8; 32] = include_bytes!(concat!(env!("OUT_DIR"), "/ota_key.bin"));

type HmacSha256 = Hmac<Sha256>;

fn reply(
    req: esp_idf_svc::http::server::Request<&mut EspHttpConnection>,
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

fn read_exact(
    req: &mut esp_idf_svc::http::server::Request<&mut EspHttpConnection>,
    buf: &mut [u8],
) -> Result<(), &'static str> {
    let mut filled = 0;
    while filled < buf.len() {
        let n = req.read(&mut buf[filled..]).map_err(|e| {
            warn!("OTA: read error: {:?}", e);
            "failed to read HMAC"
        })?;
        if n == 0 {
            return Err("missing HMAC signature");
        }
        filled += n;
    }
    Ok(())
}

fn handle_upload(
    req: &mut esp_idf_svc::http::server::Request<&mut EspHttpConnection>,
) -> Result<usize, &'static str> {
    let mut expected_hmac = [0u8; 32];
    read_exact(req, &mut expected_hmac)?;

    let mut mac = HmacSha256::new_from_slice(OTA_KEY).expect("HMAC key length must be valid");

    let mut ota = EspOta::new().map_err(|e| {
        warn!("OTA: init failed: {:?}", e);
        "OTA init failed"
    })?;
    let mut update = ota.initiate_update().map_err(|e| {
        warn!("OTA: initiate_update failed: {:?}", e);
        "OTA initiate_update failed"
    })?;

    let deadline = Instant::now() + UPLOAD_TIMEOUT;
    let mut buf = [0u8; 4096];
    let mut total: usize = 0;
    let outcome = loop {
        if Instant::now() >= deadline {
            warn!("OTA: upload timed out after {} bytes", total);
            break Err("upload timed out");
        }
        let n = match req.read(&mut buf) {
            Ok(n) => n,
            Err(e) => {
                warn!("OTA: read error: {:?}", e);
                break Err("failed to read firmware data");
            }
        };
        if n == 0 {
            break Ok(());
        }
        mac.update(&buf[..n]);
        if let Err(e) = update.write(&buf[..n]) {
            warn!("OTA: write failed at {} bytes: {:?}", total, e);
            break Err("invalid firmware image");
        }
        total += n;
    };
    if let Err(msg) = outcome {
        let _ = update.abort();
        return Err(msg);
    }

    if mac.verify_slice(&expected_hmac).is_err() {
        let _ = update.abort();
        return Err("HMAC verification failed");
    }

    update.complete().map_err(|e| {
        warn!("OTA: complete failed: {:?}", e);
        "firmware validation failed"
    })?;

    Ok(total)
}

pub fn register(server: &mut EspHttpServer<'static>) {
    server
        .fn_handler("/ota/upload", esp_idf_svc::http::Method::Post, |mut req| {
            let mut buf = [0u8; 128];
            match handle_upload(&mut req) {
                Ok(total) => {
                    info!("OTA: received {} bytes, signature valid", total);
                    let len = serde_json_core::to_slice(&OkResponse { ok: true }, &mut buf).unwrap();
                    let _ = reply(req, 200, &buf[..len]);
                    crate::reboot_after("OTA: rebooting now");
                }
                Err(msg) => {
                    warn!("OTA: {}", msg);
                    let len =
                        serde_json_core::to_slice(&ErrorResponse { error: msg }, &mut buf).unwrap();
                    let _ = reply(req, 403, &buf[..len]);
                }
            }
            Ok::<(), esp_idf_svc::sys::EspError>(())
        })
        .unwrap();
}
