use std::time::Duration;

use esp_idf_svc::http::server::{EspHttpConnection, EspHttpServer};
use esp_idf_svc::ota::EspOta;
use hmac::{Hmac, Mac, digest::KeyInit};
use log::{info, warn};
use serde::Serialize;
use sha2::Sha256;

use crate::clock::uptime;
use crate::http::{json_reply, mount_post};

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

const OTA_KEY_HEX: &str = env!("OTA_KEY");

type HmacSha256 = Hmac<Sha256>;

fn decode_key() -> [u8; 32] {
    let b = OTA_KEY_HEX.as_bytes();
    assert_eq!(b.len(), 64, "OTA_KEY must be 64 hex chars");
    let mut out = [0u8; 32];
    for i in 0..32 {
        out[i] = u8::from_str_radix(&OTA_KEY_HEX[i * 2..i * 2 + 2], 16)
            .expect("OTA_KEY must be valid hex");
    }
    out
}

fn handle_upload(
    req: &mut esp_idf_svc::http::server::Request<&mut EspHttpConnection>,
) -> Result<usize, &'static str> {
    let mut expected_hmac = [0u8; 32];
    crate::http::read_exact(
        req,
        &mut expected_hmac,
        "failed to read HMAC",
        "missing HMAC signature",
    )?;

    let mut mac = HmacSha256::new_from_slice(&decode_key()).expect("HMAC key length must be valid");

    let mut ota = EspOta::new().map_err(|e| {
        warn!("OTA: init failed: {:?}", e);
        "OTA init failed"
    })?;
    let mut update = ota.initiate_update().map_err(|e| {
        warn!("OTA: initiate_update failed: {:?}", e);
        "OTA initiate_update failed"
    })?;

    let deadline = uptime() + UPLOAD_TIMEOUT;
    let mut buf = [0u8; 4096];
    let mut total: usize = 0;
    let outcome = loop {
        if uptime() >= deadline {
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

pub fn mount(server: &mut EspHttpServer<'static>) {
    mount_post(server, "/ota/upload", |mut req| {
        let mut buf = [0u8; 128];
        match handle_upload(&mut req) {
            Ok(total) => {
                info!("OTA: received {} bytes, signature valid", total);
                let len = serde_json_core::to_slice(&OkResponse { ok: true }, &mut buf).unwrap();
                let _ = json_reply(req, 200, &buf[..len]);
                crate::reboot::reboot_after("OTA: rebooting now");
            }
            Err(msg) => {
                warn!("OTA: {}", msg);
                let len =
                    serde_json_core::to_slice(&ErrorResponse { error: msg }, &mut buf).unwrap();
                let _ = json_reply(req, 403, &buf[..len]);
            }
        }
        Ok::<(), esp_idf_svc::sys::EspError>(())
    });
}
