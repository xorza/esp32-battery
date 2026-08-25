use std::sync::LazyLock;
use std::time::Duration;

use esp_idf_svc::http::server::{EspHttpConnection, EspHttpServer, Request};
use esp_idf_svc::ota::EspOta;
use hmac::{Hmac, Mac, digest::KeyInit};
use log::{info, warn};
use sha2::Sha256;

use crate::clock::uptime;
use crate::http::{json_err, json_ok, mount_post};

/// Wall-clock timeout for the entire OTA upload. A legitimate 1.5 MB firmware
/// over the local network takes a few seconds; anything approaching this is
/// either a dead link or a slowloris-style DoS.
const UPLOAD_TIMEOUT: Duration = Duration::from_secs(60);

const OTA_KEY_HEX: &str = env!("OTA_KEY");

type HmacSha256 = Hmac<Sha256>;

static OTA_KEY: LazyLock<[u8; 32]> = LazyLock::new(|| {
    let b = OTA_KEY_HEX.as_bytes();
    assert_eq!(b.len(), 64, "OTA_KEY must be 64 hex chars");
    let mut out = [0u8; 32];
    for i in 0..32 {
        out[i] = u8::from_str_radix(&OTA_KEY_HEX[i * 2..i * 2 + 2], 16)
            .expect("OTA_KEY must be valid hex");
    }
    out
});

/// Force-decode `OTA_KEY` at boot so a malformed value panics early
/// instead of deferring until the first upload.
pub fn init() {
    LazyLock::force(&OTA_KEY);
}

/// Read the 32-byte HMAC tag the upload is prefixed with. A body that ends
/// early is a truncated upload, not a read error, so the two get different
/// messages — the client can retry one and not the other.
fn read_hmac_tag(req: &mut Request<&mut EspHttpConnection>) -> Result<[u8; 32], &'static str> {
    let mut tag = [0u8; 32];
    let mut filled = 0;
    while filled < tag.len() {
        let n = req.read(&mut tag[filled..]).map_err(|e| {
            warn!("OTA: HMAC prefix read failed: {e:?}");
            "failed to read HMAC"
        })?;
        if n == 0 {
            return Err("missing HMAC signature");
        }
        filled += n;
    }
    Ok(tag)
}

/// Derive a 32-byte subkey from the build-time OTA key, domain-separated by
/// `domain`. Lets another subsystem hold a per-device secret without a second
/// build-time key to manage, and without the key itself leaving this module.
pub(crate) fn derive_subkey(domain: &str, salt: &[u8]) -> [u8; 32] {
    let mut mac = HmacSha256::new_from_slice(&*OTA_KEY).expect("HMAC key length must be valid");
    mac.update(domain.as_bytes());
    mac.update(salt);
    mac.finalize().into_bytes().into()
}

fn handle_upload(req: &mut Request<&mut EspHttpConnection>) -> Result<usize, &'static str> {
    let expected_hmac = read_hmac_tag(req)?;

    let mut mac = HmacSha256::new_from_slice(&*OTA_KEY).expect("HMAC key length must be valid");

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
        match handle_upload(&mut req) {
            Ok(total) => {
                info!("OTA: received {} bytes, signature valid", total);
                let _ = json_ok(req);
                crate::reboot::reboot_after("OTA: rebooting now");
            }
            Err(msg) => {
                warn!("OTA: {}", msg);
                let _ = json_err(req, 403, msg);
            }
        }
        Ok::<(), esp_idf_svc::sys::EspError>(())
    });
}
