use std::fs;
use std::io::Write;
use std::path::Path;

use flate2::Compression;
use flate2::write::GzEncoder;

const WEB_ASSETS: &[&str] = &[
    "src/index.html",
    "src/ota.html",
    "src/captive_portal.html",
    "src/style.css",
];

fn gzip_web_assets() {
    let out_dir = std::env::var("OUT_DIR").unwrap();

    for asset in WEB_ASSETS {
        println!("cargo:rerun-if-changed={asset}");

        let input = fs::read(asset).unwrap();

        let mut encoder = GzEncoder::new(Vec::new(), Compression::best());
        encoder.write_all(&input).unwrap();
        let compressed = encoder.finish().unwrap();

        let out_path = Path::new(&out_dir).join(Path::new(asset).file_name().unwrap());
        fs::write(&out_path, &compressed).unwrap();

        println!(
            "cargo:warning=Gzipped {asset}: {} -> {} bytes ({:.0}%)",
            input.len(),
            compressed.len(),
            (1.0 - compressed.len() as f64 / input.len() as f64) * 100.0,
        );
    }
}

/// Propagate `OTA_KEY` (hex string) into rustc's env so `env!("OTA_KEY")`
/// works at compile time. Prefers a real env var, falls back to a `.env`
/// file in the crate root.
fn propagate_ota_key() {
    let env_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(".env");
    println!("cargo:rerun-if-changed={}", env_path.display());
    println!("cargo:rerun-if-env-changed=OTA_KEY");

    let hex = std::env::var("OTA_KEY").ok().or_else(|| {
        fs::read_to_string(&env_path).ok()?.lines().find_map(|line| {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                return None;
            }
            let (k, v) = line.split_once('=')?;
            (k.trim() == "OTA_KEY")
                .then(|| v.trim().trim_matches(|c| c == '"' || c == '\'').to_string())
        })
    });
    if let Some(hex) = hex {
        println!("cargo:rustc-env=OTA_KEY={hex}");
    }
}

fn main() {
    embuild::espidf::sysenv::output();
    gzip_web_assets();
    propagate_ota_key();
}
