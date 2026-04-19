use std::fs;
use std::io::Write;
use std::path::Path;

use flate2::write::GzEncoder;
use flate2::Compression;

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

fn write_ota_key() {
    let out_dir = std::env::var("OUT_DIR").unwrap();
    let env_path = Path::new(env!("CARGO_MANIFEST_DIR")).join(".env");
    println!("cargo:rerun-if-changed={}", env_path.display());
    println!("cargo:rerun-if-env-changed=OTA_KEY");

    let hex = std::env::var("OTA_KEY").ok().or_else(|| {
        let contents = fs::read_to_string(&env_path).ok()?;
        contents.lines().find_map(|line| {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                return None;
            }
            let (k, v) = line.split_once('=')?;
            (k.trim() == "OTA_KEY").then(|| v.trim().trim_matches(|c| c == '"' || c == '\'').to_string())
        })
    }).expect("OTA_KEY not set (env var or .env file)");

    assert_eq!(hex.len(), 64, "OTA_KEY must be 64 hex chars (32 bytes)");
    let mut bytes = [0u8; 32];
    for i in 0..32 {
        bytes[i] = u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16)
            .expect("OTA_KEY must be valid hex");
    }

    let out_path = Path::new(&out_dir).join("ota_key.bin");
    fs::write(&out_path, bytes).unwrap();
}

fn main() {
    embuild::espidf::sysenv::output();
    gzip_web_assets();
    write_ota_key();
}
