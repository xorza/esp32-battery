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

fn main() {
    embuild::espidf::sysenv::output();
    gzip_web_assets();
}
