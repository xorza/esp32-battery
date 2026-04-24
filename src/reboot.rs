use std::thread;
use std::time::Duration;

use log::info;

/// Spawn a short-lived thread that logs `msg` and reboots the device after a
/// 2 s grace period — long enough for the caller to flush an HTTP response or
/// log line before the reset.
pub fn reboot_after(msg: &'static str) {
    thread::Builder::new()
        .stack_size(4096)
        .spawn(move || {
            thread::sleep(Duration::from_secs(2));
            info!("{}", msg);
            esp_idf_svc::hal::reset::restart();
        })
        .unwrap();
}
