use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

use esp32_battery_logic::dns_packet::build_response;

pub struct DnsHandle {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl DnsHandle {
    pub fn start() -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let thread = start_responder(stop.clone());
        Self {
            stop,
            thread: Some(thread),
        }
    }
}

impl Drop for DnsHandle {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn start_responder(stop: Arc<AtomicBool>) -> JoinHandle<()> {
    thread::Builder::new()
        .name("dns".into())
        .stack_size(4096)
        .spawn(move || {
            let socket = UdpSocket::bind("0.0.0.0:53").expect("DNS bind failed");
            socket
                .set_read_timeout(Some(Duration::from_secs(1)))
                .unwrap();
            let mut req = [0u8; 512];
            let mut resp = [0u8; 512];

            while !stop.load(Ordering::Relaxed) {
                let (len, src) = match socket.recv_from(&mut req) {
                    Ok(r) => r,
                    Err(_) => continue,
                };
                if let Some(n) = build_response(&req[..len], crate::wifi::AP_GATEWAY, &mut resp) {
                    let _ = socket.send_to(&resp[..n], src);
                }
            }
        })
        .unwrap()
}
