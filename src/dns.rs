use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::Duration;

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
            let mut buf = [0u8; 512];

            while !stop.load(Ordering::Relaxed) {
                let (len, src) = match socket.recv_from(&mut buf) {
                    Ok(r) => r,
                    Err(_) => continue,
                };
                if len < 12 {
                    continue;
                }

                // Parse question section to find qtype
                let mut q_end = 12;
                while q_end < len && buf[q_end] != 0 {
                    let label_len = buf[q_end] as usize + 1;
                    if q_end + label_len >= len {
                        break;
                    }
                    q_end += label_len;
                }
                if q_end >= len || buf[q_end] != 0 {
                    continue; // malformed: label ran past end of packet
                }
                q_end += 1; // null byte
                if q_end + 4 > len {
                    continue;
                }
                let qtype = u16::from_be_bytes([buf[q_end], buf[q_end + 1]]);
                q_end += 4; // qtype(2) + qclass(2)

                let is_a_query = qtype == 1;

                // Build DNS response
                let mut resp = [0u8; 512];
                resp[0] = buf[0]; // transaction ID
                resp[1] = buf[1];
                resp[2] = 0x84 | (buf[2] & 0x01); // QR=1, AA=1, copy RD from query
                resp[3] = 0x00;
                let mut pos = 12;

                // Copy question section
                resp[pos..pos + (q_end - 12)].copy_from_slice(&buf[12..q_end]);
                pos += q_end - 12;

                if is_a_query {
                    // 1 question, 1 answer
                    resp[4..12].copy_from_slice(&[0, 1, 0, 1, 0, 0, 0, 0]);
                    // Answer: pointer to question name, type A, class IN, TTL 0, 4-byte IP
                    resp[pos..pos + 12].copy_from_slice(&[
                        0xC0, 0x0C, // name pointer
                        0, 1, // type A
                        0, 1, // class IN
                        0, 0, 0, 0, // TTL 0
                        0, 4, // RDLENGTH
                    ]);
                    resp[pos + 12..pos + 16].copy_from_slice(&crate::wifi::AP_GATEWAY);
                    pos += 16;
                } else {
                    // AAAA and others: 1 question, 0 answers (immediate negative response)
                    resp[4..12].copy_from_slice(&[0, 1, 0, 0, 0, 0, 0, 0]);
                }

                let _ = socket.send_to(&resp[..pos], src);
            }
        })
        .unwrap()
}
