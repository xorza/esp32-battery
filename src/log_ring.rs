//! In-RAM ring buffer that captures every ESP-IDF log line (native C logs and
//! Rust `log::` calls both route through the same vprintf hook). Exposed via
//! `GET /api/log` so we can triage remotely after OTA deploys.
//!
//! The hook formats into a stack buffer, appends the bytes to the ring, and
//! re-emits the formatted text on UART so local serial monitoring still works.
//! The original vprintf isn't re-called — that would require `va_copy`, which
//! Rust doesn't expose portably.
//!
//! The ring uses a `Mutex`, not a spin-lock, so any log call that races with a
//! snapshot just drops a line rather than blocking. Not ISR-safe; ESP-IDF
//! logging is only called from tasks.
//!
//! Default buffer is 16 KiB — about 150 lines of typical ESP-IDF output.

use std::ffi::c_char;
use std::sync::Mutex;
use std::sync::atomic::{AtomicPtr, Ordering};

use esp_idf_svc::sys;

unsafe extern "C" {
    fn vsnprintf(s: *mut c_char, n: usize, fmt: *const c_char, args: sys::va_list) -> i32;
}

const BUF_SIZE: usize = 16 * 1024;
/// Per-call stack buffer. Shipped ESP-IDF log lines top out around 200 bytes;
/// 512 covers the long ones while staying well under the smallest default
/// task stacks (≈2–4 KiB).
const LINE_BUF_SIZE: usize = 512;

struct Ring {
    data: Box<[u8; BUF_SIZE]>,
    /// Next write index.
    head: usize,
    /// True once the ring has wrapped; before that, valid data is `[0..head]`.
    wrapped: bool,
}

impl Ring {
    fn new() -> Self {
        Self {
            data: Box::new([0u8; BUF_SIZE]),
            head: 0,
            wrapped: false,
        }
    }

    fn write(&mut self, mut bytes: &[u8]) {
        // Oversized line: keep only the most recent BUF_SIZE bytes so wrapping
        // lands at a clean final position.
        if bytes.len() > BUF_SIZE {
            bytes = &bytes[bytes.len() - BUF_SIZE..];
            self.head = 0;
            self.wrapped = true;
        }
        let tail = BUF_SIZE - self.head;
        if bytes.len() <= tail {
            self.data[self.head..self.head + bytes.len()].copy_from_slice(bytes);
            self.head += bytes.len();
            if self.head == BUF_SIZE {
                self.head = 0;
                self.wrapped = true;
            }
        } else {
            let (first, rest) = bytes.split_at(tail);
            self.data[self.head..].copy_from_slice(first);
            self.data[..rest.len()].copy_from_slice(rest);
            self.head = rest.len();
            self.wrapped = true;
        }
    }

    fn snapshot(&self) -> Vec<u8> {
        if self.wrapped {
            let mut out = Vec::with_capacity(BUF_SIZE);
            out.extend_from_slice(&self.data[self.head..]);
            out.extend_from_slice(&self.data[..self.head]);
            out
        } else {
            self.data[..self.head].to_vec()
        }
    }
}

static RING: Mutex<Option<Ring>> = Mutex::new(None);
/// The vprintf handler that was installed before our hook took over. We don't
/// currently chain to it (we re-emit the formatted line via `printf` so the
/// UART keeps receiving logs), but the docs recommend keeping it around in
/// case we want to restore on shutdown or swap in a different transport.
static PREV_VPRINTF: AtomicPtr<()> = AtomicPtr::new(std::ptr::null_mut());

/// Call once at startup. Installs the vprintf hook.
pub fn init() {
    *RING.lock().unwrap() = Some(Ring::new());
    let prev = unsafe { sys::esp_log_set_vprintf(Some(vprintf_hook)) };
    PREV_VPRINTF.store(prev.map_or(std::ptr::null_mut(), |f| f as *mut ()), Ordering::Relaxed);
}

/// Copy the current ring contents into a fresh Vec (oldest byte first).
pub fn snapshot() -> Vec<u8> {
    RING.lock()
        .ok()
        .and_then(|g| g.as_ref().map(Ring::snapshot))
        .unwrap_or_default()
}

unsafe extern "C" fn vprintf_hook(fmt: *const c_char, args: sys::va_list) -> i32 {
    let mut buf = [0u8; LINE_BUF_SIZE];
    // SAFETY: vsnprintf is the standard libc fn; fmt/args come from ESP-IDF.
    let n = unsafe { vsnprintf(buf.as_mut_ptr() as *mut c_char, buf.len(), fmt, args) };
    if n <= 0 {
        return n;
    }
    let len = (n as usize).min(buf.len() - 1);
    let slice = &buf[..len];

    // Echo to UART so physical serial monitoring keeps working. Use printf with
    // %.*s — we can't re-call the original vprintf since args was consumed.
    unsafe {
        sys::printf(c"%.*s".as_ptr(), len as i32, slice.as_ptr());
    }

    // Best-effort append. If the mutex is contended we drop the line from the
    // ring (UART still got it) rather than block a logging task.
    if let Ok(mut guard) = RING.try_lock()
        && let Some(r) = guard.as_mut()
    {
        r.write(slice);
    }

    n
}
