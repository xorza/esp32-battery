//! ESP-IDF log capture. Hooks `esp_log_set_vprintf` so every native C log line
//! and every Rust `log::` call (which routes through the same vprintf) lands
//! in an in-RAM ring buffer, exposed via `GET /api/log` for remote triage
//! after OTA deploys. Bytes are also re-emitted via `printf` so the UART
//! monitor keeps working.
//!
//! The ring uses a `Mutex`, not a spin-lock, so any log call that races with
//! a snapshot just drops a line rather than blocking. Not ISR-safe; ESP-IDF
//! logging is only called from tasks. The original vprintf isn't re-called —
//! that would require `va_copy`, which Rust doesn't expose portably.

use std::ffi::c_char;
use std::sync::Mutex;
use std::sync::atomic::{AtomicPtr, Ordering};

use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::sys;
use esp_idf_svc::sys::EspError;

use esp32_battery_logic::Ring;

unsafe extern "C" {
    fn vsnprintf(s: *mut c_char, n: usize, fmt: *const c_char, args: sys::va_list) -> i32;
}

/// Ring capacity — about 150 lines of typical ESP-IDF output.
const BUF_SIZE: usize = 16 * 1024;
/// Per-call stack buffer. Shipped ESP-IDF log lines top out around 200 bytes;
/// 512 covers the long ones while staying well under the smallest default
/// task stacks (≈2–4 KiB).
const LINE_BUF_SIZE: usize = 512;

static RING: Mutex<Option<Ring>> = Mutex::new(None);
/// The vprintf handler that was installed before our hook took over. We don't
/// currently chain to it (we re-emit the formatted line via `printf` so the
/// UART keeps receiving logs), but the docs recommend keeping it around in
/// case we want to restore on shutdown or swap in a different transport.
static PREV_VPRINTF: AtomicPtr<()> = AtomicPtr::new(std::ptr::null_mut());

/// Call once at startup. Installs the vprintf hook.
pub fn init() {
    // Backing storage in BSS — Ring borrows it for the program's life.
    static mut RING_BUF: [u8; BUF_SIZE] = [0u8; BUF_SIZE];
    // SAFETY: init() runs once at boot; the static is referenced only
    // through the resulting Ring inside RING (mutex-guarded).
    let ptr = &raw mut RING_BUF;
    let buf: &'static mut [u8] = unsafe { &mut *ptr };
    *RING.lock().unwrap() = Some(Ring::from_buf(buf));
    let prev = unsafe { sys::esp_log_set_vprintf(Some(vprintf_hook)) };
    PREV_VPRINTF.store(
        prev.map_or(std::ptr::null_mut(), |f| f as *mut ()),
        Ordering::Relaxed,
    );
}

pub fn mount(server: &mut EspHttpServer<'static>) {
    crate::http::mount_get(server, "/api/log", |req| {
        let mut resp = req
            .into_response(
                200,
                None,
                &[
                    ("Content-Type", "text/plain; charset=utf-8"),
                    ("Cache-Control", "no-store"),
                    ("Connection", "close"),
                ],
            )
            .map_err(|e| e.0)?;
        // Stream the ring directly to the response — no Vec materialization.
        // Lock is held for the duration of the writes so the ring can't
        // wrap mid-stream and reorder bytes; logging tasks meanwhile use
        // `try_lock` and drop their line if they collide.
        let guard = RING.lock().unwrap();
        if let Some(ring) = guard.as_ref() {
            let (older, newer) = ring.slices();
            resp.write_all(older).map_err(|e| e.0)?;
            resp.write_all(newer).map_err(|e| e.0)?;
        }
        Ok::<(), EspError>(())
    });
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
