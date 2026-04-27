//! Thin wrapper over the ESP IDF task watchdog (TWDT).
//!
//! The project ships with `CONFIG_ESP_TASK_WDT_INIT=n` and idle tasks
//! unsubscribed (see `sdkconfig.defaults`) — OTA flash writes block the
//! IDLE task for more than 5 s and would otherwise trip the WDT. `main`
//! calls [`init`] once at boot; safety-critical threads then call
//! [`subscribe`] (returning a [`WdtToken`]) and feed it every loop
//! iteration.
//!
//! [`WdtToken`] is `!Send + !Sync`: the TWDT subscription is bound to
//! the FreeRTOS task that called `esp_task_wdt_add`, so feeding from a
//! different thread would feed the wrong subscription. The marker
//! makes that a compile error.
//!
//! On timeout the WDT panics → core reboots → S_INI=OFF brings the buck
//! up disabled.

use std::marker::PhantomData;
use std::time::Duration;

use esp_idf_svc::sys::{
    esp, esp_task_wdt_add, esp_task_wdt_config_t, esp_task_wdt_delete, esp_task_wdt_init,
    esp_task_wdt_reset,
};

/// Shared timeout for all subscribed tasks. Long enough to ride out
/// the slowest legitimate work item, short enough that a wedged loop
/// reboots within ~10 s.
pub const WDT_TIMEOUT: Duration = Duration::from_secs(10);

/// Initialize the task watchdog. Call exactly once from `main` before
/// any thread calls [`subscribe`].
pub fn init() {
    let cfg = esp_task_wdt_config_t {
        timeout_ms: WDT_TIMEOUT.as_millis() as u32,
        // Bitmask of cores to monitor for idle-task hangs. 0 = none —
        // we only care about subscribed tasks missing their reset.
        idle_core_mask: 0,
        // Trigger panic (→ reboot) on timeout instead of just logging.
        trigger_panic: true,
    };
    esp!(unsafe { esp_task_wdt_init(&cfg) }).expect("WDT init");
}

/// Per-thread WDT subscription handle. `!Send + !Sync` so the type
/// system enforces that [`reset`](Self::reset) and `Drop` run on the
/// same FreeRTOS task that called [`subscribe`].
pub struct WdtToken {
    _not_send_sync: PhantomData<*const ()>,
}

/// Subscribe the **current** task to the watchdog. Must be called from
/// inside the thread to be monitored, after [`init`]. Drop the returned
/// token to unsubscribe.
pub fn subscribe() -> WdtToken {
    esp!(unsafe { esp_task_wdt_add(std::ptr::null_mut()) }).expect("WDT subscribe current task");
    WdtToken {
        _not_send_sync: PhantomData,
    }
}

impl WdtToken {
    /// Feed the watchdog. Must be called at least once per [`WDT_TIMEOUT`].
    pub fn reset(&self) {
        unsafe {
            esp_task_wdt_reset();
        }
    }
}

impl Drop for WdtToken {
    fn drop(&mut self) {
        esp!(unsafe { esp_task_wdt_delete(std::ptr::null_mut()) }).expect("WDT unsubscribe");
    }
}
