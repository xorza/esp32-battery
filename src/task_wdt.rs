//! Thin wrapper over the ESP IDF task watchdog (TWDT).
//!
//! The project ships with `CONFIG_ESP_TASK_WDT_INIT=n` and idle tasks
//! unsubscribed (see `sdkconfig.defaults`) — OTA flash writes block the
//! IDLE task for more than 5 s and would otherwise trip the WDT. Only
//! safety-critical tasks (currently just the xy supervisor) opt in by
//! calling [`init_and_subscribe`] once and [`reset`] every loop iteration.
//!
//! On timeout the WDT panics → core reboots → S_INI=OFF brings the buck
//! up disabled.

use std::time::Duration;

use esp_idf_svc::sys::{
    ESP_OK, esp, esp_task_wdt_add, esp_task_wdt_config_t, esp_task_wdt_delete, esp_task_wdt_init,
    esp_task_wdt_reconfigure, esp_task_wdt_reset,
};

/// Initialize the task watchdog with `timeout` and subscribe the **current**
/// task to it. Safe to call from a thread that wasn't auto-subscribed.
///
/// Idempotent re-init: if the WDT was already initialized (e.g. by another
/// safety-critical task that came up first), falls back to `reconfigure`.
pub fn init_and_subscribe(timeout: Duration) {
    let cfg = esp_task_wdt_config_t {
        timeout_ms: timeout.as_millis() as u32,
        // Bitmask of cores to monitor for idle-task hangs. 0 = none —
        // we only care about *this* task missing its reset.
        idle_core_mask: 0,
        // Trigger panic (→ reboot) on timeout instead of just logging.
        trigger_panic: true,
    };
    if unsafe { esp_task_wdt_init(&cfg) } != ESP_OK {
        esp!(unsafe { esp_task_wdt_reconfigure(&cfg) }).expect("WDT reconfigure");
    }
    esp!(unsafe { esp_task_wdt_add(std::ptr::null_mut()) }).expect("WDT subscribe current task");
}

/// Feed the watchdog from the current task. Must be called at least once
/// per `timeout` interval after [`init_and_subscribe`].
pub fn reset() {
    unsafe {
        esp_task_wdt_reset();
    }
}

/// Unsubscribe the current task. Call this before a deliberate thread
/// exit so the WDT doesn't reboot the MCU once feeds stop arriving.
pub fn unsubscribe() {
    esp!(unsafe { esp_task_wdt_delete(std::ptr::null_mut()) }).expect("WDT unsubscribe");
}
