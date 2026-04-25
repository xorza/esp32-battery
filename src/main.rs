mod api;
mod app_state;
mod board;
mod captive_api;
mod clock;
mod dns;
mod errors;
mod history_store;
mod http;
mod ina;
#[cfg(feature = "lcd")]
mod lcd;
mod log_ring;
mod nvs_creds;
mod ota;
mod reboot;
mod wifi;
mod wifi_reset;
mod xy;

use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{info, warn};

use esp32_battery_logic::save_scheduler::{DEFAULT_SAVE_INTERVAL_S, SaveScheduler};

use crate::app_state::{EventLogHandle, EventRecorder, SensorDataHandle, Supervisor};
use crate::captive_api::SaveState;
use crate::history_store::{HistoryStore, Persister};

/// Number of consecutive 1 Hz ticks with `is_connected() == false` before we
/// tear down the host server and fall back to the captive AP. Sized to cover
/// initial DHCP/DNS at boot and brief link blips without flapping the SSID.
const CAPTIVE_AFTER_FAILURES: u32 = 15;

/// How long the captive page's "Connecting..." spinner is allowed to run
/// before we declare the submitted credentials a failure and let the user
/// re-enter them. ESP-IDF associates good creds in 3–8s typically; 20s is
/// comfortably past that without leaving the user staring forever.
const CAPTIVE_TRYING_TIMEOUT: Duration = Duration::from_secs(20);

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log_ring::init();

    // Reboot on any thread panic. Without this, a panic in (e.g.) the INA thread
    // poisons the sensor_data mutex; subsequent HTTP / LCD handlers then panic on
    // lock(), leaving a half-dead device that doesn't get caught by the watchdog.
    std::panic::set_hook(Box::new(|info| {
        log::error!("panic: {info}");
        thread::sleep(Duration::from_millis(500));
        esp_idf_svc::hal::reset::restart();
    }));

    let board = board::Board::take();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs_partition = EspDefaultNvsPartition::take().unwrap();

    let nvs = Arc::new(nvs_creds::open(nvs_partition.clone()));
    let mut creds = nvs_creds::load(&nvs);

    let clock = clock::EspClock::new();
    let history_store = HistoryStore::new(nvs_partition.clone());

    let wifi = Arc::new(Mutex::new(wifi::Wifi::new(
        board.modem,
        sysloop,
        nvs_partition,
    )));

    // Load persisted history at boot so the first commit doesn't dump a stale
    // blob and the dashboard has data before the first live commit lands.
    let mut sd = esp32_battery_logic::data::SensorData::new();
    let mut load_buf = vec![0u8; esp32_battery_logic::data::SERIALIZED_MAX_BYTES];
    if let Some(len) = history_store.load(&mut load_buf)
        && !sd.load_from_bytes(&load_buf[..len])
    {
        warn!("history blob in NVS is corrupt or from an older version — discarding");
    }

    let sensor_data: SensorDataHandle = Arc::new(Mutex::new(sd));
    let event_log: EventLogHandle =
        Arc::new(Mutex::new(esp32_battery_logic::error_log::EventLog::new()));
    let mut supervisor = Supervisor::new();
    let mut persister = Persister::new(
        sensor_data.clone(),
        history_store,
        SaveScheduler::new(DEFAULT_SAVE_INTERVAL_S),
    );

    let _sntp = clock::start_sntp(clock.clone());

    let recorder = EventRecorder::new(event_log.clone(), clock.clone());

    xy::start(board.xy, sensor_data.clone(), recorder.clone());
    ina::start(board.i2c, sensor_data.clone(), recorder);

    #[cfg(feature = "lcd")]
    lcd::start(board.lcd, sensor_data.clone(), supervisor.status.clone());

    if let Some(ref creds) = creds {
        wifi.lock().unwrap().start_sta(creds);
    }

    // Wall-clock timestamp of the current "Trying" window. Set on each
    // captive-portal submission, cleared on success or timeout.
    let mut trying_since: Option<Instant> = None;

    loop {
        thread::sleep(Duration::from_secs(1));

        persister.tick(clock.epoch_s());

        if let Some(new) = supervisor.take_pending_creds() {
            // /save is the only producer of these creds and it's only
            // mounted while the supervisor is in Captive — so the AP is
            // up and a live STA-config update is the right move.
            let state = supervisor
                .captive_save_state()
                .expect("creds posted while supervisor isn't in Captive phase");
            wifi.lock().unwrap().set_sta_creds_live(&new);
            state.store(SaveState::Trying as u8, Ordering::Relaxed);
            trying_since = Some(Instant::now());
            creds = Some(new);
        }

        // Time out the Trying spinner so the captive page can show a
        // failure and let the user re-enter creds. The AP stays up.
        if let Some(start) = trying_since
            && start.elapsed() >= CAPTIVE_TRYING_TIMEOUT
            && let Some(state) = supervisor.captive_save_state()
            && state.load(Ordering::Relaxed) == SaveState::Trying as u8
        {
            warn!("Captive: STA association timed out; flipping to Failed");
            state.store(SaveState::Failed as u8, Ordering::Relaxed);
            trying_since = None;
        }

        let connected = wifi.lock().unwrap().tick();
        if connected {
            // Captive-aware handoff: flip Connected and pause briefly so
            // the page (polling /status every ~1s) gets a chance to see
            // it and call window.close() before we tear down the AP. With
            // no pause the captive server stops accepting requests in the
            // same tick and the page never observes Connected.
            if let Some(state) = supervisor.captive_save_state() {
                info!("Captive: STA associated — broadcasting Connected");
                state.store(SaveState::Connected as u8, Ordering::Relaxed);
                trying_since = None;
                thread::sleep(Duration::from_millis(1200));
                let c = creds.as_ref().expect("captive cannot run without creds");
                wifi.lock().unwrap().start_sta(c);
            }
            let sd = sensor_data.clone();
            let el = event_log.clone();
            supervisor.on_tick_connected(|| http::start_main(sd, el, nvs.clone()));
        } else {
            let creds_tx = supervisor.creds_sender();
            supervisor.on_tick_disconnected(creds.is_some(), CAPTIVE_AFTER_FAILURES, || {
                wifi.lock().unwrap().start_ap_mixed(creds.as_ref());
                http::start_captive(creds_tx, nvs.clone(), wifi.clone())
            });
        }
    }
}
