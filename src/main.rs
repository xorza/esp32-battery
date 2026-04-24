mod api;
mod app_state;
mod board;
mod clock;
mod dns;
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
mod xy;

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{info, warn};

use esp32_battery_logic::save_scheduler::{DEFAULT_SAVE_INTERVAL_S, SaveScheduler};

pub use app_state::{AppState, uptime_s};

use crate::clock::EspClock;
use crate::nvs_creds::WifiCredentials;

/// Number of consecutive 1 Hz ticks with `is_connected() == false` before we
/// tear down the host server and fall back to the captive AP. Sized to cover
/// initial DHCP/DNS at boot and brief link blips without flapping the SSID.
const CAPTIVE_AFTER_FAILURES: u32 = 15;

fn tick_and_persist(state: &AppState, clock: &EspClock, scheduler: &mut SaveScheduler) {
    let now = clock.epoch_s();
    // Tick the data store and (if the save timer fires) serialize under one
    // lock — keeps NVS I/O out of the critical section but avoids a two-lock
    // dance + stale-state window between them.
    let payload = {
        let mut sd = state.shared.sensor_data.lock().unwrap();
        sd.tick(now);
        if scheduler.tick(now) {
            Some(sd.serialize())
        } else {
            None
        }
    };
    if let Some(bytes) = payload {
        log::info!("Emitting save payload: {} bytes", bytes.len());
        state.history_store.save(&bytes);
    }
}

fn drain_pending_creds(
    state: &mut AppState,
    wifi: &Mutex<wifi::Wifi<'static>>,
) -> Option<WifiCredentials> {
    let new = state.shared.pending_creds.lock().unwrap().take()?;
    info!("Applying credentials submitted via captive portal");
    wifi.lock().unwrap().start_sta(&new);
    state.on_creds_applied();
    Some(new)
}

fn tick_wifi(wifi: &Mutex<wifi::Wifi<'static>>) -> bool {
    let mut wf = wifi.lock().unwrap();
    wf.try_reconnect();
    wf.is_connected()
}

fn start_sntp(clock: clock::EspClock) -> esp_idf_svc::sntp::EspSntp<'static> {
    info!("Starting NTP sync");
    esp_idf_svc::sntp::EspSntp::new_with_callback(
        &esp_idf_svc::sntp::SntpConf::default(),
        move |synced_at| {
            // The SNTP callback can fire with a bogus time (bad server, DNS
            // hijack, pre-sync tick). Only flip the flag once the reported
            // epoch is within the plausibility window — otherwise a bogus
            // value reaches SensorData and poisons history.
            let secs = synced_at.as_secs();
            if clock::VALID_EPOCH_S.contains(&secs) {
                info!("NTP synced: epoch={secs}");
                clock.mark_synced();
            } else {
                warn!("NTP sync ignored: implausible epoch={secs}");
            }
        },
    )
    .unwrap()
}

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
    let history_store = history_store::HistoryStore::new(nvs_partition.clone());

    let wifi = Arc::new(Mutex::new(wifi::Wifi::new(
        board.modem,
        sysloop,
        nvs_partition,
    )));

    // Load persisted history at boot so the first commit doesn't dump a stale
    // blob and the dashboard has data before the first live commit lands.
    let mut sensor_data = esp32_battery_logic::data::SensorData::new();
    let mut load_buf = vec![0u8; esp32_battery_logic::data::SERIALIZED_MAX_BYTES];
    if let Some(len) = history_store.load(&mut load_buf)
        && !sensor_data.load_from_bytes(&load_buf[..len])
    {
        warn!("history blob in NVS is corrupt or from an older version — discarding");
    }

    let mut state = AppState::new(sensor_data, history_store);
    let mut save_scheduler = SaveScheduler::new(DEFAULT_SAVE_INTERVAL_S);

    // SNTP runs once for the whole lifetime — the client handles WiFi flaps
    // internally, so there's no reason to tear it down and restart.
    let _sntp = start_sntp(clock.clone());

    xy::start(board.xy, state.shared.clone());

    ina::start(board.i2c, state.shared.clone());

    #[cfg(feature = "lcd")]
    lcd::start(board.lcd, state.shared.clone());

    if let Some(ref creds) = creds {
        wifi.lock().unwrap().start_sta(creds);
        state.on_creds_applied();
    }

    loop {
        thread::sleep(Duration::from_secs(1));

        tick_and_persist(&state, &clock, &mut save_scheduler);

        if let Some(new) = drain_pending_creds(&mut state, &wifi) {
            creds = Some(new);
        }

        let connected = tick_wifi(&wifi);
        if connected {
            let shared = state.shared.clone();
            state.on_tick_connected(|| http::start_main(shared, nvs.clone()));
        } else {
            let shared = state.shared.clone();
            state.on_tick_disconnected(creds.is_some(), CAPTIVE_AFTER_FAILURES, || {
                wifi.lock().unwrap().start_ap_mixed(creds.as_ref());
                http::start_captive(shared, nvs.clone(), wifi.clone())
            });
        }
    }
}
