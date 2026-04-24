mod api;
mod app_state;
mod board;
mod dns;
mod http;
mod ina;
#[cfg(feature = "lcd")]
mod lcd;
mod log_ring;
mod nvs_creds;
mod ota;
mod platform;
mod wifi;
mod xy;

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{info, warn};

pub use app_state::{AppState, uptime_s};

/// HTTP server + optional captive-portal DNS hijack. Held purely for `Drop`.
/// `AppState::is_captive` is the source of truth for which kind is running.
struct ActiveServer {
    #[allow(dead_code)]
    http: EspHttpServer<'static>,
    #[allow(dead_code)]
    dns: Option<dns::DnsHandle>,
}

fn start_sntp(clock: platform::EspClock) -> esp_idf_svc::sntp::EspSntp<'static> {
    info!("Starting NTP sync");
    esp_idf_svc::sntp::EspSntp::new_with_callback(
        &esp_idf_svc::sntp::SntpConf::default(),
        move |synced_at| {
            // The SNTP callback can fire with a bogus time (bad server, DNS
            // hijack, pre-sync tick). Only flip the flag once the reported
            // epoch is within the plausibility window — otherwise a bogus
            // value reaches SensorData and poisons history.
            let secs = synced_at.as_secs();
            if platform::VALID_EPOCH_S.contains(&secs) {
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

    let clock = platform::EspClock::new();
    let history_store = platform::HistoryStore::new(nvs_partition.clone());

    let wifi = Arc::new(Mutex::new(wifi::Wifi::new(
        board.modem,
        sysloop,
        nvs_partition,
    )));

    // Load persisted history at boot so the first commit doesn't dump a stale
    // blob and the dashboard has data before the first live commit lands.
    let mut sensor_data = esp32_battery_logic::data::SensorData::new(clock.clone());
    let mut load_buf = vec![0u8; esp32_battery_logic::data::SERIALIZED_MAX_BYTES];
    if let Some(len) = history_store.load(&mut load_buf)
        && !sensor_data.load_from_bytes(&load_buf[..len])
    {
        warn!("history blob in NVS is corrupt or from an older version — discarding");
    }

    let state = AppState::new(sensor_data, history_store);

    // SNTP runs once for the whole lifetime — the client handles WiFi flaps
    // internally, so there's no reason to tear it down and restart.
    let _sntp = start_sntp(clock.clone());

    xy::start(board.xy, state.clone());

    ina::start(board.i2c, state.clone());

    #[cfg(feature = "lcd")]
    lcd::start(board.lcd, state.clone());

    if let Some(ref creds) = creds {
        wifi.lock().unwrap().start_sta(creds);
    }

    let mut server: Option<ActiveServer> = None;

    loop {
        thread::sleep(Duration::from_secs(1));

        // Persist history if a save is due. The payload is taken under the
        // sensor_data lock, then written to flash outside it so sensor
        // threads don't stall on the 50–100 ms NVS erase/write.
        let save_payload = state.sensor_data.lock().unwrap().take_save_payload();
        if let Some(bytes) = save_payload {
            state.history_store.save(&bytes);
        }

        if let Some(new_creds) = state.pending_creds.lock().unwrap().take() {
            info!("Applying credentials submitted via captive portal");
            wifi.lock().unwrap().start_sta(&new_creds);
            creds = Some(new_creds);
        }

        let connected = {
            let mut wf = wifi.lock().unwrap();
            wf.try_reconnect();
            wf.is_connected()
        };

        let want_captive = !connected;
        if server.is_none() || state.is_captive() != want_captive {
            drop(server.take());
            server = Some(if want_captive {
                warn!("WiFi disconnected, starting captive portal");
                wifi.lock().unwrap().start_ap_mixed(creds.as_ref());
                let (http, dns) = http::start_captive(state.clone(), nvs.clone(), wifi.clone());
                state.set_captive(true);
                ActiveServer { http, dns: Some(dns) }
            } else {
                info!("WiFi connected, starting main server");
                state.set_captive(false);
                ActiveServer { http: http::start_main(state.clone(), nvs.clone()), dns: None }
            });
        }
    }
}
