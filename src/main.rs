mod api;
mod app_state;
mod board;
mod dns;
mod http;
mod ina;
#[cfg(feature = "lcd")]
mod lcd;
mod nvs_creds;
mod ota;
mod platform;
mod wifi;
mod xy;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{info, warn};

pub use app_state::{AppState, uptime_s};

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

#[allow(dead_code)]
enum Server<'a> {
    /// Main HTTPS dashboard + SNTP client. The SNTP lifetime is bound to Server::Main
    /// so there is never more than one SNTP client running (two would race on the
    /// system clock). Any transition out of Main drops the variant, which drops SNTP.
    Main(
        esp_idf_svc::http::server::EspHttpServer<'a>,
        esp_idf_svc::sntp::EspSntp<'static>,
    ),
    /// Captive portal HTTP + DNS. No SNTP here — time sync is not needed for setup.
    Captive(esp_idf_svc::http::server::EspHttpServer<'a>, dns::DnsHandle),
    None,
}

fn start_sntp(flag: Arc<AtomicBool>) -> esp_idf_svc::sntp::EspSntp<'static> {
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
                flag.store(true, Ordering::Relaxed);
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
    let creds = nvs_creds::load(&nvs);

    let ntp_synced = Arc::new(AtomicBool::new(false));
    let esp_platform = platform::EspPlatform::new(nvs_partition.clone(), ntp_synced.clone());

    let wifi = Arc::new(Mutex::new(wifi::Wifi::new(
        board.modem,
        sysloop,
        nvs_partition,
    )));

    let sensor_data = esp32_battery_logic::data::SensorData::new(esp_platform);
    let state = AppState::new(ntp_synced, sensor_data);

    xy::start(board.xy, state.clone());

    ina::start_measurement_thread(board.i2c.init_bus(), state.clone());

    #[cfg(feature = "lcd")]
    lcd::start_lcd_thread(board.lcd, state.clone());

    if let Some(ref creds) = creds {
        wifi.lock().unwrap().start_sta(creds);
    }

    let mut server = Server::None;

    loop {
        thread::sleep(Duration::from_secs(1));

        let connected = {
            let mut wf = wifi.lock().unwrap();
            wf.try_reconnect();
            wf.is_connected()
        };

        match (&server, connected) {
            (Server::Main(..), true) | (Server::Captive(..), false) => {}

            (Server::None, true) => {
                info!("WiFi connected, starting main server");
                state.set_captive(false);
                // server was None here — no prior SNTP to conflict with.
                let http = http::start_main(state.clone(), nvs.clone());
                let sntp = start_sntp(state.ntp_synced.clone());
                server = Server::Main(http, sntp);
            }

            (Server::Captive(..), true) => {
                info!("WiFi reconnected, switching to STA-only");
                state.set_captive(false);
                server = Server::None;
                let creds = creds.as_ref().expect("connected requires credentials");
                wifi.lock().unwrap().start_sta(creds);
            }

            (_, false) => {
                warn!("WiFi disconnected, starting captive portal");
                // Drop the old Main (which owns SNTP) and any old Captive first,
                // before reconfiguring WiFi and starting the new captive HTTP.
                drop(std::mem::replace(&mut server, Server::None));
                wifi.lock().unwrap().start_ap_mixed(creds.as_ref());
                let (s, d) = http::start_captive(nvs.clone(), wifi.clone());
                server = Server::Captive(s, d);
                state.set_captive(true);
            }
        }
    }
}
