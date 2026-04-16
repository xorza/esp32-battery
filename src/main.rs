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

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp_idf_hal::i2c::{I2cBusDriver, config::BusConfig};
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
    Main(esp_idf_svc::http::server::EspHttpServer<'a>),
    Captive(esp_idf_svc::http::server::EspHttpServer<'a>, dns::DnsHandle),
    None,
}

fn start_sntp(flag: Arc<AtomicBool>) -> esp_idf_svc::sntp::EspSntp<'static> {
    info!("Starting NTP sync");
    esp_idf_svc::sntp::EspSntp::new_with_callback(
        &esp_idf_svc::sntp::SntpConf::default(),
        move |_| {
            info!("NTP synced");
            flag.store(true, Ordering::Relaxed);
        },
    )
    .unwrap()
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

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

    let i2c_bus: &'static I2cBusDriver = Box::leak(Box::new(
        I2cBusDriver::new(
            board.i2c.i2c,
            board.i2c.sda,
            board.i2c.scl,
            &BusConfig::new(),
        )
        .unwrap(),
    ));

    let sensor_data = esp32_battery_logic::data::SensorData::new(esp_platform);
    let state = AppState::new(ntp_synced, sensor_data);

    ina::start_measurement_thread(i2c_bus, state.clone());

    #[cfg(feature = "lcd")]
    lcd::start_lcd_thread(board.lcd, state.clone());

    if let Some(ref creds) = creds {
        wifi.lock().unwrap().start_sta(creds);
    }

    let mut server = Server::None;
    let mut sntp: Option<esp_idf_svc::sntp::EspSntp<'static>> = None;

    loop {
        thread::sleep(Duration::from_secs(1));

        let mut wf = wifi.lock().unwrap();
        wf.try_reconnect();
        let connected = wf.is_connected();
        drop(wf);

        match (&server, connected) {
            (Server::Main(_), true) | (Server::Captive(_, _), false) => {}

            (Server::None, true) => {
                info!("WiFi connected, starting main server");
                state.set_captive(false);
                drop(sntp.take());
                sntp = Some(start_sntp(state.ntp_synced.clone()));
                server = Server::Main(http::start_main(state.clone(), nvs.clone()));
            }

            (Server::Captive(_, _), true) => {
                info!("WiFi reconnected, switching to STA-only");
                state.set_captive(false);
                server = Server::None;
                let creds = creds.as_ref().expect("connected requires credentials");
                wifi.lock().unwrap().start_sta(creds);
            }

            (_, false) => {
                warn!("WiFi disconnected, starting captive portal");
                drop(sntp.take());
                drop(std::mem::replace(&mut server, Server::None));
                wifi.lock().unwrap().start_ap_mixed(creds.as_ref());
                let (s, d) = http::start_captive(nvs.clone(), wifi.clone());
                server = Server::Captive(s, d);
                state.set_captive(true);
            }
        }
    }
}
