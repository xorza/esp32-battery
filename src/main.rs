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
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::{info, warn};

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

pub fn uptime_s() -> u32 {
    (unsafe { esp_idf_svc::sys::esp_timer_get_time() } / 1_000_000) as u32
}

static NTP_SYNCED: AtomicBool = AtomicBool::new(false);
pub static CAPTIVE_PORTAL_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Returns current epoch time in seconds, or None if NTP has not synced yet.
pub fn epoch_s() -> Option<u32> {
    if !NTP_SYNCED.load(Ordering::Relaxed) {
        return None;
    }
    Some(esp_idf_svc::systime::EspSystemTime.now().as_secs() as u32)
}

#[allow(unused)]
enum Server<'a> {
    Main(esp_idf_svc::http::server::EspHttpServer<'a>),
    Captive(esp_idf_svc::http::server::EspHttpServer<'a>, dns::DnsHandle),
    None,
}

fn start_sntp() -> esp_idf_svc::sntp::EspSntp<'static> {
    info!("Starting NTP sync");
    esp_idf_svc::sntp::EspSntp::new_with_callback(&esp_idf_svc::sntp::SntpConf::default(), |_| {
        info!("NTP synced");
        NTP_SYNCED.store(true, Ordering::Relaxed);
    })
    .unwrap()
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs_partition = EspDefaultNvsPartition::take().unwrap();

    let nvs = Arc::new(nvs_creds::open(nvs_partition.clone()));
    let creds = nvs_creds::load(&nvs);

    let esp_platform = platform::EspPlatform::new(nvs_partition.clone());

    let wifi = Arc::new(Mutex::new(wifi::Wifi::new(
        peripherals.modem,
        sysloop,
        nvs_partition,
    )));

    #[cfg(feature = "esp32c3")]
    let (i2c_sda, i2c_scl) = (peripherals.pins.gpio8, peripherals.pins.gpio9);
    #[cfg(feature = "esp32c6")]
    let (i2c_sda, i2c_scl) = (peripherals.pins.gpio20, peripherals.pins.gpio23);

    let i2c_bus: &'static I2cBusDriver = Box::leak(Box::new(
        I2cBusDriver::new(peripherals.i2c0, i2c_sda, i2c_scl, &BusConfig::new()).unwrap(),
    ));
    let sensor_data = Arc::new(Mutex::new(esp32_battery_logic::data::SensorData::new(
        esp_platform,
    )));
    ina::start_measurement_thread(i2c_bus, sensor_data.clone());

    #[cfg(feature = "lcd")]
    lcd::start_lcd_thread(
        lcd::LcdPins {
            sclk: peripherals.pins.gpio7.into(),
            mosi: peripherals.pins.gpio6.into(),
            cs: peripherals.pins.gpio14.into(),
            dc: peripherals.pins.gpio15.into(),
            rst: peripherals.pins.gpio21.into(),
            blk: peripherals.pins.gpio22.into(),
            spi: peripherals.spi2,
            ledc_timer: peripherals.ledc.timer0,
            ledc_channel: peripherals.ledc.channel0,
        },
        sensor_data.clone(),
    );

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
                CAPTIVE_PORTAL_ACTIVE.store(false, Ordering::Relaxed);
                drop(sntp.take());
                sntp = Some(start_sntp());
                server = Server::Main(http::start_main(sensor_data.clone(), nvs.clone()));
            }

            (Server::Captive(_, _), true) => {
                info!("WiFi reconnected, switching to STA-only");
                CAPTIVE_PORTAL_ACTIVE.store(false, Ordering::Relaxed);
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
                CAPTIVE_PORTAL_ACTIVE.store(true, Ordering::Relaxed);
            }
        }
    }
}
