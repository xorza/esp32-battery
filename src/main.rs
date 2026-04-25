mod api;
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
mod net;
mod nvs_creds;
mod ota;
mod reboot;
mod wifi;
mod wifi_reset;
mod xy;

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use log::warn;

use esp32_battery_logic::data::SensorData;
use esp32_battery_logic::error_log::EventLog;
use esp32_battery_logic::save_scheduler::{DEFAULT_SAVE_INTERVAL_S, SaveScheduler};

use crate::clock::{EventRecorder, uptime};
use crate::history_store::{HistoryStore, Persister};
use crate::net::{LinkSeen, Net, NetStatus, NetStatusHandle, Submission};

/// How long `is_connected() == false` may persist before we tear down
/// the host server and fall back to the captive AP. The AP is a fallback
/// for "the saved creds no longer work" (rotated password, SSID gone),
/// so we wait long enough that a real outage of the user's router (ISP
/// reboot, scheduled maintenance) doesn't unnecessarily flap us into
/// captive mode and break the dashboard for everyone on the LAN.
const CAPTIVE_AFTER_DISCONNECT: Duration = Duration::from_secs(2 * 60 * 60);

/// How long the captive page's "Connecting..." spinner is allowed to
/// run before we declare the submitted credentials a failure and let
/// the user re-enter them. ESP-IDF associates good creds in 3–8s
/// typically; 20s is comfortably past that.
const CAPTIVE_TRYING_TIMEOUT: Duration = Duration::from_secs(20);

/// Supervisor loop period.
const TICK_PERIOD: Duration = Duration::from_secs(1);

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

    let sensor_data: Arc<Mutex<SensorData>> = Arc::new(Mutex::new(sd));
    let event_log: Arc<Mutex<EventLog>> =
        Arc::new(Mutex::new(esp32_battery_logic::error_log::EventLog::new()));
    let net_status = NetStatusHandle::new();
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
    lcd::start(board.lcd, sensor_data.clone(), net_status.clone());

    // Initial state: STA if we have creds, else captive. Bootstrap with
    // a host server eagerly when creds are present — `last_associated`
    // starts at boot time so the grace timer covers DHCP.
    // Radio mode tracks Net: STA-only when we have saved creds, Mixed
    // AP+STA when we don't (or after long STA outage). Per `wifi
    // flow.md`.
    let mut net = match &creds {
        Some(c) => {
            wifi.lock().unwrap().start_sta(c);
            let server = http::start_main(sensor_data.clone(), event_log.clone(), nvs.clone());
            Net::Sta {
                server,
                link_seen: LinkSeen::Never {
                    session_start: uptime(),
                },
            }
        }
        None => {
            wifi.lock().unwrap().start_ap_mixed(None);
            let state = Arc::new(Mutex::new(Submission::Idle));
            let bundle = http::start_captive(wifi.clone(), state);
            Net::Captive { bundle }
        }
    };
    // Initial LCD: bootstrap captive is always Idle, bootstrap Sta hasn't
    // associated yet. Both fall through to the obvious value.
    net_status.store(match &net {
        Net::Sta { link_seen, .. } => NetStatus::for_sta(link_seen, false, uptime()),
        Net::Captive { .. } => NetStatus::Captive,
    });

    loop {
        thread::sleep(TICK_PERIOD);
        let now = uptime();
        persister.tick(clock.epoch_s());

        let (new_net, lcd) = match net {
            Net::Captive { bundle } => {
                // Drain a fresh /save handoff (Pending → Trying) and time
                // out a stale Trying window — single critical section on
                // the submission lock. Compute the captive LCD reading
                // inside the same lock so end-of-tick doesn't re-acquire.
                let captive_lcd = {
                    let mut s = bundle.state.lock().unwrap();
                    let taken = std::mem::replace(&mut *s, Submission::Idle);
                    match taken {
                        Submission::Pending {
                            creds: new_creds,
                            since,
                        } => {
                            wifi.lock().unwrap().set_sta_creds_live(&new_creds);
                            *s = Submission::Trying { since };
                            creds = Some(new_creds);
                        }
                        Submission::Trying { since }
                            if now.saturating_sub(since) >= CAPTIVE_TRYING_TIMEOUT =>
                        {
                            *s = Submission::Failed;
                            warn!("Captive: STA association timed out; flipping to Failed");
                        }
                        other => *s = other,
                    }
                    NetStatus::for_captive(&s)
                };

                let connected = wifi.lock().unwrap().tick(creds.is_some());
                if connected {
                    let c = creds
                        .clone()
                        .expect("captive→sta transition requires creds");
                    // Persist only after the new creds successfully
                    // associated. Wrong creds therefore never overwrite
                    // a known-good pair on flash.
                    nvs_creds::save(&nvs, &c.ssid, &c.password);
                    drop(bundle);
                    // Switch radio to STA-only — AP goes down now that
                    // the user has a working network.
                    wifi.lock().unwrap().start_sta(&c);
                    let server =
                        http::start_main(sensor_data.clone(), event_log.clone(), nvs.clone());
                    let net = Net::Sta {
                        server,
                        link_seen: LinkSeen::At(now),
                    };
                    (net, NetStatus::Host)
                } else {
                    (Net::Captive { bundle }, captive_lcd)
                }
            }
            Net::Sta { server, link_seen } => {
                let connected = wifi.lock().unwrap().tick(creds.is_some());
                if connected {
                    let net = Net::Sta {
                        server,
                        link_seen: LinkSeen::At(now),
                    };
                    (net, NetStatus::Host)
                } else if now.saturating_sub(link_seen.timestamp()) >= CAPTIVE_AFTER_DISCONNECT {
                    // Long STA outage — bring up captive AP+STA so the
                    // user can correct creds while STA keeps retrying.
                    drop(server);
                    wifi.lock().unwrap().start_ap_mixed(creds.as_ref());
                    let state = Arc::new(Mutex::new(Submission::Idle));
                    let bundle = http::start_captive(wifi.clone(), state);
                    (Net::Captive { bundle }, NetStatus::Captive)
                } else {
                    let lcd = NetStatus::for_sta(&link_seen, false, now);
                    (Net::Sta { server, link_seen }, lcd)
                }
            }
        };
        net = new_net;
        net_status.store(lcd);
    }
}
