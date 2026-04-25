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

    // Bootstrap: STA if we have creds, else captive. Per `wifi flow.md`.
    let mut net = match &creds {
        Some(c) => start_sta_session(
            &wifi,
            c,
            &sensor_data,
            &event_log,
            &nvs,
            LinkSeen::Never {
                session_start: uptime(),
            },
        ),
        None => start_captive_session(&wifi, None),
    };
    net_status.store(match &net {
        Net::Sta { link_seen, .. } => NetStatus::for_sta(link_seen, false, uptime()),
        Net::Captive { .. } => NetStatus::Captive,
    });

    loop {
        thread::sleep(TICK_PERIOD);
        let now = uptime();
        persister.tick(clock.epoch_s());

        // Two-phase: decide what to do (with `&mut net` so stay cases
        // can mutate in place), then apply transitions where consuming
        // the old `net` is needed to drop server/bundle.
        let step = match &mut net {
            Net::Captive { bundle } => {
                let captive_status = drain_submission(bundle, &wifi, &mut creds, now);
                let connected = wifi.lock().unwrap().tick(creds.is_some());
                if connected {
                    Step::Promote
                } else {
                    Step::Stay(captive_status)
                }
            }
            Net::Sta { link_seen, .. } => {
                let connected = wifi.lock().unwrap().tick(creds.is_some());
                if connected {
                    *link_seen = LinkSeen::At(now);
                    Step::Stay(NetStatus::Host)
                } else if now.saturating_sub(link_seen.timestamp()) >= CAPTIVE_AFTER_DISCONNECT {
                    Step::FallBack
                } else {
                    Step::Stay(NetStatus::for_sta(link_seen, false, now))
                }
            }
        };

        let status = match step {
            Step::Stay(s) => s,
            Step::Promote => {
                // Creds just associated — persist now (wrong creds never
                // overwrite a known-good pair on flash) and promote to
                // STA-only. Reassigning `net` drops the captive bundle,
                // which stops the captive HTTP server + DNS responder.
                let c = creds
                    .clone()
                    .expect("captive→sta transition requires creds");
                nvs_creds::save(&nvs, &c.ssid, &c.password);
                net =
                    start_sta_session(&wifi, &c, &sensor_data, &event_log, &nvs, LinkSeen::At(now));
                NetStatus::Host
            }
            Step::FallBack => {
                // Long STA outage — bring up captive AP+STA so the user
                // can correct creds while STA keeps retrying. Reassigning
                // `net` drops the dashboard server.
                net = start_captive_session(&wifi, creds.as_ref());
                NetStatus::Captive
            }
        };
        net_status.store(status);
    }
}

/// One supervisor-tick decision. Decoupled from application so the
/// observation phase can borrow `&mut net` (and mutate `link_seen` in
/// place for stay-in-Sta) while transitions run after the borrow ends.
enum Step {
    /// No transition; `link_seen` may have been refreshed in place. The
    /// LCD reading was computed during observation.
    Stay(NetStatus),
    /// Captive → Sta. STA just associated with submitted creds.
    Promote,
    /// Sta → Captive. STA has been disconnected past the grace window.
    FallBack,
}

/// Drain a fresh `/save` handoff (`Pending → Trying`) and time-out a
/// stale `Trying` window — single critical section on the submission
/// lock. Returns the captive LCD reading post-drain so the caller
/// doesn't have to re-acquire the lock.
fn drain_submission(
    bundle: &net::CaptiveBundle,
    wifi: &Mutex<wifi::Wifi<'static>>,
    creds: &mut Option<nvs_creds::WifiCredentials>,
    now: Duration,
) -> NetStatus {
    let mut s = bundle.state.lock().unwrap();
    let taken = std::mem::replace(&mut *s, Submission::Idle);
    match taken {
        Submission::Pending {
            creds: new_creds,
            since,
        } => {
            wifi.lock().unwrap().set_sta_creds_live(&new_creds);
            *s = Submission::Trying { since };
            *creds = Some(new_creds);
        }
        Submission::Trying { since } if now.saturating_sub(since) >= CAPTIVE_TRYING_TIMEOUT => {
            *s = Submission::Failed;
            warn!("Captive: STA association timed out; flipping to Failed");
        }
        other => *s = other,
    }
    NetStatus::for_captive(&s)
}

/// Bring up STA-only mode + dashboard server.
fn start_sta_session(
    wifi: &Arc<Mutex<wifi::Wifi<'static>>>,
    creds: &nvs_creds::WifiCredentials,
    sensor_data: &Arc<Mutex<SensorData>>,
    event_log: &Arc<Mutex<EventLog>>,
    nvs: &Arc<esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>>,
    link_seen: LinkSeen,
) -> Net {
    wifi.lock().unwrap().start_sta(creds);
    let server = http::start_main(sensor_data.clone(), event_log.clone(), nvs.clone());
    Net::Sta {
        _server: server,
        link_seen,
    }
}

/// Bring up AP+STA Mixed + captive HTTP/DNS bundle.
fn start_captive_session(
    wifi: &Arc<Mutex<wifi::Wifi<'static>>>,
    creds: Option<&nvs_creds::WifiCredentials>,
) -> Net {
    wifi.lock().unwrap().start_ap_mixed(creds);
    let state = Arc::new(Mutex::new(Submission::Idle));
    let bundle = http::start_captive(wifi.clone(), state);
    Net::Captive { bundle }
}
