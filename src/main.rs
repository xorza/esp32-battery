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
    let boot_creds = nvs_creds::load(&nvs);

    let clock = clock::EspClock::new();
    let history_store = HistoryStore::new(nvs_partition.clone());

    let wifi = wifi::WifiDriver::new(board.modem, sysloop, nvs_partition);

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
    let mut net = match boot_creds {
        Some(c) => {
            let sta_wifi = wifi.into_sta(&c);
            start_sta_session(
                sta_wifi,
                c,
                &sensor_data,
                &event_log,
                &nvs,
                LinkSeen::Never {
                    session_start: uptime(),
                },
            )
        }
        None => {
            let mixed_wifi = wifi.into_mixed(None);
            start_captive_session(mixed_wifi, None)
        }
    };
    net_status.store(match &net {
        Net::Sta { link_seen, .. } => NetStatus::for_sta(link_seen, false, uptime()),
        Net::Captive { .. } => NetStatus::Captive,
    });

    loop {
        thread::sleep(TICK_PERIOD);
        let now = uptime();
        persister.tick(clock.epoch_s());

        // Each tick consumes `net` and rebuilds it. Mode-changing
        // transitions consume the embedded `wifi` into the other mode
        // wrapper, which moves the radio with the variant — no shared
        // `Arc<Mutex<…>>` to thread through helpers.
        let (next, status) = match net {
            Net::Captive {
                mut wifi,
                bundle,
                mut creds,
            } => {
                let captive_status = drain_submission(&bundle, &mut wifi, &mut creds, now);
                let connected = wifi.try_connect();
                // Only refresh while we're not mid-association: a fresh
                // `scan_n` competes with an in-flight associate and can
                // tank the user's submitted creds. `Captive` covers the
                // Idle/Failed sub-states; `CaptiveTrying` (Pending/Trying)
                // is excluded.
                if !connected && captive_status == NetStatus::Captive {
                    wifi.refresh_scan_if_stale(now);
                }
                if connected {
                    // `connected` here implies a Pending was drained
                    // earlier this tick (or a prior tick that landed
                    // creds and we associated since) — creds is Some.
                    let c = creds.expect("captive associated implies creds");
                    nvs_creds::save(&nvs, &c.ssid, &c.password);
                    // Drop captive bundle first so its server/dns threads
                    // join before we tear the AP down via `into_sta`.
                    drop(bundle);
                    let sta_wifi = wifi.into_sta(&c);
                    let next = start_sta_session(
                        sta_wifi,
                        c,
                        &sensor_data,
                        &event_log,
                        &nvs,
                        LinkSeen::At(now),
                    );
                    (next, NetStatus::Host)
                } else {
                    (
                        Net::Captive {
                            wifi,
                            bundle,
                            creds,
                        },
                        captive_status,
                    )
                }
            }
            Net::Sta {
                mut wifi,
                server,
                mut mdns,
                creds,
                mut link_seen,
            } => {
                let connected = wifi.try_connect();
                if connected {
                    link_seen = LinkSeen::At(now);
                    if mdns.is_none() {
                        // First associated tick — netif is up, take and
                        // configure mDNS. Stays alive until the variant
                        // is dropped on `Sta → Captive`.
                        mdns = Some(wifi::setup_mdns());
                    }
                    (
                        Net::Sta {
                            wifi,
                            server,
                            mdns,
                            creds,
                            link_seen,
                        },
                        NetStatus::Host,
                    )
                } else if now.saturating_sub(link_seen.timestamp()) >= CAPTIVE_AFTER_DISCONNECT {
                    // Long STA outage — drop the dashboard then bring up
                    // captive AP+STA so the user can correct creds while
                    // STA keeps retrying.
                    drop(server);
                    drop(mdns);
                    let mixed_wifi = wifi.into_mixed(Some(&creds));
                    let next = start_captive_session(mixed_wifi, Some(creds));
                    (next, NetStatus::Captive)
                } else {
                    let s = NetStatus::for_sta(&link_seen, false, now);
                    (
                        Net::Sta {
                            wifi,
                            server,
                            mdns,
                            creds,
                            link_seen,
                        },
                        s,
                    )
                }
            }
        };
        net = next;
        net_status.store(status);
    }
}

/// Drain a fresh `/save` handoff (`Pending → Trying`) and time-out a
/// stale `Trying` window — single critical section on the submission
/// lock. Returns the captive LCD reading post-drain so the caller
/// doesn't have to re-acquire the lock.
fn drain_submission(
    bundle: &net::CaptiveBundle,
    wifi: &mut wifi::MixedWifi<'static>,
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
            wifi.set_sta_creds(&new_creds);
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

/// Bring up the dashboard server around an already-STA radio. `creds`
/// is moved into `Net::Sta` so the variant is the single source of truth
/// for "what credentials are we currently trying."
fn start_sta_session(
    wifi: wifi::StaWifi<'static>,
    creds: nvs_creds::WifiCredentials,
    sensor_data: &Arc<Mutex<SensorData>>,
    event_log: &Arc<Mutex<EventLog>>,
    nvs: &Arc<esp_idf_svc::nvs::EspNvs<esp_idf_svc::nvs::NvsDefault>>,
    link_seen: LinkSeen,
) -> Net {
    let server = http::start_main(sensor_data.clone(), event_log.clone(), nvs.clone());
    Net::Sta {
        wifi,
        server,
        mdns: None,
        creds,
        link_seen,
    }
}

/// Bring up the captive HTTP/DNS bundle around an already-Mixed radio.
/// `creds` is the optional carry-over from a `Sta → Captive` fallback
/// (`None` on cold boot with no stored creds).
fn start_captive_session(
    wifi: wifi::MixedWifi<'static>,
    creds: Option<nvs_creds::WifiCredentials>,
) -> Net {
    let scan_cache = wifi.scan_cache();
    let state = Arc::new(Mutex::new(Submission::Idle));
    let bundle = http::start_captive(scan_cache, state);
    Net::Captive {
        wifi,
        bundle,
        creds,
    }
}
