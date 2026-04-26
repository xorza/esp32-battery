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
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::nvs::{EspDefaultNvsPartition, EspNvs, NvsDefault};
use log::warn;

use esp32_battery_logic::data::SensorData;
use esp32_battery_logic::error_log::EventLog;
use esp32_battery_logic::save_scheduler::{DEFAULT_SAVE_INTERVAL_S, SaveScheduler};

use crate::clock::{EventRecorder, uptime};
use crate::history_store::{HistoryStore, Persister};
use crate::net::{NetState, NetStatusHandle, SubmissionStatus};
use crate::nvs_creds::WifiCredentials;

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

struct StaCtx {
    sensor_data: Arc<Mutex<SensorData>>,
    event_log: Arc<Mutex<EventLog>>,
    nvs: Arc<EspNvs<NvsDefault>>,
}

impl StaCtx {
    fn start_dashboard(&self) -> EspHttpServer<'static> {
        http::start_main(
            self.sensor_data.clone(),
            self.event_log.clone(),
            self.nvs.clone(),
        )
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();
    log_ring::init();
    ota::init();

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
    lcd::start(
        board.lcd,
        sensor_data.clone(),
        event_log.clone(),
        net_status.clone(),
    );

    let sta_ctx = StaCtx {
        sensor_data,
        event_log,
        nvs: nvs.clone(),
    };

    // Bootstrap per `wifi_fsm.md`: STA if we have creds, captive otherwise.
    let mut state = match boot_creds {
        Some(creds) => {
            let sta_wifi = wifi.into_sta(&creds);
            let server = sta_ctx.start_dashboard();
            NetState::StaConnecting {
                wifi: sta_wifi,
                server,
                session_start: uptime(),
            }
        }
        None => {
            let mixed = wifi.into_mixed(None);
            let bundle = http::start_captive(mixed.scan_cache());
            NetState::BootNoCreds {
                wifi: mixed,
                bundle,
            }
        }
    };
    net_status.store(state.lcd_status());

    loop {
        thread::sleep(TICK_PERIOD);
        let now = uptime();
        persister.tick(clock.epoch_s());

        state = step(state, now, &sta_ctx);
        net_status.store(state.lcd_status());
    }
}

/// One supervisor tick. Consumes `state` and returns the next variant.
/// Each arm is the full transition logic for that state — there are no
/// fallthroughs and no shared mutable cross-state plumbing.
fn step(state: NetState, now: Duration, ctx: &StaCtx) -> NetState {
    match state {
        NetState::BootNoCreds { mut wifi, bundle } => {
            if let Some(creds) = take_submitted(&bundle) {
                return NetState::CaptiveSubmitted {
                    wifi,
                    bundle,
                    creds,
                    since: now,
                };
            }
            wifi.refresh_scan_if_stale(now);
            NetState::BootNoCreds { wifi, bundle }
        }
        NetState::CaptiveSubmitted {
            mut wifi,
            bundle,
            creds,
            since,
        } => {
            // One-tick state: apply parked creds to the radio, flip
            // status to Trying, fall through to CaptiveTrying. `since`
            // carries forward — the 20s window starts at /save time.
            wifi.set_sta_creds(&creds);
            bundle.status.store(SubmissionStatus::Trying);
            NetState::CaptiveTrying {
                wifi,
                bundle,
                creds,
                since,
            }
        }
        NetState::CaptiveTrying {
            mut wifi,
            bundle,
            creds,
            since,
        } => {
            // A second /save during the trying window overrides — drop
            // current attempt, restart with new creds.
            if let Some(new_creds) = take_submitted(&bundle) {
                return NetState::CaptiveSubmitted {
                    wifi,
                    bundle,
                    creds: new_creds,
                    since: now,
                };
            }
            if wifi.try_connect() {
                return promote_to_host(wifi, bundle, creds, ctx, now);
            }
            if now.saturating_sub(since) >= CAPTIVE_TRYING_TIMEOUT {
                bundle.status.store(SubmissionStatus::Failed);
                warn!("Captive: STA association timed out; flipping to Failed");
                return NetState::CaptiveFailed { wifi, bundle };
            }
            NetState::CaptiveTrying {
                wifi,
                bundle,
                creds,
                since,
            }
        }
        NetState::CaptiveFailed { mut wifi, bundle } => {
            if let Some(creds) = take_submitted(&bundle) {
                return NetState::CaptiveSubmitted {
                    wifi,
                    bundle,
                    creds,
                    since: now,
                };
            }
            wifi.refresh_scan_if_stale(now);
            NetState::CaptiveFailed { wifi, bundle }
        }
        NetState::CaptiveFallbackRetrying { mut wifi, bundle } => {
            // Sta→Captive carry-over: NVS already has known-good (or
            // last-known) creds and the radio is configured to retry
            // them. /save still wins if the user re-submits.
            if let Some(creds) = take_submitted(&bundle) {
                return NetState::CaptiveSubmitted {
                    wifi,
                    bundle,
                    creds,
                    since: now,
                };
            }
            if wifi.try_connect() {
                let creds = nvs_creds::load(&ctx.nvs)
                    .expect("CaptiveFallbackRetrying without NVS creds");
                return promote_to_host(wifi, bundle, creds, ctx, now);
            }
            wifi.refresh_scan_if_stale(now);
            NetState::CaptiveFallbackRetrying { wifi, bundle }
        }
        NetState::StaConnecting {
            mut wifi,
            server,
            session_start,
        } => {
            if wifi.try_connect() {
                let mdns = wifi::setup_mdns();
                return NetState::StaHost {
                    wifi,
                    server,
                    mdns,
                    last_assoc: now,
                };
            }
            if now.saturating_sub(session_start) >= CAPTIVE_AFTER_DISCONNECT {
                return fallback_to_captive(wifi, server, None, ctx);
            }
            NetState::StaConnecting {
                wifi,
                server,
                session_start,
            }
        }
        NetState::StaHost {
            mut wifi,
            server,
            mdns,
            last_assoc,
        } => {
            if wifi.try_connect() {
                NetState::StaHost {
                    wifi,
                    server,
                    mdns,
                    last_assoc: now,
                }
            } else {
                NetState::StaReassociating {
                    wifi,
                    server,
                    mdns,
                    last_assoc,
                }
            }
        }
        NetState::StaReassociating {
            mut wifi,
            server,
            mdns,
            last_assoc,
        } => {
            if wifi.try_connect() {
                return NetState::StaHost {
                    wifi,
                    server,
                    mdns,
                    last_assoc: now,
                };
            }
            if now.saturating_sub(last_assoc) >= CAPTIVE_AFTER_DISCONNECT {
                return fallback_to_captive(wifi, server, Some(mdns), ctx);
            }
            NetState::StaReassociating {
                wifi,
                server,
                mdns,
                last_assoc,
            }
        }
    }
}

/// Pop a freshly-submitted creds payload out of the captive bundle's
/// mailbox. Returns `None` if no `/save` has fired since the last drain.
fn take_submitted(bundle: &net::CaptiveBundle) -> Option<WifiCredentials> {
    bundle.mailbox.lock().unwrap().take()
}

/// Captive → STA promotion. Persist creds to NVS, drop the captive
/// bundle (joins DNS thread, stops captive HTTP), switch the radio to
/// STA-only, start the dashboard server + mDNS.
fn promote_to_host(
    wifi: wifi::MixedWifi<'static>,
    bundle: net::CaptiveBundle,
    creds: WifiCredentials,
    ctx: &StaCtx,
    now: Duration,
) -> NetState {
    nvs_creds::save(&ctx.nvs, &creds.ssid, &creds.password);
    drop(bundle);
    let sta_wifi = wifi.into_sta(&creds);
    let server = ctx.start_dashboard();
    let mdns = wifi::setup_mdns();
    NetState::StaHost {
        wifi: sta_wifi,
        server,
        mdns,
        last_assoc: now,
    }
}

/// STA → Captive fallback. Drops the dashboard, switches the radio to
/// Mixed (carrying creds so the STA half keeps retrying), brings up the
/// captive bundle.
fn fallback_to_captive(
    wifi: wifi::StaWifi<'static>,
    server: EspHttpServer<'static>,
    mdns: Option<esp_idf_svc::mdns::EspMdns>,
    ctx: &StaCtx,
) -> NetState {
    drop(server);
    drop(mdns);
    let creds =
        nvs_creds::load(&ctx.nvs).expect("STA fallback without NVS creds (boot path bug?)");
    let mixed = wifi.into_mixed(Some(&creds));
    let bundle = http::start_captive(mixed.scan_cache());
    NetState::CaptiveFallbackRetrying {
        wifi: mixed,
        bundle,
    }
}
