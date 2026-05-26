mod api;
mod board;
mod captive_api;
mod clock;
mod dns;
mod errors;
mod http;
mod ina;
#[cfg(feature = "lcd")]
mod lcd;

mod log_ring;
mod net;
mod nvs_creds;
mod ota;
mod reboot;
mod task_wdt;
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

use esp32_battery_logic::battery::Chemistry;
use esp32_battery_logic::charging::{INPUT_LVP_MARGIN_V, Profile};
use esp32_battery_logic::data::SensorData;
use esp32_battery_logic::error_log::EventLog;
use xy_modbus::SafetyLimits;

use crate::clock::{EventRecorder, uptime};
use crate::net::{LinkState, NetState, NetStatusHandle, ResetSignal, SubmissionStatus};
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

/// This board's pack: 4S LiFePO4, 50 Ah. Daily-cycle setpoints — 14.4 V
/// absorb / 13.5 V float. Currents derive from capacity via the `*_C`
/// constants in `charging`: 0.2C = 10 A CC, 0.06C = 3 A enter, 0.05C = 2.5 A
/// exit (manufacturer-standard tail). Single source of pack identity —
/// charge setpoints, hardware safety limits, and reported SoC all derive
/// from it.
pub(crate) const PACK_PROFILE: Profile = Profile::for_pack(Chemistry::LiIon, 4, 50.0);

/// Hard trip thresholds programmed into the XY's protection registers (OVP/OCP/LVP).
/// Derived from the profile so a chemistry/cell-count change moves them
/// in lockstep — no chance the OVP ceiling drifts below the absorb target.
/// Nominal DC input feeding the XY7025 buck. Drives the buck's input UVLO
/// (LVP register) — a board/supply property, not part of the pack profile.
pub(crate) const INPUT_NOMINAL_V: f32 = 24.0;
const _: () = assert!(INPUT_NOMINAL_V - INPUT_LVP_MARGIN_V > 12.0);

pub(crate) const SAFETY: SafetyLimits = PACK_PROFILE.safety_limits(INPUT_NOMINAL_V);

/// Quiet down the ESP-IDF C-side logger before any subsystem starts emitting.
/// Rust `log::` calls go through a separate path and aren't filtered here.
fn init_logging() {
    let logger = esp_idf_svc::log::init_from_esp_idf();
    // INFO chatter (handshake-per-poll from esp_https_server, wifi state
    // spam, etc.) on every ESP-IDF tag.
    logger
        .filter()
        .set_target_level("*", log::LevelFilter::Warn)
        .ok();
    // Per-connection TLS handshake failures: clients on the LAN that don't
    // have our Homelab CA installed reject the cert and send a fatal alert
    // (TLS alert 46 / certificate_unknown). Confirmed via mbedtls debug
    // logging. The 3-line E/E/W burst per failed connection drowns the log.
    for tag in ["esp-tls-mbedtls", "esp_https_server", "httpd"] {
        logger
            .filter()
            .set_target_level(tag, log::LevelFilter::Off)
            .ok();
    }
}

struct StaCtx {
    sensor_data: Arc<Mutex<SensorData>>,
    event_log: Arc<Mutex<EventLog>>,
    nvs: Arc<EspNvs<NvsDefault>>,
    reset: ResetSignal,
}

impl StaCtx {
    fn start_dashboard(&self) -> EspHttpServer<'static> {
        http::start_main(
            self.sensor_data.clone(),
            self.event_log.clone(),
            self.nvs.clone(),
            self.reset.clone(),
        )
    }
}

fn main() {
    esp_idf_svc::sys::link_patches();
    init_logging();

    log_ring::init();
    ota::init();
    task_wdt::init();

    let board = board::Board::take();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs_partition = EspDefaultNvsPartition::take().unwrap();

    let nvs = Arc::new(nvs_creds::open(nvs_partition.clone()));
    let boot_creds = nvs_creds::load(&nvs);

    let clock = clock::EspClock::new();

    let wifi = wifi::WifiDriver::new(board.modem, sysloop, nvs_partition);

    let sensor_data: Arc<Mutex<SensorData>> =
        Arc::new(Mutex::new(esp32_battery_logic::data::SensorData::new()));
    let event_log: Arc<Mutex<EventLog>> =
        Arc::new(Mutex::new(esp32_battery_logic::error_log::EventLog::new()));
    let net_status = NetStatusHandle::new();

    let _sntp = clock::start_sntp(clock.clone());

    let recorder = EventRecorder::new(event_log.clone(), clock.clone());

    // TEMP: simulate errors to verify webpage + LCD display. Remove before merging.
    // {
    //     use esp32_battery_logic::error_log::{Event, InaError, XyError};
    //     recorder.record(Event::Ina(InaError::BusVoltageRead));
    //     recorder.record(Event::Ina(InaError::CurrentRead));
    //     recorder.record(Event::Xy(XyError::ReadStatus));
    //     recorder.record(Event::Xy(XyError::SetVoltage));
    // }

    xy::start(board.xy, sensor_data.clone(), recorder.clone());
    ina::start(board.i2c, sensor_data.clone(), recorder);

    #[cfg(feature = "lcd")]
    lcd::start(
        board.lcd,
        sensor_data.clone(),
        event_log.clone(),
        net_status.clone(),
    );

    let reset = ResetSignal::new();
    let sta_ctx = StaCtx {
        sensor_data: sensor_data.clone(),
        event_log,
        nvs: nvs.clone(),
        reset,
    };

    // Bootstrap per `wifi_fsm.md`: STA if we have creds, captive otherwise.
    let mut state = match boot_creds {
        Some(creds) => {
            let sta_wifi = wifi.into_sta(&creds);
            let server = sta_ctx.start_dashboard();
            NetState::StaConnecting {
                wifi: sta_wifi,
                server,
                creds,
                session_start: uptime(),
            }
        }
        None => {
            let mixed = wifi.into_mixed(None);
            let bundle = http::start_captive(mixed.scan_cache());
            NetState::CaptiveIdle {
                wifi: mixed,
                bundle,
            }
        }
    };
    net_status.store(state.lcd_status());

    loop {
        thread::sleep(TICK_PERIOD);
        let now = uptime();
        sensor_data.lock().unwrap().tick(clock.epoch_s());

        if sta_ctx.reset.take() {
            state = force_captive_idle(state);
        }
        state = step(state, now, &sta_ctx);
        net_status.store(state.lcd_status());
    }
}

/// Drop the live STA association and return the FSM to `CaptiveIdle`.
/// Only reachable from the dashboard's `/wifi-reset`, which is mounted
/// only on the host server — captive states cannot raise the signal.
fn force_captive_idle(state: NetState) -> NetState {
    match state {
        NetState::StaConnecting { wifi, server, .. } => sta_to_captive_idle(wifi, server, None),
        NetState::StaServing {
            wifi, server, mdns, ..
        } => sta_to_captive_idle(wifi, server, Some(mdns)),
        _ => unreachable!("/wifi-reset only reachable from dashboard (Sta* states)"),
    }
}

fn sta_to_captive_idle(
    wifi: wifi::StaWifi<'static>,
    server: EspHttpServer<'static>,
    mdns: Option<esp_idf_svc::mdns::EspMdns>,
) -> NetState {
    drop(server);
    drop(mdns);
    let mixed = wifi.into_mixed(None);
    let bundle = http::start_captive(mixed.scan_cache());
    NetState::CaptiveIdle {
        wifi: mixed,
        bundle,
    }
}

/// One supervisor tick. Consumes `state` and returns the next variant.
fn step(state: NetState, now: Duration, ctx: &StaCtx) -> NetState {
    match state {
        NetState::CaptiveIdle { mut wifi, bundle } => {
            if let Some(creds) = bundle.take_creds() {
                return apply_submission(wifi, bundle, creds, now);
            }
            wifi.refresh_scan_if_stale(now);
            NetState::CaptiveIdle { wifi, bundle }
        }
        NetState::CaptiveTrying {
            wifi,
            bundle,
            creds,
            since,
        } => step_captive_trying(wifi, bundle, creds, since, now, ctx),
        NetState::CaptiveFallbackRetrying {
            mut wifi,
            bundle,
            creds,
        } => {
            // STA half is retrying the carry-over creds in the
            // background. Same ordering as CaptiveTrying: assoc-success
            // wins over a /save in the same tick.
            if wifi.try_connect() {
                return promote_to_serving(wifi, bundle, creds, ctx);
            }
            if let Some(new_creds) = bundle.take_creds() {
                return apply_submission(wifi, bundle, new_creds, now);
            }
            wifi.refresh_scan_if_stale(now);
            NetState::CaptiveFallbackRetrying {
                wifi,
                bundle,
                creds,
            }
        }
        NetState::StaConnecting {
            mut wifi,
            server,
            creds,
            session_start,
        } => {
            if wifi.try_connect() {
                return NetState::StaServing {
                    wifi,
                    server,
                    mdns: wifi::setup_mdns(),
                    creds,
                    link: LinkState::Up,
                };
            }
            if now.saturating_sub(session_start) >= CAPTIVE_AFTER_DISCONNECT {
                return fallback_to_captive(wifi, server, None, creds);
            }
            NetState::StaConnecting {
                wifi,
                server,
                creds,
                session_start,
            }
        }
        NetState::StaServing {
            mut wifi,
            server,
            mdns,
            creds,
            link,
        } => {
            let link = match (wifi.try_connect(), link) {
                (true, _) => LinkState::Up,
                (false, LinkState::Up) => LinkState::Down { since: now },
                (false, down) => down,
            };
            if let LinkState::Down { since } = link
                && now.saturating_sub(since) >= CAPTIVE_AFTER_DISCONNECT
            {
                return fallback_to_captive(wifi, server, Some(mdns), creds);
            }
            NetState::StaServing {
                wifi,
                server,
                mdns,
                creds,
                link,
            }
        }
    }
}

/// `CaptiveTrying` arm. Lifted into its own fn purely because the body
/// was the longest of the supervisor's match arms.
fn step_captive_trying(
    mut wifi: wifi::MixedWifi<'static>,
    bundle: net::CaptiveBundle,
    creds: WifiCredentials,
    since: Duration,
    now: Duration,
    ctx: &StaCtx,
) -> NetState {
    // Order matters: an associate-success on the in-flight creds wins
    // over a /save that arrived too late — otherwise we'd disconnect
    // from the network we just successfully joined.
    if wifi.try_connect() {
        return promote_to_serving(wifi, bundle, creds, ctx);
    }
    if let Some(new_creds) = bundle.take_creds() {
        return apply_submission(wifi, bundle, new_creds, now);
    }
    if now.saturating_sub(since) >= CAPTIVE_TRYING_TIMEOUT {
        bundle.set_status(SubmissionStatus::Failed);
        warn!("Captive: STA association timed out; flipping to Failed");
        return NetState::CaptiveIdle { wifi, bundle };
    }
    NetState::CaptiveTrying {
        wifi,
        bundle,
        creds,
        since,
    }
}

/// Apply freshly-submitted creds to the live radio and enter the 20 s
/// trying window.
fn apply_submission(
    mut wifi: wifi::MixedWifi<'static>,
    bundle: net::CaptiveBundle,
    creds: WifiCredentials,
    now: Duration,
) -> NetState {
    wifi.set_sta_creds(&creds);
    bundle.set_status(SubmissionStatus::Trying);
    NetState::CaptiveTrying {
        wifi,
        bundle,
        creds,
        since: now,
    }
}

/// Captive → STA promotion. Persist creds to NVS, drop the captive
/// bundle (joins DNS thread, stops captive HTTP), switch the radio to
/// STA-only, start the dashboard server + mDNS.
fn promote_to_serving(
    wifi: wifi::MixedWifi<'static>,
    bundle: net::CaptiveBundle,
    creds: WifiCredentials,
    ctx: &StaCtx,
) -> NetState {
    nvs_creds::save(&ctx.nvs, &creds);
    // Publish "connected" and linger so the captive page's 1 Hz /status
    // poll can read it before the AP disappears with the bundle.
    bundle.set_status(SubmissionStatus::Connected);
    thread::sleep(Duration::from_millis(1500));
    drop(bundle);
    let sta_wifi = wifi.into_sta(&creds);
    let server = ctx.start_dashboard();
    let mdns = wifi::setup_mdns();
    NetState::StaServing {
        wifi: sta_wifi,
        server,
        mdns,
        creds,
        link: LinkState::Up,
    }
}

/// STA → Captive fallback. Drops the dashboard, switches the radio to
/// Mixed (carrying creds so the STA half keeps retrying), brings up the
/// captive bundle.
fn fallback_to_captive(
    wifi: wifi::StaWifi<'static>,
    server: EspHttpServer<'static>,
    mdns: Option<esp_idf_svc::mdns::EspMdns>,
    creds: WifiCredentials,
) -> NetState {
    drop(server);
    drop(mdns);
    let mixed = wifi.into_mixed(Some(&creds));
    let bundle = http::start_captive(mixed.scan_cache());
    NetState::CaptiveFallbackRetrying {
        wifi: mixed,
        bundle,
        creds,
    }
}
