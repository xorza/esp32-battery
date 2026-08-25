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

use esp32_battery_logic::Chemistry;
use esp32_battery_logic::EventLog;
use esp32_battery_logic::SensorData;
use esp32_battery_logic::{INPUT_LVP_MARGIN_V, Profile};
use xy_modbus::SafetyLimits;

use esp32_battery_logic::{NetAction, NetPhase, NetPoll, NetSupervisor, WifiCredentials};

use crate::clock::{EventRecorder, LoopTimer, uptime};
use crate::net::{NetResources, NetStatusHandle, ResetSignal, SubmissionStatus};

/// Supervisor loop period.
const TICK_PERIOD: Duration = Duration::from_secs(1);

/// This board's pack: 4S LiFePO4, 50 Ah. Daily-cycle setpoints — 14.4 V
/// absorb / 13.5 V float. Currents derive from capacity via the `*_C`
/// constants in `charging`: 0.2C = 10 A CC, 0.06C = 3 A enter, 0.05C = 2.5 A
/// exit (manufacturer-standard tail). Single source of pack identity —
/// charge setpoints, hardware safety limits, and reported SoC all derive
/// from it.
pub(crate) const PACK_PROFILE: Profile = Profile::for_pack(Chemistry::LiFePo4, 4, 50.0);
// pub(crate) const PACK_PROFILE: Profile = Profile::for_pack(Chemistry::LiIon, 3, 17.0);

/// Hard trip thresholds programmed into the XY's protection registers (OVP/OCP/LVP).
/// Derived from the profile so a chemistry/cell-count change moves them
/// in lockstep — no chance the OVP ceiling drifts below the absorb target.
/// Nominal DC input feeding the XY7025 buck. Drives the buck's input UVLO
/// (LVP register) — a board/supply property, not part of the pack profile.
// pub(crate) const INPUT_NOMINAL_V: f32 = 19.0;
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
        Arc::new(Mutex::new(esp32_battery_logic::SensorData::new()));
    let event_log: Arc<Mutex<EventLog>> =
        Arc::new(Mutex::new(esp32_battery_logic::EventLog::new()));
    let net_status = NetStatusHandle::new();

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

    let reset = ResetSignal::new();
    let sta_ctx = StaCtx {
        sensor_data: sensor_data.clone(),
        event_log,
        nvs: nvs.clone(),
        reset,
    };

    // Bootstrap per `net_fsm.md`: STA if we have creds, captive otherwise.
    let mut supervisor = NetSupervisor::new(boot_creds, uptime());
    let mut resources = match supervisor.phase() {
        NetPhase::StaConnecting { creds, .. } => NetResources::Sta {
            wifi: wifi.into_sta(creds),
            server: sta_ctx.start_dashboard(),
            mdns: None,
        },
        _ => {
            let mixed = wifi.into_mixed(None);
            let bundle = http::start_captive(mixed.scan_cache());
            NetResources::Mixed {
                wifi: mixed,
                bundle,
            }
        }
    };
    net_status.store(supervisor.phase().lcd_status());

    // `try_connect` below can block for seconds on a slow association, so an
    // iteration is not `TICK_PERIOD` long and the staleness clocks are charged
    // the measured interval instead.
    let mut timer = LoopTimer::start();
    loop {
        thread::sleep(TICK_PERIOD);
        let elapsed = timer.lap();
        let now = timer.now();
        sensor_data.lock().unwrap().tick(clock.epoch_s(), elapsed);

        let poll = NetPoll {
            now,
            associated: resources.try_connect(supervisor.phase()),
            submitted: resources.take_creds(),
            reset_requested: sta_ctx.reset.take(),
        };
        let action = supervisor.tick(poll);
        resources = apply_net_action(action, resources, now, &sta_ctx);
        net_status.store(supervisor.phase().lcd_status());
    }
}

/// Carry out one `NetAction` against the resources the firmware owns. The
/// supervisor has already moved; this only makes the radio and the
/// servers match where it went.
fn apply_net_action(
    action: NetAction,
    mut resources: NetResources,
    now: Duration,
    ctx: &StaCtx,
) -> NetResources {
    match action {
        NetAction::Nothing => resources,
        NetAction::RefreshScan => {
            resources.refresh_scan_if_stale(now);
            resources
        }
        NetAction::ApplyCreds(creds) => {
            match &mut resources {
                NetResources::Mixed { wifi, bundle } => {
                    wifi.set_sta_creds(&creds);
                    bundle.set_status(SubmissionStatus::Trying);
                }
                other => other.warn_out_of_step("ApplyCreds"),
            }
            resources
        }
        NetAction::MarkSubmissionFailed => {
            warn!("Captive: STA association timed out; flipping to Failed");
            match &mut resources {
                NetResources::Mixed { wifi, bundle } => {
                    // The phase drops the credentials on this transition; the
                    // radio has to as well, or the STA half keeps retrying a
                    // pair the supervisor no longer knows about.
                    wifi.clear_sta_creds();
                    bundle.set_status(SubmissionStatus::Failed);
                }
                other => other.warn_out_of_step("MarkSubmissionFailed"),
            }
            resources
        }
        NetAction::StartMdns => {
            match &mut resources {
                NetResources::Sta { mdns, .. } => *mdns = Some(wifi::setup_mdns()),
                other => other.warn_out_of_step("StartMdns"),
            }
            resources
        }
        NetAction::PromoteToSta(creds) => promote_to_sta(resources, ctx, &creds),
        NetAction::FallbackToCaptive(creds) => sta_to_captive(resources, Some(&creds)),
        NetAction::ForceCaptive => sta_to_captive(resources, None),
    }
}

/// Captive → STA. Persist the credentials that just worked, let the
/// captive page see `Connected`, then drop the bundle (which joins the
/// DNS thread and stops the captive HTTP server), switch the radio to
/// STA-only, and bring the dashboard and mDNS up.
fn promote_to_sta(resources: NetResources, ctx: &StaCtx, creds: &WifiCredentials) -> NetResources {
    let NetResources::Mixed { wifi, bundle } = resources else {
        resources.warn_out_of_step("PromoteToSta");
        return resources;
    };
    nvs_creds::save(&ctx.nvs, creds);
    // Linger so the captive page's 1 Hz /status poll picks up `Connected`
    // before the AP disappears along with the bundle.
    bundle.set_status(SubmissionStatus::Connected);
    thread::sleep(Duration::from_millis(1500));
    drop(bundle);
    NetResources::Sta {
        wifi: wifi.into_sta(creds),
        server: ctx.start_dashboard(),
        mdns: Some(wifi::setup_mdns()),
    }
}

/// STA → captive. Drops the dashboard and mDNS, switches the radio to
/// Mixed, and mounts the captive bundle. `creds` are carried onto the
/// radio so the STA half keeps retrying in the background; `None` leaves
/// it bare, which is what `/wifi-reset` wants.
fn sta_to_captive(resources: NetResources, creds: Option<&WifiCredentials>) -> NetResources {
    let NetResources::Sta { wifi, server, mdns } = resources else {
        resources.warn_out_of_step("captive fallback");
        return resources;
    };
    drop(server);
    drop(mdns);
    let mixed = wifi.into_mixed(creds);
    let bundle = http::start_captive(mixed.scan_cache());
    NetResources::Mixed {
        wifi: mixed,
        bundle,
    }
}
