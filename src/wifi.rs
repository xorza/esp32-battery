use esp_idf_hal::modem::WifiModemPeripheral;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::ipv4::{
    ClientConfiguration as IpClientConfiguration, Configuration as IpConfiguration,
    DHCPClientSettings, Mask, RouterConfiguration, Subnet,
};
use esp_idf_svc::mdns::EspMdns;
use esp_idf_svc::netif::{EspNetif, NetifConfiguration};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{
    AccessPointConfiguration, BlockingWifi, ClientConfiguration, Configuration, EspWifi, WifiDriver,
};
use log::{info, warn};

use crate::nvs_creds::WifiCredentials;

use std::net::Ipv4Addr;

const HOSTNAME: &str = "battery-esp32";
const HTTP_PORT: u16 = 80;
pub const AP_SSID: &str = "Battery-Setup";
pub const AP_PASS: &str = "01010101";
pub const AP_GATEWAY: [u8; 4] = [192, 168, 71, 1];
const MAX_SCAN_APS: usize = 10;

pub type ScanResult = heapless::Vec<(heapless::String<32>, i8), MAX_SCAN_APS>;

#[derive(Copy, Clone, PartialEq, Eq)]
pub enum LinkState {
    /// No STA credentials configured — captive portal needs to collect them.
    NoCreds,
    /// Have creds but not currently associated — reconnect is worth attempting.
    Disassociated,
    /// STA is associated; the host server can run.
    Associated,
}

/// Current STA RSSI in dBm, or 0 when not associated. Reads the live AP
/// record via `esp_wifi_sta_get_ap_info`; the call is cheap and doesn't
/// require a `Wifi` handle.
pub fn sta_rssi() -> i32 {
    let mut ap_info: esp_idf_svc::sys::wifi_ap_record_t = unsafe { std::mem::zeroed() };
    if unsafe { esp_idf_svc::sys::esp_wifi_sta_get_ap_info(&mut ap_info) } == 0 {
        ap_info.rssi as i32
    } else {
        0
    }
}

fn sta_config(creds: &WifiCredentials) -> ClientConfiguration {
    ClientConfiguration {
        ssid: creds.ssid.as_str().try_into().unwrap(),
        password: creds.password.as_str().try_into().unwrap(),
        ..Default::default()
    }
}

/// Tracks whether the driver is started and whether we have STA credentials
/// configured. The four reachable combinations are Idle (both false), Sta
/// (started + has creds), or ApMixed (started, optional creds for the
/// embedded STA half). The pre-F4 representation used two independent
/// `bool`s, which had two unreachable shapes.
enum Mode {
    Idle,
    Sta,
    ApMixed { has_sta_creds: bool },
}

impl Mode {
    fn started(&self) -> bool {
        !matches!(self, Mode::Idle)
    }

    fn has_sta_creds(&self) -> bool {
        matches!(
            self,
            Mode::Sta
                | Mode::ApMixed {
                    has_sta_creds: true
                }
        )
    }
}

pub struct Wifi<'d> {
    wifi: BlockingWifi<EspWifi<'d>>,
    mdns: Option<EspMdns>,
    mode: Mode,
}

impl<'d> Wifi<'d> {
    pub fn new(
        modem: impl WifiModemPeripheral + 'd,
        sysloop: EspSystemEventLoop,
        nvs: EspDefaultNvsPartition,
    ) -> Self {
        let sta_netif = EspNetif::new_with_conf(&NetifConfiguration {
            ip_configuration: Some(IpConfiguration::Client(IpClientConfiguration::DHCP(
                DHCPClientSettings {
                    hostname: Some(HOSTNAME.try_into().unwrap()),
                },
            ))),
            ..NetifConfiguration::wifi_default_client()
        })
        .unwrap();

        let ap_netif = EspNetif::new_with_conf(&NetifConfiguration {
            ip_configuration: Some(IpConfiguration::Router(RouterConfiguration {
                subnet: Subnet {
                    gateway: Ipv4Addr::from(AP_GATEWAY),
                    mask: Mask(24),
                },
                dhcp_enabled: true,
                dns: Some(Ipv4Addr::from(AP_GATEWAY)),
                secondary_dns: None,
            })),
            ..NetifConfiguration::wifi_default_router()
        })
        .unwrap();

        let wifi = BlockingWifi::wrap(
            EspWifi::wrap_all(
                WifiDriver::new(modem, sysloop.clone(), Some(nvs)).unwrap(),
                sta_netif,
                ap_netif,
            )
            .unwrap(),
            sysloop,
        )
        .unwrap();

        Self {
            wifi,
            mdns: None,
            mode: Mode::Idle,
        }
    }

    fn start_with(&mut self, config: Configuration) {
        if self.mode.started() {
            let _ = self.wifi.stop();
        }
        self.wifi.set_configuration(&config).unwrap();
        self.wifi.start().unwrap();
    }

    pub fn start_sta(&mut self, creds: &WifiCredentials) {
        info!("Starting WiFi STA for '{}'", creds.ssid);
        self.start_with(Configuration::Client(sta_config(creds)));
        self.mode = Mode::Sta;
    }

    /// Update STA credentials in the running mixed (AP+STA) mode without
    /// stopping the radio — so the captive AP stays associated with the
    /// user's phone while the STA half retries against the new SSID. Should
    /// only be called while already in `ApMixed` mode; panics otherwise.
    pub fn set_sta_creds_live(&mut self, creds: &WifiCredentials) {
        assert!(
            matches!(self.mode, Mode::ApMixed { .. }),
            "set_sta_creds_live requires ApMixed mode"
        );
        info!("Updating STA creds for '{}' (live)", creds.ssid);
        let ap = AccessPointConfiguration {
            ssid: AP_SSID.try_into().unwrap(),
            password: AP_PASS.try_into().unwrap(),
            auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
            channel: 1,
            max_connections: 4,
            ..Default::default()
        };
        let sta = sta_config(creds);
        self.wifi
            .set_configuration(&Configuration::Mixed(sta, ap))
            .unwrap();
        self.mode = Mode::ApMixed {
            has_sta_creds: true,
        };
        // Drop the old (failing) association attempt; kick off a fresh
        // connect against the new creds. Errors are non-fatal — the
        // supervisor's per-tick reconnect will retry on its own.
        let _ = self.wifi.disconnect();
        let _ = self.wifi.connect();
    }

    /// Switch to mixed AP+STA mode. AP serves captive portal, STA keeps retrying.
    pub fn start_ap_mixed(&mut self, creds: Option<&WifiCredentials>) {
        info!("Starting AP: {}", AP_SSID);

        let ap = AccessPointConfiguration {
            ssid: AP_SSID.try_into().unwrap(),
            password: AP_PASS.try_into().unwrap(),
            auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
            channel: 1,
            max_connections: 4,
            ..Default::default()
        };

        // Always use Mixed mode so the STA interface is available for WiFi scanning.
        let sta = creds.map_or_else(ClientConfiguration::default, sta_config);
        self.start_with(Configuration::Mixed(sta, ap));
        self.mode = Mode::ApMixed {
            has_sta_creds: creds.is_some(),
        };
        info!("AP started");
    }

    /// Distinct STA states the supervisor reasons about. `NoCreds` and
    /// `Disassociated` both mean "not currently routable", but they take
    /// different recovery paths — only `Disassociated` is worth retrying.
    pub fn link_state(&self) -> LinkState {
        if !self.mode.has_sta_creds() {
            LinkState::NoCreds
        } else if self.wifi.is_connected().unwrap_or(false) {
            LinkState::Associated
        } else {
            LinkState::Disassociated
        }
    }

    fn try_reconnect(&mut self) {
        if self.link_state() != LinkState::Disassociated {
            return;
        }
        if self.wifi.connect().is_ok() {
            let _ = self.wifi.wait_netif_up();
            self.setup_mdns();
        }
    }

    /// Supervisor tick: try reconnect (no-op unless `Disassociated`) and
    /// return the post-reconnect link state.
    pub fn tick(&mut self) -> LinkState {
        self.try_reconnect();
        self.link_state()
    }

    /// Scan for visible access points, deduplicated by SSID (strongest signal kept),
    /// sorted by signal strength descending.
    pub fn scan(&mut self) -> ScanResult {
        let mut entries = ScanResult::new();

        match self.wifi.scan_n::<MAX_SCAN_APS>() {
            Err(e) => {
                warn!("WiFi scan failed: {:?}", e);
            }
            Ok((aps, _)) => {
                for ap in &aps {
                    if ap.ssid.is_empty() {
                        continue;
                    }
                    if let Some(existing) = entries.iter_mut().find(|e| e.0 == ap.ssid) {
                        if ap.signal_strength > existing.1 {
                            existing.1 = ap.signal_strength;
                        }
                    } else {
                        let _ = entries.push((ap.ssid.clone(), ap.signal_strength));
                    }
                }
                entries.sort_unstable_by_key(|e| std::cmp::Reverse(e.1));
            }
        }

        entries
    }

    fn setup_mdns(&mut self) {
        if self.mdns.is_some() {
            return;
        }
        let mut mdns = EspMdns::take().unwrap();
        mdns.set_hostname(HOSTNAME).unwrap();
        mdns.set_instance_name(HOSTNAME).unwrap();
        mdns.add_service(None, "_http", "_tcp", HTTP_PORT, &[])
            .unwrap();
        info!("mDNS: {}.local", HOSTNAME);
        self.mdns = Some(mdns);
    }
}
