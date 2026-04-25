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

pub struct Wifi<'d> {
    wifi: BlockingWifi<EspWifi<'d>>,
    mdns: Option<EspMdns>,
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

        Self { wifi, mdns: None }
    }

    fn start_with(&mut self, config: Configuration) {
        // stop() errors when the driver isn't running; ignore — start()
        // is what matters and will surface a real failure.
        let _ = self.wifi.stop();
        self.wifi.set_configuration(&config).unwrap();
        self.wifi.start().unwrap();
    }

    pub fn start_sta(&mut self, creds: &WifiCredentials) {
        info!("Starting WiFi STA for '{}'", creds.ssid);
        self.start_with(Configuration::Client(sta_config(creds)));
    }

    /// Update STA credentials in the running mixed (AP+STA) mode without
    /// stopping the radio — so the captive AP stays associated with the
    /// user's phone while the STA half retries against the new SSID. The
    /// supervisor only calls this from the captive arm; no state-machine
    /// guard inside `Wifi`.
    pub fn set_sta_creds_live(&mut self, creds: &WifiCredentials) {
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
        info!("AP started");
    }

    pub fn is_connected(&self) -> bool {
        self.wifi.is_connected().unwrap_or(false)
    }

    /// Supervisor tick: when STA creds are configured and we're not
    /// associated, attempt a reconnect. Returns whether we are associated
    /// post-attempt. Caller passes `has_sta_creds` because credential
    /// presence is supervisor-owned state (in `main`'s `creds: Option`).
    pub fn tick(&mut self, has_sta_creds: bool) -> bool {
        if has_sta_creds && !self.is_connected() && self.wifi.connect().is_ok() {
            let _ = self.wifi.wait_netif_up();
            self.setup_mdns();
        }
        self.is_connected()
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
