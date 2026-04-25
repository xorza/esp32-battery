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
use std::sync::{Arc, Mutex};
use std::time::Duration;

const HOSTNAME: &str = "battery-esp32";
const HTTP_PORT: u16 = 80;
pub const AP_SSID: &str = "Battery-Setup";
pub const AP_PASS: &str = "01010101";
pub const AP_GATEWAY: [u8; 4] = [192, 168, 71, 1];
const MAX_SCAN_APS: usize = 10;
/// Max staleness before `refresh_scan_if_stale` runs another `scan_n`.
/// The supervisor pays the cost (during its captive-arm tick); `/scan`
/// reads only the cache and returns instantly.
const SCAN_CACHE_TTL: Duration = Duration::from_secs(10);

pub type ScanResult = heapless::Vec<(heapless::String<32>, i8), MAX_SCAN_APS>;

/// Cache of the most recent scan, refreshed by the supervisor and read
/// (without touching the `Wifi` mutex) by the `/scan` handler.
/// `at == None` means "never scanned this session" — the next supervisor
/// tick refreshes immediately.
pub struct CachedScan {
    pub at: Option<Duration>,
    pub entries: ScanResult,
}

pub type ScanCache = Arc<Mutex<CachedScan>>;

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

fn ap_config() -> AccessPointConfiguration {
    AccessPointConfiguration {
        ssid: AP_SSID.try_into().unwrap(),
        password: AP_PASS.try_into().unwrap(),
        auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
        channel: 1,
        max_connections: 4,
        ..Default::default()
    }
}

pub struct Wifi<'d> {
    wifi: BlockingWifi<EspWifi<'d>>,
    scan_cache: ScanCache,
}

/// Take and configure the mDNS responder for the dashboard. Owned by
/// `Net::Sta` (the dashboard's lifecycle is the mDNS responder's
/// lifecycle); dropping it on `Sta → Captive` lets a later promote
/// `EspMdns::take()` again.
pub fn setup_mdns() -> EspMdns {
    let mut mdns = EspMdns::take().unwrap();
    mdns.set_hostname(HOSTNAME).unwrap();
    mdns.set_instance_name(HOSTNAME).unwrap();
    mdns.add_service(None, "_http", "_tcp", HTTP_PORT, &[])
        .unwrap();
    info!("mDNS: {}.local", HOSTNAME);
    mdns
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
            scan_cache: Arc::new(Mutex::new(CachedScan {
                at: None,
                entries: ScanResult::new(),
            })),
        }
    }

    /// Handle to the shared scan cache. The captive `/scan` handler reads
    /// it without locking the `Wifi` mutex, so a long `scan_n` cannot
    /// stall the supervisor's per-second tick (and vice versa).
    pub fn scan_cache(&self) -> ScanCache {
        self.scan_cache.clone()
    }

    /// Re-run `scan_n` if the cached result is older than `SCAN_CACHE_TTL`.
    /// Caller decides when scanning is safe — currently only the captive
    /// arm of the supervisor, when STA isn't mid-association (scanning
    /// disrupts an in-flight associate).
    pub fn refresh_scan_if_stale(&mut self, now: Duration) {
        let stale = {
            let c = self.scan_cache.lock().unwrap();
            c.at.is_none_or(|t| now.saturating_sub(t) >= SCAN_CACHE_TTL)
        };
        if !stale {
            return;
        }
        let entries = self.scan_now();
        let mut c = self.scan_cache.lock().unwrap();
        c.at = Some(now);
        c.entries = entries;
    }

    fn start_with(&mut self, config: Configuration) {
        // stop() errors when the driver isn't running; ignore — start()
        // is what matters and will surface a real failure.
        let _ = self.wifi.stop();
        self.wifi.set_configuration(&config).unwrap();
        self.wifi.start().unwrap();
    }

    /// Switch to STA-only. Tears down the AP — used after the user's
    /// creds successfully associate, so the device runs in client mode
    /// for normal operation.
    pub fn start_sta(&mut self, creds: &WifiCredentials) {
        info!("Starting WiFi STA for '{}'", creds.ssid);
        self.start_with(Configuration::Client(sta_config(creds)));
    }

    /// Update STA credentials in the running mixed (AP+STA) mode without
    /// stopping the radio — so the captive AP stays associated with the
    /// user's phone while the STA half retries against the new SSID.
    pub fn set_sta_creds_live(&mut self, creds: &WifiCredentials) {
        info!("Updating STA creds for '{}' (live)", creds.ssid);
        self.wifi
            .set_configuration(&Configuration::Mixed(sta_config(creds), ap_config()))
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
        // Always use Mixed mode so the STA interface is available for WiFi scanning.
        let sta = creds.map_or_else(ClientConfiguration::default, sta_config);
        self.start_with(Configuration::Mixed(sta, ap_config()));
        info!("AP started");
    }

    pub fn is_connected(&self) -> bool {
        self.wifi.is_connected().unwrap_or(false)
    }

    /// Single connect attempt. Returns post-attempt connection state.
    /// Caller invokes only when STA creds are configured (so the SSID
    /// in `Configuration` isn't empty); waits for the netif so a `true`
    /// return means downstream binds (mDNS, HTTP) can run immediately.
    pub fn try_connect(&mut self) -> bool {
        if self.is_connected() {
            return true;
        }
        if self.wifi.connect().is_ok() {
            let _ = self.wifi.wait_netif_up();
        }
        self.is_connected()
    }

    /// Scan for visible access points, deduplicated by SSID (strongest signal kept),
    /// sorted by signal strength descending. Private — callers go through
    /// `refresh_scan_if_stale` so the cost is paid at most once per
    /// `SCAN_CACHE_TTL`.
    fn scan_now(&mut self) -> ScanResult {
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
}
