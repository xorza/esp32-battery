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
    AccessPointConfiguration, BlockingWifi, ClientConfiguration, Configuration, EspWifi,
    WifiDriver as EspWifiDriver,
};
use log::{info, warn};

use crate::nvs_creds::WifiCredentials;

use std::net::Ipv4Addr;
use std::sync::{Arc, Mutex};
use std::time::Duration;

pub const HOSTNAME: &str = "battery-esp32";
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
/// (without touching the radio) by the `/scan` handler. `at == None`
/// means "never scanned this session" — the next supervisor tick refreshes
/// immediately.
pub struct CachedScan {
    pub at: Option<Duration>,
    pub entries: ScanResult,
}

pub type ScanCache = Arc<Mutex<CachedScan>>;

/// Current STA RSSI in dBm, or 0 when not associated. Reads the live AP
/// record via `esp_wifi_sta_get_ap_info`; the call is cheap and doesn't
/// require a `Wifi` handle (esp-wifi keeps this state in a global), so
/// the dashboard's `/api` handler can read it without locking the radio.
pub fn sta_rssi() -> i32 {
    let mut ap_info: esp_idf_svc::sys::wifi_ap_record_t = unsafe { std::mem::zeroed() };
    if unsafe { esp_idf_svc::sys::esp_wifi_sta_get_ap_info(&mut ap_info) } == 0 {
        ap_info.rssi as i32
    } else {
        0
    }
}

/// Current STA IPv4 address, or `None` when not associated / DHCP not
/// yet complete. Reads via the global `esp_netif` registry (key
/// `WIFI_STA_DEF`) — same "no `Wifi` handle needed" pattern as
/// `sta_rssi`, so the LCD thread can poll it without touching the
/// supervisor's state.
#[allow(dead_code)] // consumed by the lcd thread
pub fn sta_ip() -> Option<std::net::Ipv4Addr> {
    let netif =
        unsafe { esp_idf_svc::sys::esp_netif_get_handle_from_ifkey(c"WIFI_STA_DEF".as_ptr()) };
    if netif.is_null() {
        return None;
    }
    let mut info: esp_idf_svc::sys::esp_netif_ip_info_t = unsafe { std::mem::zeroed() };
    if unsafe { esp_idf_svc::sys::esp_netif_get_ip_info(netif, &mut info) } != 0 {
        return None;
    }
    if info.ip.addr == 0 {
        return None;
    }
    // esp_ip4_addr_t::addr is the 4 octets packed little-endian (net byte
    // order), so to_le_bytes() yields [a, b, c, d] in dotted-quad order.
    let [a, b, c, d] = info.ip.addr.to_le_bytes();
    Some(std::net::Ipv4Addr::new(a, b, c, d))
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

/// Raw radio driver. Created once at boot and immediately consumed into
/// either `StaWifi` or `MixedWifi` via `into_sta` / `into_mixed`. Mode
/// switches consume the current mode wrapper and produce the other one,
/// so the type system enforces "the radio is in mode X" at every call
/// site instead of relying on a documented call ordering.
pub struct WifiDriver<'d> {
    wifi: BlockingWifi<EspWifi<'d>>,
}

impl<'d> WifiDriver<'d> {
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
                EspWifiDriver::new(modem, sysloop.clone(), Some(nvs)).unwrap(),
                sta_netif,
                ap_netif,
            )
            .unwrap(),
            sysloop,
        )
        .unwrap();

        Self { wifi }
    }

    fn start_with(&mut self, config: Configuration) {
        // stop() errors when the driver isn't running; ignore — start()
        // is what matters and will surface a real failure.
        let _ = self.wifi.stop();
        self.wifi.set_configuration(&config).unwrap();
        self.wifi.start().unwrap();
    }

    pub fn into_sta(mut self, creds: &WifiCredentials) -> StaWifi<'d> {
        info!("Starting WiFi STA for '{}'", creds.ssid);
        self.start_with(Configuration::Client(sta_config(creds)));
        StaWifi { driver: self }
    }

    pub fn into_mixed(mut self, creds: Option<&WifiCredentials>) -> MixedWifi<'d> {
        info!("Starting AP: {}", AP_SSID);
        // Always Mixed mode so the STA interface is available for scanning.
        let sta = creds.map_or_else(ClientConfiguration::default, sta_config);
        self.start_with(Configuration::Mixed(sta, ap_config()));
        info!("AP started");
        MixedWifi {
            driver: self,
            scan_cache: Arc::new(Mutex::new(CachedScan {
                at: None,
                entries: ScanResult::new(),
            })),
            sta_configured: creds.is_some(),
        }
    }
}

/// STA-only mode: dashboard up, AP torn down. The only operation the
/// supervisor needs is "did we associate this tick" — exposed as
/// `try_connect`. To fall back to the captive AP, consume into `MixedWifi`.
pub struct StaWifi<'d> {
    driver: WifiDriver<'d>,
}

impl<'d> StaWifi<'d> {
    /// Single connect attempt. Returns post-attempt connection state.
    /// Waits for the netif so a `true` return means downstream binds
    /// (mDNS, HTTP) can run immediately.
    pub fn try_connect(&mut self) -> bool {
        if self.driver.wifi.is_connected().unwrap_or(false) {
            return true;
        }
        if self.driver.wifi.connect().is_ok() {
            let _ = self.driver.wifi.wait_netif_up();
        }
        self.driver.wifi.is_connected().unwrap_or(false)
    }

    pub fn into_mixed(self, creds: Option<&WifiCredentials>) -> MixedWifi<'d> {
        self.driver.into_mixed(creds)
    }
}

/// Mixed AP+STA mode: captive portal + STA half retrying. Carries the
/// scan cache (only sane in mixed mode — STA-only scan would disrupt the
/// dashboard) and a `sta_configured` flag so `try_connect` can short-
/// circuit before any creds were ever supplied (avoids per-tick
/// "connecting to ''" log spam at cold boot).
pub struct MixedWifi<'d> {
    driver: WifiDriver<'d>,
    scan_cache: ScanCache,
    sta_configured: bool,
}

impl<'d> MixedWifi<'d> {
    /// Handle to the shared scan cache. The captive `/scan` handler reads
    /// it without touching the radio, so a long `scan_n` cannot stall the
    /// supervisor's per-second tick (and vice versa).
    pub fn scan_cache(&self) -> ScanCache {
        self.scan_cache.clone()
    }

    /// Update STA credentials in the running mixed mode without stopping
    /// the radio — the captive AP stays associated with the user's phone
    /// while the STA half retries against the new SSID.
    pub fn set_sta_creds(&mut self, creds: &WifiCredentials) {
        info!("Updating STA creds for '{}' (live)", creds.ssid);
        self.driver
            .wifi
            .set_configuration(&Configuration::Mixed(sta_config(creds), ap_config()))
            .unwrap();
        // Drop the old (failing) association attempt; kick off a fresh
        // connect against the new creds. Errors are non-fatal — the
        // supervisor's per-tick reconnect retries on its own.
        let _ = self.driver.wifi.disconnect();
        let _ = self.driver.wifi.connect();
        self.sta_configured = true;
    }

    /// Returns post-attempt connection state. Until creds are configured,
    /// this is just a read of `is_connected()` — connecting to an empty
    /// SSID logs an error per second otherwise.
    pub fn try_connect(&mut self) -> bool {
        if !self.sta_configured {
            return self.driver.wifi.is_connected().unwrap_or(false);
        }
        if self.driver.wifi.is_connected().unwrap_or(false) {
            return true;
        }
        if self.driver.wifi.connect().is_ok() {
            let _ = self.driver.wifi.wait_netif_up();
        }
        self.driver.wifi.is_connected().unwrap_or(false)
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

    pub fn into_sta(self, creds: &WifiCredentials) -> StaWifi<'d> {
        self.driver.into_sta(creds)
    }

    /// Scan for visible APs, deduplicated by SSID (strongest signal kept),
    /// sorted by signal strength descending. Private — callers go through
    /// `refresh_scan_if_stale` so the cost is paid at most once per
    /// `SCAN_CACHE_TTL`.
    fn scan_now(&mut self) -> ScanResult {
        let mut entries = ScanResult::new();

        match self.driver.wifi.scan_n::<MAX_SCAN_APS>() {
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
