//! The radio: the two mode wrappers, the setup AP, and the scan cache.
//!
//! `WifiDriver` is built once at boot and immediately consumed into either
//! `StaWifi` (dashboard up, AP torn down) or `MixedWifi` (captive portal up,
//! STA half retrying behind it). A mode switch consumes one wrapper and
//! produces the other, so "the radio is in mode X" is something the caller
//! holds rather than a call order it has to remember.
//!
//! The two free functions at the top read live radio state out of esp-idf's
//! own globals, which needs no `Wifi` handle — that is what lets `/api` and
//! the LCD thread read RSSI and IP without contending for the supervisor's
//! radio.

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

use esp32_battery_logic::WifiCredentials;

use core::fmt::Write as _;
use std::net::Ipv4Addr;
use std::sync::{Arc, LazyLock, Mutex};
use std::time::Duration;

pub const HOSTNAME: &str = "battery";
const HTTP_PORT: u16 = 80;
pub const AP_SSID: &str = "Battery-Setup";
/// Setup-AP passphrase, derived per unit rather than written down here — a
/// constant would let anyone who has read this repository onto any unit's
/// setup AP for as long as one is up.
///
/// `HMAC(ota key, "ap-pass" ‖ SoftAP MAC)`, rendered as 12 hex characters.
/// The MAC alone would buy nothing, since it is the BSSID and goes out in
/// every beacon; the build-time OTA key is what makes the result unguessable
/// from radio range. 48 bits is far past reach for an AP that is only up
/// while the unit has no working credentials.
///
/// The owner reads it off the LCD's WiFi Setup screen, or out of the boot log
/// on a build with no panel. That second route also puts it in the `/api/log`
/// ring — which is reachable only from the dashboard, i.e. only once the unit
/// is already on the network this passphrase exists to get it onto.
pub static AP_PASS: LazyLock<heapless::String<12>> = LazyLock::new(|| {
    let mut mac = [0u8; 6];
    // WIFI_SOFTAP: the address this AP will actually beacon under, so the
    // passphrase is tied to the interface it protects.
    let err = unsafe {
        esp_idf_svc::sys::esp_read_mac(
            mac.as_mut_ptr(),
            esp_idf_svc::sys::esp_mac_type_t_ESP_MAC_WIFI_SOFTAP,
        )
    };
    assert_eq!(err, 0, "esp_read_mac(WIFI_SOFTAP) failed");

    let subkey = crate::ota::derive_subkey("ap-pass", &mac);
    let mut pass = heapless::String::new();
    for byte in &subkey[..6] {
        write!(pass, "{byte:02x}").expect("12 hex chars fit a String<12>");
    }
    pass
});
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

fn log_got_ip() {
    match sta_ip() {
        Some(ip) => info!("STA up: got IP {ip}"),
        None => info!("STA up: netif ready (no IP yet)"),
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
        password: AP_PASS.as_str().try_into().unwrap(),
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

/// Raw radio driver, before a mode has been chosen. Holds the netif pair —
/// DHCP client for STA, router with the AP subnet for the AP half — so both
/// modes are configured the same way whichever one it is consumed into.
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

    /// Mixed AP+STA, optionally carrying STA credentials. Built here so the
    /// cold start (`into_mixed`) and the live swap (`apply_sta_config`) can
    /// never disagree about the AP half.
    fn mixed_config(creds: Option<&WifiCredentials>) -> Configuration {
        let sta = creds.map_or_else(ClientConfiguration::default, sta_config);
        Configuration::Mixed(sta, ap_config())
    }

    /// One association attempt, returning the state afterwards. Waits for the
    /// netif so a `true` return means downstream binds (mDNS, HTTP) can run
    /// immediately. Callers gate on `NetPhase::polls_association` — a phase
    /// with no credentials must not reach here, or the radio logs a failed
    /// connect to an empty SSID every tick.
    fn try_connect(&mut self) -> bool {
        if self.wifi.is_connected().unwrap_or(false) {
            return true;
        }
        if self.wifi.connect().is_ok() && self.wifi.wait_netif_up().is_ok() {
            log_got_ip();
        }
        self.wifi.is_connected().unwrap_or(false)
    }

    pub fn into_sta(mut self, creds: &WifiCredentials) -> StaWifi<'d> {
        info!("Starting WiFi STA for '{}'", creds.ssid);
        self.start_with(Configuration::Client(sta_config(creds)));
        StaWifi { driver: self }
    }

    pub fn into_mixed(mut self, creds: Option<&WifiCredentials>) -> MixedWifi<'d> {
        // Logged so a build without a panel still has somewhere to read the
        // passphrase; on one with a panel the WiFi Setup screen shows it.
        info!("Starting AP: {} / {}", AP_SSID, AP_PASS.as_str());
        // Always Mixed mode so the STA interface is available for scanning.
        self.start_with(Self::mixed_config(creds));
        info!("AP started");
        MixedWifi {
            driver: self,
            scan_cache: Arc::new(Mutex::new(CachedScan {
                at: None,
                entries: ScanResult::new(),
            })),
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
    pub fn try_connect(&mut self) -> bool {
        self.driver.try_connect()
    }

    pub fn into_mixed(self, creds: Option<&WifiCredentials>) -> MixedWifi<'d> {
        self.driver.into_mixed(creds)
    }
}

/// Mixed AP+STA mode: captive portal up, STA half retrying behind it.
/// Carries the scan cache, which is only sane in mixed mode — an STA-only
/// scan would disrupt the dashboard.
pub struct MixedWifi<'d> {
    driver: WifiDriver<'d>,
    scan_cache: ScanCache,
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
        self.apply_sta_config(Some(creds));
        // Kick off a fresh connect against the new creds. Non-fatal — the
        // supervisor's per-tick reconnect retries on its own.
        let _ = self.driver.wifi.connect();
    }

    /// Drop the credentials the STA half is retrying, without stopping the
    /// radio — the captive AP stays associated with the user's phone. Used
    /// when a submission's association budget expires: the phase drops the
    /// creds, so leaving them applied would have the STA half retrying
    /// credentials the supervisor has already forgotten.
    pub fn clear_sta_creds(&mut self) {
        info!("Clearing STA creds (live)");
        self.apply_sta_config(None);
    }

    /// Swap the STA half's configuration without stopping the radio, so
    /// the captive AP stays associated with the user's phone throughout.
    /// Drops any association in flight; `None` leaves the STA half bare.
    fn apply_sta_config(&mut self, creds: Option<&WifiCredentials>) {
        self.driver
            .wifi
            .set_configuration(&WifiDriver::mixed_config(creds))
            .unwrap();
        let _ = self.driver.wifi.disconnect();
    }

    pub fn try_connect(&mut self) -> bool {
        self.driver.try_connect()
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
