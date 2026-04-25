//! Captive-mode HTTP server (port 80). Mounts the captive API + common
//! assets, then registers the wildcard portal page LAST so named routes
//! take precedence over the catch-all.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use log::info;

use crate::captive_api;
use crate::dns::DnsHandle;
use crate::net::{CaptiveBundle, CaptiveStateHandle};
use crate::wifi::Wifi;

use super::{create_server, serve_common_assets, serve_static};

pub fn start(wifi: Arc<Mutex<Wifi<'static>>>, state: CaptiveStateHandle) -> CaptiveBundle {
    let dns = DnsHandle::start();

    let mut server = create_server(8192, true, 4, Some(Duration::from_secs(2)), false);

    captive_api::mount(&mut server, wifi, state.clone());
    serve_common_assets(&mut server);

    // Wildcard fallback — must be the last fn_handler call so the named
    // routes above (and `/style.css`, `/favicon.ico`) win the URI match.
    serve_static(
        &mut server,
        "/*",
        "text/html",
        "no-cache",
        include_bytes!(concat!(env!("OUT_DIR"), "/captive_portal.html")),
        true,
    );

    info!("Captive portal started");

    CaptiveBundle {
        _server: server,
        _dns: dns,
        state,
    }
}
