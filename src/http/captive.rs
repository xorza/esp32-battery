//! Captive-mode HTTP server (port 80). Mounts the captive API + common
//! assets, then registers the wildcard portal page LAST so named routes
//! take precedence over the catch-all.

use log::info;

use crate::captive_api;
use crate::dns::DnsHandle;
use crate::net::{CaptiveBundle, SubmissionStatusHandle, new_creds_mailbox};
use crate::wifi::ScanCache;

use super::{ServerConfig, create_server, serve_common_assets, serve_static};

pub fn start(scan_cache: ScanCache) -> CaptiveBundle {
    let dns = DnsHandle::start();
    let mailbox = new_creds_mailbox();
    let status = SubmissionStatusHandle::new();

    let mut server = create_server(ServerConfig {
        stack_size: 8192,
        max_sockets: 4,
        wildcard: true,
        https: false,
    });

    captive_api::mount(&mut server, scan_cache, mailbox.clone(), status.clone());
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
        mailbox,
        status,
    }
}
