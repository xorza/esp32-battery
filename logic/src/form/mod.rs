pub fn url_decode(input: &str, out: &mut [u8]) -> usize {
    let bytes = input.as_bytes();
    let mut i = 0;
    let mut o = 0;
    while i < bytes.len() && o < out.len() {
        if bytes[i] == b'+' {
            out[o] = b' ';
            i += 1;
        } else if bytes[i] == b'%' && i + 2 < bytes.len() {
            let hi = char::from(bytes[i + 1]).to_digit(16);
            let lo = char::from(bytes[i + 2]).to_digit(16);
            if let (Some(h), Some(l)) = (hi, lo) {
                out[o] = (h * 16 + l) as u8;
                i += 3;
            } else {
                out[o] = bytes[i];
                i += 1;
            }
        } else {
            out[o] = bytes[i];
            i += 1;
        }
        o += 1;
    }
    o
}

/// Parse URL-encoded form body "ssid=value&pass=value".
/// Returns raw encoded values — caller must url_decode them.
pub fn parse_form(body: &str) -> Option<(&str, &str)> {
    let mut ssid = None;
    let mut pass = None;
    for pair in body.split('&') {
        if let Some((key, value)) = pair.split_once('=') {
            match key {
                "ssid" => ssid = Some(value),
                "pass" => pass = Some(value),
                _ => {}
            }
        }
    }
    Some((ssid?, pass.unwrap_or("")))
}

#[cfg(test)]
mod tests;
