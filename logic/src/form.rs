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
mod tests {
    use super::*;

    #[test]
    fn url_decode_basic() {
        let mut buf = [0u8; 64];
        let len = url_decode("hello+world", &mut buf);
        assert_eq!(&buf[..len], b"hello world");
    }

    #[test]
    fn url_decode_percent() {
        let mut buf = [0u8; 64];
        let len = url_decode("a%20b%21", &mut buf);
        assert_eq!(&buf[..len], b"a b!");
    }

    #[test]
    fn url_decode_empty() {
        let mut buf = [0u8; 64];
        let len = url_decode("", &mut buf);
        assert_eq!(len, 0);
    }

    #[test]
    fn url_decode_no_encoding() {
        let mut buf = [0u8; 64];
        let len = url_decode("plain", &mut buf);
        assert_eq!(&buf[..len], b"plain");
    }

    #[test]
    fn url_decode_truncates_at_buffer_limit() {
        let mut buf = [0u8; 3];
        let len = url_decode("abcdef", &mut buf);
        assert_eq!(len, 3);
        assert_eq!(&buf[..len], b"abc");
    }

    #[test]
    fn url_decode_invalid_percent_passthrough() {
        let mut buf = [0u8; 64];
        let len = url_decode("%ZZ", &mut buf);
        // Invalid hex digits: '%' is passed through, then Z, Z follow
        assert_eq!(&buf[..len], b"%ZZ");
    }

    #[test]
    fn parse_form_both_fields() {
        let (ssid, pass) = parse_form("ssid=MyNetwork&pass=secret123").unwrap();
        assert_eq!(ssid, "MyNetwork");
        assert_eq!(pass, "secret123");
    }

    #[test]
    fn parse_form_no_password() {
        let (ssid, pass) = parse_form("ssid=OpenNet&pass=").unwrap();
        assert_eq!(ssid, "OpenNet");
        assert_eq!(pass, "");
    }

    #[test]
    fn parse_form_missing_ssid() {
        assert!(parse_form("pass=secret").is_none());
    }

    #[test]
    fn parse_form_missing_pass_key() {
        let (ssid, pass) = parse_form("ssid=Test").unwrap();
        assert_eq!(ssid, "Test");
        assert_eq!(pass, "");
    }

    #[test]
    fn parse_form_encoded_values() {
        // parse_form returns raw encoded strings
        let (ssid, pass) = parse_form("ssid=My%20Net&pass=p%40ss").unwrap();
        assert_eq!(ssid, "My%20Net");
        assert_eq!(pass, "p%40ss");

        // url_decode handles the actual decoding
        let mut buf = [0u8; 64];
        let len = url_decode(ssid, &mut buf);
        assert_eq!(std::str::from_utf8(&buf[..len]).unwrap(), "My Net");

        let len = url_decode(pass, &mut buf);
        assert_eq!(std::str::from_utf8(&buf[..len]).unwrap(), "p@ss");
    }

    #[test]
    fn parse_form_empty() {
        assert!(parse_form("").is_none());
    }
}
