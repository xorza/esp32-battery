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
