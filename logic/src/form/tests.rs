use super::*;

#[test]
fn url_decode_truncates_at_buffer_limit() {
    let mut buf = [0u8; 3];
    let len = url_decode("abcdef", &mut buf);
    assert_eq!(len, 3);
    assert_eq!(&buf[..len], b"abc");
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
fn url_decode_handles_every_encoding_the_form_sends() {
    // `+` is a space, `%XX` is a byte, and anything that isn't valid hex
    // after a `%` passes through untouched rather than being dropped.
    let cases: [(&str, &[u8]); 5] = [
        ("hello+world", b"hello world"),
        ("a%20b%21", b"a b!"),
        ("plain", b"plain"),
        ("", b""),
        ("%ZZ", b"%ZZ"),
    ];
    for (input, want) in cases {
        let mut buf = [0u8; 64];
        let len = url_decode(input, &mut buf);
        assert_eq!(&buf[..len], want, "url_decode({input:?})");
    }
}

#[test]
fn parse_form_rejects_bodies_without_an_ssid() {
    for body in ["pass=secret", ""] {
        assert!(parse_form(body).is_none(), "{body:?}");
    }
}
