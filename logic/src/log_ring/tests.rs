use super::*;

fn ring(cap: usize) -> Ring {
    Ring::from_buf(Box::leak(vec![0u8; cap].into_boxed_slice()))
}

fn collect(r: &Ring) -> Vec<u8> {
    let (a, b) = r.slices();
    [a, b].concat()
}

#[test]
fn fresh_ring_snapshot_is_empty() {
    let r = ring(16);
    assert_eq!(collect(&r), Vec::<u8>::new());
}

#[test]
fn write_below_capacity_preserves_order() {
    let mut r = ring(16);
    r.write(b"hello");
    r.write(b" world");
    assert_eq!(collect(&r), b"hello world");
}

#[test]
fn fill_exactly_to_capacity_marks_wrapped() {
    let mut r = ring(8);
    r.write(b"abcdefgh");
    // After exact fill, head=0 and wrapped=true.
    assert_eq!(collect(&r), b"abcdefgh");
    // Next byte goes to position 0; snapshot becomes "bcdefghX"
    r.write(b"X");
    assert_eq!(collect(&r), b"bcdefghX");
}

#[test]
fn wrap_around_mid_write_orders_oldest_first() {
    let mut r = ring(8);
    r.write(b"123456");
    // head=6, not wrapped
    r.write(b"ABCDEF");
    // 6 used + 6 written = 12, capacity 8 — wrap. Keeps last 8 bytes:
    // "56ABCDEF" expected (we lose "1234").
    assert_eq!(collect(&r), b"56ABCDEF");
}

#[test]
fn oversized_write_keeps_only_last_capacity_bytes() {
    let mut r = ring(8);
    r.write(b"earlier");
    // 20-byte input into 8-byte ring: keep last 8.
    r.write(b"AAAAAAAAAAAAXXYYZZWQ");
    assert_eq!(collect(&r), b"XXYYZZWQ");
}

#[test]
fn snapshot_after_full_wrap_is_in_chronological_order() {
    let mut r = ring(4);
    for &b in b"abcdefgh" {
        r.write(&[b]);
    }
    // Last 4 bytes of input are "efgh"; chronological order preserved.
    assert_eq!(collect(&r), b"efgh");
}

#[test]
fn write_split_exactly_at_boundary() {
    let mut r = ring(8);
    r.write(b"abcd");
    // tail=4, write exactly tail bytes — head wraps to 0, wrapped=true.
    r.write(b"efgh");
    assert_eq!(collect(&r), b"abcdefgh");
    r.write(b"i");
    assert_eq!(collect(&r), b"bcdefghi");
}

#[test]
#[should_panic]
fn zero_capacity_panics() {
    ring(0);
}
