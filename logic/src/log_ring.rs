//! Fixed-capacity byte ring buffer for log capture.
//!
//! Pure data structure — no I/O, no logging hook. The vprintf hook in `src/`
//! owns the global instance and feeds bytes into it; HTTP `/api/log` calls
//! `snapshot()` to read out the contents in chronological order.

pub struct Ring {
    data: Box<[u8]>,
    /// Next write index.
    head: usize,
    /// True once the ring has wrapped; before that, valid data is `[0..head]`.
    wrapped: bool,
}

impl Ring {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "Ring capacity must be > 0");
        Self {
            data: vec![0u8; capacity].into_boxed_slice(),
            head: 0,
            wrapped: false,
        }
    }

    pub fn capacity(&self) -> usize {
        self.data.len()
    }

    pub fn write(&mut self, mut bytes: &[u8]) {
        let cap = self.data.len();
        // Oversized line: keep only the most recent `cap` bytes so wrapping
        // lands at a clean final position (head=0, fully wrapped).
        if bytes.len() > cap {
            bytes = &bytes[bytes.len() - cap..];
            self.head = 0;
            self.wrapped = true;
        }
        let tail = cap - self.head;
        if bytes.len() <= tail {
            self.data[self.head..self.head + bytes.len()].copy_from_slice(bytes);
            self.head += bytes.len();
            if self.head == cap {
                self.head = 0;
                self.wrapped = true;
            }
        } else {
            let (first, rest) = bytes.split_at(tail);
            self.data[self.head..].copy_from_slice(first);
            self.data[..rest.len()].copy_from_slice(rest);
            self.head = rest.len();
            self.wrapped = true;
        }
    }

    pub fn snapshot(&self) -> Vec<u8> {
        if self.wrapped {
            let mut out = Vec::with_capacity(self.data.len());
            out.extend_from_slice(&self.data[self.head..]);
            out.extend_from_slice(&self.data[..self.head]);
            out
        } else {
            self.data[..self.head].to_vec()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fresh_ring_snapshot_is_empty() {
        let r = Ring::new(16);
        assert_eq!(r.snapshot(), Vec::<u8>::new());
    }

    #[test]
    fn write_below_capacity_preserves_order() {
        let mut r = Ring::new(16);
        r.write(b"hello");
        r.write(b" world");
        assert_eq!(r.snapshot(), b"hello world");
    }

    #[test]
    fn fill_exactly_to_capacity_marks_wrapped() {
        let mut r = Ring::new(8);
        r.write(b"abcdefgh");
        // After exact fill, head=0 and wrapped=true.
        assert_eq!(r.snapshot(), b"abcdefgh");
        // Next byte goes to position 0; snapshot becomes "bcdefghX"
        r.write(b"X");
        assert_eq!(r.snapshot(), b"bcdefghX");
    }

    #[test]
    fn wrap_around_mid_write_orders_oldest_first() {
        let mut r = Ring::new(8);
        r.write(b"123456");
        // head=6, not wrapped
        r.write(b"ABCDEF");
        // 6 used + 6 written = 12, capacity 8 — wrap. Keeps last 8 bytes:
        // "56ABCDEF" expected (we lose "1234").
        assert_eq!(r.snapshot(), b"56ABCDEF");
    }

    #[test]
    fn oversized_write_keeps_only_last_capacity_bytes() {
        let mut r = Ring::new(8);
        r.write(b"earlier");
        // 20-byte input into 8-byte ring: keep last 8.
        r.write(b"AAAAAAAAAAAAXXYYZZWQ");
        assert_eq!(r.snapshot(), b"XXYYZZWQ");
    }

    #[test]
    fn snapshot_after_full_wrap_is_in_chronological_order() {
        let mut r = Ring::new(4);
        for &b in b"abcdefgh" {
            r.write(&[b]);
        }
        // Last 4 bytes of input are "efgh"; chronological order preserved.
        assert_eq!(r.snapshot(), b"efgh");
    }

    #[test]
    fn write_split_exactly_at_boundary() {
        let mut r = Ring::new(8);
        r.write(b"abcd");
        // tail=4, write exactly tail bytes — head wraps to 0, wrapped=true.
        r.write(b"efgh");
        assert_eq!(r.snapshot(), b"abcdefgh");
        r.write(b"i");
        assert_eq!(r.snapshot(), b"bcdefghi");
    }

    #[test]
    #[should_panic]
    fn zero_capacity_panics() {
        Ring::new(0);
    }
}
