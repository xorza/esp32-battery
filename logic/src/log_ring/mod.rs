//! Fixed-capacity byte ring buffer for log capture.
//!
//! Pure data structure — no I/O, no logging hook. The vprintf hook in `src/`
//! owns the global instance and feeds bytes into it; HTTP `/api/log` calls
//! `slices()` to read out the contents in chronological order.
//!
//! Storage is a caller-provided `&'static mut [u8]`, so firmware can place
//! it in BSS instead of the heap. Tests use `Box::leak` for the same shape.

pub struct Ring {
    data: &'static mut [u8],
    /// Next write index.
    head: usize,
    /// True once the ring has wrapped; before that, valid data is `[0..head]`.
    wrapped: bool,
}

impl Ring {
    pub fn from_buf(data: &'static mut [u8]) -> Self {
        assert!(!data.is_empty(), "Ring capacity must be > 0");
        data.fill(0);
        Self {
            data,
            head: 0,
            wrapped: false,
        }
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

    /// Borrow the ring contents as up to two contiguous slices in
    /// chronological order: the older half (post-head) followed by the
    /// newer half (pre-head). Either slice may be empty. Avoids the
    /// allocation a `Vec`-returning snapshot would do.
    pub fn slices(&self) -> (&[u8], &[u8]) {
        if self.wrapped {
            (&self.data[self.head..], &self.data[..self.head])
        } else {
            (&self.data[..self.head], &[])
        }
    }
}

#[cfg(test)]
mod tests;
