//! Captive-portal DNS responder framing. Pure byte math — no sockets.
//!
//! Answers A-record queries with a fixed IP; sends 0 answers for everything
//! else (AAAA / TXT / SRV) so clients fall back fast instead of hanging on a
//! retry timer.

/// Parses a DNS request from `req` and writes a response into `out`. Returns
/// the response length, or `None` if the request is malformed.
///
/// - A-record queries get a single answer pointing to `answer_ip`.
/// - Any other qtype gets a valid response with 0 answers (immediate
///   negative — clients move on without retrying).
pub fn build_response(req: &[u8], answer_ip: [u8; 4], out: &mut [u8]) -> Option<usize> {
    let q_end = parse_question_end(req)?;
    let qtype = u16::from_be_bytes([req[q_end - 4], req[q_end - 3]]);
    let is_a = qtype == 1;

    // Header(12) + echoed question + (A: 16-byte answer | other: 0).
    let answer_len = if is_a { 16 } else { 0 };
    let total = 12 + (q_end - 12) + answer_len;
    if out.len() < total {
        return None;
    }

    out[0] = req[0]; // txn ID
    out[1] = req[1];
    out[2] = 0x84 | (req[2] & 0x01); // QR=1, AA=1, copy RD from query
    out[3] = 0x00;

    if is_a {
        out[4..12].copy_from_slice(&[0, 1, 0, 1, 0, 0, 0, 0]); // qdcount=1, ancount=1
    } else {
        out[4..12].copy_from_slice(&[0, 1, 0, 0, 0, 0, 0, 0]); // qdcount=1, ancount=0
    }

    out[12..q_end].copy_from_slice(&req[12..q_end]);

    if is_a {
        let pos = q_end;
        out[pos..pos + 12].copy_from_slice(&[
            0xC0, 0x0C, // name pointer to offset 12
            0, 1, // type A
            0, 1, // class IN
            0, 0, 0, 0, // TTL 0
            0, 4, // RDLENGTH
        ]);
        out[pos + 12..pos + 16].copy_from_slice(&answer_ip);
    }

    Some(total)
}

/// Walks labels in the question section starting at offset 12 and returns
/// the offset just past `qclass` (i.e. first byte after the question).
/// Returns `None` if the labels run past the end of the packet or the
/// trailing qtype/qclass don't fit.
fn parse_question_end(req: &[u8]) -> Option<usize> {
    if req.len() < 12 {
        return None;
    }
    let mut q_end = 12;
    while q_end < req.len() && req[q_end] != 0 {
        let label_len = req[q_end] as usize + 1;
        if q_end + label_len >= req.len() {
            return None;
        }
        q_end += label_len;
    }
    if q_end >= req.len() || req[q_end] != 0 {
        return None;
    }
    q_end += 1; // null terminator
    if q_end + 4 > req.len() {
        return None;
    }
    Some(q_end + 4) // qtype(2) + qclass(2)
}

#[cfg(test)]
mod tests;
