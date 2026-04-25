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
mod tests {
    use super::*;

    /// Build a minimal A query for "example": header(12) + 1-label "example"
    /// (7+1 bytes) + null + qtype(2) + qclass(2) = 24 bytes.
    fn a_query(txn_id: [u8; 2]) -> Vec<u8> {
        let mut q = Vec::new();
        q.extend_from_slice(&txn_id);
        q.extend_from_slice(&[0x01, 0x00]); // flags: standard query, RD=1
        q.extend_from_slice(&[0, 1, 0, 0, 0, 0, 0, 0]); // qd=1, an=0, ns=0, ar=0
        q.push(7);
        q.extend_from_slice(b"example");
        q.push(0); // null terminator
        q.extend_from_slice(&[0, 1]); // qtype A
        q.extend_from_slice(&[0, 1]); // qclass IN
        q
    }

    fn aaaa_query() -> Vec<u8> {
        let mut q = a_query([0xAB, 0xCD]);
        // last 4 bytes are qtype/qclass; flip qtype to AAAA (28)
        let len = q.len();
        q[len - 4] = 0;
        q[len - 3] = 28;
        q
    }

    #[test]
    fn a_query_returns_28_bytes() {
        // header(12) + question(12: 1+7+1+2+2 = 13... wait) — recompute:
        // labels: 1 length byte + 7 chars + 1 null = 9; qtype 2 + qclass 2 = 4; total 13.
        // request total = 12 + 13 = 25.
        // response = 12 (header) + 13 (echoed question) + 16 (answer) = 41.
        let req = a_query([0x12, 0x34]);
        assert_eq!(req.len(), 25);
        let mut out = [0u8; 64];
        let n = build_response(&req, [192, 168, 71, 1], &mut out).unwrap();
        assert_eq!(n, 41);
        // Echoed transaction ID
        assert_eq!(&out[0..2], &[0x12, 0x34]);
        // Flags: QR=1, AA=1, RD=1 => 0x85, 0x00
        assert_eq!(&out[2..4], &[0x85, 0x00]);
        // qd=1, an=1
        assert_eq!(&out[4..8], &[0, 1, 0, 1]);
        // Answer section starts after header(12)+question(13)=25
        assert_eq!(&out[25..27], &[0xC0, 0x0C]); // name pointer
        assert_eq!(&out[27..29], &[0, 1]); // type A
        assert_eq!(&out[29..31], &[0, 1]); // class IN
        // RDLENGTH 4, then IP
        assert_eq!(&out[35..37], &[0, 4]);
        assert_eq!(&out[37..41], &[192, 168, 71, 1]);
    }

    #[test]
    fn aaaa_query_returns_zero_answers() {
        let req = aaaa_query();
        let mut out = [0u8; 64];
        let n = build_response(&req, [192, 168, 71, 1], &mut out).unwrap();
        // header(12) + echoed question(13) + 0 answers = 25
        assert_eq!(n, 25);
        assert_eq!(&out[0..2], &[0xAB, 0xCD]);
        // ancount=0
        assert_eq!(&out[6..8], &[0, 0]);
    }

    #[test]
    fn rd_bit_echoes_into_response_flags() {
        let mut req = a_query([0, 0]);
        req[2] = 0x00; // RD=0
        let mut out = [0u8; 64];
        build_response(&req, [1, 2, 3, 4], &mut out).unwrap();
        assert_eq!(out[2], 0x84); // QR=1, AA=1, RD=0

        req[2] = 0x01; // RD=1
        build_response(&req, [1, 2, 3, 4], &mut out).unwrap();
        assert_eq!(out[2], 0x85); // QR=1, AA=1, RD=1
    }

    #[test]
    fn truncated_header_rejected() {
        let mut out = [0u8; 64];
        assert!(build_response(&[0u8; 11], [0; 4], &mut out).is_none());
    }

    #[test]
    fn label_running_past_end_rejected() {
        let mut req = vec![0u8; 12];
        req.push(50); // label claims 50 bytes but only ~12 follow
        req.extend_from_slice(b"short");
        let mut out = [0u8; 64];
        assert!(build_response(&req, [0; 4], &mut out).is_none());
    }

    #[test]
    fn missing_qtype_qclass_rejected() {
        // Header + label + null but no qtype/qclass
        let mut req = vec![0u8; 12];
        req.push(7);
        req.extend_from_slice(b"example");
        req.push(0);
        let mut out = [0u8; 64];
        assert!(build_response(&req, [0; 4], &mut out).is_none());
    }

    #[test]
    fn output_buffer_too_small_rejected() {
        let req = a_query([0, 0]);
        let mut out = [0u8; 30]; // need 41
        assert!(build_response(&req, [0; 4], &mut out).is_none());
    }

    #[test]
    fn multi_label_name_preserved_in_answer() {
        // "captive.example" = [7]captive[7]example[0]
        let mut req = vec![0xCA, 0xFE]; // txn
        req.extend_from_slice(&[0x01, 0x00]); // flags
        req.extend_from_slice(&[0, 1, 0, 0, 0, 0, 0, 0]);
        req.push(7);
        req.extend_from_slice(b"captive");
        req.push(7);
        req.extend_from_slice(b"example");
        req.push(0);
        req.extend_from_slice(&[0, 1, 0, 1]); // A, IN

        // Question is: 8 + 8 + 1 + 4 = 21 bytes after header.
        // Total request = 12 + 21 = 33; response = 12 + 21 + 16 = 49.
        let mut out = [0u8; 64];
        let n = build_response(&req, [10, 0, 0, 5], &mut out).unwrap();
        assert_eq!(n, 49);
        // Echoed labels start at offset 12
        assert_eq!(&out[12..20], &[7, b'c', b'a', b'p', b't', b'i', b'v', b'e']);
        assert_eq!(&out[20..28], &[7, b'e', b'x', b'a', b'm', b'p', b'l', b'e']);
        assert_eq!(out[28], 0);
    }
}
