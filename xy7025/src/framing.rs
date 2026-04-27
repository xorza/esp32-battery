//! Modbus-RTU on-wire framing — pure functions, no I/O.
//!
//! Use these to build a [`crate::ModbusTransport`] over your platform's
//! UART. The codec is general Modbus-RTU (function codes `0x03`, `0x06`,
//! `0x10`); nothing in here is XY-specific.
//!
//! CRC-16 is the standard reflected polynomial `0xA001`, seeded
//! `0xFFFF`, no final XOR. The CRC is appended low-byte first.

use crate::transport::ModbusError;

pub const FN_READ_HOLDING: u8 = 0x03;
pub const FN_WRITE_SINGLE: u8 = 0x06;
pub const FN_WRITE_MULTIPLE: u8 = 0x10;

/// Maximum Modbus-RTU ADU size (slave + PDU + CRC).
pub const MAX_ADU: usize = 256;

/// Maximum registers in a single `Write Multiple Holdings` request
/// (Modbus standard limit).
pub const MAX_WRITE_REGS: usize = 123;

/// Why [`build_write_multiple_request`] could not assemble a frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FrameError {
    /// `values` was empty or exceeded [`MAX_WRITE_REGS`] (123).
    InvalidLength(usize),
    /// `out` was smaller than the assembled frame (header + payload + CRC).
    BufferTooSmall {
        needed: usize,
        actual: usize,
    },
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidLength(n) => write!(f, "invalid register count {n}"),
            Self::BufferTooSmall { needed, actual } => {
                write!(f, "buffer too small (need {needed}, have {actual})")
            }
        }
    }
}

impl core::error::Error for FrameError {}

/// Standard Modbus-RTU CRC-16.
pub fn crc16_modbus(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= b as u16;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

fn append_crc(buf: &mut [u8], len: usize) {
    let crc = crc16_modbus(&buf[..len]);
    buf[len] = crc as u8;
    buf[len + 1] = (crc >> 8) as u8;
}

/// Build a `Read Holding Registers` (FC `0x03`) request frame.
pub fn build_read_request(slave: u8, addr: u16, count: u16) -> [u8; 8] {
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_READ_HOLDING;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    req[4..6].copy_from_slice(&count.to_be_bytes());
    append_crc(&mut req, 6);
    req
}

/// Build a `Write Single Holding Register` (FC `0x06`) request frame.
pub fn build_write_single_request(slave: u8, addr: u16, value: u16) -> [u8; 8] {
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_WRITE_SINGLE;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    req[4..6].copy_from_slice(&value.to_be_bytes());
    append_crc(&mut req, 6);
    req
}

/// Build a `Write Multiple Holding Registers` (FC `0x10`) request into
/// `out`, returning the number of bytes written. `out` must be at
/// least `9 + 2 * values.len()` bytes.
pub fn build_write_multiple_request(
    slave: u8,
    addr: u16,
    values: &[u16],
    out: &mut [u8],
) -> Result<usize, FrameError> {
    if values.is_empty() || values.len() > MAX_WRITE_REGS {
        return Err(FrameError::InvalidLength(values.len()));
    }
    let bc = 2 * values.len();
    let len = 7 + bc + 2;
    if out.len() < len {
        return Err(FrameError::BufferTooSmall {
            needed: len,
            actual: out.len(),
        });
    }
    out[0] = slave;
    out[1] = FN_WRITE_MULTIPLE;
    out[2..4].copy_from_slice(&addr.to_be_bytes());
    out[4..6].copy_from_slice(&(values.len() as u16).to_be_bytes());
    out[6] = bc as u8;
    for (i, v) in values.iter().enumerate() {
        out[7 + 2 * i..9 + 2 * i].copy_from_slice(&v.to_be_bytes());
    }
    append_crc(out, 7 + bc);
    Ok(len)
}

fn check_crc(resp: &[u8], len: usize) -> Result<(), ModbusError> {
    if resp.len() < len {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    let got = u16::from_le_bytes([resp[len - 2], resp[len - 1]]);
    let calc = crc16_modbus(&resp[..len - 2]);
    if got == calc {
        Ok(())
    } else {
        Err(ModbusError::BadCrc)
    }
}

fn check_exception(resp: &[u8], slave: u8) -> Result<(), ModbusError> {
    if resp.len() < 5 {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    if resp[0] != slave {
        return Err(ModbusError::BadSlave(resp[0]));
    }
    if resp[1] & 0x80 != 0 {
        check_crc(resp, 5)?;
        return Err(ModbusError::Exception(resp[2]));
    }
    Ok(())
}

/// Parse a `Read Holding Registers` response into `out`. The expected
/// register count is `out.len()`.
pub fn parse_read_response(resp: &[u8], slave: u8, out: &mut [u16]) -> Result<(), ModbusError> {
    check_exception(resp, slave)?;
    let count = out.len();
    let expected_len = 5 + 2 * count;
    if resp[1] != FN_READ_HOLDING || resp[2] as usize != 2 * count {
        return Err(ModbusError::BadHeader);
    }
    check_crc(resp, expected_len)?;
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = u16::from_be_bytes([resp[3 + 2 * i], resp[4 + 2 * i]]);
    }
    Ok(())
}

/// Parse a `Write Single Holding Register` response. Per Modbus spec the
/// response echoes the request byte-for-byte.
pub fn parse_write_single_response(resp: &[u8], req: &[u8; 8]) -> Result<(), ModbusError> {
    check_exception(resp, req[0])?;
    check_crc(resp, 8)?;
    if resp.len() < 8 || resp[..8] != req[..] {
        return Err(ModbusError::BadHeader);
    }
    Ok(())
}

/// Parse a `Write Multiple Holding Registers` response. The response is
/// always 8 bytes: slave, fc, start addr, qty, CRC.
pub fn parse_write_multiple_response(
    resp: &[u8],
    slave: u8,
    addr: u16,
    qty: u16,
) -> Result<(), ModbusError> {
    check_exception(resp, slave)?;
    check_crc(resp, 8)?;
    if resp[1] != FN_WRITE_MULTIPLE
        || u16::from_be_bytes([resp[2], resp[3]]) != addr
        || u16::from_be_bytes([resp[4], resp[5]]) != qty
    {
        return Err(ModbusError::BadHeader);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;

    #[test]
    fn crc_known_vectors() {
        assert_eq!(crc16_modbus(&[]), 0xFFFF);
        assert_eq!(crc16_modbus(&[0x01]), 0x807E);
        // Read 1 reg at 0x001F from slave 1.
        assert_eq!(crc16_modbus(&[0x01, 0x03, 0x00, 0x1F, 0x00, 0x01]), 0xCCB5);
        // Write 0x0001 to reg 0x0012 from slave 1.
        assert_eq!(crc16_modbus(&[0x01, 0x06, 0x00, 0x12, 0x00, 0x01]), 0x0FE8);
    }

    #[test]
    fn crc_detects_bit_flips() {
        let base = [0x01u8, 0x03, 0x00, 0x00, 0x00, 0x06];
        let base_crc = crc16_modbus(&base);
        for i in 0..base.len() {
            for bit in 0..8 {
                let mut f = base;
                f[i] ^= 1 << bit;
                assert_ne!(crc16_modbus(&f), base_crc);
            }
        }
    }

    #[test]
    fn build_read_matches_known() {
        let req = build_read_request(0x01, 0x001F, 1);
        assert_eq!(req, [0x01, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xB5, 0xCC]);
    }

    #[test]
    fn build_write_single_matches_known() {
        let req = build_write_single_request(0x01, 0x0012, 0x0001);
        assert_eq!(req, [0x01, 0x06, 0x00, 0x12, 0x00, 0x01, 0xE8, 0x0F]);
    }

    #[test]
    fn build_write_multiple_layout() {
        // Wire-level example from README §6.3: write LVP=1000, OVP=1500,
        // OCP=1250 to 0x0052..=0x0054, slave 1.
        let mut buf = [0u8; 32];
        let n =
            build_write_multiple_request(0x01, 0x0052, &[1000, 1500, 1250], &mut buf).unwrap();
        // 7 (header) + 6 (payload) + 2 (CRC) = 15
        assert_eq!(n, 15);
        // Header: slave, FC, start addr, qty, byte count.
        assert_eq!(
            buf[..7],
            [0x01, 0x10, 0x00, 0x52, 0x00, 0x03, 0x06]
        );
        // Payload: 1000=0x03E8, 1500=0x05DC, 1250=0x04E2.
        assert_eq!(buf[7..13], [0x03, 0xE8, 0x05, 0xDC, 0x04, 0xE2]);
    }

    #[test]
    fn build_write_multiple_rejects_oversize() {
        let mut buf = [0u8; 16];
        assert!(
            build_write_multiple_request(0x01, 0x0050, &[0; 14], &mut buf).is_err()
        );
    }

    fn read_resp(slave: u8, values: &[u16]) -> std::vec::Vec<u8> {
        let mut out = std::vec::Vec::new();
        out.push(slave);
        out.push(FN_READ_HOLDING);
        out.push((values.len() * 2) as u8);
        for v in values {
            out.extend_from_slice(&v.to_be_bytes());
        }
        let crc = crc16_modbus(&out);
        out.push(crc as u8);
        out.push((crc >> 8) as u8);
        out
    }

    #[test]
    fn parse_read_six_regs() {
        let frame = read_resp(0x01, &[1360, 1000, 1350, 0, 0, 4800]);
        let mut out = [0u16; 6];
        parse_read_response(&frame, 0x01, &mut out).unwrap();
        assert_eq!(out, [1360, 1000, 1350, 0, 0, 4800]);
    }

    #[test]
    fn parse_read_rejects_wrong_slave() {
        let frame = read_resp(0x02, &[0x1234]);
        let mut out = [0u16; 1];
        assert!(matches!(
            parse_read_response(&frame, 0x01, &mut out),
            Err(ModbusError::BadSlave(0x02))
        ));
    }

    #[test]
    fn parse_read_rejects_bad_crc() {
        let mut frame = read_resp(0x01, &[0x1234]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let mut out = [0u16; 1];
        assert!(matches!(
            parse_read_response(&frame, 0x01, &mut out),
            Err(ModbusError::BadCrc)
        ));
    }

    #[test]
    fn parse_read_exception_with_valid_crc() {
        let mut frame = std::vec![0x01u8, 0x83, 0x02];
        let crc = crc16_modbus(&frame);
        frame.push(crc as u8);
        frame.push((crc >> 8) as u8);
        let mut out = [0u16; 1];
        assert!(matches!(
            parse_read_response(&frame, 0x01, &mut out),
            Err(ModbusError::Exception(0x02))
        ));
    }

    #[test]
    fn parse_read_exception_with_bad_crc_is_bad_crc() {
        let frame = [0x01u8, 0x83, 0x02, 0x00, 0x00];
        let mut out = [0u16; 1];
        assert!(matches!(
            parse_read_response(&frame, 0x01, &mut out),
            Err(ModbusError::BadCrc)
        ));
    }

    #[test]
    fn parse_write_single_valid_echo() {
        let req = build_write_single_request(0x01, 0x0012, 0x0001);
        parse_write_single_response(&req, &req).unwrap();
    }

    #[test]
    fn parse_write_single_rejects_value_mismatch() {
        let req = build_write_single_request(0x01, 0x0012, 0x0001);
        let mut resp = req;
        resp[5] = 0x02;
        let crc = crc16_modbus(&resp[..6]);
        resp[6] = crc as u8;
        resp[7] = (crc >> 8) as u8;
        assert!(matches!(
            parse_write_single_response(&resp, &req),
            Err(ModbusError::BadHeader)
        ));
    }

    #[test]
    fn parse_write_single_exception_returns_exception() {
        let req = build_write_single_request(0x01, 0x0012, 0x0001);
        let mut frame = std::vec![0x01u8, 0x86, 0x03];
        let crc = crc16_modbus(&frame);
        frame.push(crc as u8);
        frame.push((crc >> 8) as u8);
        assert!(matches!(
            parse_write_single_response(&frame, &req),
            Err(ModbusError::Exception(0x03))
        ));
    }

    #[test]
    fn parse_write_multiple_valid() {
        // Standard echo response: slave, fc, addr, qty, CRC.
        let mut frame = [0x01u8, 0x10, 0x00, 0x52, 0x00, 0x03, 0, 0];
        let crc = crc16_modbus(&frame[..6]);
        frame[6] = crc as u8;
        frame[7] = (crc >> 8) as u8;
        parse_write_multiple_response(&frame, 0x01, 0x0052, 3).unwrap();
    }

    #[test]
    fn parse_write_multiple_rejects_addr_mismatch() {
        let mut frame = [0x01u8, 0x10, 0x00, 0x52, 0x00, 0x03, 0, 0];
        let crc = crc16_modbus(&frame[..6]);
        frame[6] = crc as u8;
        frame[7] = (crc >> 8) as u8;
        assert!(matches!(
            parse_write_multiple_response(&frame, 0x01, 0x0050, 3),
            Err(ModbusError::BadHeader)
        ));
    }
}
