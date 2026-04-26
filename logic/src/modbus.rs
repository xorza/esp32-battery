//! Modbus-RTU frame building and parsing. Pure functions — no I/O — so they
//! can be unit-tested on host. The UART transport lives in the firmware's
//! `xy` module.

pub const FN_READ_HOLDING: u8 = 0x03;
pub const FN_WRITE_HOLDING: u8 = 0x06;

/// Standard Modbus-RTU CRC-16 (reflected polynomial 0xA001, seed 0xFFFF).
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

pub fn build_read_request(slave: u8, addr: u16, count: u16) -> [u8; 8] {
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_READ_HOLDING;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    req[4..6].copy_from_slice(&count.to_be_bytes());
    let crc = crc16_modbus(&req[..6]);
    req[6] = crc as u8;
    req[7] = (crc >> 8) as u8;
    req
}

pub fn build_write_request(slave: u8, addr: u16, value: u16) -> [u8; 8] {
    let mut req = [0u8; 8];
    req[0] = slave;
    req[1] = FN_WRITE_HOLDING;
    req[2..4].copy_from_slice(&addr.to_be_bytes());
    req[4..6].copy_from_slice(&value.to_be_bytes());
    let crc = crc16_modbus(&req[..6]);
    req[6] = crc as u8;
    req[7] = (crc >> 8) as u8;
    req
}

fn check_crc(resp: &[u8], expected_len: usize) -> Result<(), ModbusError> {
    let crc_got = u16::from_le_bytes([resp[expected_len - 2], resp[expected_len - 1]]);
    let crc_calc = crc16_modbus(&resp[..expected_len - 2]);
    if crc_got == crc_calc {
        Ok(())
    } else {
        Err(ModbusError::BadCrc)
    }
}

pub fn parse_read_response(resp: &[u8], slave: u8, count: u16) -> Result<Vec<u16>, ModbusError> {
    if resp.len() < 5 {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    if resp[0] != slave {
        return Err(ModbusError::BadSlave(resp[0]));
    }
    // Exception frame is 5 bytes. CRC must validate before trusting the code.
    if resp[1] & 0x80 != 0 {
        check_crc(resp, 5)?;
        return Err(ModbusError::Exception(resp[2]));
    }
    let expected_len = 5 + 2 * count as usize;
    if resp[1] != FN_READ_HOLDING || resp[2] as usize != 2 * count as usize {
        return Err(ModbusError::BadHeader);
    }
    if resp.len() < expected_len {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    check_crc(resp, expected_len)?;
    Ok((0..count as usize)
        .map(|i| u16::from_be_bytes([resp[3 + 2 * i], resp[4 + 2 * i]]))
        .collect())
}

pub fn parse_write_response(resp: &[u8], req: &[u8; 8]) -> Result<(), ModbusError> {
    if resp.len() < 5 {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    if resp[0] != req[0] {
        return Err(ModbusError::BadSlave(resp[0]));
    }
    if resp[1] & 0x80 != 0 {
        check_crc(resp, 5)?;
        return Err(ModbusError::Exception(resp[2]));
    }
    if resp.len() < 8 {
        return Err(ModbusError::ShortResponse(resp.len()));
    }
    check_crc(resp, 8)?;
    // Spec: successful write response echoes the request byte-for-byte.
    if resp[..8] != req[..] {
        return Err(ModbusError::BadHeader);
    }
    Ok(())
}

pub enum ModbusError {
    ShortResponse(usize),
    BadSlave(u8),
    BadHeader,
    BadCrc,
    Exception(u8),
}

impl std::fmt::Display for ModbusError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ModbusError::ShortResponse(n) => write!(f, "short response ({n} bytes)"),
            ModbusError::BadSlave(a) => write!(f, "wrong slave id 0x{a:02X}"),
            ModbusError::BadHeader => write!(f, "malformed header"),
            ModbusError::BadCrc => write!(f, "CRC mismatch"),
            ModbusError::Exception(c) => write!(f, "modbus exception 0x{c:02X}"),
        }
    }
}

/// Unified error for a Modbus-RTU transaction: transport (UART) or
/// protocol (codec). Lives here so consumers don't have to define
/// their own wrapper.
pub enum RtuError {
    UartRead,
    UartWrite,
    Modbus(ModbusError),
}

impl std::fmt::Display for RtuError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UartRead => write!(f, "UART read failed"),
            Self::UartWrite => write!(f, "UART write failed"),
            Self::Modbus(e) => std::fmt::Display::fmt(e, f),
        }
    }
}

impl From<ModbusError> for RtuError {
    fn from(e: ModbusError) -> Self {
        Self::Modbus(e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- CRC ---

    #[test]
    fn crc_empty() {
        assert_eq!(crc16_modbus(&[]), 0xFFFF);
    }

    #[test]
    fn crc_read_request() {
        // Read 1 reg at 0x001F from slave 1: on-wire trailing bytes B5 CC.
        assert_eq!(crc16_modbus(&[0x01, 0x03, 0x00, 0x1F, 0x00, 0x01]), 0xCCB5);
    }

    #[test]
    fn crc_write_request() {
        // Write 0x0001 to reg 0x0012 from slave 1: on-wire trailing bytes E8 0F.
        assert_eq!(crc16_modbus(&[0x01, 0x06, 0x00, 0x12, 0x00, 0x01]), 0x0FE8);
    }

    #[test]
    fn crc_single_byte() {
        assert_eq!(crc16_modbus(&[0x01]), 0x807E);
    }

    #[test]
    fn crc_detects_single_bit_flip() {
        let base = [0x01u8, 0x03, 0x00, 0x00, 0x00, 0x06];
        let base_crc = crc16_modbus(&base);
        for byte_idx in 0..base.len() {
            for bit in 0..8 {
                let mut flipped = base;
                flipped[byte_idx] ^= 1 << bit;
                assert_ne!(
                    crc16_modbus(&flipped),
                    base_crc,
                    "bit {bit} of byte {byte_idx}"
                );
            }
        }
    }

    // --- Request building ---

    #[test]
    fn build_read_matches_crc_vector() {
        let req = build_read_request(0x01, 0x001F, 0x0001);
        assert_eq!(req, [0x01, 0x03, 0x00, 0x1F, 0x00, 0x01, 0xB5, 0xCC]);
    }

    #[test]
    fn build_write_matches_crc_vector() {
        let req = build_write_request(0x01, 0x0012, 0x0001);
        assert_eq!(req, [0x01, 0x06, 0x00, 0x12, 0x00, 0x01, 0xE8, 0x0F]);
    }

    // --- Read response parsing ---

    fn read_resp(slave: u8, values: &[u16]) -> Vec<u8> {
        let mut out = Vec::new();
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
    fn parse_read_valid_single_reg() {
        let frame = read_resp(0x01, &[0x1234]);
        let Ok(out) = parse_read_response(&frame, 0x01, 1) else {
            panic!("expected Ok");
        };
        assert_eq!(out, vec![0x1234]);
    }

    #[test]
    fn parse_read_valid_six_regs() {
        let frame = read_resp(0x01, &[1360, 1000, 1350, 0, 0, 4800]);
        let Ok(out) = parse_read_response(&frame, 0x01, 6) else {
            panic!("expected Ok");
        };
        assert_eq!(out, vec![1360, 1000, 1350, 0, 0, 4800]);
    }

    #[test]
    fn parse_read_rejects_wrong_slave() {
        let frame = read_resp(0x02, &[0x1234]);
        assert!(matches!(
            parse_read_response(&frame, 0x01, 1),
            Err(ModbusError::BadSlave(0x02))
        ));
    }

    #[test]
    fn parse_read_rejects_bad_crc() {
        let mut frame = read_resp(0x01, &[0x1234]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        assert!(matches!(
            parse_read_response(&frame, 0x01, 1),
            Err(ModbusError::BadCrc)
        ));
    }

    #[test]
    fn parse_read_rejects_short() {
        let frame = [0x01, 0x03, 0x02, 0x00];
        assert!(matches!(
            parse_read_response(&frame, 0x01, 1),
            Err(ModbusError::ShortResponse(4))
        ));
    }

    #[test]
    fn parse_read_rejects_truncated_payload() {
        // Header claims 2 payload bytes; frame cut before CRC (5 bytes, need 7).
        let frame = [0x01, 0x03, 0x02, 0x12, 0x34];
        assert!(matches!(
            parse_read_response(&frame, 0x01, 1),
            Err(ModbusError::ShortResponse(5))
        ));
    }

    #[test]
    fn parse_read_rejects_wrong_function() {
        let mut frame = vec![0x01u8, 0x04, 0x02, 0x12, 0x34];
        let crc = crc16_modbus(&frame);
        frame.push(crc as u8);
        frame.push((crc >> 8) as u8);
        assert!(matches!(
            parse_read_response(&frame, 0x01, 1),
            Err(ModbusError::BadHeader)
        ));
    }

    #[test]
    fn parse_read_rejects_wrong_byte_count() {
        // Frame reports 1 register but caller expects 2.
        let frame = read_resp(0x01, &[0x1234]);
        assert!(matches!(
            parse_read_response(&frame, 0x01, 2),
            Err(ModbusError::BadHeader)
        ));
    }

    #[test]
    fn parse_read_exception_with_valid_crc() {
        let mut frame = vec![0x01u8, 0x83, 0x02]; // illegal data address
        let crc = crc16_modbus(&frame);
        frame.push(crc as u8);
        frame.push((crc >> 8) as u8);
        assert!(matches!(
            parse_read_response(&frame, 0x01, 1),
            Err(ModbusError::Exception(0x02))
        ));
    }

    #[test]
    fn parse_read_exception_with_bad_crc_is_bad_crc() {
        // Exception flag set, CRC is garbage — must NOT surface as Exception.
        let frame = [0x01u8, 0x83, 0x02, 0x00, 0x00];
        assert!(matches!(
            parse_read_response(&frame, 0x01, 1),
            Err(ModbusError::BadCrc)
        ));
    }

    // --- Write response parsing ---

    #[test]
    fn parse_write_valid_echo() {
        let req = build_write_request(0x01, 0x0012, 0x0001);
        assert!(parse_write_response(&req, &req).is_ok());
    }

    #[test]
    fn parse_write_rejects_value_mismatch() {
        let req = build_write_request(0x01, 0x0012, 0x0001);
        let mut resp = req;
        resp[5] = 0x02;
        let crc = crc16_modbus(&resp[..6]);
        resp[6] = crc as u8;
        resp[7] = (crc >> 8) as u8;
        assert!(matches!(
            parse_write_response(&resp, &req),
            Err(ModbusError::BadHeader)
        ));
    }

    #[test]
    fn parse_write_rejects_bad_crc() {
        let req = build_write_request(0x01, 0x0012, 0x0001);
        let mut resp = req;
        resp[7] ^= 0xFF;
        assert!(matches!(
            parse_write_response(&resp, &req),
            Err(ModbusError::BadCrc)
        ));
    }

    #[test]
    fn parse_write_exception_returns_exception_not_short() {
        // 5-byte exception. Previous firmware code returned ShortResponse here.
        let req = build_write_request(0x01, 0x0012, 0x0001);
        let mut frame = vec![0x01u8, 0x86, 0x03]; // 0x06|0x80, illegal value
        let crc = crc16_modbus(&frame);
        frame.push(crc as u8);
        frame.push((crc >> 8) as u8);
        assert!(matches!(
            parse_write_response(&frame, &req),
            Err(ModbusError::Exception(0x03))
        ));
    }

    #[test]
    fn parse_write_rejects_wrong_slave() {
        let req = build_write_request(0x01, 0x0012, 0x0001);
        let mut resp = req;
        resp[0] = 0x02;
        let crc = crc16_modbus(&resp[..6]);
        resp[6] = crc as u8;
        resp[7] = (crc >> 8) as u8;
        assert!(matches!(
            parse_write_response(&resp, &req),
            Err(ModbusError::BadSlave(0x02))
        ));
    }

    #[test]
    fn parse_write_rejects_short() {
        let req = build_write_request(0x01, 0x0012, 0x0001);
        let frame = [0x01u8, 0x06, 0x00, 0x12];
        assert!(matches!(
            parse_write_response(&frame, &req),
            Err(ModbusError::ShortResponse(4))
        ));
    }
}
