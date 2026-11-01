//! Frame = header(version, msg_type, payload_len u16 LE) + payload + crc16 LE.
//! CRC-16/CCITT-FALSE over header+payload.

use super::{DecodeError, MAX_FRAME_LEN, PROTOCOL_VERSION};
use crc::{Crc, CRC_16_IBM_3740};

/// Header bytes before the payload.
pub const HEADER_LEN: usize = 4;
/// CRC bytes after the payload.
pub const CRC_LEN: usize = 2;
/// Largest payload that fits in [`MAX_FRAME_LEN`].
pub const MAX_PAYLOAD_LEN: usize = MAX_FRAME_LEN - HEADER_LEN - CRC_LEN;

// CRC-16/IBM-3740 is the catalogue name of CRC-16/CCITT-FALSE.
const CRC16: Crc<u16> = Crc::<u16>::new(&CRC_16_IBM_3740);

/// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
pub fn crc16(data: &[u8]) -> u16 {
    CRC16.checksum(data)
}

/// Build a decoded frame (header + payload + crc).
///
/// Payloads come from the fixed message layouts and are always far below
/// [`MAX_PAYLOAD_LEN`]; a larger payload is a programming error.
pub fn build(msg_type: u8, payload: &[u8]) -> Vec<u8> {
    debug_assert!(payload.len() <= MAX_PAYLOAD_LEN, "payload too long");
    let len = payload.len() as u16;
    let mut out = Vec::with_capacity(HEADER_LEN + payload.len() + CRC_LEN);
    out.push(PROTOCOL_VERSION);
    out.push(msg_type);
    out.extend_from_slice(&len.to_le_bytes());
    out.extend_from_slice(payload);
    let crc = crc16(&out);
    out.extend_from_slice(&crc.to_le_bytes());
    out
}

/// Validate a decoded frame and return (msg_type, payload).
///
/// Check order: length bounds → CRC → version → payload_len field.
/// The message type is not checked here (see `decode_payload`).
pub fn parse(frame: &[u8]) -> Result<(u8, &[u8]), DecodeError> {
    if frame.len() < HEADER_LEN + CRC_LEN {
        return Err(DecodeError::TooShort);
    }
    if frame.len() > MAX_FRAME_LEN {
        return Err(DecodeError::TooLong);
    }
    let (body, crc_bytes) = frame.split_at(frame.len() - CRC_LEN);
    let rx_crc = u16::from_le_bytes([crc_bytes[0], crc_bytes[1]]);
    if crc16(body) != rx_crc {
        return Err(DecodeError::BadCrc);
    }
    let (header, payload) = body.split_at(HEADER_LEN);
    if header[0] != PROTOCOL_VERSION {
        return Err(DecodeError::BadVersion(header[0]));
    }
    let payload_len = u16::from_le_bytes([header[2], header[3]]) as usize;
    if payload_len != payload.len() {
        return Err(DecodeError::LengthMismatch);
    }
    Ok((header[1], payload))
}

/// build + COBS encode + 0x00 delimiter.
pub fn to_wire(msg_type: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = super::cobs::encode(&build(msg_type, payload));
    out.push(0);
    out
}
