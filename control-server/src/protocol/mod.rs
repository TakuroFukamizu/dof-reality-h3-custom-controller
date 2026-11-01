//! PC ⇄ CoreS3 USB protocol v1. The normative spec is
//! `docs/control-server/protocol.md`; keep the two in lockstep.
//!
//! Layering:
//! - [`cobs`]: COBS byte stuffing + `0x00` delimiting, with a streaming
//!   [`cobs::FrameAccumulator`] that resynchronizes after garbage/oversize input.
//! - [`frame`]: header + payload + CRC-16/CCITT-FALSE.
//! - this module: typed messages and their fixed little-endian payload layouts.

pub mod cobs;
pub mod frame;

use crate::types::{Counts, DeviceId, FaultCode};
use serde::Serialize;

pub const PROTOCOL_VERSION: u8 = 1;
/// Maximum decoded frame (header + payload + crc).
pub const MAX_FRAME_LEN: usize = 256;
/// Maximum COBS-encoded frame excluding the 0x00 delimiter.
pub const MAX_ENCODED_LEN: usize = 300;

/// Message type ids (protocol §4).
pub mod msg_type {
    pub const HELLO_REQ: u8 = 0x01;
    pub const CONFIG: u8 = 0x02;
    pub const ARM_PREPARE: u8 = 0x03;
    pub const START: u8 = 0x04;
    pub const SET_POSITION: u8 = 0x05;
    pub const CONTROLLED_STOP: u8 = 0x06;
    pub const DISARM: u8 = 0x07;
    pub const FAULT_RESET: u8 = 0x08;
    pub const CLOCK_SYNC: u8 = 0x09;
    pub const HOST_FAULT: u8 = 0x0A;
    pub const SET_REFERENCE: u8 = 0x0B;
    pub const CALIBRATE: u8 = 0x0C;

    pub const HELLO: u8 = 0x81;
    pub const ACK: u8 = 0x82;
    pub const STATUS: u8 = 0x83;
    pub const FAULT_EVENT: u8 = 0x84;
    pub const CLOCK_SYNC_REPLY: u8 = 0x85;
    pub const AXIS_SELECT: u8 = 0x86;
    pub const CAL_RESULT: u8 = 0x87;
}

/// ACK result (protocol §3.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AckResult {
    Accepted = 0,
    Rejected = 1,
    Duplicate = 2,
}

/// ACK reject reason (protocol §3.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AckReason {
    None = 0,
    BadState = 1,
    EpochMismatch = 2,
    NotConfigured = 3,
    InvalidParam = 4,
    NotReferenced = 5,
    FaultActive = 6,
    FaultCausePresent = 7,
}

/// CALIBRATE `mode` (protocol §4 `0x0C`).
pub mod calibrate_mode {
    pub const CANCEL: u8 = 0;
    pub const START: u8 = 1;
}

/// CALIBRATE `flags` (protocol §4 `0x0C`, byte 5). Only the power-controller
/// axis (CONFIG `role_flags` bit0) interprets them; other axes ignore them.
/// Bits 1..7 are reserved (sent as 0, ignored on decode).
pub mod calibrate_flags {
    /// The power axis keeps the main power on after its own calibration
    /// (stays CALIBRATING in POWER_HOLD) until CALIBRATE(0) or the firmware
    /// hold timeout (protocol §1.2, DESIGN.md §11).
    pub const KEEP_POWER: u8 = 0x01;
}

/// CAL_RESULT `result` (protocol §4 `0x87`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum CalResultCode {
    Ok = 0,
    Failed = 1,
    Cancelled = 2,
}

impl CalResultCode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => CalResultCode::Ok,
            1 => CalResultCode::Failed,
            2 => CalResultCode::Cancelled,
            _ => return None,
        })
    }
    pub fn to_u8(self) -> u8 {
        self as u8
    }
}

/// Length of the CAL_RESULT `reason` field (ASCII, NUL padded).
pub const CAL_REASON_LEN: usize = 8;

/// Role flags in CONFIG.
pub mod role_flags {
    pub const POWER_CONTROLLER: u8 = 1 << 0;
}

/// HELLO capability bits.
pub mod capabilities {
    pub const POSITION_MODE: u32 = 1 << 0;
    pub const APPLY_AT: u32 = 1 << 1;
    pub const CURRENT_SENSE: u32 = 1 << 2;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Config {
    pub request_id: u32,
    pub session_epoch: u32,
    pub calibration_hash: u32,
    pub soft_min_count: Counts,
    pub soft_max_count: Counts,
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
    pub following_error_limit_count: u32,
    pub overspeed_count_s: u32,
    pub command_timeout_ms: u16,
    pub stop_timeout_ms: u16,
    pub role_flags: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct SetPosition {
    pub session_epoch: u32,
    pub seq: u32,
    pub group_frame_id: u32,
    pub target_count: Counts,
    pub max_vel_count_s: u32,
    pub max_acc_count_s2: u32,
    pub valid_for_ms: u16,
    /// 0 = apply on receipt. Non-zero requires a valid clock sync (not in MVP).
    pub apply_at_us: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Hello {
    pub request_id: u32,
    pub protocol_version: u8,
    /// Axis (1..=3) the operator selected on the MCU, 0 = none yet. The server
    /// assigns the port from this (protocol §1.1); `device_id` is only a
    /// configured cross-check.
    pub selected_axis: u8,
    /// Raw MCU state byte (see [`crate::types::McuState::from_u8`]).
    pub mcu_state: u8,
    pub device_id: DeviceId,
    pub boot_id: u32,
    pub fw_version: [u8; 3],
    pub capabilities: u32,
    pub calibration_hash: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Ack {
    pub request_id: u32,
    pub acked_type: u8,
    pub result: AckResult,
    pub reason: AckReason,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Status {
    pub boot_id: u32,
    pub mcu_time_us: u64,
    /// Raw MCU state byte (see [`crate::types::McuState::from_u8`]).
    pub mcu_state: u8,
    /// Same value as HELLO (offset 13, was reserved; old firmware sends 0).
    pub selected_axis: u8,
    pub fault_code: FaultCode,
    pub flags: u16,
    pub pwm_permille: i16,
    pub actual_count: Counts,
    pub target_count: Counts,
    pub velocity_count_s: i32,
    pub following_error_count: i32,
    pub last_applied_frame: u32,
    pub last_applied_seq: u32,
    pub last_rx_seq: u32,
    /// Meaningful only when `flags & CURRENT_VALID`.
    pub current_ma: i32,
    pub rx_error_count: u16,
    pub stale_drop_count: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct FaultEvent {
    pub boot_id: u32,
    pub mcu_time_us: u64,
    pub fault_code: FaultCode,
    pub mcu_state_before: u8,
    pub actual_count: Counts,
}

/// Sent once when the operator switches the axis on the MCU (protocol §1.1,
/// §4 `0x86`). No ACK. The PC also notices the switch from the next HELLO /
/// STATUS `selected_axis`, so a lost AXIS_SELECT is caught up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct AxisSelect {
    pub boot_id: u32,
    /// New value (1..=3, 0 = none).
    pub selected_axis: u8,
    pub prev_selected_axis: u8,
    /// Raw MCU state byte at the switch (DISARMED under the MCU rules).
    pub mcu_state: u8,
}

/// Sent once when an end-stop calibration ends (protocol §1.2, §4 `0x87`):
/// after the shutdown sequence and before the STATUS showing DISARMED.
/// Counts are in the MCU's `actual_count` coordinate with the P end at 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct CalResult {
    pub boot_id: u32,
    pub result: CalResultCode,
    /// Step reached: 0 none, 1 to P end, 2 to N end, 3 to neutral.
    pub step: u8,
    /// Count of the N end (positive for sign +1, negative for sign −1).
    pub n_end_count: Counts,
    pub neutral_count: Counts,
    /// Sign of the unit's axis profile (±1).
    pub sign: i8,
    /// ASCII reason, NUL padded (`ok`, `stall`, `slow`, `travel`, `timeout`,
    /// `stop`, `estop`, …). See [`CalResult::reason_str`].
    #[serde(serialize_with = "serialize_reason")]
    pub reason: [u8; CAL_REASON_LEN],
}

impl CalResult {
    /// All-zero placeholder (struct update syntax in tests / helpers).
    pub const EMPTY: CalResult = CalResult {
        boot_id: 0,
        result: CalResultCode::Failed,
        step: 0,
        n_end_count: 0,
        neutral_count: 0,
        sign: 0,
        reason: [0; CAL_REASON_LEN],
    };

    /// `reason` up to the first NUL, lossily as UTF-8.
    pub fn reason_str(&self) -> String {
        reason_str(&self.reason)
    }

    /// Build the fixed `reason` field from text (truncated to 8 bytes, NUL padded).
    pub fn reason_bytes(text: &str) -> [u8; CAL_REASON_LEN] {
        let mut out = [0u8; CAL_REASON_LEN];
        for (dst, src) in out.iter_mut().zip(text.bytes()) {
            *dst = src;
        }
        out
    }
}

fn reason_str(r: &[u8; CAL_REASON_LEN]) -> String {
    let end = r.iter().position(|&b| b == 0).unwrap_or(CAL_REASON_LEN);
    String::from_utf8_lossy(&r[..end]).into_owned()
}

fn serialize_reason<S: serde::Serializer>(r: &[u8; CAL_REASON_LEN], s: S) -> Result<S::Ok, S::Error> {
    s.serialize_str(&reason_str(r))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct ClockSyncReply {
    pub request_id: u32,
    pub pc_time_us: u64,
    pub mcu_rx_us: u64,
    pub mcu_tx_us: u64,
}

/// Messages the PC sends to an axis MCU.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(tag = "type", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum HostMsg {
    HelloReq {
        request_id: u32,
    },
    Config(Config),
    ArmPrepare {
        request_id: u32,
        session_epoch: u32,
    },
    Start {
        request_id: u32,
        session_epoch: u32,
    },
    SetPosition(SetPosition),
    ControlledStop {
        request_id: u32,
        session_epoch: u32,
        reason: u8,
    },
    Disarm {
        request_id: u32,
    },
    FaultReset {
        request_id: u32,
    },
    ClockSync {
        request_id: u32,
        pc_time_us: u64,
    },
    HostFault {
        request_id: u32,
        fault_code: FaultCode,
    },
    SetReference {
        request_id: u32,
        reference_count: Counts,
    },
    /// Start (`mode` 1) or cancel (`mode` 0) the MCU-driven end-stop
    /// calibration (protocol §1.2). `flags`: [`calibrate_flags`] (decoded as
    /// received; unknown bits are not an error).
    Calibrate {
        request_id: u32,
        mode: u8,
        flags: u8,
    },
}

/// Messages an axis MCU sends to the PC.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(tag = "type", rename_all = "SCREAMING_SNAKE_CASE")]
pub enum McuMsg {
    Hello(Hello),
    Ack(Ack),
    Status(Status),
    FaultEvent(FaultEvent),
    ClockSyncReply(ClockSyncReply),
    AxisSelect(AxisSelect),
    CalResult(CalResult),
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum DecodeError {
    #[error("frame shorter than header+crc")]
    TooShort,
    #[error("frame longer than {MAX_FRAME_LEN} bytes")]
    TooLong,
    #[error("crc mismatch")]
    BadCrc,
    #[error("unsupported protocol version {0}")]
    BadVersion(u8),
    #[error("payload_len field does not match frame length")]
    LengthMismatch,
    #[error("unknown message type 0x{0:02x}")]
    UnknownType(u8),
    #[error("payload length {got} invalid for type 0x{msg_type:02x} (expected {expected})")]
    BadPayloadLen {
        msg_type: u8,
        expected: usize,
        got: usize,
    },
    #[error("invalid field value: {0}")]
    BadField(&'static str),
    #[error("COBS decode error")]
    Cobs,
}

impl AckResult {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => AckResult::Accepted,
            1 => AckResult::Rejected,
            2 => AckResult::Duplicate,
            _ => return None,
        })
    }
    pub fn to_u8(self) -> u8 {
        self as u8
    }
}

impl AckReason {
    pub fn from_u16(v: u16) -> Option<Self> {
        use AckReason::*;
        Some(match v {
            0 => None,
            1 => BadState,
            2 => EpochMismatch,
            3 => NotConfigured,
            4 => InvalidParam,
            5 => NotReferenced,
            6 => FaultActive,
            7 => FaultCausePresent,
            _ => return Option::None,
        })
    }
    pub fn to_u16(self) -> u16 {
        self as u16
    }
}

/// Fixed payload length of a PC → MCU message type, `None` if unknown.
pub const fn host_payload_len(msg_type: u8) -> Option<usize> {
    use msg_type::*;
    Some(match msg_type {
        HELLO_REQ => 4,
        CONFIG => 44,
        ARM_PREPARE => 8,
        START => 8,
        SET_POSITION => 36,
        CONTROLLED_STOP => 12,
        DISARM => 4,
        FAULT_RESET => 4,
        CLOCK_SYNC => 12,
        HOST_FAULT => 8,
        SET_REFERENCE => 8,
        CALIBRATE => 8,
        _ => return None,
    })
}

/// Fixed payload length of a MCU → PC message type, `None` if unknown.
pub const fn mcu_payload_len(msg_type: u8) -> Option<usize> {
    use msg_type::*;
    Some(match msg_type {
        HELLO => 32,
        ACK => 8,
        STATUS => 56,
        FAULT_EVENT => 20,
        CLOCK_SYNC_REPLY => 28,
        AXIS_SELECT => 8,
        CAL_RESULT => 28,
        _ => return None,
    })
}

/// Little-endian payload writer.
struct Writer(Vec<u8>);

impl Writer {
    fn with_len(n: usize) -> Self {
        Writer(Vec::with_capacity(n))
    }
    fn u8(&mut self, v: u8) -> &mut Self {
        self.0.push(v);
        self
    }
    fn u16(&mut self, v: u16) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn i16(&mut self, v: i16) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn u32(&mut self, v: u32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn i32(&mut self, v: i32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn u64(&mut self, v: u64) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    fn bytes(&mut self, b: &[u8]) -> &mut Self {
        self.0.extend_from_slice(b);
        self
    }
    fn finish(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.0)
    }
}

/// Little-endian payload reader. Length is validated up front, but every read
/// is still bounds-checked so malformed input can never panic.
struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Reader { buf, pos: 0 }
    }
    fn take<const N: usize>(&mut self) -> Result<[u8; N], DecodeError> {
        let bytes = self
            .buf
            .get(self.pos..self.pos + N)
            .and_then(|s| <[u8; N]>::try_from(s).ok())
            .ok_or(DecodeError::TooShort)?;
        self.pos += N;
        Ok(bytes)
    }
    fn skip(&mut self, n: usize) -> Result<(), DecodeError> {
        if self.pos + n > self.buf.len() {
            return Err(DecodeError::TooShort);
        }
        self.pos += n;
        Ok(())
    }
    fn u8(&mut self) -> Result<u8, DecodeError> {
        Ok(self.take::<1>()?[0])
    }
    fn u16(&mut self) -> Result<u16, DecodeError> {
        Ok(u16::from_le_bytes(self.take()?))
    }
    fn i16(&mut self) -> Result<i16, DecodeError> {
        Ok(i16::from_le_bytes(self.take()?))
    }
    fn u32(&mut self) -> Result<u32, DecodeError> {
        Ok(u32::from_le_bytes(self.take()?))
    }
    fn i32(&mut self) -> Result<i32, DecodeError> {
        Ok(i32::from_le_bytes(self.take()?))
    }
    fn u64(&mut self) -> Result<u64, DecodeError> {
        Ok(u64::from_le_bytes(self.take()?))
    }
}

/// Check that `payload` has exactly the length defined for `msg_type`.
fn check_len(
    msg_type: u8,
    payload: &[u8],
    table: fn(u8) -> Option<usize>,
) -> Result<(), DecodeError> {
    let expected = table(msg_type).ok_or(DecodeError::UnknownType(msg_type))?;
    if payload.len() != expected {
        return Err(DecodeError::BadPayloadLen {
            msg_type,
            expected,
            got: payload.len(),
        });
    }
    Ok(())
}

impl HostMsg {
    /// The `msg_type` byte for this message.
    pub fn msg_type(&self) -> u8 {
        use self::msg_type as t;
        match self {
            HostMsg::HelloReq { .. } => t::HELLO_REQ,
            HostMsg::Config(_) => t::CONFIG,
            HostMsg::ArmPrepare { .. } => t::ARM_PREPARE,
            HostMsg::Start { .. } => t::START,
            HostMsg::SetPosition(_) => t::SET_POSITION,
            HostMsg::ControlledStop { .. } => t::CONTROLLED_STOP,
            HostMsg::Disarm { .. } => t::DISARM,
            HostMsg::FaultReset { .. } => t::FAULT_RESET,
            HostMsg::ClockSync { .. } => t::CLOCK_SYNC,
            HostMsg::HostFault { .. } => t::HOST_FAULT,
            HostMsg::SetReference { .. } => t::SET_REFERENCE,
            HostMsg::Calibrate { .. } => t::CALIBRATE,
        }
    }

    /// Encode payload bytes only (no header/crc/COBS), fixed LE layout per spec.
    /// Reserved fields are written as 0.
    pub fn encode_payload(&self) -> Vec<u8> {
        let mut w = Writer::with_len(host_payload_len(self.msg_type()).unwrap_or(0));
        match *self {
            HostMsg::HelloReq { request_id }
            | HostMsg::Disarm { request_id }
            | HostMsg::FaultReset { request_id } => w.u32(request_id),
            HostMsg::Config(c) => w
                .u32(c.request_id)
                .u32(c.session_epoch)
                .u32(c.calibration_hash)
                .i32(c.soft_min_count)
                .i32(c.soft_max_count)
                .u32(c.max_vel_count_s)
                .u32(c.max_acc_count_s2)
                .u32(c.following_error_limit_count)
                .u32(c.overspeed_count_s)
                .u16(c.command_timeout_ms)
                .u16(c.stop_timeout_ms)
                .u8(c.role_flags)
                .u8(0)
                .u16(0),
            HostMsg::ArmPrepare {
                request_id,
                session_epoch,
            }
            | HostMsg::Start {
                request_id,
                session_epoch,
            } => w.u32(request_id).u32(session_epoch),
            HostMsg::SetPosition(s) => w
                .u32(s.session_epoch)
                .u32(s.seq)
                .u32(s.group_frame_id)
                .i32(s.target_count)
                .u32(s.max_vel_count_s)
                .u32(s.max_acc_count_s2)
                .u16(s.valid_for_ms)
                .u16(0)
                .u64(s.apply_at_us),
            HostMsg::ControlledStop {
                request_id,
                session_epoch,
                reason,
            } => w.u32(request_id).u32(session_epoch).u8(reason).u8(0).u16(0),
            HostMsg::ClockSync {
                request_id,
                pc_time_us,
            } => w.u32(request_id).u64(pc_time_us),
            HostMsg::HostFault {
                request_id,
                fault_code,
            } => w.u32(request_id).u16(fault_code.to_u16()).u16(0),
            HostMsg::SetReference {
                request_id,
                reference_count,
            } => w.u32(request_id).i32(reference_count),
            HostMsg::Calibrate {
                request_id,
                mode,
                flags,
            } => w.u32(request_id).u8(mode).u8(flags).u16(0),
        };
        w.finish()
    }

    /// Decode from `msg_type` + payload (used by the MCU-side simulator).
    /// Reserved fields are ignored.
    pub fn decode_payload(msg_type: u8, payload: &[u8]) -> Result<Self, DecodeError> {
        use self::msg_type as t;
        check_len(msg_type, payload, host_payload_len)?;
        let mut r = Reader::new(payload);
        Ok(match msg_type {
            t::HELLO_REQ => HostMsg::HelloReq {
                request_id: r.u32()?,
            },
            t::CONFIG => {
                let c = Config {
                    request_id: r.u32()?,
                    session_epoch: r.u32()?,
                    calibration_hash: r.u32()?,
                    soft_min_count: r.i32()?,
                    soft_max_count: r.i32()?,
                    max_vel_count_s: r.u32()?,
                    max_acc_count_s2: r.u32()?,
                    following_error_limit_count: r.u32()?,
                    overspeed_count_s: r.u32()?,
                    command_timeout_ms: r.u16()?,
                    stop_timeout_ms: r.u16()?,
                    role_flags: r.u8()?,
                };
                r.skip(3)?;
                HostMsg::Config(c)
            }
            t::ARM_PREPARE => HostMsg::ArmPrepare {
                request_id: r.u32()?,
                session_epoch: r.u32()?,
            },
            t::START => HostMsg::Start {
                request_id: r.u32()?,
                session_epoch: r.u32()?,
            },
            t::SET_POSITION => {
                let session_epoch = r.u32()?;
                let seq = r.u32()?;
                let group_frame_id = r.u32()?;
                let target_count = r.i32()?;
                let max_vel_count_s = r.u32()?;
                let max_acc_count_s2 = r.u32()?;
                let valid_for_ms = r.u16()?;
                r.skip(2)?;
                let apply_at_us = r.u64()?;
                HostMsg::SetPosition(SetPosition {
                    session_epoch,
                    seq,
                    group_frame_id,
                    target_count,
                    max_vel_count_s,
                    max_acc_count_s2,
                    valid_for_ms,
                    apply_at_us,
                })
            }
            t::CONTROLLED_STOP => {
                let m = HostMsg::ControlledStop {
                    request_id: r.u32()?,
                    session_epoch: r.u32()?,
                    reason: r.u8()?,
                };
                r.skip(3)?;
                m
            }
            t::DISARM => HostMsg::Disarm {
                request_id: r.u32()?,
            },
            t::FAULT_RESET => HostMsg::FaultReset {
                request_id: r.u32()?,
            },
            t::CLOCK_SYNC => HostMsg::ClockSync {
                request_id: r.u32()?,
                pc_time_us: r.u64()?,
            },
            t::HOST_FAULT => {
                let m = HostMsg::HostFault {
                    request_id: r.u32()?,
                    fault_code: FaultCode::from_u16(r.u16()?),
                };
                r.skip(2)?;
                m
            }
            t::SET_REFERENCE => HostMsg::SetReference {
                request_id: r.u32()?,
                reference_count: r.i32()?,
            },
            t::CALIBRATE => {
                let m = HostMsg::Calibrate {
                    request_id: r.u32()?,
                    mode: r.u8()?,
                    flags: r.u8()?,
                };
                r.skip(2)?;
                m
            }
            other => return Err(DecodeError::UnknownType(other)),
        })
    }

    /// Full wire bytes: COBS(header+payload+crc) followed by 0x00.
    pub fn to_wire(&self) -> Vec<u8> {
        frame::to_wire(self.msg_type(), &self.encode_payload())
    }
}

impl McuMsg {
    pub fn msg_type(&self) -> u8 {
        use self::msg_type as t;
        match self {
            McuMsg::Hello(_) => t::HELLO,
            McuMsg::Ack(_) => t::ACK,
            McuMsg::Status(_) => t::STATUS,
            McuMsg::FaultEvent(_) => t::FAULT_EVENT,
            McuMsg::ClockSyncReply(_) => t::CLOCK_SYNC_REPLY,
            McuMsg::AxisSelect(_) => t::AXIS_SELECT,
            McuMsg::CalResult(_) => t::CAL_RESULT,
        }
    }

    /// Encode payload bytes only, fixed LE layout per spec. Reserved fields = 0.
    pub fn encode_payload(&self) -> Vec<u8> {
        let mut w = Writer::with_len(mcu_payload_len(self.msg_type()).unwrap_or(0));
        match *self {
            McuMsg::Hello(h) => w
                .u32(h.request_id)
                .u8(h.protocol_version)
                .u8(h.selected_axis)
                .u8(h.mcu_state)
                .u8(0)
                .u64(h.device_id.0)
                .u32(h.boot_id)
                .u8(h.fw_version[0])
                .u8(h.fw_version[1])
                .u8(h.fw_version[2])
                .u8(0)
                .u32(h.capabilities)
                .u32(h.calibration_hash),
            McuMsg::Ack(a) => w
                .u32(a.request_id)
                .u8(a.acked_type)
                .u8(a.result.to_u8())
                .u16(a.reason.to_u16()),
            McuMsg::Status(s) => w
                .u32(s.boot_id)
                .u64(s.mcu_time_us)
                .u8(s.mcu_state)
                .u8(s.selected_axis)
                .u16(s.fault_code.to_u16())
                .u16(s.flags)
                .i16(s.pwm_permille)
                .i32(s.actual_count)
                .i32(s.target_count)
                .i32(s.velocity_count_s)
                .i32(s.following_error_count)
                .u32(s.last_applied_frame)
                .u32(s.last_applied_seq)
                .u32(s.last_rx_seq)
                .i32(s.current_ma)
                .u16(s.rx_error_count)
                .u16(s.stale_drop_count),
            McuMsg::FaultEvent(f) => w
                .u32(f.boot_id)
                .u64(f.mcu_time_us)
                .u16(f.fault_code.to_u16())
                .u8(f.mcu_state_before)
                .u8(0)
                .i32(f.actual_count),
            McuMsg::ClockSyncReply(c) => w
                .u32(c.request_id)
                .u64(c.pc_time_us)
                .u64(c.mcu_rx_us)
                .u64(c.mcu_tx_us),
            McuMsg::AxisSelect(a) => w
                .u32(a.boot_id)
                .u8(a.selected_axis)
                .u8(a.prev_selected_axis)
                .u8(a.mcu_state)
                .u8(0),
            McuMsg::CalResult(c) => w
                .u32(c.boot_id)
                .u8(c.result.to_u8())
                .u8(c.step)
                .u16(0)
                .i32(c.n_end_count)
                .i32(c.neutral_count)
                .u8(c.sign as u8)
                .u8(0)
                .u16(0)
                .bytes(&c.reason),
        };
        w.finish()
    }

    /// Decode from `msg_type` + payload. Reserved fields are ignored; unknown
    /// ACK result/reason → `BadField`; unknown fault codes → `FaultCode::Unknown`;
    /// `mcu_state` is kept raw.
    pub fn decode_payload(msg_type: u8, payload: &[u8]) -> Result<Self, DecodeError> {
        use self::msg_type as t;
        check_len(msg_type, payload, mcu_payload_len)?;
        let mut r = Reader::new(payload);
        Ok(match msg_type {
            t::HELLO => {
                let request_id = r.u32()?;
                let protocol_version = r.u8()?;
                let selected_axis = r.u8()?;
                let mcu_state = r.u8()?;
                r.skip(1)?;
                let device_id = DeviceId(r.u64()?);
                let boot_id = r.u32()?;
                let fw_version = [r.u8()?, r.u8()?, r.u8()?];
                r.skip(1)?;
                McuMsg::Hello(Hello {
                    request_id,
                    protocol_version,
                    selected_axis,
                    mcu_state,
                    device_id,
                    boot_id,
                    fw_version,
                    capabilities: r.u32()?,
                    calibration_hash: r.u32()?,
                })
            }
            t::ACK => McuMsg::Ack(Ack {
                request_id: r.u32()?,
                acked_type: r.u8()?,
                result: AckResult::from_u8(r.u8()?).ok_or(DecodeError::BadField("ack.result"))?,
                reason: AckReason::from_u16(r.u16()?).ok_or(DecodeError::BadField("ack.reason"))?,
            }),
            t::STATUS => {
                let boot_id = r.u32()?;
                let mcu_time_us = r.u64()?;
                let mcu_state = r.u8()?;
                let selected_axis = r.u8()?;
                McuMsg::Status(Status {
                    boot_id,
                    mcu_time_us,
                    mcu_state,
                    selected_axis,
                    fault_code: FaultCode::from_u16(r.u16()?),
                    flags: r.u16()?,
                    pwm_permille: r.i16()?,
                    actual_count: r.i32()?,
                    target_count: r.i32()?,
                    velocity_count_s: r.i32()?,
                    following_error_count: r.i32()?,
                    last_applied_frame: r.u32()?,
                    last_applied_seq: r.u32()?,
                    last_rx_seq: r.u32()?,
                    current_ma: r.i32()?,
                    rx_error_count: r.u16()?,
                    stale_drop_count: r.u16()?,
                })
            }
            t::FAULT_EVENT => {
                let boot_id = r.u32()?;
                let mcu_time_us = r.u64()?;
                let fault_code = FaultCode::from_u16(r.u16()?);
                let mcu_state_before = r.u8()?;
                r.skip(1)?;
                McuMsg::FaultEvent(FaultEvent {
                    boot_id,
                    mcu_time_us,
                    fault_code,
                    mcu_state_before,
                    actual_count: r.i32()?,
                })
            }
            t::CLOCK_SYNC_REPLY => McuMsg::ClockSyncReply(ClockSyncReply {
                request_id: r.u32()?,
                pc_time_us: r.u64()?,
                mcu_rx_us: r.u64()?,
                mcu_tx_us: r.u64()?,
            }),
            t::AXIS_SELECT => {
                let a = AxisSelect {
                    boot_id: r.u32()?,
                    selected_axis: r.u8()?,
                    prev_selected_axis: r.u8()?,
                    mcu_state: r.u8()?,
                };
                r.skip(1)?;
                McuMsg::AxisSelect(a)
            }
            t::CAL_RESULT => {
                let boot_id = r.u32()?;
                let result = CalResultCode::from_u8(r.u8()?)
                    .ok_or(DecodeError::BadField("cal_result.result"))?;
                let step = r.u8()?;
                r.skip(2)?;
                let n_end_count = r.i32()?;
                let neutral_count = r.i32()?;
                let sign = r.u8()? as i8;
                r.skip(3)?;
                let reason: [u8; CAL_REASON_LEN] = r.take()?;
                McuMsg::CalResult(CalResult {
                    boot_id,
                    result,
                    step,
                    n_end_count,
                    neutral_count,
                    sign,
                    reason,
                })
            }
            other => return Err(DecodeError::UnknownType(other)),
        })
    }

    pub fn to_wire(&self) -> Vec<u8> {
        frame::to_wire(self.msg_type(), &self.encode_payload())
    }
}

/// COBS-decode + parse one encoded frame (delimiter stripped) into raw parts.
fn decode_frame_parts(encoded: &[u8]) -> Result<(u8, Vec<u8>), DecodeError> {
    if encoded.len() > MAX_ENCODED_LEN {
        return Err(DecodeError::TooLong);
    }
    let frame = cobs::decode(encoded).ok_or(DecodeError::Cobs)?;
    let (msg_type, payload) = frame::parse(&frame)?;
    Ok((msg_type, payload.to_vec()))
}

/// Decode one MCU → PC frame as produced by [`cobs::FrameAccumulator`]
/// (COBS-encoded, without the 0x00 delimiter).
pub fn decode_mcu_frame(encoded: &[u8]) -> Result<McuMsg, DecodeError> {
    let (t, payload) = decode_frame_parts(encoded)?;
    McuMsg::decode_payload(t, &payload)
}

/// Decode one PC → MCU frame (COBS-encoded, without the 0x00 delimiter).
pub fn decode_host_frame(encoded: &[u8]) -> Result<HostMsg, DecodeError> {
    let (t, payload) = decode_frame_parts(encoded)?;
    HostMsg::decode_payload(t, &payload)
}
