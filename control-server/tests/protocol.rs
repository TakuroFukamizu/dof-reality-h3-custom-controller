//! USB protocol v1 codec tests: COBS, framing/CRC, message layouts, resync.

use h3cs::protocol::cobs::{self, FrameAccumulator};
use h3cs::protocol::frame;
use h3cs::protocol::*;
use h3cs::types::{DeviceId, FaultCode};

// ---------- helpers ----------

/// Deterministic xorshift64* PRNG (no extra deps).
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn u8(&mut self) -> u8 {
        self.next() as u8
    }
    fn u16(&mut self) -> u16 {
        self.next() as u16
    }
    fn u32(&mut self) -> u32 {
        self.next() as u32
    }
    fn i32(&mut self) -> i32 {
        self.next() as i32
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
    /// Bias toward edge values so boundaries get hit often.
    fn edgy_u32(&mut self) -> u32 {
        match self.below(6) {
            0 => 0,
            1 => u32::MAX,
            2 => 1,
            _ => self.u32(),
        }
    }
    fn edgy_i32(&mut self) -> i32 {
        match self.below(6) {
            0 => 0,
            1 => i32::MIN,
            2 => i32::MAX,
            3 => -1,
            _ => self.i32(),
        }
    }
    fn edgy_u64(&mut self) -> u64 {
        match self.below(5) {
            0 => 0,
            1 => u64::MAX,
            _ => self.next(),
        }
    }
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.u8()).collect()
    }
}

/// Hand-built frame with arbitrary header fields and a correct CRC, so tests of
/// version/type/length checks are not masked by a CRC failure.
fn raw_frame(version: u8, msg_type: u8, payload_len_field: u16, payload: &[u8]) -> Vec<u8> {
    let mut f = vec![version, msg_type];
    f.extend_from_slice(&payload_len_field.to_le_bytes());
    f.extend_from_slice(payload);
    let crc = frame::crc16(&f);
    f.extend_from_slice(&crc.to_le_bytes());
    f
}

fn encoded(frame_bytes: &[u8]) -> Vec<u8> {
    cobs::encode(frame_bytes)
}

fn wire_to_encoded(wire: &[u8]) -> &[u8] {
    assert_eq!(wire.last(), Some(&0));
    &wire[..wire.len() - 1]
}

const ALL_FAULTS: [FaultCode; 14] = [
    FaultCode::None,
    FaultCode::CommandTimeout,
    FaultCode::FollowingError,
    FaultCode::Overspeed,
    FaultCode::SoftLimit,
    FaultCode::EncoderFault,
    FaultCode::ControlOverrun,
    FaultCode::ConfigInvalid,
    FaultCode::StopTimeout,
    FaultCode::PowerFault,
    FaultCode::HostFault,
    FaultCode::NotReferenced,
    FaultCode::LocalEstop,
    FaultCode::Unknown,
];

fn rand_host(r: &mut Rng) -> HostMsg {
    match r.below(12) {
        0 => HostMsg::HelloReq {
            request_id: r.edgy_u32(),
        },
        1 => HostMsg::Config(Config {
            request_id: r.edgy_u32(),
            session_epoch: r.edgy_u32(),
            calibration_hash: r.edgy_u32(),
            soft_min_count: r.edgy_i32(),
            soft_max_count: r.edgy_i32(),
            max_vel_count_s: r.edgy_u32(),
            max_acc_count_s2: r.edgy_u32(),
            following_error_limit_count: r.edgy_u32(),
            overspeed_count_s: r.edgy_u32(),
            command_timeout_ms: r.u16(),
            stop_timeout_ms: r.u16(),
            role_flags: r.u8(),
        }),
        2 => HostMsg::ArmPrepare {
            request_id: r.edgy_u32(),
            session_epoch: r.edgy_u32(),
        },
        3 => HostMsg::Start {
            request_id: r.edgy_u32(),
            session_epoch: r.edgy_u32(),
        },
        4 => HostMsg::SetPosition(SetPosition {
            session_epoch: r.edgy_u32(),
            seq: r.edgy_u32(),
            group_frame_id: r.edgy_u32(),
            target_count: r.edgy_i32(),
            max_vel_count_s: r.edgy_u32(),
            max_acc_count_s2: r.edgy_u32(),
            valid_for_ms: r.u16(),
            apply_at_us: r.edgy_u64(),
        }),
        5 => HostMsg::ControlledStop {
            request_id: r.edgy_u32(),
            session_epoch: r.edgy_u32(),
            reason: r.u8(),
        },
        6 => HostMsg::Disarm {
            request_id: r.edgy_u32(),
        },
        7 => HostMsg::FaultReset {
            request_id: r.edgy_u32(),
        },
        8 => HostMsg::ClockSync {
            request_id: r.edgy_u32(),
            pc_time_us: r.edgy_u64(),
        },
        9 => HostMsg::HostFault {
            request_id: r.edgy_u32(),
            fault_code: ALL_FAULTS[r.below(ALL_FAULTS.len() as u64) as usize],
        },
        10 => HostMsg::SetReference {
            request_id: r.edgy_u32(),
            reference_count: r.edgy_i32(),
        },
        _ => HostMsg::Calibrate {
            request_id: r.edgy_u32(),
            mode: r.u8(),
            flags: r.u8(),
        },
    }
}

const ALL_RESULTS: [AckResult; 3] = [
    AckResult::Accepted,
    AckResult::Rejected,
    AckResult::Duplicate,
];
const ALL_REASONS: [AckReason; 8] = [
    AckReason::None,
    AckReason::BadState,
    AckReason::EpochMismatch,
    AckReason::NotConfigured,
    AckReason::InvalidParam,
    AckReason::NotReferenced,
    AckReason::FaultActive,
    AckReason::FaultCausePresent,
];

const ALL_CAL_RESULTS: [CalResultCode; 3] = [CalResultCode::Ok, CalResultCode::Failed, CalResultCode::Cancelled];

fn rand_mcu(r: &mut Rng) -> McuMsg {
    match r.below(7) {
        0 => McuMsg::Hello(Hello {
            request_id: r.edgy_u32(),
            protocol_version: r.u8(),
            selected_axis: r.u8(),
            mcu_state: r.u8(),
            device_id: DeviceId(r.edgy_u64()),
            boot_id: r.edgy_u32(),
            fw_version: [r.u8(), r.u8(), r.u8()],
            capabilities: r.edgy_u32(),
            calibration_hash: r.edgy_u32(),
        }),
        1 => McuMsg::Ack(Ack {
            request_id: r.edgy_u32(),
            acked_type: r.u8(),
            result: ALL_RESULTS[r.below(3) as usize],
            reason: ALL_REASONS[r.below(8) as usize],
        }),
        2 => McuMsg::Status(Status {
            boot_id: r.edgy_u32(),
            mcu_time_us: r.edgy_u64(),
            mcu_state: r.u8(),
            selected_axis: r.u8(),
            fault_code: ALL_FAULTS[r.below(ALL_FAULTS.len() as u64) as usize],
            flags: r.u16(),
            pwm_permille: r.u16() as i16,
            actual_count: r.edgy_i32(),
            target_count: r.edgy_i32(),
            velocity_count_s: r.edgy_i32(),
            following_error_count: r.edgy_i32(),
            last_applied_frame: r.edgy_u32(),
            last_applied_seq: r.edgy_u32(),
            last_rx_seq: r.edgy_u32(),
            current_ma: r.edgy_i32(),
            rx_error_count: r.u16(),
            stale_drop_count: r.u16(),
        }),
        3 => McuMsg::FaultEvent(FaultEvent {
            boot_id: r.edgy_u32(),
            mcu_time_us: r.edgy_u64(),
            fault_code: ALL_FAULTS[r.below(ALL_FAULTS.len() as u64) as usize],
            mcu_state_before: r.u8(),
            actual_count: r.edgy_i32(),
        }),
        4 => McuMsg::ClockSyncReply(ClockSyncReply {
            request_id: r.edgy_u32(),
            pc_time_us: r.edgy_u64(),
            mcu_rx_us: r.edgy_u64(),
            mcu_tx_us: r.edgy_u64(),
        }),
        5 => McuMsg::AxisSelect(AxisSelect {
            boot_id: r.edgy_u32(),
            selected_axis: r.u8(),
            prev_selected_axis: r.u8(),
            mcu_state: r.u8(),
        }),
        _ => {
            let mut reason = [0u8; CAL_REASON_LEN];
            let n = r.below(CAL_REASON_LEN as u64 + 1) as usize;
            for b in reason.iter_mut().take(n) {
                *b = b'a' + (r.u8() % 26);
            }
            McuMsg::CalResult(CalResult {
                boot_id: r.edgy_u32(),
                result: ALL_CAL_RESULTS[r.below(3) as usize],
                step: r.u8(),
                n_end_count: r.edgy_i32(),
                neutral_count: r.edgy_i32(),
                sign: r.u8() as i8,
                reason,
            })
        }
    }
}

fn host_rt(m: HostMsg) {
    let wire = m.to_wire();
    assert_eq!(decode_host_frame(wire_to_encoded(&wire)), Ok(m), "{m:?}");
    let p = m.encode_payload();
    assert_eq!(Some(p.len()), host_payload_len(m.msg_type()), "{m:?}");
    assert_eq!(HostMsg::decode_payload(m.msg_type(), &p), Ok(m));
}

fn mcu_rt(m: McuMsg) {
    let wire = m.to_wire();
    assert_eq!(decode_mcu_frame(wire_to_encoded(&wire)), Ok(m), "{m:?}");
    let p = m.encode_payload();
    assert_eq!(Some(p.len()), mcu_payload_len(m.msg_type()), "{m:?}");
    assert_eq!(McuMsg::decode_payload(m.msg_type(), &p), Ok(m));
}

// ---------- CRC ----------

#[test]
fn crc16_ccitt_false_check_value() {
    assert_eq!(frame::crc16(b"123456789"), 0x29B1);
    assert_eq!(frame::crc16(b""), 0xFFFF);
}

#[test]
fn frame_layout_header_and_crc_le() {
    let f = frame::build(0x42, &[0xAA, 0xBB, 0xCC]);
    assert_eq!(&f[..4], &[0x01, 0x42, 0x03, 0x00]);
    assert_eq!(&f[4..7], &[0xAA, 0xBB, 0xCC]);
    let crc = frame::crc16(&f[..7]);
    assert_eq!(&f[7..], &crc.to_le_bytes());
    assert_eq!(frame::parse(&f), Ok((0x42, &[0xAA, 0xBB, 0xCC][..])));
}

// ---------- COBS ----------

fn seq(range: std::ops::RangeInclusive<u8>) -> Vec<u8> {
    range.collect()
}

fn cat(parts: &[&[u8]]) -> Vec<u8> {
    parts.concat()
}

/// Wikipedia "Encoding examples" (COBS article).
fn cobs_vectors() -> Vec<(Vec<u8>, Vec<u8>)> {
    vec![
        (vec![], vec![0x01]),
        (vec![0x00], vec![0x01, 0x01]),
        (vec![0x00, 0x00], vec![0x01, 0x01, 0x01]),
        (vec![0x00, 0x11, 0x00], vec![0x01, 0x02, 0x11, 0x01]),
        (
            vec![0x11, 0x22, 0x00, 0x33],
            vec![0x03, 0x11, 0x22, 0x02, 0x33],
        ),
        (
            vec![0x11, 0x22, 0x33, 0x44],
            vec![0x05, 0x11, 0x22, 0x33, 0x44],
        ),
        (
            vec![0x11, 0x00, 0x00, 0x00],
            vec![0x02, 0x11, 0x01, 0x01, 0x01],
        ),
        // 254 non-zero bytes: one full group, no trailing block.
        (seq(0x01..=0xFE), cat(&[&[0xFF], &seq(0x01..=0xFE)])),
        // 00 then 254 non-zero bytes.
        (seq(0x00..=0xFE), cat(&[&[0x01, 0xFF], &seq(0x01..=0xFE)])),
        // 255 non-zero bytes: full group then a 1-byte group.
        (
            seq(0x01..=0xFF),
            cat(&[&[0xFF], &seq(0x01..=0xFE), &[0x02, 0xFF]]),
        ),
        // 254 non-zero bytes then 00.
        (
            cat(&[&seq(0x02..=0xFF), &[0x00]]),
            cat(&[&[0xFF], &seq(0x02..=0xFF), &[0x01, 0x01]]),
        ),
        // 253 non-zero, 00, 01.
        (
            cat(&[&seq(0x03..=0xFF), &[0x00, 0x01]]),
            cat(&[&[0xFE], &seq(0x03..=0xFF), &[0x02, 0x01]]),
        ),
    ]
}

#[test]
fn cobs_standard_vectors() {
    for (i, (raw, enc)) in cobs_vectors().into_iter().enumerate() {
        assert_eq!(cobs::encode(&raw), enc, "encode vector {i}");
        assert_eq!(cobs::decode(&enc), Some(raw), "decode vector {i}");
    }
}

#[test]
fn cobs_runs_of_zeros() {
    for n in 1..=600 {
        let raw = vec![0u8; n];
        let enc = cobs::encode(&raw);
        assert_eq!(enc, vec![0x01; n + 1]);
        assert_eq!(cobs::decode(&enc), Some(raw));
    }
}

#[test]
fn cobs_accepts_trailing_empty_group_after_full_group() {
    let a = cat(&[&[0xFF], &seq(0x01..=0xFE)]);
    let b = cat(&[&a, &[0x01]]);
    assert_eq!(cobs::decode(&a), cobs::decode(&b));
    assert_eq!(cobs::decode(&b), Some(seq(0x01..=0xFE)));
}

#[test]
fn cobs_rejects_malformed() {
    assert_eq!(cobs::decode(&[]), None);
    assert_eq!(cobs::decode(&[0x00]), None);
    assert_eq!(cobs::decode(&[0x03, 0x11, 0x00]), None); // embedded zero
    assert_eq!(cobs::decode(&[0x05, 0x11, 0x22]), None); // overrun
    assert_eq!(cobs::decode(&[0x02, 0x11, 0x00, 0x01]), None);
}

#[test]
fn cobs_encoded_never_contains_zero_and_round_trips() {
    let mut r = Rng(0x1234_5678_9ABC_DEF0);
    for len in 0..=700usize {
        let mut raw = r.bytes(len);
        // Sprinkle zeros at a varying density.
        let density = r.below(4);
        for b in raw.iter_mut() {
            if r.below(4) < density {
                *b = 0;
            }
        }
        let enc = cobs::encode(&raw);
        assert!(!enc.contains(&0), "len {len}");
        assert!(enc.len() <= len + len / 254 + 1, "len {len}: {}", enc.len());
        assert_eq!(cobs::decode(&enc), Some(raw), "len {len}");
    }
}

#[test]
fn cobs_decode_random_garbage_never_panics() {
    let mut r = Rng(42);
    for _ in 0..5000 {
        let len = r.below(400) as usize;
        let g = r.bytes(len);
        let _ = cobs::decode(&g);
        let _ = decode_mcu_frame(&g);
        let _ = decode_host_frame(&g);
        let _ = frame::parse(&g);
        for t in 0..=255u8 {
            if r.below(32) == 0 {
                let _ = McuMsg::decode_payload(t, &g);
                let _ = HostMsg::decode_payload(t, &g);
            }
        }
    }
}

// ---------- FrameAccumulator ----------

#[test]
fn accumulator_splits_frames_and_ignores_empty() {
    let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
    let out = acc.push(&[0x00, 0x00, 0x02, 0x11, 0x00, 0x00, 0x00, 0x03, 0x22]);
    assert_eq!(out, vec![vec![0x02, 0x11]]);
    // Partial frame is held until its delimiter arrives.
    assert_eq!(acc.push(&[0x33, 0x00]), vec![vec![0x03, 0x22, 0x33]]);
    assert_eq!(acc.dropped(), 0);
}

#[test]
fn accumulator_byte_by_byte_equals_bulk() {
    let mut r = Rng(7);
    let mut stream = Vec::new();
    for _ in 0..200 {
        if r.below(2) == 0 {
            stream.extend(rand_host(&mut r).to_wire());
        } else {
            stream.extend(rand_mcu(&mut r).to_wire());
        }
        if r.below(5) == 0 {
            stream.push(0); // stray delimiter
        }
    }
    let bulk = FrameAccumulator::new(MAX_ENCODED_LEN).push(&stream);
    let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
    let mut single = Vec::new();
    for b in &stream {
        single.extend(acc.push(std::slice::from_ref(b)));
    }
    assert_eq!(bulk, single);
    assert_eq!(bulk.len(), 200);

    // Random chunking too.
    let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
    let mut chunked = Vec::new();
    let mut i = 0;
    while i < stream.len() {
        let n = (1 + r.below(70) as usize).min(stream.len() - i);
        chunked.extend(acc.push(&stream[i..i + n]));
        i += n;
    }
    assert_eq!(bulk, chunked);
}

#[test]
fn accumulator_oversize_boundary_and_resync() {
    let mut acc = FrameAccumulator::new(300);
    let exact = vec![0x55u8; 300];
    let over = vec![0x66u8; 301];
    let mut stream = exact.clone();
    stream.push(0);
    stream.extend(&over);
    stream.push(0);
    stream.extend([0x02, 0x77, 0x00]);
    let out = acc.push(&stream);
    assert_eq!(out, vec![exact, vec![0x02, 0x77]]);
    assert_eq!(acc.dropped(), 1);

    // A very long oversize run counts once and resyncs at the next delimiter.
    let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
    let mut s = vec![0xEEu8; 5000];
    let good = HostMsg::HelloReq { request_id: 9 }.to_wire();
    s.extend(&good);
    s.extend(&good);
    let out = acc.push(&s);
    assert_eq!(acc.dropped(), 1);
    // The 5000-byte run absorbs the first frame (no delimiter between them).
    assert_eq!(out.len(), 1);
    assert_eq!(
        decode_host_frame(&out[0]),
        Ok(HostMsg::HelloReq { request_id: 9 })
    );
}

#[test]
fn accumulator_clear_discards_partial() {
    let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
    assert!(acc.push(&[0x05, 0x01, 0x02]).is_empty());
    acc.clear();
    assert_eq!(acc.push(&[0x02, 0x09, 0x00]), vec![vec![0x02, 0x09]]);
}

/// Feed a stream through an accumulator and decode each frame as MCU messages.
fn decode_stream_mcu(stream: &[u8]) -> Vec<Result<McuMsg, DecodeError>> {
    FrameAccumulator::new(MAX_ENCODED_LEN)
        .push(stream)
        .iter()
        .map(|f| decode_mcu_frame(f))
        .collect()
}

fn sample_status() -> McuMsg {
    McuMsg::Status(Status {
        boot_id: 0xDEADBEEF,
        mcu_time_us: 123_456_789,
        mcu_state: 3,
        selected_axis: 2,
        fault_code: FaultCode::None,
        flags: 0b1111,
        pwm_permille: -250,
        actual_count: -100,
        target_count: -98,
        velocity_count_s: 0,
        following_error_count: 2,
        last_applied_frame: 77,
        last_applied_seq: 78,
        last_rx_seq: 78,
        current_ma: 0,
        rx_error_count: 0,
        stale_drop_count: 0,
    })
}

#[test]
fn resync_after_garbage() {
    let good = sample_status();
    let mut stream = vec![0x13, 0x37, 0xFF, 0x80, 0x01]; // line noise, mid-frame start
    stream.push(0);
    stream.extend(good.to_wire());
    let res = decode_stream_mcu(&stream);
    assert_eq!(res.len(), 2);
    assert!(res[0].is_err());
    assert_eq!(res[1], Ok(good));
}

#[test]
fn resync_after_truncated_frame() {
    let good = sample_status();
    let wire = good.to_wire();
    let cut = &wire[..wire.len() / 2];

    // Truncated then delimiter (e.g. MCU reset mid-frame).
    let mut s = cut.to_vec();
    s.push(0);
    s.extend(&wire);
    let res = decode_stream_mcu(&s);
    assert_eq!(res.len(), 2);
    assert!(res[0].is_err());
    assert_eq!(res[1], Ok(good));

    // Truncated without delimiter: merges with the next frame (lost), the one after is fine.
    let mut s = cut.to_vec();
    s.extend(&wire);
    s.extend(&wire);
    let res = decode_stream_mcu(&s);
    assert_eq!(res.len(), 2);
    assert!(res[0].is_err());
    assert_eq!(res[1], Ok(good));
}

#[test]
fn resync_after_oversize_frame() {
    let good = sample_status();
    let mut s = vec![0x42u8; MAX_ENCODED_LEN + 50];
    s.push(0);
    s.extend(good.to_wire());
    let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
    let frames = acc.push(&s);
    assert_eq!(acc.dropped(), 1);
    assert_eq!(frames.len(), 1);
    assert_eq!(decode_mcu_frame(&frames[0]), Ok(good));
}

#[test]
fn bad_crc_is_rejected_and_stream_recovers() {
    let good = sample_status();
    let mut f = frame::build(good.msg_type(), &good.encode_payload());
    f[10] ^= 0x01;
    let mut s = encoded(&f);
    s.push(0);
    s.extend(good.to_wire());
    let res = decode_stream_mcu(&s);
    assert_eq!(res, vec![Err(DecodeError::BadCrc), Ok(good)]);

    // Corrupt CRC bytes themselves.
    let mut f = frame::build(good.msg_type(), &good.encode_payload());
    let n = f.len();
    f[n - 1] ^= 0x80;
    assert_eq!(frame::parse(&f), Err(DecodeError::BadCrc));
}

// ---------- frame / payload validation ----------

#[test]
fn frame_parse_errors() {
    assert_eq!(frame::parse(&[]), Err(DecodeError::TooShort));
    assert_eq!(frame::parse(&[1, 2, 0, 0, 0]), Err(DecodeError::TooShort));
    let big = raw_frame(1, msg_type::STATUS, 251, &[0u8; 251]);
    assert_eq!(big.len(), MAX_FRAME_LEN + 1);
    assert_eq!(frame::parse(&big), Err(DecodeError::TooLong));
    let max = raw_frame(1, 0x7F, 250, &[0u8; 250]);
    assert_eq!(max.len(), MAX_FRAME_LEN);
    assert_eq!(
        frame::parse(&max).map(|(t, p)| (t, p.len())),
        Ok((0x7F, 250))
    );
    // Empty payload is a valid frame structurally.
    assert_eq!(
        frame::parse(&raw_frame(1, 0x01, 0, &[])),
        Ok((0x01, &[][..]))
    );
}

#[test]
fn wrong_version_rejected() {
    let p = HostMsg::HelloReq { request_id: 1 }.encode_payload();
    for v in [0u8, 2, 0xFF] {
        let f = raw_frame(v, msg_type::HELLO_REQ, 4, &p);
        assert_eq!(frame::parse(&f), Err(DecodeError::BadVersion(v)));
        assert_eq!(
            decode_host_frame(&encoded(&f)),
            Err(DecodeError::BadVersion(v))
        );
    }
}

#[test]
fn payload_len_field_mismatch_rejected() {
    let p = HostMsg::HelloReq { request_id: 1 }.encode_payload();
    for field in [0u16, 3, 5, 0xFFFF] {
        let f = raw_frame(1, msg_type::HELLO_REQ, field, &p);
        assert_eq!(
            frame::parse(&f),
            Err(DecodeError::LengthMismatch),
            "field {field}"
        );
    }
}

#[test]
fn wrong_payload_len_for_type_rejected() {
    // Header is self-consistent, but the length is not the type's defined length.
    let f = raw_frame(1, msg_type::HELLO_REQ, 5, &[1, 0, 0, 0, 0]);
    assert_eq!(
        decode_host_frame(&encoded(&f)),
        Err(DecodeError::BadPayloadLen {
            msg_type: msg_type::HELLO_REQ,
            expected: 4,
            got: 5
        })
    );
    let st = sample_status().encode_payload();
    let f = raw_frame(1, msg_type::STATUS, 55, &st[..55]);
    assert_eq!(
        decode_mcu_frame(&encoded(&f)),
        Err(DecodeError::BadPayloadLen {
            msg_type: msg_type::STATUS,
            expected: 56,
            got: 55
        })
    );
    // Every type rejects ±1 byte.
    for t in 0..=255u8 {
        if let Some(n) = host_payload_len(t) {
            for bad in [n - 1, n + 1] {
                assert_eq!(
                    HostMsg::decode_payload(t, &vec![0u8; bad]),
                    Err(DecodeError::BadPayloadLen {
                        msg_type: t,
                        expected: n,
                        got: bad
                    })
                );
            }
        }
        if let Some(n) = mcu_payload_len(t) {
            for bad in [n - 1, n + 1] {
                assert_eq!(
                    McuMsg::decode_payload(t, &vec![0u8; bad]),
                    Err(DecodeError::BadPayloadLen {
                        msg_type: t,
                        expected: n,
                        got: bad
                    })
                );
            }
        }
    }
}

#[test]
fn unknown_type_rejected_including_cross_direction() {
    let f = raw_frame(1, 0x7E, 4, &[1, 0, 0, 0]);
    assert_eq!(
        decode_host_frame(&encoded(&f)),
        Err(DecodeError::UnknownType(0x7E))
    );
    assert_eq!(
        decode_mcu_frame(&encoded(&f)),
        Err(DecodeError::UnknownType(0x7E))
    );
    // HELLO_REQ is not a MCU→PC message and HELLO is not PC→MCU.
    let req = HostMsg::HelloReq { request_id: 1 }.to_wire();
    assert_eq!(
        decode_mcu_frame(wire_to_encoded(&req)),
        Err(DecodeError::UnknownType(0x01))
    );
    let hello = McuMsg::decode_payload(msg_type::HELLO, &[0u8; 32])
        .unwrap()
        .to_wire();
    assert_eq!(
        decode_host_frame(wire_to_encoded(&hello)),
        Err(DecodeError::UnknownType(0x81))
    );
    assert_eq!(
        HostMsg::decode_payload(0x00, &[]),
        Err(DecodeError::UnknownType(0x00))
    );
    assert_eq!(
        McuMsg::decode_payload(0x89, &[]),
        Err(DecodeError::UnknownType(0x89))
    );
}

#[test]
fn payload_length_tables_match_spec() {
    use msg_type::*;
    let host = [
        (HELLO_REQ, 4),
        (CONFIG, 44),
        (ARM_PREPARE, 8),
        (START, 8),
        (SET_POSITION, 36),
        (CONTROLLED_STOP, 12),
        (DISARM, 4),
        (FAULT_RESET, 4),
        (CLOCK_SYNC, 12),
        (HOST_FAULT, 8),
        (SET_REFERENCE, 8),
        (CALIBRATE, 8),
        (SET_LIMITS, 16),
    ];
    let mcu = [
        (HELLO, 32),
        (ACK, 8),
        (STATUS, 56),
        (FAULT_EVENT, 20),
        (CLOCK_SYNC_REPLY, 28),
        (AXIS_SELECT, 8),
        (CAL_RESULT, 28),
        (LIMITS, 24),
    ];
    let mut n_host = 0;
    let mut n_mcu = 0;
    for t in 0..=255u8 {
        n_host += host_payload_len(t).is_some() as usize;
        n_mcu += mcu_payload_len(t).is_some() as usize;
    }
    assert_eq!((n_host, n_mcu), (host.len(), mcu.len()));
    for (t, n) in host {
        assert_eq!(host_payload_len(t), Some(n));
    }
    for (t, n) in mcu {
        assert_eq!(mcu_payload_len(t), Some(n));
    }
    // Largest frame fits comfortably within limits.
    let largest = frame::HEADER_LEN + 56 + frame::CRC_LEN;
    assert!(largest <= MAX_FRAME_LEN);
    assert!(largest + largest / 254 < MAX_ENCODED_LEN);
}

#[test]
fn ack_unknown_result_or_reason_is_bad_field() {
    let mut p = McuMsg::Ack(Ack {
        request_id: 5,
        acked_type: msg_type::CONFIG,
        result: AckResult::Accepted,
        reason: AckReason::None,
    })
    .encode_payload();
    p[5] = 3;
    assert_eq!(
        McuMsg::decode_payload(msg_type::ACK, &p),
        Err(DecodeError::BadField("ack.result"))
    );
    p[5] = 1;
    p[6] = 8;
    assert_eq!(
        McuMsg::decode_payload(msg_type::ACK, &p),
        Err(DecodeError::BadField("ack.reason"))
    );
    p[6] = 0;
    p[7] = 1; // reason = 0x0100
    assert_eq!(
        McuMsg::decode_payload(msg_type::ACK, &p),
        Err(DecodeError::BadField("ack.reason"))
    );
}

#[test]
fn unknown_fault_code_maps_to_unknown_and_state_kept_raw() {
    let mut p = sample_status().encode_payload();
    p[12] = 0xEE; // mcu_state outside the enum
    p[14..16].copy_from_slice(&13u16.to_le_bytes()); // undefined fault code (12 = LOCAL_ESTOP is defined)
    match McuMsg::decode_payload(msg_type::STATUS, &p) {
        Ok(McuMsg::Status(s)) => {
            assert_eq!(s.mcu_state, 0xEE);
            assert_eq!(s.fault_code, FaultCode::Unknown);
        }
        other => panic!("{other:?}"),
    }
    let p = HostMsg::HostFault {
        request_id: 1,
        fault_code: FaultCode::None,
    }
    .encode_payload();
    let mut q = p.clone();
    q[4..6].copy_from_slice(&0x1234u16.to_le_bytes());
    assert_eq!(
        HostMsg::decode_payload(msg_type::HOST_FAULT, &q),
        Ok(HostMsg::HostFault {
            request_id: 1,
            fault_code: FaultCode::Unknown
        })
    );
}

#[test]
fn reserved_fields_written_zero_and_ignored_on_decode() {
    // (message payload, reserved byte offsets)
    let cfg = HostMsg::Config(Config {
        request_id: u32::MAX,
        session_epoch: u32::MAX,
        calibration_hash: u32::MAX,
        soft_min_count: -1,
        soft_max_count: -1,
        max_vel_count_s: u32::MAX,
        max_acc_count_s2: u32::MAX,
        following_error_limit_count: u32::MAX,
        overspeed_count_s: u32::MAX,
        command_timeout_ms: u16::MAX,
        stop_timeout_ms: u16::MAX,
        role_flags: 0xFF,
    });
    let sp = HostMsg::SetPosition(SetPosition {
        session_epoch: u32::MAX,
        seq: u32::MAX,
        group_frame_id: u32::MAX,
        target_count: -1,
        max_vel_count_s: u32::MAX,
        max_acc_count_s2: u32::MAX,
        valid_for_ms: u16::MAX,
        apply_at_us: u64::MAX,
    });
    let cs = HostMsg::ControlledStop {
        request_id: u32::MAX,
        session_epoch: u32::MAX,
        reason: 0xFF,
    };
    let hf = HostMsg::HostFault {
        request_id: u32::MAX,
        fault_code: FaultCode::Unknown,
    };
    let host_cases: [(HostMsg, &[usize]); 4] = [
        (cfg, &[41, 42, 43]),
        (sp, &[26, 27]),
        (cs, &[9, 10, 11]),
        (hf, &[6, 7]),
    ];
    for (m, reserved) in host_cases {
        let mut p = m.encode_payload();
        for &i in reserved {
            assert_eq!(p[i], 0, "{m:?} byte {i}");
            p[i] = 0xA5;
        }
        assert_eq!(HostMsg::decode_payload(m.msg_type(), &p), Ok(m));
    }

    let hello = McuMsg::Hello(Hello {
        request_id: u32::MAX,
        protocol_version: 0xFF,
        selected_axis: 0xFF,
        mcu_state: 0xFF,
        device_id: DeviceId(u64::MAX),
        boot_id: u32::MAX,
        fw_version: [0xFF; 3],
        capabilities: u32::MAX,
        calibration_hash: u32::MAX,
    });
    let st = sample_status();
    let fe = McuMsg::FaultEvent(FaultEvent {
        boot_id: u32::MAX,
        mcu_time_us: u64::MAX,
        fault_code: FaultCode::Unknown,
        mcu_state_before: 0xFF,
        actual_count: -1,
    });
    let sel = McuMsg::AxisSelect(AxisSelect {
        boot_id: u32::MAX,
        selected_axis: 0xFF,
        prev_selected_axis: 0xFF,
        mcu_state: 0xFF,
    });
    // STATUS offset 13 is `selected_axis` since protocol §1.1 (no longer reserved).
    let mcu_cases: [(McuMsg, &[usize]); 3] = [(hello, &[7, 23]), (fe, &[15]), (sel, &[7])];
    assert_eq!(st.encode_payload()[13], 2, "STATUS byte 13 carries selected_axis");
    for (m, reserved) in mcu_cases {
        let mut p = m.encode_payload();
        for &i in reserved {
            assert_eq!(p[i], 0, "{m:?} byte {i}");
            p[i] = 0xA5;
        }
        assert_eq!(McuMsg::decode_payload(m.msg_type(), &p), Ok(m));
    }
}

#[test]
fn axis_select_layout_and_length() {
    // protocol §4: 0x86, 8 bytes: u32 boot_id / u8 selected_axis / u8 prev / u8 mcu_state / u8 reserved.
    assert_eq!(mcu_payload_len(msg_type::AXIS_SELECT), Some(8));
    let m = McuMsg::AxisSelect(AxisSelect {
        boot_id: 0x89AB_CDEF,
        selected_axis: 2,
        prev_selected_axis: 1,
        mcu_state: 1,
    });
    assert_eq!(m.msg_type(), 0x86);
    assert_eq!(m.encode_payload(), [0xEF, 0xCD, 0xAB, 0x89, 2, 1, 1, 0]);
    mcu_rt(m);
    // Wrong length is rejected, never panics.
    assert!(matches!(
        McuMsg::decode_payload(msg_type::AXIS_SELECT, &[0u8; 7]),
        Err(DecodeError::BadPayloadLen { msg_type: 0x86, expected: 8, got: 7 })
    ));
    assert!(McuMsg::decode_payload(msg_type::AXIS_SELECT, &[]).is_err());
}

#[test]
fn calibrate_layout_and_length() {
    // protocol §4: 0x0C, 8 bytes: u32 request_id / u8 mode / u8 reserved / u16 reserved.
    assert_eq!(host_payload_len(msg_type::CALIBRATE), Some(8));
    let m = HostMsg::Calibrate { request_id: 7, mode: calibrate_mode::START, flags: 0 };
    assert_eq!(m.msg_type(), 0x0C);
    assert_eq!(m.encode_payload(), [7, 0, 0, 0, 1, 0, 0, 0]);
    host_rt(m);
    host_rt(HostMsg::Calibrate { request_id: 7, mode: calibrate_mode::CANCEL, flags: 0 });
    // flags (KEEP_POWER) is byte 5; the two reserved bytes are ignored on decode; wrong length is rejected.
    assert_eq!(HostMsg::Calibrate { request_id: 7, mode: 1, flags: 1 }.encode_payload(), [7, 0, 0, 0, 1, 1, 0, 0]);
    assert_eq!(
        HostMsg::decode_payload(msg_type::CALIBRATE, &[7, 0, 0, 0, 0, 0xAA, 0xBB, 0xCC]),
        Ok(HostMsg::Calibrate { request_id: 7, mode: 0, flags: 0xAA })
    );
    assert!(matches!(
        HostMsg::decode_payload(msg_type::CALIBRATE, &[0u8; 4]),
        Err(DecodeError::BadPayloadLen { msg_type: 0x0C, expected: 8, got: 4 })
    ));
}

#[test]
fn cal_result_layout_reason_padding_and_unknown_result() {
    // protocol §4: 0x87, 28 bytes: u32 boot_id / u8 result / u8 step / u16 reserved /
    // i32 n_end_count / i32 neutral_count / i8 sign / u8 reserved / u16 reserved / u8[8] reason.
    assert_eq!(mcu_payload_len(msg_type::CAL_RESULT), Some(28));
    let m = McuMsg::CalResult(CalResult {
        boot_id: 0x89AB_CDEF,
        result: CalResultCode::Failed,
        step: 2,
        n_end_count: -5551,
        neutral_count: -2775,
        sign: -1,
        reason: CalResult::reason_bytes("stall"),
    });
    assert_eq!(m.msg_type(), 0x87);
    let p = m.encode_payload();
    assert_eq!(&p[..4], &[0xEF, 0xCD, 0xAB, 0x89]);
    assert_eq!(&p[4..8], &[1, 2, 0, 0]);
    assert_eq!(&p[8..12], &(-5551i32).to_le_bytes());
    assert_eq!(&p[12..16], &(-2775i32).to_le_bytes());
    assert_eq!(&p[16..20], &[0xFF, 0, 0, 0]);
    assert_eq!(&p[20..], b"stall\0\0\0");
    mcu_rt(m);
    let McuMsg::CalResult(c) = m else { unreachable!() };
    assert_eq!(c.reason_str(), "stall");
    // reason_bytes truncates to 8 and never pads with anything but NUL.
    assert_eq!(&CalResult::reason_bytes("timeout_long"), b"timeout_");
    assert_eq!(CalResult { reason: [b'o', b'k', 0, 0, 0, 0, 0, 0], ..CalResult::EMPTY }.reason_str(), "ok");
    // An unknown result code is a decode error (the frame is discarded).
    let mut bad = p.clone();
    bad[4] = 3;
    assert_eq!(
        McuMsg::decode_payload(msg_type::CAL_RESULT, &bad),
        Err(DecodeError::BadField("cal_result.result"))
    );
    // Reserved bytes are ignored on decode.
    let mut res = p.clone();
    res[6] = 0xAA;
    res[17] = 0xBB;
    assert_eq!(McuMsg::decode_payload(msg_type::CAL_RESULT, &res), Ok(m));
    assert!(matches!(
        McuMsg::decode_payload(msg_type::CAL_RESULT, &[0u8; 27]),
        Err(DecodeError::BadPayloadLen { msg_type: 0x87, expected: 28, got: 27 })
    ));
}

#[test]
fn status_selected_axis_is_byte_13_and_old_frames_decode_as_zero() {
    let McuMsg::Status(s) = sample_status() else { unreachable!() };
    let m = McuMsg::Status(Status { selected_axis: 3, ..s });
    let mut p = m.encode_payload();
    assert_eq!(p[13], 3);
    // A frame from firmware that still writes reserved = 0 decodes with selected_axis 0.
    p[13] = 0;
    assert_eq!(
        McuMsg::decode_payload(msg_type::STATUS, &p),
        Ok(McuMsg::Status(Status { selected_axis: 0, ..s }))
    );
}

// ---------- round trips ----------

#[test]
fn round_trip_every_host_message_edge_values() {
    for (u, i, w, h) in [
        (0u32, 0i32, 0u64, 0u16),
        (u32::MAX, i32::MIN, u64::MAX, u16::MAX),
        (u32::MAX, i32::MAX, u64::MAX, u16::MAX),
        (1, -1, 1, 1),
    ] {
        let msgs = [
            HostMsg::HelloReq { request_id: u },
            HostMsg::Config(Config {
                request_id: u,
                session_epoch: u,
                calibration_hash: u,
                soft_min_count: i,
                soft_max_count: i,
                max_vel_count_s: u,
                max_acc_count_s2: u,
                following_error_limit_count: u,
                overspeed_count_s: u,
                command_timeout_ms: h,
                stop_timeout_ms: h,
                role_flags: h as u8,
            }),
            HostMsg::ArmPrepare {
                request_id: u,
                session_epoch: u,
            },
            HostMsg::Start {
                request_id: u,
                session_epoch: u,
            },
            HostMsg::SetPosition(SetPosition {
                session_epoch: u,
                seq: u,
                group_frame_id: u,
                target_count: i,
                max_vel_count_s: u,
                max_acc_count_s2: u,
                valid_for_ms: h,
                apply_at_us: w,
            }),
            HostMsg::ControlledStop {
                request_id: u,
                session_epoch: u,
                reason: h as u8,
            },
            HostMsg::Disarm { request_id: u },
            HostMsg::FaultReset { request_id: u },
            HostMsg::ClockSync {
                request_id: u,
                pc_time_us: w,
            },
            HostMsg::SetReference {
                request_id: u,
                reference_count: i,
            },
            HostMsg::Calibrate {
                request_id: u,
                mode: h as u8,
                flags: 0,
            },
            HostMsg::SetLimits(SetLimits {
                request_id: u,
                duty_max_percent: h as u8,
                duty_min_percent: h as u8,
                kp_percent_per_100: h as u8,
                ff_count_s_per_percent: h,
                deadband_count: h,
                deadband_out_count: h,
                stop_acc_count_s2: h,
            }),
        ];
        for m in msgs {
            host_rt(m);
        }
        for f in ALL_FAULTS {
            host_rt(HostMsg::HostFault {
                request_id: u,
                fault_code: f,
            });
        }
    }
}

#[test]
fn round_trip_every_mcu_message_edge_values() {
    for (u, i, w, h, b) in [
        (0u32, 0i32, 0u64, 0u16, 0u8),
        (u32::MAX, i32::MIN, u64::MAX, u16::MAX, u8::MAX),
        (u32::MAX, i32::MAX, u64::MAX, u16::MAX, u8::MAX),
    ] {
        mcu_rt(McuMsg::Hello(Hello {
            request_id: u,
            protocol_version: b,
            selected_axis: b,
            mcu_state: b,
            device_id: DeviceId(w),
            boot_id: u,
            fw_version: [b, 0, b],
            capabilities: u,
            calibration_hash: u,
        }));
        for result in ALL_RESULTS {
            for reason in ALL_REASONS {
                mcu_rt(McuMsg::Ack(Ack {
                    request_id: u,
                    acked_type: b,
                    result,
                    reason,
                }));
            }
        }
        for pwm in [i16::MIN, -1000, 0, 1000, i16::MAX] {
            for f in ALL_FAULTS {
                mcu_rt(McuMsg::Status(Status {
                    boot_id: u,
                    mcu_time_us: w,
                    mcu_state: b,
                    selected_axis: b,
                    fault_code: f,
                    flags: h,
                    pwm_permille: pwm,
                    actual_count: i,
                    target_count: i,
                    velocity_count_s: i,
                    following_error_count: i,
                    last_applied_frame: u,
                    last_applied_seq: u,
                    last_rx_seq: u,
                    current_ma: i,
                    rx_error_count: h,
                    stale_drop_count: h,
                }));
            }
        }
        for f in ALL_FAULTS {
            mcu_rt(McuMsg::FaultEvent(FaultEvent {
                boot_id: u,
                mcu_time_us: w,
                fault_code: f,
                mcu_state_before: b,
                actual_count: i,
            }));
        }
        mcu_rt(McuMsg::ClockSyncReply(ClockSyncReply {
            request_id: u,
            pc_time_us: w,
            mcu_rx_us: w,
            mcu_tx_us: w,
        }));
        mcu_rt(McuMsg::AxisSelect(AxisSelect {
            boot_id: u,
            selected_axis: b,
            prev_selected_axis: b.wrapping_sub(1),
            mcu_state: b,
        }));
        for result in ALL_CAL_RESULTS {
            mcu_rt(McuMsg::CalResult(CalResult {
                boot_id: u,
                result,
                step: b,
                n_end_count: i,
                neutral_count: i,
                sign: b as i8,
                reason: [b; CAL_REASON_LEN],
            }));
        }
        mcu_rt(McuMsg::Limits(Limits {
            boot_id: u,
            duty_max_percent: b,
            duty_min_percent: b,
            kp_percent_per_100: b,
            abs_duty_max_percent: b,
            ff_count_s_per_percent: h,
            deadband_count: h,
            deadband_out_count: h,
            abs_max_vel_count_s: u,
            abs_max_acc_count_s2: u,
        }));
    }
}

#[test]
fn property_random_messages_round_trip_through_stream() {
    let mut r = Rng(0x00C0_FFEE_D00D_F00D);
    for round in 0..50 {
        let mut host = Vec::new();
        let mut mcu = Vec::new();
        let mut host_stream = Vec::new();
        let mut mcu_stream = Vec::new();
        for _ in 0..40 {
            let h = rand_host(&mut r);
            host_stream.extend(h.to_wire());
            host.push(h);
            let m = rand_mcu(&mut r);
            mcu_stream.extend(m.to_wire());
            mcu.push(m);
        }
        let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
        let got: Vec<_> = acc
            .push(&host_stream)
            .iter()
            .map(|f| decode_host_frame(f))
            .collect();
        assert_eq!(
            got,
            host.iter().map(|m| Ok(*m)).collect::<Vec<_>>(),
            "round {round}"
        );
        let mut acc = FrameAccumulator::new(MAX_ENCODED_LEN);
        let got: Vec<_> = acc
            .push(&mcu_stream)
            .iter()
            .map(|f| decode_mcu_frame(f))
            .collect();
        assert_eq!(
            got,
            mcu.iter().map(|m| Ok(*m)).collect::<Vec<_>>(),
            "round {round}"
        );
        assert_eq!(acc.dropped(), 0);
    }
}

#[test]
fn property_single_bit_errors_are_rejected() {
    // A single flipped bit must be caught (CRC-16 detects all 1-bit errors);
    // it either fails or (if it hits COBS structure) still fails at some layer.
    let mut r = Rng(99);
    for _ in 0..3000 {
        let m = rand_mcu(&mut r);
        let mut wire = m.to_wire();
        let n = wire.len() - 1;
        let pos = r.below(n as u64) as usize;
        let bit = 1u8 << r.below(8);
        wire[pos] ^= bit;
        let frames = FrameAccumulator::new(MAX_ENCODED_LEN).push(&wire);
        for f in frames {
            let res = decode_mcu_frame(&f);
            assert!(res.is_err(), "1-bit error at {pos} decoded as {res:?}");
        }
    }
}

#[test]
fn decode_frame_rejects_overlong_encoded_input() {
    let long = vec![0x01u8; MAX_ENCODED_LEN + 1];
    assert_eq!(decode_mcu_frame(&long), Err(DecodeError::TooLong));
    assert_eq!(decode_host_frame(&[]), Err(DecodeError::Cobs));
    assert_eq!(
        decode_host_frame(&[0x03, 0x00, 0x01]),
        Err(DecodeError::Cobs)
    );
}
