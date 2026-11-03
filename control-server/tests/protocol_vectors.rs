//! Fixed wire-byte test vectors for USB protocol v1.
//!
//! The same vectors are published in `docs/control-server/protocol.md` §7
//! (between the VECTORS markers) for the CoreS3 firmware. `doc_matches_vectors`
//! keeps the two byte-for-byte identical. To regenerate the doc block:
//! `cargo test --test protocol_vectors -- --ignored --nocapture print_vectors`.

use h3cs::protocol::cobs::FrameAccumulator;
use h3cs::protocol::*;
use h3cs::types::{DeviceId, FaultCode};

fn sample_pmic() -> PmicReport {
    let mut regs = [0u8; PMIC_REG_LEN];
    let mut reg_valid = 0u64;
    for (i, v) in [
        (0usize, 0x21u8),
        (1, 0x08),
        (2, 0x48),
        (3, 0x33),
        (4, 0x36),
        (5, 0x38),
        (7, 0x05),
        (10, 0x03),
        (12, 0x01),
        (13, 0x20),
        (15, 0x1F),
        (19, 0x00),
        (20, 0x00),
        (21, 0x00),
        (25, 0x01),
    ] {
        regs[i] = v;
        reg_valid |= 1u64 << i;
    }
    PmicReport {
        kind: pmic_kind::PERIODIC,
        flags: pmic_flags::RTC_OK
            | pmic_flags::MARK_OK
            | pmic_flags::VBUS_OK
            | pmic_flags::VSYS_OK
            | pmic_flags::VBAT_OK
            | pmic_flags::TS_OK
            | pmic_flags::TDIE_OK,
        read_ok: 14,
        read_tried: 14,
        reset_reason: 1,
        por: 0,
        seq: 3,
        mcu_time_us: 5_000_000,
        last_up_s: 120,
        vbus: 5000,
        vsys: 3300,
        vbat: 4100,
        ts: 4900,
        tdie: 250,
        last_vbus: 4980,
        last_vsys: 3290,
        last_vbat: 4080,
        last_s0: 0x20,
        last_s1: 0x08,
        i2c_err: 0,
        reg_valid,
        regs,
    }
}

#[derive(Clone, Copy)]
enum Msg {
    Host(HostMsg),
    Mcu(McuMsg),
}

impl Msg {
    fn msg_type(&self) -> u8 {
        match self {
            Msg::Host(m) => m.msg_type(),
            Msg::Mcu(m) => m.msg_type(),
        }
    }
    fn payload(&self) -> Vec<u8> {
        match self {
            Msg::Host(m) => m.encode_payload(),
            Msg::Mcu(m) => m.encode_payload(),
        }
    }
    fn wire(&self) -> Vec<u8> {
        match self {
            Msg::Host(m) => m.to_wire(),
            Msg::Mcu(m) => m.to_wire(),
        }
    }
    fn direction(&self) -> &'static str {
        match self {
            Msg::Host(_) => "PC → MCU",
            Msg::Mcu(_) => "MCU → PC",
        }
    }
    /// Field values as compact JSON in layout order, without the `type` tag.
    fn fields_json(&self) -> String {
        let s = match self {
            Msg::Host(m) => serde_json::to_string(m),
            Msg::Mcu(m) => serde_json::to_string(m),
        }
        .expect("serialize");
        // Internally tagged: `{"type":"NAME",...}` → `{...}`.
        let (_, rest) = s.split_once(',').expect("has fields");
        format!("{{{rest}")
    }
}

struct Vector {
    title: &'static str,
    msg: Msg,
    frame_hex: &'static str,
    wire_hex: &'static str,
}

const BOOT_ID: u32 = 0x89AB_CDEF;

fn vectors() -> Vec<Vector> {
    use msg_type as t;
    vec![
        Vector {
            title: "HELLO_REQ",
            msg: Msg::Host(HostMsg::HelloReq { request_id: 1 }),
            frame_hex: "01 01 04 00 01 00 00 00 99 BE",
            wire_hex: "04 01 01 04 02 01 01 01 03 99 BE 00",
        },
        Vector {
            title: "CONFIG",
            msg: Msg::Host(HostMsg::Config(Config {
                request_id: 2,
                session_epoch: 3,
                calibration_hash: 0x1234_5678,
                soft_min_count: -2700,
                soft_max_count: 2700,
                max_vel_count_s: 4000,
                max_acc_count_s2: 20000,
                following_error_limit_count: 200,
                overspeed_count_s: 6000,
                command_timeout_ms: 100,
                stop_timeout_ms: 1500,
                role_flags: role_flags::POWER_CONTROLLER,
            })),
            frame_hex: "01 02 2C 00 02 00 00 00 03 00 00 00 78 56 34 12 74 F5 FF FF 8C 0A 00 00 A0 0F 00 00 20 4E 00 00 C8 00 00 00 70 17 00 00 64 00 DC 05 01 00 00 00 D2 48",
            wire_hex: "04 01 02 2C 02 02 01 01 02 03 01 01 0B 78 56 34 12 74 F5 FF FF 8C 0A 01 03 A0 0F 01 03 20 4E 01 02 C8 01 01 03 70 17 01 02 64 04 DC 05 01 01 01 03 D2 48 00",
        },
        Vector {
            title: "ARM_PREPARE",
            msg: Msg::Host(HostMsg::ArmPrepare { request_id: 4, session_epoch: 3 }),
            frame_hex: "01 03 08 00 04 00 00 00 03 00 00 00 8C 21",
            wire_hex: "04 01 03 08 02 04 01 01 02 03 01 01 03 8C 21 00",
        },
        Vector {
            title: "START",
            msg: Msg::Host(HostMsg::Start { request_id: 5, session_epoch: 3 }),
            frame_hex: "01 04 08 00 05 00 00 00 03 00 00 00 E2 4F",
            wire_hex: "04 01 04 08 02 05 01 01 02 03 01 01 03 E2 4F 00",
        },
        Vector {
            title: "SET_POSITION",
            msg: Msg::Host(HostMsg::SetPosition(SetPosition {
                session_epoch: 3,
                seq: 1000,
                group_frame_id: 1234,
                target_count: -1234,
                max_vel_count_s: 4000,
                max_acc_count_s2: 20000,
                valid_for_ms: 50,
                apply_at_us: 0,
            })),
            frame_hex: "01 05 24 00 03 00 00 00 E8 03 00 00 D2 04 00 00 2E FB FF FF A0 0F 00 00 20 4E 00 00 32 00 00 00 00 00 00 00 00 00 00 00 1D 07",
            wire_hex: "04 01 05 24 02 03 01 01 03 E8 03 01 03 D2 04 01 07 2E FB FF FF A0 0F 01 03 20 4E 01 02 32 01 01 01 01 01 01 01 01 01 01 03 1D 07 00",
        },
        Vector {
            title: "CONTROLLED_STOP",
            msg: Msg::Host(HostMsg::ControlledStop { request_id: 6, session_epoch: 3, reason: 1 }),
            frame_hex: "01 06 0C 00 06 00 00 00 03 00 00 00 01 00 00 00 74 B7",
            wire_hex: "04 01 06 0C 02 06 01 01 02 03 01 01 02 01 01 01 03 74 B7 00",
        },
        Vector {
            title: "DISARM",
            msg: Msg::Host(HostMsg::Disarm { request_id: 7 }),
            frame_hex: "01 07 04 00 07 00 00 00 25 38",
            wire_hex: "04 01 07 04 02 07 01 01 03 25 38 00",
        },
        Vector {
            title: "FAULT_RESET",
            msg: Msg::Host(HostMsg::FaultReset { request_id: 8 }),
            frame_hex: "01 08 04 00 08 00 00 00 22 66",
            wire_hex: "04 01 08 04 02 08 01 01 03 22 66 00",
        },
        Vector {
            title: "CLOCK_SYNC",
            msg: Msg::Host(HostMsg::ClockSync { request_id: 9, pc_time_us: 1_000_000 }),
            frame_hex: "01 09 0C 00 09 00 00 00 40 42 0F 00 00 00 00 00 58 CB",
            wire_hex: "04 01 09 0C 02 09 01 01 04 40 42 0F 01 01 01 01 03 58 CB 00",
        },
        Vector {
            title: "HOST_FAULT",
            msg: Msg::Host(HostMsg::HostFault { request_id: 10, fault_code: FaultCode::HostFault }),
            frame_hex: "01 0A 08 00 0A 00 00 00 0A 00 00 00 2D 36",
            wire_hex: "04 01 0A 08 02 0A 01 01 02 0A 01 01 03 2D 36 00",
        },
        Vector {
            title: "SET_REFERENCE",
            msg: Msg::Host(HostMsg::SetReference { request_id: 11, reference_count: -2770 }),
            frame_hex: "01 0B 08 00 0B 00 00 00 2E F5 FF FF A5 41",
            wire_hex: "04 01 0B 08 02 0B 01 01 07 2E F5 FF FF A5 41 00",
        },
        Vector {
            title: "HELLO",
            msg: Msg::Mcu(McuMsg::Hello(Hello {
                request_id: 1,
                protocol_version: PROTOCOL_VERSION,
                selected_axis: 1, // byte 5: the unit selected axis 1
                mcu_state: 1, // DISARMED
                device_id: DeviceId(0x5131_0000_0000_0001),
                boot_id: BOOT_ID,
                fw_version: [0, 1, 0],
                capabilities: capabilities::POSITION_MODE,
                calibration_hash: 0,
            })),
            frame_hex: "01 81 20 00 01 00 00 00 01 01 01 00 01 00 00 00 00 00 31 51 EF CD AB 89 00 01 00 00 01 00 00 00 00 00 00 00 1E 98",
            wire_hex: "04 01 81 20 02 01 01 01 04 01 01 01 02 01 01 01 01 01 07 31 51 EF CD AB 89 02 01 01 02 01 01 01 01 01 01 01 03 1E 98 00",
        },
        Vector {
            title: "ACK ACCEPTED",
            msg: Msg::Mcu(McuMsg::Ack(Ack {
                request_id: 2,
                acked_type: t::CONFIG,
                result: AckResult::Accepted,
                reason: AckReason::None,
            })),
            frame_hex: "01 82 08 00 02 00 00 00 02 00 00 00 10 86",
            wire_hex: "04 01 82 08 02 02 01 01 02 02 01 01 03 10 86 00",
        },
        Vector {
            title: "ACK REJECTED / EPOCH_MISMATCH",
            msg: Msg::Mcu(McuMsg::Ack(Ack {
                request_id: 4,
                acked_type: t::ARM_PREPARE,
                result: AckResult::Rejected,
                reason: AckReason::EpochMismatch,
            })),
            frame_hex: "01 82 08 00 04 00 00 00 03 01 02 00 3D 21",
            wire_hex: "04 01 82 08 02 04 01 01 04 03 01 02 03 3D 21 00",
        },
        Vector {
            title: "STATUS",
            msg: Msg::Mcu(McuMsg::Status(Status {
                boot_id: BOOT_ID,
                mcu_time_us: 1_234_567_890,
                mcu_state: 4, // ACTIVE
                selected_axis: 1, // byte 13 (was reserved): axis 1 (matches POWER_CONTROLLER in flags)
                fault_code: FaultCode::None,
                flags: 0x001F, // POWER_ON_CMD|BRAKE_RELEASE_CMD|POSITION_REFERENCED|CONFIGURED|POWER_CONTROLLER
                pwm_permille: -250,
                actual_count: -1230,
                target_count: -1234,
                velocity_count_s: -150,
                following_error_count: 4,
                last_applied_frame: 1234,
                last_applied_seq: 1000,
                last_rx_seq: 1000,
                current_ma: 0,
                rx_error_count: 1,
                stale_drop_count: 2,
            })),
            frame_hex: "01 83 38 00 EF CD AB 89 D2 02 96 49 00 00 00 00 04 01 00 00 1F 00 06 FF 32 FB FF FF 2E FB FF FF 6A FF FF FF 04 00 00 00 D2 04 00 00 E8 03 00 00 E8 03 00 00 00 00 00 00 01 00 02 00 49 36",
            wire_hex: "04 01 83 38 09 EF CD AB 89 D2 02 96 49 01 01 01 03 04 01 01 02 1F 10 06 FF 32 FB FF FF 2E FB FF FF 6A FF FF FF 04 01 01 03 D2 04 01 03 E8 03 01 03 E8 03 01 01 01 01 01 02 01 02 02 03 49 36 00",
        },
        Vector {
            title: "FAULT_EVENT",
            msg: Msg::Mcu(McuMsg::FaultEvent(FaultEvent {
                boot_id: BOOT_ID,
                mcu_time_us: 1_234_600_000,
                fault_code: FaultCode::FollowingError,
                mcu_state_before: 4, // ACTIVE
                actual_count: -1500,
            })),
            frame_hex: "01 84 14 00 EF CD AB 89 40 80 96 49 00 00 00 00 02 00 04 00 24 FA FF FF 3F 2B",
            wire_hex: "04 01 84 14 09 EF CD AB 89 40 80 96 49 01 01 01 02 02 02 04 07 24 FA FF FF 3F 2B 00",
        },
        Vector {
            title: "CLOCK_SYNC_REPLY",
            msg: Msg::Mcu(McuMsg::ClockSyncReply(ClockSyncReply {
                request_id: 9,
                pc_time_us: 1_000_000,
                mcu_rx_us: 5_000_000,
                mcu_tx_us: 5_000_120,
            })),
            frame_hex: "01 85 1C 00 09 00 00 00 40 42 0F 00 00 00 00 00 40 4B 4C 00 00 00 00 00 B8 4B 4C 00 00 00 00 00 A9 A7",
            wire_hex: "04 01 85 1C 02 09 01 01 04 40 42 0F 01 01 01 01 04 40 4B 4C 01 01 01 01 04 B8 4B 4C 01 01 01 01 03 A9 A7 00",
        },
        Vector {
            title: "AXIS_SELECT",
            msg: Msg::Mcu(McuMsg::AxisSelect(AxisSelect {
                boot_id: BOOT_ID,
                selected_axis: 2,
                prev_selected_axis: 1,
                mcu_state: 1, // DISARMED
            })),
            frame_hex: "01 86 08 00 EF CD AB 89 02 01 01 00 32 C1",
            wire_hex: "04 01 86 08 08 EF CD AB 89 02 01 01 03 32 C1 00",
        },
        Vector {
            title: "CALIBRATE",
            msg: Msg::Host(HostMsg::Calibrate { request_id: 7, mode: calibrate_mode::START, flags: 0 }),
            frame_hex: "01 0C 08 00 07 00 00 00 01 00 00 00 A2 8F",
            wire_hex: "04 01 0C 08 02 07 01 01 02 01 01 01 03 A2 8F 00",
        },
        Vector {
            title: "CAL_RESULT",
            msg: Msg::Mcu(McuMsg::CalResult(CalResult {
                boot_id: BOOT_ID,
                result: CalResultCode::Ok,
                step: 3,
                n_end_count: 5551,
                neutral_count: 2775,
                sign: 1,
                reason: CalResult::reason_bytes("ok"),
            })),
            frame_hex: "01 87 1C 00 EF CD AB 89 00 03 00 00 AF 15 00 00 D7 0A 00 00 01 00 00 00 6F 6B 00 00 00 00 00 00 7A C9",
            wire_hex: "04 01 87 1C 05 EF CD AB 89 02 03 01 03 AF 15 01 03 D7 0A 01 02 01 01 01 03 6F 6B 01 01 01 01 01 03 7A C9 00",
        },
        Vector {
            title: "SET_LIMITS",
            msg: Msg::Host(HostMsg::SetLimits(SetLimits {
                request_id: 13,
                duty_max_percent: 40,
                duty_min_percent: 3,
                kp_percent_per_100: 2,
                ff_count_s_per_percent: 100,
                deadband_count: 10,
                deadband_out_count: 20,
            })),
            frame_hex: "01 0D 10 00 0D 00 00 00 28 03 02 00 64 00 0A 00 14 00 00 00 40 08",
            wire_hex: "04 01 0D 10 02 0D 01 01 04 28 03 02 02 64 02 0A 02 14 01 01 03 40 08 00",
        },
        Vector {
            title: "LIMITS",
            msg: Msg::Mcu(McuMsg::Limits(Limits {
                boot_id: BOOT_ID,
                duty_max_percent: 40,
                duty_min_percent: 3,
                kp_percent_per_100: 2,
                abs_duty_max_percent: 60,
                ff_count_s_per_percent: 100,
                deadband_count: 10,
                deadband_out_count: 20,
                abs_max_vel_count_s: 6000,
                abs_max_acc_count_s2: 40000,
            })),
            frame_hex: "01 88 18 00 EF CD AB 89 28 03 02 3C 64 00 0A 00 14 00 00 00 70 17 00 00 40 9C 00 00 BF DB",
            wire_hex: "04 01 88 18 0A EF CD AB 89 28 03 02 3C 64 02 0A 02 14 01 01 03 70 17 01 03 40 9C 01 03 BF DB 00",
        },
        Vector {
            title: "PMIC_REPORT",
            msg: Msg::Mcu(McuMsg::PmicReport(sample_pmic())),
            frame_hex: "01 89 5C 00 02 7F 0E 00 0E 00 01 00 03 00 00 00 40 4B 4C 00 00 00 00 00 78 00 00 00 88 13 E4 0C 04 10 24 13 FA 00 74 13 DA 0C F0 0F 20 08 00 00 BF B4 38 02 00 00 00 00 21 08 48 33 36 38 00 05 00 00 03 00 01 20 00 1F 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 9D 58",
            wire_hex: "04 01 89 5C 04 02 7F 0E 02 0E 02 01 02 03 01 01 04 40 4B 4C 01 01 01 01 02 78 01 01 0A 88 13 E4 0C 04 10 24 13 FA 09 74 13 DA 0C F0 0F 20 08 01 05 BF B4 38 02 01 01 01 07 21 08 48 33 36 38 02 05 01 02 03 03 01 20 02 1F 01 01 01 01 01 01 01 01 02 01 01 01 01 01 01 01 01 01 01 01 01 01 01 03 9D 58 00",
        },
    ]
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn frame_bytes(m: &Msg) -> Vec<u8> {
    frame::build(m.msg_type(), &m.payload())
}

/// Markdown for the protocol.md §7 block, generated from the vectors above.
fn markdown() -> String {
    let mut s = String::new();
    s.push_str(
        "`frame` は COBS 前のフレーム（header + payload + crc）、`wire` は COBS 符号化して区切り `0x00` を付けた USB 上のバイト列。\
         いずれも 16 進・空白区切り。「値」は各フィールドの値（JSON、フィールドはレイアウト順、数値は 10 進。\
         `device_id` は 16 桁 hex、`fault_code` / `result` / `reason` は名前で表記。CAL_RESULT の `reason` は NUL を除いた ASCII 文字列）。\n",
    );
    for (i, v) in vectors().iter().enumerate() {
        let f = frame_bytes(&v.msg);
        let w = v.msg.wire();
        s.push_str(&format!(
            "\n### 7.{n} {title}（`0x{t:02X}`、{dir}）\n\n値: `{fields}`\n\n```text\nframe ({fl} byte): {fh}\nwire  ({wl} byte): {wh}\n```\n",
            n = i + 1,
            title = v.title,
            t = v.msg.msg_type(),
            dir = v.msg.direction(),
            fields = v.msg.fields_json(),
            fl = f.len(),
            fh = hex(&f),
            wl = w.len(),
            wh = hex(&w),
        ));
    }
    s
}

#[test]
fn vectors_match_expected_bytes() {
    for v in vectors() {
        assert_eq!(hex(&frame_bytes(&v.msg)), v.frame_hex, "{} frame", v.title);
        assert_eq!(hex(&v.msg.wire()), v.wire_hex, "{} wire", v.title);
    }
}

#[test]
fn vectors_decode_from_wire_bytes() {
    for v in vectors() {
        let wire: Vec<u8> = v
            .wire_hex
            .split(' ')
            .map(|b| u8::from_str_radix(b, 16).expect("hex"))
            .collect();
        let frames = FrameAccumulator::new(MAX_ENCODED_LEN).push(&wire);
        assert_eq!(frames.len(), 1, "{}", v.title);
        match v.msg {
            Msg::Host(m) => assert_eq!(decode_host_frame(&frames[0]), Ok(m), "{}", v.title),
            Msg::Mcu(m) => assert_eq!(decode_mcu_frame(&frames[0]), Ok(m), "{}", v.title),
        }
    }
}

#[test]
fn vectors_cover_every_message_type() {
    let mut seen: Vec<u8> = vectors().iter().map(|v| v.msg.msg_type()).collect();
    seen.sort();
    seen.dedup();
    let mut all: Vec<u8> = (0..=255u8)
        .filter(|&t| host_payload_len(t).is_some() || mcu_payload_len(t).is_some())
        .collect();
    all.sort();
    assert_eq!(seen, all);
}

#[test]
fn doc_matches_vectors() {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../docs/control-server/protocol.md"
    );
    let doc = std::fs::read_to_string(path).expect("read protocol.md");
    let begin = "<!-- VECTORS:BEGIN -->";
    let end = "<!-- VECTORS:END -->";
    let a = doc.find(begin).expect("BEGIN marker") + begin.len();
    let b = doc.find(end).expect("END marker");
    let block = doc[a..b].trim();
    let want = markdown();
    assert!(
        block == want.trim(),
        "protocol.md §7 vectors are out of date; regenerate with \
         `cargo test --test protocol_vectors -- --ignored --nocapture print_vectors`"
    );
}

/// Prints the doc block and the Rust hex literals for this file.
#[test]
#[ignore]
fn print_vectors() {
    println!("----- markdown -----\n{}", markdown());
    println!("----- rust -----");
    for v in vectors() {
        println!(
            "{}\n            frame_hex: \"{}\",",
            v.title,
            hex(&frame_bytes(&v.msg))
        );
        println!("            wire_hex: \"{}\",", hex(&v.msg.wire()));
    }
}
