// v1_codec のホストテスト。test_host.cpp から include する（CHECK マクロは test_host.cpp のもの）。
// protocol.md §7 の全テストベクタ（frame / wire の hex をそのまま埋め込む）で encode / decode の両方向を確認し、
// 不正フレームの破棄（CRC・長さ・版・未知種別・256 byte 超）も見る。
#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "v1_codec.h"

namespace v1test {

// "01 02 0A" のような 16 進・空白区切りをバイト列にする。
inline std::vector<uint8_t> hex(const char* text) {
  std::vector<uint8_t> out;
  const char* p = text;
  while (*p) {
    while (*p == ' ') ++p;
    if (!*p) break;
    char* end = nullptr;
    out.push_back(static_cast<uint8_t>(std::strtoul(p, &end, 16)));
    p = end;
  }
  return out;
}

inline bool same(const std::vector<uint8_t>& a, const uint8_t* b, size_t n) {
  return a.size() == n && std::memcmp(a.data(), b, n) == 0;
}

// PC → MCU ベクタ: wire → HostMsg（check が値を見る）、HostMsg → frame / wire がベクタと一致。
inline void check_host_vector(const char* frame_hex, const char* wire_hex, const v1::HostMsg& expected,
                              bool (*equal)(const v1::HostMsg&, const v1::HostMsg&)) {
  const std::vector<uint8_t> frame = hex(frame_hex);
  const std::vector<uint8_t> wire = hex(wire_hex);
  // decode: wire（区切りを除く）→ struct
  v1::HostMsg got{};
  CHECK(v1::decode_host_encoded(wire.data(), wire.size() - 1, &got));
  CHECK(got.type == expected.type);
  CHECK(equal(got, expected));
  // decode: frame → struct
  v1::HostMsg got2{};
  CHECK(v1::decode_host_frame(frame.data(), frame.size(), &got2));
  CHECK(equal(got2, expected));
  // encode: struct → frame / wire
  uint8_t fbuf[v1::MAX_FRAME_LEN];
  const size_t flen = v1::encode_host_frame(expected, fbuf, sizeof(fbuf));
  CHECK(same(frame, fbuf, flen));
  uint8_t wbuf[v1::MAX_WIRE_LEN];
  const size_t wlen = v1::encode_host_wire(expected, wbuf, sizeof(wbuf));
  CHECK(same(wire, wbuf, wlen));
  // COBS の往復
  uint8_t dec[v1::MAX_FRAME_LEN];
  size_t dlen = 0;
  CHECK(v1::cobs_decode(wire.data(), wire.size() - 1, dec, sizeof(dec), &dlen));
  CHECK(same(frame, dec, dlen));
}

inline void check_mcu_vector(const char* frame_hex, const char* wire_hex, const v1::McuMsg& expected,
                             bool (*equal)(const v1::McuMsg&, const v1::McuMsg&)) {
  const std::vector<uint8_t> frame = hex(frame_hex);
  const std::vector<uint8_t> wire = hex(wire_hex);
  uint8_t fbuf[v1::MAX_FRAME_LEN];
  const size_t flen = v1::encode_mcu_frame(expected, fbuf, sizeof(fbuf));
  CHECK(same(frame, fbuf, flen));
  uint8_t wbuf[v1::MAX_WIRE_LEN];
  const size_t wlen = v1::encode_mcu_wire(expected, wbuf, sizeof(wbuf));
  CHECK(same(wire, wbuf, wlen));
  v1::McuMsg got{};
  CHECK(v1::decode_mcu_encoded(wire.data(), wire.size() - 1, &got));
  CHECK(got.type == expected.type);
  CHECK(equal(got, expected));
  v1::McuMsg got2{};
  CHECK(v1::decode_mcu_frame(frame.data(), frame.size(), &got2));
  CHECK(equal(got2, expected));
}

inline bool eq_hello_req(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.hello_req.request_id == b.hello_req.request_id;
}
inline bool eq_config(const v1::HostMsg& a, const v1::HostMsg& b) {
  const v1::Config& x = a.config;
  const v1::Config& y = b.config;
  return x.request_id == y.request_id && x.session_epoch == y.session_epoch &&
         x.calibration_hash == y.calibration_hash && x.soft_min_count == y.soft_min_count &&
         x.soft_max_count == y.soft_max_count && x.max_vel_count_s == y.max_vel_count_s &&
         x.max_acc_count_s2 == y.max_acc_count_s2 &&
         x.following_error_limit_count == y.following_error_limit_count &&
         x.overspeed_count_s == y.overspeed_count_s && x.command_timeout_ms == y.command_timeout_ms &&
         x.stop_timeout_ms == y.stop_timeout_ms && x.role_flags == y.role_flags;
}
inline bool eq_arm(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.arm_prepare.request_id == b.arm_prepare.request_id &&
         a.arm_prepare.session_epoch == b.arm_prepare.session_epoch;
}
inline bool eq_start(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.start.request_id == b.start.request_id && a.start.session_epoch == b.start.session_epoch;
}
inline bool eq_set_position(const v1::HostMsg& a, const v1::HostMsg& b) {
  const v1::SetPosition& x = a.set_position;
  const v1::SetPosition& y = b.set_position;
  return x.session_epoch == y.session_epoch && x.seq == y.seq && x.group_frame_id == y.group_frame_id &&
         x.target_count == y.target_count && x.max_vel_count_s == y.max_vel_count_s &&
         x.max_acc_count_s2 == y.max_acc_count_s2 && x.valid_for_ms == y.valid_for_ms &&
         x.apply_at_us == y.apply_at_us;
}
inline bool eq_stop(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.controlled_stop.request_id == b.controlled_stop.request_id &&
         a.controlled_stop.session_epoch == b.controlled_stop.session_epoch &&
         a.controlled_stop.reason == b.controlled_stop.reason;
}
inline bool eq_disarm(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.disarm.request_id == b.disarm.request_id;
}
inline bool eq_fault_reset(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.fault_reset.request_id == b.fault_reset.request_id;
}
inline bool eq_clock_sync(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.clock_sync.request_id == b.clock_sync.request_id && a.clock_sync.pc_time_us == b.clock_sync.pc_time_us;
}
inline bool eq_host_fault(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.host_fault.request_id == b.host_fault.request_id && a.host_fault.fault_code == b.host_fault.fault_code;
}
inline bool eq_set_reference(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.set_reference.request_id == b.set_reference.request_id &&
         a.set_reference.reference_count == b.set_reference.reference_count;
}
inline bool eq_calibrate(const v1::HostMsg& a, const v1::HostMsg& b) {
  return a.calibrate.request_id == b.calibrate.request_id && a.calibrate.mode == b.calibrate.mode &&
         a.calibrate.flags == b.calibrate.flags;
}

inline bool eq_hello(const v1::McuMsg& a, const v1::McuMsg& b) {
  const v1::Hello& x = a.hello;
  const v1::Hello& y = b.hello;
  return x.request_id == y.request_id && x.protocol_version == y.protocol_version &&
         x.selected_axis == y.selected_axis && x.mcu_state == y.mcu_state && x.device_id == y.device_id &&
         x.boot_id == y.boot_id && x.fw_major == y.fw_major && x.fw_minor == y.fw_minor &&
         x.fw_patch == y.fw_patch && x.capabilities == y.capabilities && x.calibration_hash == y.calibration_hash;
}
inline bool eq_ack(const v1::McuMsg& a, const v1::McuMsg& b) {
  return a.ack.request_id == b.ack.request_id && a.ack.acked_type == b.ack.acked_type &&
         a.ack.result == b.ack.result && a.ack.reason == b.ack.reason;
}
inline bool eq_status(const v1::McuMsg& a, const v1::McuMsg& b) {
  const v1::Status& x = a.status;
  const v1::Status& y = b.status;
  return x.boot_id == y.boot_id && x.mcu_time_us == y.mcu_time_us && x.mcu_state == y.mcu_state &&
         x.selected_axis == y.selected_axis && x.fault_code == y.fault_code && x.flags == y.flags &&
         x.pwm_permille == y.pwm_permille && x.actual_count == y.actual_count &&
         x.target_count == y.target_count && x.velocity_count_s == y.velocity_count_s &&
         x.following_error_count == y.following_error_count && x.last_applied_frame == y.last_applied_frame &&
         x.last_applied_seq == y.last_applied_seq && x.last_rx_seq == y.last_rx_seq &&
         x.current_ma == y.current_ma && x.rx_error_count == y.rx_error_count &&
         x.stale_drop_count == y.stale_drop_count;
}
inline bool eq_fault_event(const v1::McuMsg& a, const v1::McuMsg& b) {
  const v1::FaultEvent& x = a.fault_event;
  const v1::FaultEvent& y = b.fault_event;
  return x.boot_id == y.boot_id && x.mcu_time_us == y.mcu_time_us && x.fault_code == y.fault_code &&
         x.mcu_state_before == y.mcu_state_before && x.actual_count == y.actual_count;
}
inline bool eq_clock_reply(const v1::McuMsg& a, const v1::McuMsg& b) {
  const v1::ClockSyncReply& x = a.clock_sync_reply;
  const v1::ClockSyncReply& y = b.clock_sync_reply;
  return x.request_id == y.request_id && x.pc_time_us == y.pc_time_us && x.mcu_rx_us == y.mcu_rx_us &&
         x.mcu_tx_us == y.mcu_tx_us;
}
inline bool eq_axis_select(const v1::McuMsg& a, const v1::McuMsg& b) {
  const v1::AxisSelect& x = a.axis_select;
  const v1::AxisSelect& y = b.axis_select;
  return x.boot_id == y.boot_id && x.selected_axis == y.selected_axis &&
         x.prev_selected_axis == y.prev_selected_axis && x.mcu_state == y.mcu_state;
}
inline bool eq_cal_result(const v1::McuMsg& a, const v1::McuMsg& b) {
  const v1::CalResult& x = a.cal_result;
  const v1::CalResult& y = b.cal_result;
  return x.boot_id == y.boot_id && x.result == y.result && x.step == y.step && x.n_end_count == y.n_end_count &&
         x.neutral_count == y.neutral_count && x.sign == y.sign &&
         std::memcmp(x.reason, y.reason, v1::CAL_REASON_LEN) == 0;
}

// ---- protocol.md §7 のベクタ -------------------------------------------------

inline void test_vectors_host() {
  {  // 7.1 HELLO_REQ
    v1::HostMsg m{};
    m.type = v1::msg::HELLO_REQ;
    m.hello_req.request_id = 1;
    check_host_vector("01 01 04 00 01 00 00 00 99 BE", "04 01 01 04 02 01 01 01 03 99 BE 00", m, eq_hello_req);
  }
  {  // 7.2 CONFIG
    v1::HostMsg m{};
    m.type = v1::msg::CONFIG;
    m.config = {2, 3, 305419896u, -2700, 2700, 4000, 20000, 200, 6000, 100, 1500, 1};
    check_host_vector(
        "01 02 2C 00 02 00 00 00 03 00 00 00 78 56 34 12 74 F5 FF FF 8C 0A 00 00 A0 0F 00 00 20 4E 00 00 C8 00 00 "
        "00 70 17 00 00 64 00 DC 05 01 00 00 00 D2 48",
        "04 01 02 2C 02 02 01 01 02 03 01 01 0B 78 56 34 12 74 F5 FF FF 8C 0A 01 03 A0 0F 01 03 20 4E 01 02 C8 01 "
        "01 03 70 17 01 02 64 04 DC 05 01 01 01 03 D2 48 00",
        m, eq_config);
  }
  {  // 7.3 ARM_PREPARE
    v1::HostMsg m{};
    m.type = v1::msg::ARM_PREPARE;
    m.arm_prepare = {4, 3};
    check_host_vector("01 03 08 00 04 00 00 00 03 00 00 00 8C 21",
                      "04 01 03 08 02 04 01 01 02 03 01 01 03 8C 21 00", m, eq_arm);
  }
  {  // 7.4 START
    v1::HostMsg m{};
    m.type = v1::msg::START;
    m.start = {5, 3};
    check_host_vector("01 04 08 00 05 00 00 00 03 00 00 00 E2 4F",
                      "04 01 04 08 02 05 01 01 02 03 01 01 03 E2 4F 00", m, eq_start);
  }
  {  // 7.5 SET_POSITION
    v1::HostMsg m{};
    m.type = v1::msg::SET_POSITION;
    m.set_position = {3, 1000, 1234, -1234, 4000, 20000, 50, 0};
    check_host_vector(
        "01 05 24 00 03 00 00 00 E8 03 00 00 D2 04 00 00 2E FB FF FF A0 0F 00 00 20 4E 00 00 32 00 00 00 00 00 00 "
        "00 00 00 00 00 1D 07",
        "04 01 05 24 02 03 01 01 03 E8 03 01 03 D2 04 01 07 2E FB FF FF A0 0F 01 03 20 4E 01 02 32 01 01 01 01 01 "
        "01 01 01 01 01 03 1D 07 00",
        m, eq_set_position);
  }
  {  // 7.6 CONTROLLED_STOP
    v1::HostMsg m{};
    m.type = v1::msg::CONTROLLED_STOP;
    m.controlled_stop = {6, 3, 1};
    check_host_vector("01 06 0C 00 06 00 00 00 03 00 00 00 01 00 00 00 74 B7",
                      "04 01 06 0C 02 06 01 01 02 03 01 01 02 01 01 01 03 74 B7 00", m, eq_stop);
  }
  {  // 7.7 DISARM
    v1::HostMsg m{};
    m.type = v1::msg::DISARM;
    m.disarm = {7};
    check_host_vector("01 07 04 00 07 00 00 00 25 38", "04 01 07 04 02 07 01 01 03 25 38 00", m, eq_disarm);
  }
  {  // 7.8 FAULT_RESET
    v1::HostMsg m{};
    m.type = v1::msg::FAULT_RESET;
    m.fault_reset = {8};
    check_host_vector("01 08 04 00 08 00 00 00 22 66", "04 01 08 04 02 08 01 01 03 22 66 00", m, eq_fault_reset);
  }
  {  // 7.9 CLOCK_SYNC
    v1::HostMsg m{};
    m.type = v1::msg::CLOCK_SYNC;
    m.clock_sync = {9, 1000000};
    check_host_vector("01 09 0C 00 09 00 00 00 40 42 0F 00 00 00 00 00 58 CB",
                      "04 01 09 0C 02 09 01 01 04 40 42 0F 01 01 01 01 03 58 CB 00", m, eq_clock_sync);
  }
  {  // 7.10 HOST_FAULT
    v1::HostMsg m{};
    m.type = v1::msg::HOST_FAULT;
    m.host_fault = {10, v1::fault::HOST_FAULT};
    check_host_vector("01 0A 08 00 0A 00 00 00 0A 00 00 00 2D 36",
                      "04 01 0A 08 02 0A 01 01 02 0A 01 01 03 2D 36 00", m, eq_host_fault);
  }
  {  // 7.11 SET_REFERENCE
    v1::HostMsg m{};
    m.type = v1::msg::SET_REFERENCE;
    m.set_reference = {11, -2770};
    check_host_vector("01 0B 08 00 0B 00 00 00 2E F5 FF FF A5 41",
                      "04 01 0B 08 02 0B 01 01 07 2E F5 FF FF A5 41 00", m, eq_set_reference);
  }
  {  // 7.19 CALIBRATE（request_id 7、mode 1 = 開始）。CRC は独立した実装（Python）でも確認した値
    v1::HostMsg m{};
    m.type = v1::msg::CALIBRATE;
    m.calibrate = {7, 1, 0};
    check_host_vector("01 0C 08 00 07 00 00 00 01 00 00 00 A2 8F",
                      "04 01 0C 08 02 07 01 01 02 01 01 01 03 A2 8F 00", m, eq_calibrate);
    CHECK(v1::host_payload_len(v1::msg::CALIBRATE) == 8);
    // mode 0（中止）も同じ長さ
    m.calibrate = {12, 0, 0};
    uint8_t f[v1::MAX_FRAME_LEN];
    CHECK(v1::encode_host_frame(m, f, sizeof(f)) == 4 + 8 + 2);
    v1::HostMsg d{};
    CHECK(v1::decode_host_frame(f, 14, &d) && d.calibrate.request_id == 12 && d.calibrate.mode == 0 && d.calibrate.flags == 0);
    // flags（byte 5、bit0 = KEEP_POWER）は符号化・復号で保たれ、予約の 2 バイトは 0
    m.calibrate = {13, 1, v1::cal_flags::KEEP_POWER};
    CHECK(v1::encode_host_frame(m, f, sizeof(f)) == 4 + 8 + 2);
    CHECK(f[4 + 4] == 1 && f[4 + 5] == 0x01 && f[4 + 6] == 0 && f[4 + 7] == 0);
    CHECK(v1::decode_host_frame(f, 14, &d) && d.calibrate.request_id == 13 && d.calibrate.mode == 1 &&
          d.calibrate.flags == v1::cal_flags::KEEP_POWER);
  }
}

inline void test_vectors_mcu() {
  {  // 7.12 HELLO
    v1::McuMsg m{};
    m.type = v1::msg::HELLO;
    m.hello = {1, 1, 1, 1, 0x5131000000000001ull, 2309737967u, 0, 1, 0, 1, 0};
    check_mcu_vector(
        "01 81 20 00 01 00 00 00 01 01 01 00 01 00 00 00 00 00 31 51 EF CD AB 89 00 01 00 00 01 00 00 00 00 00 00 "
        "00 1E 98",
        "04 01 81 20 02 01 01 01 04 01 01 01 02 01 01 01 01 01 07 31 51 EF CD AB 89 02 01 01 02 01 01 01 01 01 01 "
        "01 03 1E 98 00",
        m, eq_hello);
  }
  {  // 7.13 ACK ACCEPTED
    v1::McuMsg m{};
    m.type = v1::msg::ACK;
    m.ack = {2, 2, v1::ack_result::ACCEPTED, v1::ack_reason::NONE};
    check_mcu_vector("01 82 08 00 02 00 00 00 02 00 00 00 10 86",
                     "04 01 82 08 02 02 01 01 02 02 01 01 03 10 86 00", m, eq_ack);
  }
  {  // 7.14 ACK REJECTED / EPOCH_MISMATCH
    v1::McuMsg m{};
    m.type = v1::msg::ACK;
    m.ack = {4, 3, v1::ack_result::REJECTED, v1::ack_reason::EPOCH_MISMATCH};
    check_mcu_vector("01 82 08 00 04 00 00 00 03 01 02 00 3D 21",
                     "04 01 82 08 02 04 01 01 04 03 01 02 03 3D 21 00", m, eq_ack);
  }
  {  // 7.15 STATUS
    v1::McuMsg m{};
    m.type = v1::msg::STATUS;
    m.status = {2309737967u, 1234567890ull, 4, 1, v1::fault::NONE, 31, -250, -1230, -1234, -150, 4, 1234, 1000, 1000,
                0, 1, 2};
    check_mcu_vector(
        "01 83 38 00 EF CD AB 89 D2 02 96 49 00 00 00 00 04 01 00 00 1F 00 06 FF 32 FB FF FF 2E FB FF FF 6A FF FF "
        "FF 04 00 00 00 D2 04 00 00 E8 03 00 00 E8 03 00 00 00 00 00 00 01 00 02 00 49 36",
        "04 01 83 38 09 EF CD AB 89 D2 02 96 49 01 01 01 03 04 01 01 02 1F 10 06 FF 32 FB FF FF 2E FB FF FF 6A FF "
        "FF FF 04 01 01 03 D2 04 01 03 E8 03 01 03 E8 03 01 01 01 01 01 02 01 02 02 03 49 36 00",
        m, eq_status);
  }
  {  // 7.16 FAULT_EVENT
    v1::McuMsg m{};
    m.type = v1::msg::FAULT_EVENT;
    m.fault_event = {2309737967u, 1234600000ull, v1::fault::FOLLOWING_ERROR, 4, -1500};
    check_mcu_vector("01 84 14 00 EF CD AB 89 40 80 96 49 00 00 00 00 02 00 04 00 24 FA FF FF 3F 2B",
                     "04 01 84 14 09 EF CD AB 89 40 80 96 49 01 01 01 02 02 02 04 07 24 FA FF FF 3F 2B 00", m,
                     eq_fault_event);
  }
  {  // 7.17 CLOCK_SYNC_REPLY
    v1::McuMsg m{};
    m.type = v1::msg::CLOCK_SYNC_REPLY;
    m.clock_sync_reply = {9, 1000000ull, 5000000ull, 5000120ull};
    check_mcu_vector(
        "01 85 1C 00 09 00 00 00 40 42 0F 00 00 00 00 00 40 4B 4C 00 00 00 00 00 B8 4B 4C 00 00 00 00 00 A9 A7",
        "04 01 85 1C 02 09 01 01 04 40 42 0F 01 01 01 01 04 40 4B 4C 01 01 01 01 04 B8 4B 4C 01 01 01 01 03 A9 A7 "
        "00",
        m, eq_clock_reply);
  }
  {  // 7.18 AXIS_SELECT
    v1::McuMsg m{};
    m.type = v1::msg::AXIS_SELECT;
    m.axis_select = {2309737967u, 2, 1, 1};
    check_mcu_vector("01 86 08 00 EF CD AB 89 02 01 01 00 32 C1",
                     "04 01 86 08 08 EF CD AB 89 02 01 01 03 32 C1 00", m, eq_axis_select);
  }
  {  // 7.20 CAL_RESULT（boot_id 0x89ABCDEF、OK、step 3、n_end 5551、neutral 2775、sign +1、reason "ok"）
    v1::McuMsg m{};
    m.type = v1::msg::CAL_RESULT;
    m.cal_result.boot_id = 2309737967u;
    m.cal_result.result = v1::cal_result::OK;
    m.cal_result.step = v1::cal_step::TO_NEUTRAL;
    m.cal_result.n_end_count = 5551;
    m.cal_result.neutral_count = 2775;
    m.cal_result.sign = 1;
    std::memcpy(m.cal_result.reason, "ok\0\0\0\0\0\0", v1::CAL_REASON_LEN);
    check_mcu_vector(
        "01 87 1C 00 EF CD AB 89 00 03 00 00 AF 15 00 00 D7 0A 00 00 01 00 00 00 6F 6B 00 00 00 00 00 00 7A C9",
        "04 01 87 1C 05 EF CD AB 89 02 03 01 03 AF 15 01 03 D7 0A 01 02 01 01 01 03 6F 6B 01 01 01 01 01 03 7A C9 00",
        m, eq_cal_result);
    CHECK(v1::mcu_payload_len(v1::msg::CAL_RESULT) == 28);
    // 負の値と sign −1、8 文字ちょうどの reason（終端なし）も往復する
    m.cal_result.result = v1::cal_result::FAILED;
    m.cal_result.step = v1::cal_step::TO_N;
    m.cal_result.n_end_count = -5551;
    m.cal_result.neutral_count = -2775;
    m.cal_result.sign = -1;
    std::memcpy(m.cal_result.reason, "abcdefgh", v1::CAL_REASON_LEN);
    uint8_t w[v1::MAX_WIRE_LEN];
    const size_t n = v1::encode_mcu_wire(m, w, sizeof(w));
    CHECK(n > 0);
    v1::McuMsg d{};
    CHECK(v1::decode_mcu_encoded(w, n - 1, &d) && eq_cal_result(d, m) && d.cal_result.sign == -1);
    // 長さ違い（27 byte）は捨てる
    uint8_t p[28] = {0};
    CHECK(!v1::decode_mcu_payload(v1::msg::CAL_RESULT, p, 27, &d));
    v1::HostMsg h{};
    CHECK(!v1::decode_host_payload(v1::msg::CALIBRATE, p, 9, &h));
    CHECK(!v1::decode_host_payload(v1::msg::CALIBRATE, p, 7, &h));
    CHECK(v1::decode_host_payload(v1::msg::CALIBRATE, p, 8, &h) && h.calibrate.mode == 0);
  }
}

// ---- 破棄されるフレーム ------------------------------------------------------

inline void test_crc_known_values() {
  // CRC-16/CCITT-FALSE の check 値（"123456789" → 0x29B1）
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(v1::crc16(s, sizeof(s)) == 0x29B1);
  CHECK(v1::crc16(s, 0) == 0xFFFF);
}

inline void test_rejections() {
  const std::vector<uint8_t> good = hex("01 07 04 00 07 00 00 00 25 38");  // DISARM
  v1::HostMsg m{};
  CHECK(v1::decode_host_frame(good.data(), good.size(), &m));

  // CRC 不一致
  std::vector<uint8_t> bad = good;
  bad.back() ^= 0x5A;
  CHECK(!v1::decode_host_frame(bad.data(), bad.size(), &m));

  // version 不一致（CRC は正しく付け直す）
  bad = good;
  bad[0] = 2;
  v1::put_u16(&bad[bad.size() - 2], v1::crc16(bad.data(), bad.size() - 2));
  CHECK(!v1::decode_host_frame(bad.data(), bad.size(), &m));

  // 未知の種別
  bad = good;
  bad[1] = 0x7F;
  v1::put_u16(&bad[bad.size() - 2], v1::crc16(bad.data(), bad.size() - 2));
  CHECK(!v1::decode_host_frame(bad.data(), bad.size(), &m));
  // MCU 側の種別を PC → MCU として受けても未知
  bad[1] = v1::msg::ACK;
  v1::put_u16(&bad[bad.size() - 2], v1::crc16(bad.data(), bad.size() - 2));
  CHECK(!v1::decode_host_frame(bad.data(), bad.size(), &m));

  // payload_len が実長と違う（header の値だけ変える）
  bad = good;
  bad[2] = 5;
  v1::put_u16(&bad[bad.size() - 2], v1::crc16(bad.data(), bad.size() - 2));
  CHECK(!v1::decode_host_frame(bad.data(), bad.size(), &m));

  // 実長が種別の定義長と違う（DISARM に 5 byte の payload。header も合わせる）
  bad = good;
  bad.insert(bad.begin() + 8, 0x00);
  bad[2] = 5;
  v1::put_u16(&bad[bad.size() - 2], v1::crc16(bad.data(), bad.size() - 2));
  CHECK(!v1::decode_host_frame(bad.data(), bad.size(), &m));

  // 短すぎる
  CHECK(!v1::decode_host_frame(good.data(), 5, &m));

  // 256 byte 超のフレーム（CRC は正しい）
  std::vector<uint8_t> big(257, 0x11);
  big[0] = 1;
  big[1] = v1::msg::DISARM;
  v1::put_u16(&big[2], 251);
  v1::put_u16(&big[255], v1::crc16(big.data(), 255));
  CHECK(!v1::decode_host_frame(big.data(), big.size(), &m));
  // COBS 経由でも復号後 256 超は捨てる
  uint8_t enc[320];
  const size_t elen = v1::cobs_encode(big.data(), big.size(), enc, sizeof(enc));
  CHECK(elen > 0);
  CHECK(!v1::decode_host_encoded(enc, elen, &m));

  // COBS: ブロック内に 0x00（不正）
  const std::vector<uint8_t> z = hex("03 01 00 02 05");
  uint8_t dec[16];
  size_t dlen = 0;
  CHECK(!v1::cobs_decode(z.data(), z.size(), dec, sizeof(dec), &dlen));
  // COBS: コードが末尾を越える
  const std::vector<uint8_t> over = hex("05 01 02");
  CHECK(!v1::cobs_decode(over.data(), over.size(), dec, sizeof(dec), &dlen));
  // COBS: 空入力
  CHECK(!v1::cobs_decode(over.data(), 0, dec, sizeof(dec), &dlen));
}

inline void test_cobs_edge_cases() {
  // Wikipedia の例: 00 → 01 01、00 00 → 01 01 01、11 22 00 33 → 03 11 22 02 33、11 22 33 44 → 05 11 22 33 44
  uint8_t out[600];
  uint8_t dec[600];
  size_t dlen = 0;
  {
    const uint8_t in[] = {0x00};
    CHECK(v1::cobs_encode(in, 1, out, sizeof(out)) == 2 && out[0] == 1 && out[1] == 1);
  }
  {
    const uint8_t in[] = {0x00, 0x00};
    CHECK(v1::cobs_encode(in, 2, out, sizeof(out)) == 3 && out[0] == 1 && out[1] == 1 && out[2] == 1);
  }
  {
    const uint8_t in[] = {0x11, 0x22, 0x00, 0x33};
    const std::vector<uint8_t> want = hex("03 11 22 02 33");
    CHECK(same(want, out, v1::cobs_encode(in, 4, out, sizeof(out))));
    CHECK(v1::cobs_decode(out, want.size(), dec, sizeof(dec), &dlen) && dlen == 4 && std::memcmp(dec, in, 4) == 0);
  }
  {
    // 非ゼロ 254 byte: 末尾に 01 を付けない。付いていても受け付ける。
    uint8_t in[254];
    for (int i = 0; i < 254; ++i) in[i] = static_cast<uint8_t>(i + 1);
    const size_t n = v1::cobs_encode(in, 254, out, sizeof(out));
    CHECK(n == 255 && out[0] == 0xFF);
    CHECK(v1::cobs_decode(out, n, dec, sizeof(dec), &dlen) && dlen == 254 && std::memcmp(dec, in, 254) == 0);
    out[255] = 0x01;
    CHECK(v1::cobs_decode(out, 256, dec, sizeof(dec), &dlen) && dlen == 254 && std::memcmp(dec, in, 254) == 0);
    // 255 byte 非ゼロ: FF ブロックの後に 02 xx
    uint8_t in2[255];
    for (int i = 0; i < 255; ++i) in2[i] = static_cast<uint8_t>((i % 254) + 1);
    const size_t n2 = v1::cobs_encode(in2, 255, out, sizeof(out));
    CHECK(n2 == 257 && out[0] == 0xFF && out[255] == 0x02);
    CHECK(v1::cobs_decode(out, n2, dec, sizeof(dec), &dlen) && dlen == 255 && std::memcmp(dec, in2, 255) == 0);
    // 254 byte 非ゼロ + 00: FF ブロックの後に 01 01（Wikipedia の例 9）
    uint8_t in3[255];
    std::memcpy(in3, in, 254);
    in3[254] = 0;
    const size_t n3 = v1::cobs_encode(in3, 255, out, sizeof(out));
    CHECK(n3 == 257 && out[0] == 0xFF && out[255] == 0x01 && out[256] == 0x01);
    CHECK(v1::cobs_decode(out, n3, dec, sizeof(dec), &dlen) && dlen == 255 && std::memcmp(dec, in3, 255) == 0);
  }
  // 容量不足
  {
    const uint8_t in[] = {0x11, 0x22, 0x33};
    CHECK(v1::cobs_encode(in, 3, out, 3) == 0);
    CHECK(v1::cobs_encode(in, 3, out, 4) == 4);
  }
}

inline void test_frame_splitter() {
  v1::FrameSplitter sp;
  const std::vector<uint8_t> a = hex("04 01 07 04 02 07 01 01 03 25 38 00");
  const std::vector<uint8_t> b = hex("04 01 08 04 02 08 01 01 03 22 66 00");
  // 2 フレーム連続 + 空フレーム（00 00）
  std::vector<uint8_t> stream = a;
  stream.push_back(0);
  stream.push_back(0);
  stream.insert(stream.end(), b.begin(), b.end());
  int frames = 0;
  for (uint8_t c : stream) {
    if (sp.push(c)) {
      frames += 1;
      v1::HostMsg m{};
      CHECK(v1::decode_host_encoded(sp.buf, sp.len, &m));
      CHECK(m.type == (frames == 1 ? v1::msg::DISARM : v1::msg::FAULT_RESET));
    }
  }
  CHECK(frames == 2);
  CHECK(sp.dropped == 0);
  // 300 byte 超は捨てて次の 00 まで読み飛ばし、その後のフレームは取れる
  for (int i = 0; i < 400; ++i) CHECK(!sp.push(0x55));
  CHECK(sp.dropped == 1);
  CHECK(!sp.push(0x00));
  frames = 0;
  for (uint8_t c : a) {
    if (sp.push(c)) frames += 1;
  }
  CHECK(frames == 1 && sp.len == a.size() - 1);
  // 取り出した後の次のバイトで空になる
  CHECK(!sp.push(0x04));
  CHECK(sp.len == 1);
  sp.clear();
  CHECK(sp.len == 0);
}

inline void run_all() {
  test_crc_known_values();
  test_vectors_host();
  test_vectors_mcu();
  test_rejections();
  test_cobs_edge_cases();
  test_frame_splitter();
}

}  // namespace v1test
