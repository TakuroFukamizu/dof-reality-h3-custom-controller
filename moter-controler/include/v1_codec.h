#pragma once

// USB プロトコル v1（docs/control-server/protocol.md）のフレーム層とメッセージ層。
// 純粋（Arduino 非依存、ヘッダのみ）。<stdint.h> / <string.h> だけに依存し、動的確保をしない。
//
//   USB バイト列:  COBS(frame) 0x00  COBS(frame) 0x00 ...
//   frame:         version(1) msg_type(1) payload_len(u16 LE) payload(N) crc16(LE)
//
// 復号は長さ・版・種別・CRC を検査し、不正なフレームは false を返すだけ（呼び出し側が rx_error として数える）。
// 整数はすべてリトルエンディアン。符号付きは 2 の補数。

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace v1 {

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr size_t MAX_FRAME_LEN = 256;      // 復号後（header + payload + crc）
constexpr size_t MAX_ENCODED_LEN = 300;    // COBS 符号化後、区切りを除く
constexpr size_t MAX_WIRE_LEN = 262;       // COBS(256 byte) は最大 258 byte + 区切り 1（余裕を持たせる）
constexpr size_t HEADER_LEN = 4;
constexpr size_t CRC_LEN = 2;
constexpr size_t MAX_PAYLOAD_LEN = MAX_FRAME_LEN - HEADER_LEN - CRC_LEN;

// --- 種別（protocol.md §4） ---
namespace msg {
constexpr uint8_t HELLO_REQ = 0x01;
constexpr uint8_t CONFIG = 0x02;
constexpr uint8_t ARM_PREPARE = 0x03;
constexpr uint8_t START = 0x04;
constexpr uint8_t SET_POSITION = 0x05;
constexpr uint8_t CONTROLLED_STOP = 0x06;
constexpr uint8_t DISARM = 0x07;
constexpr uint8_t FAULT_RESET = 0x08;
constexpr uint8_t CLOCK_SYNC = 0x09;
constexpr uint8_t HOST_FAULT = 0x0A;
constexpr uint8_t SET_REFERENCE = 0x0B;
constexpr uint8_t CALIBRATE = 0x0C;
constexpr uint8_t SET_LIMITS = 0x0D;  // Duty 上下限・Kp・FF・不感帯の置き換え（motion-limits 設計 §4.2）

constexpr uint8_t HELLO = 0x81;
constexpr uint8_t ACK = 0x82;
constexpr uint8_t STATUS = 0x83;
constexpr uint8_t FAULT_EVENT = 0x84;
constexpr uint8_t CLOCK_SYNC_REPLY = 0x85;
constexpr uint8_t AXIS_SELECT = 0x86;
constexpr uint8_t CAL_RESULT = 0x87;
constexpr uint8_t LIMITS = 0x88;  // SET_LIMITS 受理直後の有効値と絶対上限（motion-limits 設計 §4.3）
}  // namespace msg

// --- 共通の値（protocol.md §3） ---
namespace state {
constexpr uint8_t BOOT = 0;
constexpr uint8_t DISARMED = 1;
constexpr uint8_t ARMING = 2;
constexpr uint8_t HOLDING = 3;
constexpr uint8_t ACTIVE = 4;
constexpr uint8_t STOPPING = 5;
constexpr uint8_t DISARMING = 6;
constexpr uint8_t FAULT = 7;
constexpr uint8_t CALIBRATING = 8;  // CALIBRATE 受理後、端当て校正の間（protocol.md §1.2）。終了で DISARMED
}  // namespace state

namespace fault {
constexpr uint16_t NONE = 0;
constexpr uint16_t COMMAND_TIMEOUT = 1;
constexpr uint16_t FOLLOWING_ERROR = 2;
constexpr uint16_t OVERSPEED = 3;
constexpr uint16_t SOFT_LIMIT = 4;
constexpr uint16_t ENCODER_FAULT = 5;
constexpr uint16_t CONTROL_OVERRUN = 6;
constexpr uint16_t CONFIG_INVALID = 7;
constexpr uint16_t STOP_TIMEOUT = 8;
constexpr uint16_t POWER_FAULT = 9;
constexpr uint16_t HOST_FAULT = 10;
constexpr uint16_t NOT_REFERENCED = 11;
constexpr uint16_t LOCAL_ESTOP = 12;  // MCU の画面の非常停止ボタン（protocol.md §3.2 に追記）
constexpr uint16_t UNKNOWN = 0xFFFF;
}  // namespace fault

namespace ack_result {
constexpr uint8_t ACCEPTED = 0;
constexpr uint8_t REJECTED = 1;
constexpr uint8_t DUPLICATE = 2;
}  // namespace ack_result

namespace ack_reason {
constexpr uint16_t NONE = 0;
constexpr uint16_t BAD_STATE = 1;
constexpr uint16_t EPOCH_MISMATCH = 2;
constexpr uint16_t NOT_CONFIGURED = 3;
constexpr uint16_t INVALID_PARAM = 4;
constexpr uint16_t NOT_REFERENCED = 5;
constexpr uint16_t FAULT_ACTIVE = 6;
constexpr uint16_t FAULT_CAUSE_PRESENT = 7;
}  // namespace ack_reason

namespace flags {
constexpr uint16_t POWER_ON_CMD = 1u << 0;
constexpr uint16_t BRAKE_RELEASE_CMD = 1u << 1;
constexpr uint16_t POSITION_REFERENCED = 1u << 2;
constexpr uint16_t CONFIGURED = 1u << 3;
constexpr uint16_t POWER_CONTROLLER = 1u << 4;
constexpr uint16_t CURRENT_VALID = 1u << 5;
constexpr uint16_t CLOCK_SYNCED = 1u << 6;
}  // namespace flags

namespace role_flags {
constexpr uint8_t POWER_CONTROLLER = 1u << 0;
}

namespace capabilities {
constexpr uint32_t POSITION_MODE = 1u << 0;
constexpr uint32_t APPLY_AT = 1u << 1;
constexpr uint32_t CURRENT_SENSE = 1u << 2;
constexpr uint32_t LIMITS = 1u << 3;  // SET_LIMITS / LIMITS に対応（motion-limits 設計 §4.1）
}  // namespace capabilities

// CALIBRATE の mode（protocol.md §4）
namespace cal_mode {
constexpr uint8_t CANCEL = 0;
constexpr uint8_t START = 1;
}  // namespace cal_mode

// CAL_RESULT の result / step（protocol.md §4）
namespace cal_result {
constexpr uint8_t OK = 0;
constexpr uint8_t FAILED = 1;
constexpr uint8_t CANCELLED = 2;
}  // namespace cal_result

namespace cal_step {
constexpr uint8_t NONE = 0;
constexpr uint8_t TO_P = 1;
constexpr uint8_t TO_N = 2;
constexpr uint8_t TO_NEUTRAL = 3;
constexpr uint8_t POWER_HOLD = 4;  // KEEP_POWER: 校正が終わって主電源だけ保っている
}  // namespace cal_step

namespace cal_flags {
constexpr uint8_t KEEP_POWER = 0x01;
}  // namespace cal_flags

constexpr size_t CAL_REASON_LEN = 8;  // CAL_RESULT の reason（ASCII、NUL 埋め。8 文字ちょうどなら終端なし）

// --- 種別ごとの payload 長。未知の種別は -1 ---
inline int host_payload_len(uint8_t type) {
  switch (type) {
    case msg::HELLO_REQ: return 4;
    case msg::CONFIG: return 44;
    case msg::ARM_PREPARE: return 8;
    case msg::START: return 8;
    case msg::SET_POSITION: return 36;
    case msg::CONTROLLED_STOP: return 12;
    case msg::DISARM: return 4;
    case msg::FAULT_RESET: return 4;
    case msg::CLOCK_SYNC: return 12;
    case msg::HOST_FAULT: return 8;
    case msg::SET_REFERENCE: return 8;
    case msg::CALIBRATE: return 8;
    case msg::SET_LIMITS: return 16;
    default: return -1;
  }
}

inline int mcu_payload_len(uint8_t type) {
  switch (type) {
    case msg::HELLO: return 32;
    case msg::ACK: return 8;
    case msg::STATUS: return 56;
    case msg::FAULT_EVENT: return 20;
    case msg::CLOCK_SYNC_REPLY: return 28;
    case msg::AXIS_SELECT: return 8;
    case msg::CAL_RESULT: return 28;
    case msg::LIMITS: return 24;
    default: return -1;
  }
}

// --- CRC-16/CCITT-FALSE（poly 0x1021、init 0xFFFF、反転なし、xorout 0） ---
inline uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// --- COBS（Cheshire & Baker）。区切り 0x00 は付けない / 取らない ---
// 非ゼロ 254 byte の連続で終わるときは末尾に空ブロック 0x01 を付けない（Wikipedia の例と同じ）。
// 戻り値は出力長。容量不足なら 0。
inline size_t cobs_encode(const uint8_t* in, size_t n, uint8_t* out, size_t cap) {
  if (cap == 0) return 0;
  size_t code_idx = 0;
  size_t len = 1;
  uint8_t code = 1;
  for (size_t i = 0; i < n; ++i) {
    if (code == 0xFF) {
      out[code_idx] = code;
      code_idx = len;
      if (len >= cap) return 0;
      len += 1;
      code = 1;
    }
    const uint8_t b = in[i];
    if (b == 0) {
      out[code_idx] = code;
      code_idx = len;
      if (len >= cap) return 0;
      len += 1;
      code = 1;
    } else {
      if (len >= cap) return 0;
      out[len++] = b;
      code += 1;
    }
  }
  out[code_idx] = code;
  return len;
}

// 空入力・ブロック内の 0x00・長さ超過は false。0xFF ブロックの後の末尾 0x01 は受け付ける。
inline bool cobs_decode(const uint8_t* in, size_t n, uint8_t* out, size_t cap, size_t* out_len) {
  if (n == 0) return false;
  size_t o = 0;
  size_t i = 0;
  while (i < n) {
    const uint8_t code = in[i];
    if (code == 0) return false;
    i += 1;
    const size_t block = static_cast<size_t>(code) - 1;
    if (i + block > n) return false;
    for (size_t k = 0; k < block; ++k) {
      if (in[i + k] == 0) return false;
      if (o >= cap) return false;
      out[o++] = in[i + k];
    }
    i += block;
    if (code < 0xFF && i < n) {
      if (o >= cap) return false;
      out[o++] = 0;
    }
  }
  *out_len = o;
  return true;
}

// --- LE 読み書き ---
inline void put_u8(uint8_t* p, uint8_t v) { p[0] = v; }
inline void put_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
inline void put_u32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline void put_u64(uint8_t* p, uint64_t v) {
  for (int i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(v >> (8 * i));
}
inline void put_i16(uint8_t* p, int16_t v) { put_u16(p, static_cast<uint16_t>(v)); }
inline void put_i32(uint8_t* p, int32_t v) { put_u32(p, static_cast<uint32_t>(v)); }
inline uint8_t get_u8(const uint8_t* p) { return p[0]; }
inline uint16_t get_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
inline uint32_t get_u32(const uint8_t* p) {
  uint32_t v = 0;
  for (int i = 3; i >= 0; --i) v = (v << 8) | p[i];
  return v;
}
inline uint64_t get_u64(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 7; i >= 0; --i) v = (v << 8) | p[i];
  return v;
}
inline int16_t get_i16(const uint8_t* p) { return static_cast<int16_t>(get_u16(p)); }
inline int32_t get_i32(const uint8_t* p) { return static_cast<int32_t>(get_u32(p)); }

// --- フレーム（header + payload + crc） ---
// 戻り値はフレーム長。容量不足・payload 超過なら 0。
inline size_t frame_build(uint8_t type, const uint8_t* payload, size_t n, uint8_t* out, size_t cap) {
  const size_t total = HEADER_LEN + n + CRC_LEN;
  if (n > MAX_PAYLOAD_LEN || total > cap) return 0;
  out[0] = PROTOCOL_VERSION;
  out[1] = type;
  put_u16(out + 2, static_cast<uint16_t>(n));
  if (n > 0) memcpy(out + HEADER_LEN, payload, n);
  put_u16(out + HEADER_LEN + n, crc16(out, HEADER_LEN + n));
  return total;
}

// 検査順: 長さ → CRC → version → payload_len。種別は検査しない（decode_* で長さと一緒に見る）。
inline bool frame_parse(const uint8_t* frame, size_t n, uint8_t* type, const uint8_t** payload, size_t* plen) {
  if (n < HEADER_LEN + CRC_LEN || n > MAX_FRAME_LEN) return false;
  const size_t body = n - CRC_LEN;
  if (crc16(frame, body) != get_u16(frame + body)) return false;
  if (frame[0] != PROTOCOL_VERSION) return false;
  const size_t declared = get_u16(frame + 2);
  if (declared != body - HEADER_LEN) return false;
  *type = frame[1];
  *payload = frame + HEADER_LEN;
  *plen = declared;
  return true;
}

// --- メッセージ構造体（protocol.md §4。union に入れるので既定初期化子は持たない） ---
struct HelloReq {
  uint32_t request_id;
};
struct Config {
  uint32_t request_id;
  uint32_t session_epoch;
  uint32_t calibration_hash;
  int32_t soft_min_count;
  int32_t soft_max_count;
  uint32_t max_vel_count_s;
  uint32_t max_acc_count_s2;
  uint32_t following_error_limit_count;
  uint32_t overspeed_count_s;
  uint16_t command_timeout_ms;
  uint16_t stop_timeout_ms;
  uint8_t role_flags;
};
struct ArmPrepare {
  uint32_t request_id;
  uint32_t session_epoch;
};
struct Start {
  uint32_t request_id;
  uint32_t session_epoch;
};
struct SetPosition {
  uint32_t session_epoch;
  uint32_t seq;
  uint32_t group_frame_id;
  int32_t target_count;
  uint32_t max_vel_count_s;
  uint32_t max_acc_count_s2;
  uint16_t valid_for_ms;
  uint64_t apply_at_us;
};
struct ControlledStop {
  uint32_t request_id;
  uint32_t session_epoch;
  uint8_t reason;
};
struct Disarm {
  uint32_t request_id;
};
struct FaultReset {
  uint32_t request_id;
};
struct ClockSync {
  uint32_t request_id;
  uint64_t pc_time_us;
};
struct HostFault {
  uint32_t request_id;
  uint16_t fault_code;
};
struct SetReference {
  uint32_t request_id;
  int32_t reference_count;
};
struct Calibrate {
  uint32_t request_id;
  uint8_t mode;  // cal_mode::START / CANCEL
  uint8_t flags;  // bit0 = KEEP_POWER（電源担当軸だけが解釈。校正が終わっても主電源を保つ）
};
struct SetLimits {
  uint32_t request_id;
  uint8_t duty_max_percent;        // 1〜100。絶対上限（V1_ABS_DUTY_MAX）を超える値はクランプ
  uint8_t duty_min_percent;        // 0〜duty_max
  uint8_t kp_percent_per_100;
  uint16_t ff_count_s_per_percent; // > 0
  uint16_t deadband_count;
  uint16_t deadband_out_count;     // ≥ deadband_count
};

struct Hello {
  uint32_t request_id;
  uint8_t protocol_version;
  uint8_t selected_axis;
  uint8_t mcu_state;
  uint64_t device_id;
  uint32_t boot_id;
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t fw_patch;
  uint32_t capabilities;
  uint32_t calibration_hash;
};
struct Ack {
  uint32_t request_id;
  uint8_t acked_type;
  uint8_t result;
  uint16_t reason;
};
struct Status {
  uint32_t boot_id;
  uint64_t mcu_time_us;
  uint8_t mcu_state;
  uint8_t selected_axis;
  uint16_t fault_code;
  uint16_t flags;
  int16_t pwm_permille;
  int32_t actual_count;
  int32_t target_count;
  int32_t velocity_count_s;
  int32_t following_error_count;
  uint32_t last_applied_frame;
  uint32_t last_applied_seq;
  uint32_t last_rx_seq;
  int32_t current_ma;
  uint16_t rx_error_count;
  uint16_t stale_drop_count;
};
struct FaultEvent {
  uint32_t boot_id;
  uint64_t mcu_time_us;
  uint16_t fault_code;
  uint8_t mcu_state_before;
  int32_t actual_count;
};
struct ClockSyncReply {
  uint32_t request_id;
  uint64_t pc_time_us;
  uint64_t mcu_rx_us;
  uint64_t mcu_tx_us;
};
struct AxisSelect {
  uint32_t boot_id;
  uint8_t selected_axis;
  uint8_t prev_selected_axis;
  uint8_t mcu_state;
};
struct CalResult {
  uint32_t boot_id;
  uint8_t result;              // cal_result::*
  uint8_t step;                // cal_step::*（終了時の段階）
  int32_t n_end_count;         // N 端の count（P 端 = 0 の座標。符号は掛けない）
  int32_t neutral_count;       // 中立の count（同上）
  int8_t sign;                 // 系統プロファイルの符号 ±1
  char reason[CAL_REASON_LEN]; // ASCII、NUL 埋め
};
struct Limits {
  uint32_t boot_id;
  uint8_t duty_max_percent;        // 有効値（クランプ後）
  uint8_t duty_min_percent;
  uint8_t kp_percent_per_100;
  uint8_t abs_duty_max_percent;    // 絶対上限（config.h の V1_ABS_DUTY_MAX）
  uint16_t ff_count_s_per_percent;
  uint16_t deadband_count;
  uint16_t deadband_out_count;
  uint32_t abs_max_vel_count_s;    // 絶対上限（V1_ABS_MAX_VEL_COUNT_S）
  uint32_t abs_max_acc_count_s2;   // 絶対上限（V1_ABS_MAX_ACC_COUNT_S2）
};

// PC → MCU の 1 メッセージ。type で union の有効な要素が決まる。
struct HostMsg {
  uint8_t type;
  union {
    HelloReq hello_req;
    Config config;
    ArmPrepare arm_prepare;
    Start start;
    SetPosition set_position;
    ControlledStop controlled_stop;
    Disarm disarm;
    FaultReset fault_reset;
    ClockSync clock_sync;
    HostFault host_fault;
    SetReference set_reference;
    Calibrate calibrate;
    SetLimits set_limits;
  };
};

// MCU → PC の 1 メッセージ。
struct McuMsg {
  uint8_t type;
  union {
    Hello hello;
    Ack ack;
    Status status;
    FaultEvent fault_event;
    ClockSyncReply clock_sync_reply;
    AxisSelect axis_select;
    CalResult cal_result;
    Limits limits;
  };
};

// --- payload のエンコード（戻り値は長さ。cap 不足・未知種別は 0） ---
inline size_t encode_host_payload(const HostMsg& m, uint8_t* out, size_t cap) {
  const int need = host_payload_len(m.type);
  if (need < 0 || static_cast<size_t>(need) > cap) return 0;
  memset(out, 0, static_cast<size_t>(need));
  switch (m.type) {
    case msg::HELLO_REQ:
      put_u32(out, m.hello_req.request_id);
      break;
    case msg::CONFIG: {
      const Config& c = m.config;
      put_u32(out + 0, c.request_id);
      put_u32(out + 4, c.session_epoch);
      put_u32(out + 8, c.calibration_hash);
      put_i32(out + 12, c.soft_min_count);
      put_i32(out + 16, c.soft_max_count);
      put_u32(out + 20, c.max_vel_count_s);
      put_u32(out + 24, c.max_acc_count_s2);
      put_u32(out + 28, c.following_error_limit_count);
      put_u32(out + 32, c.overspeed_count_s);
      put_u16(out + 36, c.command_timeout_ms);
      put_u16(out + 38, c.stop_timeout_ms);
      put_u8(out + 40, c.role_flags);
      break;
    }
    case msg::ARM_PREPARE:
      put_u32(out + 0, m.arm_prepare.request_id);
      put_u32(out + 4, m.arm_prepare.session_epoch);
      break;
    case msg::START:
      put_u32(out + 0, m.start.request_id);
      put_u32(out + 4, m.start.session_epoch);
      break;
    case msg::SET_POSITION: {
      const SetPosition& s = m.set_position;
      put_u32(out + 0, s.session_epoch);
      put_u32(out + 4, s.seq);
      put_u32(out + 8, s.group_frame_id);
      put_i32(out + 12, s.target_count);
      put_u32(out + 16, s.max_vel_count_s);
      put_u32(out + 20, s.max_acc_count_s2);
      put_u16(out + 24, s.valid_for_ms);
      put_u64(out + 28, s.apply_at_us);
      break;
    }
    case msg::CONTROLLED_STOP:
      put_u32(out + 0, m.controlled_stop.request_id);
      put_u32(out + 4, m.controlled_stop.session_epoch);
      put_u8(out + 8, m.controlled_stop.reason);
      break;
    case msg::DISARM:
      put_u32(out, m.disarm.request_id);
      break;
    case msg::FAULT_RESET:
      put_u32(out, m.fault_reset.request_id);
      break;
    case msg::CLOCK_SYNC:
      put_u32(out + 0, m.clock_sync.request_id);
      put_u64(out + 4, m.clock_sync.pc_time_us);
      break;
    case msg::HOST_FAULT:
      put_u32(out + 0, m.host_fault.request_id);
      put_u16(out + 4, m.host_fault.fault_code);
      break;
    case msg::SET_REFERENCE:
      put_u32(out + 0, m.set_reference.request_id);
      put_i32(out + 4, m.set_reference.reference_count);
      break;
    case msg::CALIBRATE:
      put_u32(out + 0, m.calibrate.request_id);
      put_u8(out + 4, m.calibrate.mode);
      put_u8(out + 5, m.calibrate.flags);
      break;
    case msg::SET_LIMITS: {
      const SetLimits& l = m.set_limits;
      put_u32(out + 0, l.request_id);
      put_u8(out + 4, l.duty_max_percent);
      put_u8(out + 5, l.duty_min_percent);
      put_u8(out + 6, l.kp_percent_per_100);
      // 7: reserved = 0
      put_u16(out + 8, l.ff_count_s_per_percent);
      put_u16(out + 10, l.deadband_count);
      put_u16(out + 12, l.deadband_out_count);
      // 14: u16 reserved = 0
      break;
    }
    default:
      return 0;
  }
  return static_cast<size_t>(need);
}

inline size_t encode_mcu_payload(const McuMsg& m, uint8_t* out, size_t cap) {
  const int need = mcu_payload_len(m.type);
  if (need < 0 || static_cast<size_t>(need) > cap) return 0;
  memset(out, 0, static_cast<size_t>(need));
  switch (m.type) {
    case msg::HELLO: {
      const Hello& h = m.hello;
      put_u32(out + 0, h.request_id);
      put_u8(out + 4, h.protocol_version);
      put_u8(out + 5, h.selected_axis);
      put_u8(out + 6, h.mcu_state);
      put_u64(out + 8, h.device_id);
      put_u32(out + 16, h.boot_id);
      put_u8(out + 20, h.fw_major);
      put_u8(out + 21, h.fw_minor);
      put_u8(out + 22, h.fw_patch);
      put_u32(out + 24, h.capabilities);
      put_u32(out + 28, h.calibration_hash);
      break;
    }
    case msg::ACK:
      put_u32(out + 0, m.ack.request_id);
      put_u8(out + 4, m.ack.acked_type);
      put_u8(out + 5, m.ack.result);
      put_u16(out + 6, m.ack.reason);
      break;
    case msg::STATUS: {
      const Status& s = m.status;
      put_u32(out + 0, s.boot_id);
      put_u64(out + 4, s.mcu_time_us);
      put_u8(out + 12, s.mcu_state);
      put_u8(out + 13, s.selected_axis);
      put_u16(out + 14, s.fault_code);
      put_u16(out + 16, s.flags);
      put_i16(out + 18, s.pwm_permille);
      put_i32(out + 20, s.actual_count);
      put_i32(out + 24, s.target_count);
      put_i32(out + 28, s.velocity_count_s);
      put_i32(out + 32, s.following_error_count);
      put_u32(out + 36, s.last_applied_frame);
      put_u32(out + 40, s.last_applied_seq);
      put_u32(out + 44, s.last_rx_seq);
      put_i32(out + 48, s.current_ma);
      put_u16(out + 52, s.rx_error_count);
      put_u16(out + 54, s.stale_drop_count);
      break;
    }
    case msg::FAULT_EVENT: {
      const FaultEvent& f = m.fault_event;
      put_u32(out + 0, f.boot_id);
      put_u64(out + 4, f.mcu_time_us);
      put_u16(out + 12, f.fault_code);
      put_u8(out + 14, f.mcu_state_before);
      put_i32(out + 16, f.actual_count);
      break;
    }
    case msg::CLOCK_SYNC_REPLY: {
      const ClockSyncReply& r = m.clock_sync_reply;
      put_u32(out + 0, r.request_id);
      put_u64(out + 4, r.pc_time_us);
      put_u64(out + 12, r.mcu_rx_us);
      put_u64(out + 20, r.mcu_tx_us);
      break;
    }
    case msg::AXIS_SELECT:
      put_u32(out + 0, m.axis_select.boot_id);
      put_u8(out + 4, m.axis_select.selected_axis);
      put_u8(out + 5, m.axis_select.prev_selected_axis);
      put_u8(out + 6, m.axis_select.mcu_state);
      break;
    case msg::CAL_RESULT: {
      const CalResult& c = m.cal_result;
      put_u32(out + 0, c.boot_id);
      put_u8(out + 4, c.result);
      put_u8(out + 5, c.step);
      put_i32(out + 8, c.n_end_count);
      put_i32(out + 12, c.neutral_count);
      put_u8(out + 16, static_cast<uint8_t>(c.sign));
      memcpy(out + 20, c.reason, CAL_REASON_LEN);
      break;
    }
    case msg::LIMITS: {
      const Limits& l = m.limits;
      put_u32(out + 0, l.boot_id);
      put_u8(out + 4, l.duty_max_percent);
      put_u8(out + 5, l.duty_min_percent);
      put_u8(out + 6, l.kp_percent_per_100);
      put_u8(out + 7, l.abs_duty_max_percent);
      put_u16(out + 8, l.ff_count_s_per_percent);
      put_u16(out + 10, l.deadband_count);
      put_u16(out + 12, l.deadband_out_count);
      // 14: u16 reserved = 0
      put_u32(out + 16, l.abs_max_vel_count_s);
      put_u32(out + 20, l.abs_max_acc_count_s2);
      break;
    }
    default:
      return 0;
  }
  return static_cast<size_t>(need);
}

// --- payload のデコード（長さが種別の定義長と違えば false） ---
inline bool decode_host_payload(uint8_t type, const uint8_t* p, size_t n, HostMsg* out) {
  const int need = host_payload_len(type);
  if (need < 0 || static_cast<size_t>(need) != n) return false;
  memset(out, 0, sizeof(*out));
  out->type = type;
  switch (type) {
    case msg::HELLO_REQ:
      out->hello_req.request_id = get_u32(p);
      break;
    case msg::CONFIG: {
      Config& c = out->config;
      c.request_id = get_u32(p + 0);
      c.session_epoch = get_u32(p + 4);
      c.calibration_hash = get_u32(p + 8);
      c.soft_min_count = get_i32(p + 12);
      c.soft_max_count = get_i32(p + 16);
      c.max_vel_count_s = get_u32(p + 20);
      c.max_acc_count_s2 = get_u32(p + 24);
      c.following_error_limit_count = get_u32(p + 28);
      c.overspeed_count_s = get_u32(p + 32);
      c.command_timeout_ms = get_u16(p + 36);
      c.stop_timeout_ms = get_u16(p + 38);
      c.role_flags = get_u8(p + 40);
      break;
    }
    case msg::ARM_PREPARE:
      out->arm_prepare.request_id = get_u32(p + 0);
      out->arm_prepare.session_epoch = get_u32(p + 4);
      break;
    case msg::START:
      out->start.request_id = get_u32(p + 0);
      out->start.session_epoch = get_u32(p + 4);
      break;
    case msg::SET_POSITION: {
      SetPosition& s = out->set_position;
      s.session_epoch = get_u32(p + 0);
      s.seq = get_u32(p + 4);
      s.group_frame_id = get_u32(p + 8);
      s.target_count = get_i32(p + 12);
      s.max_vel_count_s = get_u32(p + 16);
      s.max_acc_count_s2 = get_u32(p + 20);
      s.valid_for_ms = get_u16(p + 24);
      s.apply_at_us = get_u64(p + 28);
      break;
    }
    case msg::CONTROLLED_STOP:
      out->controlled_stop.request_id = get_u32(p + 0);
      out->controlled_stop.session_epoch = get_u32(p + 4);
      out->controlled_stop.reason = get_u8(p + 8);
      break;
    case msg::DISARM:
      out->disarm.request_id = get_u32(p);
      break;
    case msg::FAULT_RESET:
      out->fault_reset.request_id = get_u32(p);
      break;
    case msg::CLOCK_SYNC:
      out->clock_sync.request_id = get_u32(p + 0);
      out->clock_sync.pc_time_us = get_u64(p + 4);
      break;
    case msg::HOST_FAULT:
      out->host_fault.request_id = get_u32(p + 0);
      out->host_fault.fault_code = get_u16(p + 4);
      break;
    case msg::SET_REFERENCE:
      out->set_reference.request_id = get_u32(p + 0);
      out->set_reference.reference_count = get_i32(p + 4);
      break;
    case msg::CALIBRATE:
      out->calibrate.request_id = get_u32(p + 0);
      out->calibrate.mode = get_u8(p + 4);
      out->calibrate.flags = get_u8(p + 5);
      break;
    case msg::SET_LIMITS: {
      SetLimits& l = out->set_limits;
      l.request_id = get_u32(p + 0);
      l.duty_max_percent = get_u8(p + 4);
      l.duty_min_percent = get_u8(p + 5);
      l.kp_percent_per_100 = get_u8(p + 6);
      l.ff_count_s_per_percent = get_u16(p + 8);
      l.deadband_count = get_u16(p + 10);
      l.deadband_out_count = get_u16(p + 12);
      break;  // 7・14 の予約は読まない
    }
    default:
      return false;
  }
  return true;
}

inline bool decode_mcu_payload(uint8_t type, const uint8_t* p, size_t n, McuMsg* out) {
  const int need = mcu_payload_len(type);
  if (need < 0 || static_cast<size_t>(need) != n) return false;
  memset(out, 0, sizeof(*out));
  out->type = type;
  switch (type) {
    case msg::HELLO: {
      Hello& h = out->hello;
      h.request_id = get_u32(p + 0);
      h.protocol_version = get_u8(p + 4);
      h.selected_axis = get_u8(p + 5);
      h.mcu_state = get_u8(p + 6);
      h.device_id = get_u64(p + 8);
      h.boot_id = get_u32(p + 16);
      h.fw_major = get_u8(p + 20);
      h.fw_minor = get_u8(p + 21);
      h.fw_patch = get_u8(p + 22);
      h.capabilities = get_u32(p + 24);
      h.calibration_hash = get_u32(p + 28);
      break;
    }
    case msg::ACK:
      out->ack.request_id = get_u32(p + 0);
      out->ack.acked_type = get_u8(p + 4);
      out->ack.result = get_u8(p + 5);
      out->ack.reason = get_u16(p + 6);
      break;
    case msg::STATUS: {
      Status& s = out->status;
      s.boot_id = get_u32(p + 0);
      s.mcu_time_us = get_u64(p + 4);
      s.mcu_state = get_u8(p + 12);
      s.selected_axis = get_u8(p + 13);
      s.fault_code = get_u16(p + 14);
      s.flags = get_u16(p + 16);
      s.pwm_permille = get_i16(p + 18);
      s.actual_count = get_i32(p + 20);
      s.target_count = get_i32(p + 24);
      s.velocity_count_s = get_i32(p + 28);
      s.following_error_count = get_i32(p + 32);
      s.last_applied_frame = get_u32(p + 36);
      s.last_applied_seq = get_u32(p + 40);
      s.last_rx_seq = get_u32(p + 44);
      s.current_ma = get_i32(p + 48);
      s.rx_error_count = get_u16(p + 52);
      s.stale_drop_count = get_u16(p + 54);
      break;
    }
    case msg::FAULT_EVENT: {
      FaultEvent& f = out->fault_event;
      f.boot_id = get_u32(p + 0);
      f.mcu_time_us = get_u64(p + 4);
      f.fault_code = get_u16(p + 12);
      f.mcu_state_before = get_u8(p + 14);
      f.actual_count = get_i32(p + 16);
      break;
    }
    case msg::CLOCK_SYNC_REPLY: {
      ClockSyncReply& r = out->clock_sync_reply;
      r.request_id = get_u32(p + 0);
      r.pc_time_us = get_u64(p + 4);
      r.mcu_rx_us = get_u64(p + 12);
      r.mcu_tx_us = get_u64(p + 20);
      break;
    }
    case msg::AXIS_SELECT:
      out->axis_select.boot_id = get_u32(p + 0);
      out->axis_select.selected_axis = get_u8(p + 4);
      out->axis_select.prev_selected_axis = get_u8(p + 5);
      out->axis_select.mcu_state = get_u8(p + 6);
      break;
    case msg::CAL_RESULT: {
      CalResult& c = out->cal_result;
      c.boot_id = get_u32(p + 0);
      c.result = get_u8(p + 4);
      c.step = get_u8(p + 5);
      c.n_end_count = get_i32(p + 8);
      c.neutral_count = get_i32(p + 12);
      c.sign = static_cast<int8_t>(get_u8(p + 16));
      memcpy(c.reason, p + 20, CAL_REASON_LEN);
      break;
    }
    case msg::LIMITS: {
      Limits& l = out->limits;
      l.boot_id = get_u32(p + 0);
      l.duty_max_percent = get_u8(p + 4);
      l.duty_min_percent = get_u8(p + 5);
      l.kp_percent_per_100 = get_u8(p + 6);
      l.abs_duty_max_percent = get_u8(p + 7);
      l.ff_count_s_per_percent = get_u16(p + 8);
      l.deadband_count = get_u16(p + 10);
      l.deadband_out_count = get_u16(p + 12);
      l.abs_max_vel_count_s = get_u32(p + 16);
      l.abs_max_acc_count_s2 = get_u32(p + 20);
      break;  // 14 の予約は読まない
    }
    default:
      return false;
  }
  return true;
}

// --- フレーム単位・ワイヤ単位の変換 ---
inline size_t encode_host_frame(const HostMsg& m, uint8_t* out, size_t cap) {
  uint8_t payload[MAX_PAYLOAD_LEN];
  const size_t n = encode_host_payload(m, payload, sizeof(payload));
  if (n == 0) return 0;
  return frame_build(m.type, payload, n, out, cap);
}

inline size_t encode_mcu_frame(const McuMsg& m, uint8_t* out, size_t cap) {
  uint8_t payload[MAX_PAYLOAD_LEN];
  const size_t n = encode_mcu_payload(m, payload, sizeof(payload));
  if (n == 0) return 0;
  return frame_build(m.type, payload, n, out, cap);
}

// COBS 符号化して区切り 0x00 を付けた USB 上のバイト列。戻り値は長さ、容量不足なら 0。
inline size_t frame_to_wire(const uint8_t* frame, size_t n, uint8_t* out, size_t cap) {
  if (cap == 0) return 0;
  const size_t enc = cobs_encode(frame, n, out, cap - 1);
  if (enc == 0) return 0;
  out[enc] = 0;
  return enc + 1;
}

inline size_t encode_host_wire(const HostMsg& m, uint8_t* out, size_t cap) {
  uint8_t frame[MAX_FRAME_LEN];
  const size_t n = encode_host_frame(m, frame, sizeof(frame));
  if (n == 0) return 0;
  return frame_to_wire(frame, n, out, cap);
}

inline size_t encode_mcu_wire(const McuMsg& m, uint8_t* out, size_t cap) {
  uint8_t frame[MAX_FRAME_LEN];
  const size_t n = encode_mcu_frame(m, frame, sizeof(frame));
  if (n == 0) return 0;
  return frame_to_wire(frame, n, out, cap);
}

// 復号後フレーム → メッセージ。CRC / 版 / 種別 / 長さのどれかが不正なら false。
inline bool decode_host_frame(const uint8_t* frame, size_t n, HostMsg* out) {
  uint8_t type = 0;
  const uint8_t* payload = nullptr;
  size_t plen = 0;
  if (!frame_parse(frame, n, &type, &payload, &plen)) return false;
  return decode_host_payload(type, payload, plen, out);
}

inline bool decode_mcu_frame(const uint8_t* frame, size_t n, McuMsg* out) {
  uint8_t type = 0;
  const uint8_t* payload = nullptr;
  size_t plen = 0;
  if (!frame_parse(frame, n, &type, &payload, &plen)) return false;
  return decode_mcu_payload(type, payload, plen, out);
}

// COBS 符号化済み（区切りを除く）→ メッセージ。復号後が 256 byte を超えるフレームも false。
inline bool decode_host_encoded(const uint8_t* encoded, size_t n, HostMsg* out) {
  uint8_t frame[MAX_FRAME_LEN];
  size_t flen = 0;
  if (!cobs_decode(encoded, n, frame, sizeof(frame), &flen)) return false;
  return decode_host_frame(frame, flen, out);
}

inline bool decode_mcu_encoded(const uint8_t* encoded, size_t n, McuMsg* out) {
  uint8_t frame[MAX_FRAME_LEN];
  size_t flen = 0;
  if (!cobs_decode(encoded, n, frame, sizeof(frame), &flen)) return false;
  return decode_mcu_frame(frame, flen, out);
}

// --- 受信バイト列を 0x00 で区切って 1 フレームずつ取り出す ---
// push() が true を返したら buf[0..len) が 1 フレーム（COBS 符号化のまま、区切りなし）。
// 取り出したら次の push() で自動的に空になる。MAX_ENCODED_LEN を超えたフレームは捨てて次の 0x00 まで
// 読み飛ばし（再同期）、dropped を数える。0x00 の連続（空フレーム）は数えずに無視する。
struct FrameSplitter {
  uint8_t buf[MAX_ENCODED_LEN];
  size_t len = 0;
  bool skipping = false;
  uint32_t dropped = 0;

  bool push(uint8_t b) {
    if (ready_) {
      len = 0;
      ready_ = false;
    }
    if (b == 0) {
      if (skipping) {
        skipping = false;
        return false;
      }
      if (len == 0) return false;
      ready_ = true;
      return true;
    }
    if (skipping) return false;
    if (len >= MAX_ENCODED_LEN) {
      skipping = true;
      dropped += 1;
      len = 0;
      return false;
    }
    buf[len++] = b;
    return false;
  }

  void clear() {
    len = 0;
    skipping = false;
    ready_ = false;
  }

 private:
  bool ready_ = false;
};

}  // namespace v1
