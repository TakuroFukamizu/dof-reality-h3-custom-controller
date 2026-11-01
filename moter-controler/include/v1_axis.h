#pragma once

// USB プロトコル v1 の 1 軸状態機械 V1Axis。
// GPIO・時計・NVS・Serial を持たない（auto_move.h と同じ流儀）。制御タスクが
//   handle_frame(受信フレーム, now_us) / tick(now_us, 生の PCNT カウント)
// を呼び、出力（power_on_cmd / brake_release_cmd / duty_percent）と送信メッセージ列（outbox）を読む。
// 振る舞いは docs/control-server/protocol.md §3〜§6 と、PC 側の参照モデル control-server/src/sim_axis.rs に合わせる。
// 設計: docs/superpowers/specs/2026-10-31-moter-controler-v1-protocol-design.md §3。
//
// 座標: actual_count = raw − reference_offset。系統の符号は掛けない（符号は PC 側の校正）。
// Duty の向き: NVS プロファイルの sign（+1 なら速度 − でカウント増）から決める。
//   カウントを増やしたいなら duty_signed = −sign × duty。
// 時刻はマイクロ秒で注入する（MCU では esp_timer_get_time()）。

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "v1_codec.h"

// 調整値。既定は config.h（実機で詰めるまでの仮値）。テストで差し替えられるよう構造体にしている。
struct V1Tuning {
  int kp_percent_per_100 = V1_KP_PERCENT_PER_100;
  int ff_count_s_per_percent = V1_FF_COUNT_S_PER_PERCENT;
  int32_t deadband = V1_DEADBAND;
  int32_t deadband_out = V1_DEADBAND_OUT;
  int duty_min = V1_DUTY_MIN;
  int duty_max = V1_DUTY_MAX;
  int stall_duty = V1_STALL_DUTY;
  uint32_t stall_ms = STALL_MS;
  uint32_t overrun_ms = V1_OVERRUN_MS;
  uint32_t power_settle_ms = POWER_SETTLE_MS;
  uint32_t brake_release_wait_ms = BRAKE_RELEASE_WAIT_MS;
  uint32_t brake_apply_wait_ms = BRAKE_APPLY_WAIT_MS;
  uint32_t status_period_ms = 20;
  uint32_t velocity_window_ms = 100;
};

// 位置制御の指令値（カウントが増える向きを正とした %）。err = 目標 − 実位置、vel = 軌道の目標速度 [count/s]。
// フィードフォワード（vel / ff）＋比例（err × kp / 100）。目標が止まっている（vel = 0）ときだけ不感帯を使い、
// 入る側 deadband / 出る側 deadband_out のヒステリシスで到達帯の縁での往復を防ぐ。
// 0 でなければ |指令| を duty_min 以上 duty_max 以下に収める。
inline int v1_duty_command(int64_t err, double vel, bool& in_band, const V1Tuning& t) {
  const int64_t err_abs = err < 0 ? -err : err;
  if (vel == 0.0) {
    if (in_band) {
      if (err_abs <= t.deadband_out) return 0;
      in_band = false;
    } else if (err_abs <= t.deadband) {
      in_band = true;
      return 0;
    }
  } else {
    in_band = false;
  }
  double cmd = static_cast<double>(err) * t.kp_percent_per_100 / 100.0;
  if (t.ff_count_s_per_percent > 0) cmd += vel / t.ff_count_s_per_percent;
  if (cmd == 0.0) return 0;
  double mag = cmd < 0 ? -cmd : cmd;
  if (mag < t.duty_min) mag = t.duty_min;
  if (mag > t.duty_max) mag = t.duty_max;
  const int m = static_cast<int>(mag);
  return cmd < 0 ? -m : m;
}

struct V1Identity {
  uint64_t device_id = 0;  // eFuse MAC から作る（上位 16 bit は 0x5131）。v1_device_id_from_mac
  uint32_t boot_id = 0;    // 起動ごとの乱数
  uint8_t fw_major = 0;
  uint8_t fw_minor = 1;
  uint8_t fw_patch = 0;
};

// device_id = 0x5131 << 48 | MAC を表記順（mac[0] が先頭オクテット）に並べた 48 bit。
// 例: 30:ED:A0:0B:20:40 → 0x513130EDA00B2040。下 16 bit（画面の `dev XXXX`）は末尾 2 オクテット。
// ESP.getEfuseMac() は先頭オクテットを最下位に置く（逆順）ので使わない。
inline uint64_t v1_device_id_from_mac(const uint8_t mac[6]) {
  uint64_t id = 0x5131ull << 48;
  for (int i = 0; i < 6; ++i) {
    id |= static_cast<uint64_t>(mac[i]) << (8 * (5 - i));
  }
  return id;
}

class V1Axis {
 public:
  static constexpr size_t kOutboxCap = 40;   // 1 周期の最大: 受信 16 件分の ACK + STATUS + FAULT_EVENT + AXIS_SELECT
  static constexpr size_t kDedupeDepth = 16;

  V1Tuning tuning;

  // BOOT → DISARMED。selected_axis は NVS の motor、sign はその系統のプロファイルの符号。
  void begin(const V1Identity& id, uint8_t selected_axis, int8_t sign, uint64_t now_us, int64_t raw) {
    id_ = id;
    selected_axis_ = selected_axis;
    sign_ = sign < 0 ? -1 : 1;
    boot_origin_us_ = now_us;
    last_tick_us_ = now_us;
    now_us_ = now_us;
    raw_ = raw;
    reference_offset_ = 0;
    vel_anchor_raw_ = raw;
    vel_anchor_us_ = now_us;
    state_ = v1::state::DISARMED;
    hold_here();
    began_ = true;
  }

  bool began() const { return began_; }
  void set_sign(int8_t sign) { sign_ = sign < 0 ? -1 : 1; }

  // --- 入力 ---------------------------------------------------------------

  // COBS 符号化済み（区切りなし）の 1 フレーム。復号できなければ rx_error を数えるだけで応答しない。
  void handle_frame(const uint8_t* encoded, size_t n, uint64_t now_us) {
    v1::HostMsg m;
    if (!v1::decode_host_encoded(encoded, n, &m)) {
      record_rx_error();
      return;
    }
    handle(m, now_us);
  }

  // 復号済みメッセージ。物理は進めない（tick が進める）。ACK → STATUS の順で outbox に積む。
  void handle(const v1::HostMsg& m, uint64_t now_us) {
    now_us_ = now_us;
    dispatch(m, now_us);
    emit_status_if_changed();
  }

  // 5 ms ごと。タイマー → 軌道 → 異常検出 → Duty → STATUS。
  void tick(uint64_t now_us, int64_t raw) {
    if (!began_) return;
    const uint64_t dt_us = now_us > last_tick_us_ ? now_us - last_tick_us_ : 0;
    last_tick_us_ = now_us;
    now_us_ = now_us;
    raw_ = raw;
    update_velocity(now_us);

    // 制御周期の逸脱。止まっている状態（DISARMED / FAULT）では見ない（起動直後や待機中の間隔は制御に影響しない）。
    if (!parked() && dt_us > static_cast<uint64_t>(tuning.overrun_ms) * 1000ull) {
      latch_fault(v1::fault::CONTROL_OVERRUN, now_us);
    }

    check_timers(now_us);
    if (position_control_on()) {
      update_setpoint(static_cast<double>(dt_us) / 1e6);
      check_motion_faults(now_us);
    }
    if (state_ == v1::state::STOPPING && setpoint_vel_ == 0.0) {
      finish_stop(now_us);
    }
    update_duty(now_us);
    emit_status_if_changed();
    emit_periodic_status(now_us);
  }

  // 系統選択（protocol.md §1.1）。DISARMED のときだけ。CONFIGURED / POSITION_REFERENCED / calibration_hash を消し、
  // AXIS_SELECT を 1 回。受け付けたら true。
  bool select_axis(uint8_t axis, uint64_t now_us) {
    now_us_ = now_us;
    if (state_ != v1::state::DISARMED) return false;
    if (axis == selected_axis_) return true;
    const uint8_t prev = selected_axis_;
    selected_axis_ = axis;
    has_config_ = false;
    referenced_ = false;
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::AXIS_SELECT;
    m.axis_select.boot_id = id_.boot_id;
    m.axis_select.selected_axis = axis;
    m.axis_select.prev_selected_axis = prev;
    m.axis_select.mcu_state = state_;
    push(m);
    emit_status_if_changed();
    return true;
  }

  // 画面の非常停止。ラッチで即 FAULT(LOCAL_ESTOP)（減速中の異常コードがあればそれを保つ）。
  // ラッチ中は FAULT_RESET が FAULT_CAUSE_PRESENT。解除しても FAULT は FAULT_RESET まで残る。
  void set_local_estop(bool latched, uint64_t now_us) {
    if (latched == estop_latched_) return;
    now_us_ = now_us;
    estop_latched_ = latched;
    if (latched) {
      const uint16_t code = (state_ == v1::state::STOPPING && stop_kind_ == StopKind::Fault)
                                ? stop_fault_code_
                                : v1::fault::LOCAL_ESTOP;
      latch_fault(code, now_us);
      emit_status_if_changed();
    }
  }

  // メッセージ層より下で捨てたフレーム（COBS・300 byte 超・受信キュー満杯）を数える。
  void record_rx_error() {
    if (rx_error_count_ < 0xFFFF) rx_error_count_ += 1;
  }

  // --- 出力（送信メッセージ） -------------------------------------------------

  size_t pending() const { return out_count_; }
  const v1::McuMsg* peek() const { return out_count_ ? &outbox_[out_head_] : nullptr; }
  void pop() {
    if (out_count_ == 0) return;
    out_head_ = (out_head_ + 1) % kOutboxCap;
    out_count_ -= 1;
  }
  bool pop(v1::McuMsg* m) {
    if (out_count_ == 0) return false;
    *m = outbox_[out_head_];
    pop();
    return true;
  }

  // --- 出力（アクチュエータ指令） ---------------------------------------------

  bool power_on_cmd() const { return power_controller() && powered_state(); }
  bool brake_release_cmd() const { return brake_release_cmd_; }
  int duty_percent() const { return duty_; }               // 符号付き %（MD13S へそのまま）
  int16_t pwm_permille() const {                            // 正が count の増える向き
    return static_cast<int16_t>(-static_cast<int>(sign_) * duty_ * 10);
  }

  // --- 参照 -----------------------------------------------------------------

  uint8_t state() const { return state_; }
  // STATUS に出す異常コード（FAULT、または COMMAND_TIMEOUT による減速中）
  uint16_t fault_code() const {
    if (state_ == v1::state::FAULT) return fault_code_;
    if (state_ == v1::state::STOPPING && stop_kind_ == StopKind::Fault) return stop_fault_code_;
    return v1::fault::NONE;
  }
  int32_t actual_count() const { return clamp_count(raw_ - reference_offset_); }
  int32_t target_count() const { return to_count(setpoint_); }
  int32_t control_target_count() const { return to_count(control_target_); }
  int32_t velocity_count_s() const { return velocity_; }
  int32_t following_error_count() const {
    return clamp_count(static_cast<int64_t>(actual_count()) - static_cast<int64_t>(target_count()));
  }
  bool configured() const { return has_config_; }
  bool referenced() const { return referenced_; }
  const v1::Config* config() const { return has_config_ ? &config_ : nullptr; }
  uint8_t selected_axis() const { return selected_axis_; }
  int8_t sign() const { return sign_; }
  bool estop_latched() const { return estop_latched_; }
  bool has_rx() const { return has_rx_; }
  uint64_t last_rx_us() const { return last_rx_us_; }
  uint16_t rx_error_count() const { return rx_error_count_; }
  uint16_t stale_drop_count() const { return stale_drop_count_; }
  uint32_t last_rx_seq() const { return last_rx_seq_; }
  uint32_t last_applied_seq() const { return last_applied_seq_; }
  uint32_t last_applied_frame() const { return last_applied_frame_; }
  uint32_t calibration_hash() const { return has_config_ ? config_.calibration_hash : 0; }
  uint32_t boot_id() const { return id_.boot_id; }
  uint64_t device_id() const { return id_.device_id; }
  uint16_t last_host_fault_code() const { return last_host_fault_code_; }
  uint64_t mcu_time_us(uint64_t now_us) const { return now_us > boot_origin_us_ ? now_us - boot_origin_us_ : 0; }

  uint16_t flags() const {
    uint16_t f = 0;
    if (power_controller()) {
      f |= v1::flags::POWER_CONTROLLER;
      if (powered_state()) f |= v1::flags::POWER_ON_CMD;
    }
    if (brake_release_cmd_) f |= v1::flags::BRAKE_RELEASE_CMD;
    if (referenced_) f |= v1::flags::POSITION_REFERENCED;
    if (has_config_) f |= v1::flags::CONFIGURED;
    return f;
  }

  // いま送るとしたら、の STATUS。
  v1::Status status(uint64_t now_us) const {
    v1::Status s;
    memset(&s, 0, sizeof(s));
    s.boot_id = id_.boot_id;
    s.mcu_time_us = mcu_time_us(now_us);
    s.mcu_state = state_;
    s.selected_axis = selected_axis_;
    s.fault_code = fault_code();
    s.flags = flags();
    s.pwm_permille = pwm_permille();
    s.actual_count = actual_count();
    s.target_count = target_count();
    s.velocity_count_s = velocity_;
    s.following_error_count = following_error_count();
    s.last_applied_frame = last_applied_frame_;
    s.last_applied_seq = last_applied_seq_;
    s.last_rx_seq = last_rx_seq_;
    s.current_ma = 0;  // 電流計測なし（CURRENT_VALID は立てない）
    s.rx_error_count = rx_error_count_;
    s.stale_drop_count = stale_drop_count_;
    return s;
  }

 private:
  enum class StopKind : uint8_t { Hold, Disarm, Fault };

  struct Accepted {
    uint8_t type;
    uint32_t request_id;
    uint8_t len;
    uint8_t payload[48];  // CONFIG の 44 byte が最大
  };

  // --- 識別・選択 ---
  V1Identity id_{};
  uint8_t selected_axis_ = 0;
  int8_t sign_ = 1;
  bool began_ = false;
  uint64_t boot_origin_us_ = 0;
  uint64_t last_tick_us_ = 0;
  uint64_t now_us_ = 0;  // 直近に handle / tick へ渡された時刻（即時 STATUS の mcu_time_us 用）

  // --- 状態 ---
  uint8_t state_ = v1::state::BOOT;
  uint16_t fault_code_ = v1::fault::NONE;
  StopKind stop_kind_ = StopKind::Hold;
  uint16_t stop_fault_code_ = v1::fault::NONE;
  uint16_t last_host_fault_code_ = v1::fault::NONE;
  bool estop_latched_ = false;

  v1::Config config_{};
  bool has_config_ = false;
  bool referenced_ = false;

  // --- タイマー（絶対 us） ---
  uint64_t arming_started_us_ = 0;
  uint64_t brake_release_at_us_ = 0;
  uint64_t disarming_started_us_ = 0;
  uint64_t stopping_started_us_ = 0;
  uint64_t lease_us_ = 0;
  uint64_t valid_until_us_ = 0;
  bool brake_release_cmd_ = false;

  // --- SET_POSITION ---
  uint32_t last_rx_seq_ = 0;
  uint32_t last_applied_seq_ = 0;
  uint32_t last_applied_frame_ = 0;
  uint16_t rx_error_count_ = 0;
  uint16_t stale_drop_count_ = 0;
  bool has_rx_ = false;
  uint64_t last_rx_us_ = 0;

  // --- 位置・軌道 ---
  int64_t raw_ = 0;
  int64_t reference_offset_ = 0;
  double setpoint_ = 0.0;
  double setpoint_vel_ = 0.0;
  double control_target_ = 0.0;
  double frame_max_vel_ = 0.0;
  double frame_max_acc_ = 0.0;
  int duty_ = 0;

  // 速度（100 ms 窓）
  int64_t vel_anchor_raw_ = 0;
  uint64_t vel_anchor_us_ = 0;
  int32_t velocity_ = 0;

  // 無パルス（ENCODER_FAULT）
  bool stall_running_ = false;
  bool in_band_ = false;
  uint64_t stall_start_us_ = 0;
  uint64_t stall_last_change_us_ = 0;
  int64_t stall_last_raw_ = 0;

  // --- DUPLICATE 判定（受理した命令、古い順） ---
  Accepted accepted_[kDedupeDepth]{};
  size_t accepted_count_ = 0;

  // --- outbox（リング） ---
  v1::McuMsg outbox_[kOutboxCap]{};
  size_t out_head_ = 0;
  size_t out_count_ = 0;

  // --- STATUS ---
  bool has_next_status_ = false;
  uint64_t next_status_us_ = 0;
  bool has_last_key_ = false;
  uint8_t key_state_ = 0;
  uint16_t key_fault_ = 0;
  uint16_t key_flags_ = 0;
  uint8_t key_axis_ = 0;

  // ------------------------------------------------------------------------
  // 補助
  // ------------------------------------------------------------------------

  static int32_t clamp_count(int64_t v) {
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return static_cast<int32_t>(v);
  }
  static int32_t to_count(double v) { return clamp_count(static_cast<int64_t>(llround(v))); }
  static uint64_t ms_to_us(uint64_t ms) { return ms * 1000ull; }

  bool parked() const {
    return state_ == v1::state::DISARMED || state_ == v1::state::FAULT || state_ == v1::state::BOOT;
  }
  bool powered_state() const {
    return state_ == v1::state::ARMING || state_ == v1::state::HOLDING || state_ == v1::state::ACTIVE ||
           state_ == v1::state::STOPPING || state_ == v1::state::DISARMING;
  }
  bool power_controller() const {
    return has_config_ && (config_.role_flags & v1::role_flags::POWER_CONTROLLER) != 0;
  }
  // 位置ループが動いている（ARMING のブレーキ解除以降・HOLDING・ACTIVE・STOPPING・DISARMING）
  bool position_control_on() const {
    switch (state_) {
      case v1::state::ARMING: return brake_release_cmd_;
      case v1::state::HOLDING:
      case v1::state::ACTIVE:
      case v1::state::STOPPING:
      case v1::state::DISARMING: return true;
      default: return false;
    }
  }
  bool inside_soft_window() const {
    const int32_t a = actual_count();
    return a >= config_.soft_min_count && a <= config_.soft_max_count;
  }
  // FAULT_RESET を拒む「残る原因」。v1 では画面の非常停止ボタンのラッチだけ（設計 §3）。
  // ENCODER_FAULT の無パルスは FAULT 中（ブレーキ作動・PWM 0）には観測できないので原因として残さず、解除は通す
  // （POSITION_REFERENCED は消える）。
  bool fault_cause_present() const { return estop_latched_; }

  void hold_here() {
    const double a = static_cast<double>(actual_count());
    setpoint_ = a;
    setpoint_vel_ = 0.0;
    control_target_ = a;
  }

  // ------------------------------------------------------------------------
  // 命令の振り分け（protocol.md §4）
  // ------------------------------------------------------------------------

  void dispatch(const v1::HostMsg& m, uint64_t now) {
    has_rx_ = true;
    last_rx_us_ = now;
    switch (m.type) {
      case v1::msg::HELLO_REQ: on_hello_req(m.hello_req.request_id); break;
      case v1::msg::CLOCK_SYNC: on_clock_sync(m.clock_sync, now); break;
      case v1::msg::SET_POSITION: on_set_position(m.set_position, now); break;
      case v1::msg::HOST_FAULT: on_host_fault(m.host_fault, now); break;
      case v1::msg::CONFIG: command(m, m.config.request_id, now); break;
      case v1::msg::ARM_PREPARE: command(m, m.arm_prepare.request_id, now); break;
      case v1::msg::START: command(m, m.start.request_id, now); break;
      case v1::msg::CONTROLLED_STOP: command(m, m.controlled_stop.request_id, now); break;
      case v1::msg::DISARM: command(m, m.disarm.request_id, now); break;
      case v1::msg::FAULT_RESET: command(m, m.fault_reset.request_id, now); break;
      case v1::msg::SET_REFERENCE: command(m, m.set_reference.request_id, now); break;
      default: break;  // decode 済みなので来ない
    }
  }

  // 命令ごとの条件。受理なら ack_reason::NONE、拒否なら理由。
  uint16_t run_command(const v1::HostMsg& m, uint64_t now) {
    switch (m.type) {
      case v1::msg::CONFIG: return on_config(m.config);
      case v1::msg::ARM_PREPARE: return on_arm_prepare(m.arm_prepare.session_epoch, now);
      case v1::msg::START: return on_start(m.start.session_epoch, now);
      case v1::msg::CONTROLLED_STOP: return on_controlled_stop(m.controlled_stop.session_epoch, now);
      case v1::msg::DISARM: return on_disarm(now);
      case v1::msg::FAULT_RESET: return on_fault_reset();
      case v1::msg::SET_REFERENCE: return on_set_reference(m.set_reference.reference_count);
      default: return v1::ack_reason::BAD_STATE;
    }
  }

  // ACK を返す命令の共通処理（HOST_FAULT 以外）: request_id 0 → DUPLICATE → 異常ゲート → 命令ごとの条件。
  void command(const v1::HostMsg& m, uint32_t request_id, uint64_t now) {
    if (request_id == 0) {
      push_ack(request_id, m.type, v1::ack_result::REJECTED, v1::ack_reason::INVALID_PARAM);
      return;
    }
    uint8_t payload[v1::MAX_PAYLOAD_LEN];
    const size_t plen = v1::encode_host_payload(m, payload, sizeof(payload));
    if (is_duplicate(m.type, request_id, payload, plen)) {
      push_ack(request_id, m.type, v1::ack_result::DUPLICATE, v1::ack_reason::NONE);
      return;
    }
    uint16_t reason;
    if (m.type != v1::msg::FAULT_RESET && fault_code() != v1::fault::NONE) {
      reason = v1::ack_reason::FAULT_ACTIVE;
    } else {
      reason = run_command(m, now);
    }
    if (reason == v1::ack_reason::NONE) {
      remember(m.type, request_id, payload, plen);
      push_ack(request_id, m.type, v1::ack_result::ACCEPTED, v1::ack_reason::NONE);
    } else {
      // 拒否は副作用が無いので記憶しない（再送は改めて評価する）
      push_ack(request_id, m.type, v1::ack_result::REJECTED, reason);
    }
  }

  bool is_duplicate(uint8_t type, uint32_t request_id, const uint8_t* payload, size_t plen) const {
    for (size_t i = 0; i < accepted_count_; ++i) {
      const Accepted& a = accepted_[i];
      if (a.type == type && a.request_id == request_id && a.len == plen && memcmp(a.payload, payload, plen) == 0) {
        return true;
      }
    }
    return false;
  }

  // 同じ request_id の古い記憶は置き換え、満杯なら最古を捨てる。
  void remember(uint8_t type, uint32_t request_id, const uint8_t* payload, size_t plen) {
    size_t w = 0;
    for (size_t r = 0; r < accepted_count_; ++r) {
      if (accepted_[r].request_id != request_id) {
        if (w != r) accepted_[w] = accepted_[r];
        w += 1;
      }
    }
    accepted_count_ = w;
    if (accepted_count_ >= kDedupeDepth) {
      for (size_t i = 1; i < accepted_count_; ++i) accepted_[i - 1] = accepted_[i];
      accepted_count_ -= 1;
    }
    Accepted& a = accepted_[accepted_count_++];
    a.type = type;
    a.request_id = request_id;
    a.len = static_cast<uint8_t>(plen > sizeof(a.payload) ? sizeof(a.payload) : plen);
    memset(a.payload, 0, sizeof(a.payload));
    memcpy(a.payload, payload, a.len);
  }

  void on_hello_req(uint32_t request_id) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::HELLO;
    m.hello.request_id = request_id;
    m.hello.protocol_version = v1::PROTOCOL_VERSION;
    m.hello.selected_axis = selected_axis_;
    m.hello.mcu_state = state_;
    m.hello.device_id = id_.device_id;
    m.hello.boot_id = id_.boot_id;
    m.hello.fw_major = id_.fw_major;
    m.hello.fw_minor = id_.fw_minor;
    m.hello.fw_patch = id_.fw_patch;
    m.hello.capabilities = v1::capabilities::POSITION_MODE;
    m.hello.calibration_hash = calibration_hash();
    push(m);
  }

  void on_clock_sync(const v1::ClockSync& c, uint64_t now) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::CLOCK_SYNC_REPLY;
    m.clock_sync_reply.request_id = c.request_id;
    m.clock_sync_reply.pc_time_us = c.pc_time_us;
    m.clock_sync_reply.mcu_rx_us = mcu_time_us(now);
    m.clock_sync_reply.mcu_tx_us = mcu_time_us(now);
    push(m);
  }

  // CONFIG: DISARMED のときだけ。§4 の表の検査だけ。epoch が変われば last_rx_seq = 0。
  uint16_t on_config(const v1::Config& c) {
    if (state_ != v1::state::DISARMED) return v1::ack_reason::BAD_STATE;
    const bool valid = c.soft_min_count < c.soft_max_count && c.max_vel_count_s > 0 && c.max_acc_count_s2 > 0 &&
                       c.following_error_limit_count > 0 && c.overspeed_count_s > 0 && c.command_timeout_ms > 0 &&
                       c.stop_timeout_ms > 0;
    if (!valid) return v1::ack_reason::INVALID_PARAM;
    if (!has_config_ || config_.session_epoch != c.session_epoch) {
      last_rx_seq_ = 0;  // seq の単調性は epoch ごと
    }
    config_ = c;
    has_config_ = true;
    return v1::ack_reason::NONE;
  }

  // ARM_PREPARE: 拒否理由の順序は §4 の表。受理で ARMING（主電源 ON 指令・ブレーキ作動のまま）。
  uint16_t on_arm_prepare(uint32_t epoch, uint64_t now) {
    if (state_ != v1::state::DISARMED) return v1::ack_reason::BAD_STATE;
    if (!has_config_) return v1::ack_reason::NOT_CONFIGURED;
    if (!referenced_) return v1::ack_reason::NOT_REFERENCED;
    if (config_.session_epoch != epoch) return v1::ack_reason::EPOCH_MISMATCH;
    if (estop_latched_) return v1::ack_reason::BAD_STATE;  // ローカル操作中（設計 §3。通常は FAULT 側で弾かれる）
    if (!inside_soft_window()) return v1::ack_reason::BAD_STATE;  // 解除しても SOFT_LIMIT になるだけ
    state_ = v1::state::ARMING;
    arming_started_us_ = now;
    lease_us_ = now;
    valid_until_us_ = 0;  // START に数えるのは以後に受けたフレームだけ
    brake_release_cmd_ = false;
    hold_here();
    return v1::ack_reason::NONE;
  }

  // START: HOLDING・epoch 一致・期限内の SET_POSITION。目標は次の SET_POSITION まで保持位置のまま。
  uint16_t on_start(uint32_t epoch, uint64_t now) {
    if (state_ != v1::state::HOLDING) return v1::ack_reason::BAD_STATE;
    if (!has_config_) return v1::ack_reason::NOT_CONFIGURED;
    if (config_.session_epoch != epoch) return v1::ack_reason::EPOCH_MISMATCH;
    if (now > valid_until_us_) return v1::ack_reason::BAD_STATE;  // 受理しても次の tick で期限切れになる
    state_ = v1::state::ACTIVE;
    control_target_ = setpoint_;
    frame_max_vel_ = static_cast<double>(config_.max_vel_count_s);
    frame_max_acc_ = static_cast<double>(config_.max_acc_count_s2);
    return v1::ack_reason::NONE;
  }

  // SET_POSITION: ACK なし。古い → stale_drop、apply_at ≠ 0 → rx_error、有効 → lease。ACTIVE でだけ目標に反映。
  void on_set_position(const v1::SetPosition& m, uint64_t now) {
    if (!has_config_ || m.session_epoch != config_.session_epoch || m.seq <= last_rx_seq_) {
      if (stale_drop_count_ < 0xFFFF) stale_drop_count_ += 1;
      return;
    }
    if (m.apply_at_us != 0) {
      record_rx_error();  // v1 では時刻同期が無いので予約適用はできない。lease も延長しない
      return;
    }
    last_rx_seq_ = m.seq;
    lease_us_ = now;
    valid_until_us_ = now + ms_to_us(m.valid_for_ms);
    if (state_ == v1::state::ACTIVE) {
      int32_t target = m.target_count;
      if (target < config_.soft_min_count) target = config_.soft_min_count;
      if (target > config_.soft_max_count) target = config_.soft_max_count;
      control_target_ = static_cast<double>(target);
      const uint32_t vmax = m.max_vel_count_s < config_.max_vel_count_s ? m.max_vel_count_s : config_.max_vel_count_s;
      const uint32_t amax =
          m.max_acc_count_s2 < config_.max_acc_count_s2 ? m.max_acc_count_s2 : config_.max_acc_count_s2;
      frame_max_vel_ = static_cast<double>(vmax);
      frame_max_acc_ = static_cast<double>(amax);
      last_applied_seq_ = m.seq;
      last_applied_frame_ = m.group_frame_id;
    }
  }

  // CONTROLLED_STOP: epoch 一致。ACTIVE → STOPPING、それ以外は何もせず受理。
  uint16_t on_controlled_stop(uint32_t epoch, uint64_t now) {
    if (!has_config_ || config_.session_epoch != epoch) return v1::ack_reason::EPOCH_MISMATCH;
    if (state_ == v1::state::ACTIVE) begin_stop(StopKind::Hold, now);
    return v1::ack_reason::NONE;
  }

  // DISARM: 動いていれば減速してから DISARMING。DISARMED / DISARMING では何もしない。
  uint16_t on_disarm(uint64_t now) {
    switch (state_) {
      case v1::state::DISARMED:
      case v1::state::DISARMING:
      case v1::state::BOOT: break;
      case v1::state::ARMING:
      case v1::state::HOLDING: enter_disarming(now); break;
      case v1::state::ACTIVE: begin_stop(StopKind::Disarm, now); break;
      case v1::state::STOPPING: stop_kind_ = StopKind::Disarm; break;
      case v1::state::FAULT: return v1::ack_reason::FAULT_ACTIVE;  // 異常ゲートで先に弾かれる
      default: break;
    }
    return v1::ack_reason::NONE;
  }

  // FAULT_RESET: FAULT のときだけ。原因が残っていれば FAULT_CAUSE_PRESENT。解除後 DISARMED。
  uint16_t on_fault_reset() {
    if (state_ != v1::state::FAULT) return v1::ack_reason::BAD_STATE;
    if (fault_cause_present()) return v1::ack_reason::FAULT_CAUSE_PRESENT;
    if (fault_code_ == v1::fault::ENCODER_FAULT) referenced_ = false;
    state_ = v1::state::DISARMED;
    fault_code_ = v1::fault::NONE;
    stop_kind_ = StopKind::Hold;
    hold_here();
    return v1::ack_reason::NONE;
  }

  // HOST_FAULT: 共通規則を飛ばして毎回実行、常に ACCEPTED。記憶しない。
  void on_host_fault(const v1::HostFault& h, uint64_t now) {
    last_host_fault_code_ = h.fault_code;
    const uint16_t code = (state_ == v1::state::STOPPING && stop_kind_ == StopKind::Fault) ? stop_fault_code_
                                                                                            : v1::fault::HOST_FAULT;
    latch_fault(code, now);
    push_ack(h.request_id, v1::msg::HOST_FAULT, v1::ack_result::ACCEPTED, v1::ack_reason::NONE);
  }

  // SET_REFERENCE: DISARMED のときだけ。現在の生カウントを reference_count と定義する（符号は掛けない）。
  uint16_t on_set_reference(int32_t reference_count) {
    if (state_ != v1::state::DISARMED) return v1::ack_reason::BAD_STATE;
    reference_offset_ = raw_ - static_cast<int64_t>(reference_count);
    hold_here();
    referenced_ = true;
    return v1::ack_reason::NONE;
  }

  // ------------------------------------------------------------------------
  // 遷移
  // ------------------------------------------------------------------------

  void begin_stop(StopKind kind, uint64_t now) {
    state_ = v1::state::STOPPING;
    stop_kind_ = kind;
    stopping_started_us_ = now;
  }

  // 減速してからラッチする異常（COMMAND_TIMEOUT）。止まっていれば即ラッチ。
  void begin_fault_stop(uint16_t code, uint64_t now) {
    if (setpoint_vel_ == 0.0) {
      latch_fault(code, now);
    } else if (state_ == v1::state::STOPPING) {
      stop_kind_ = StopKind::Fault;  // 減速の開始時刻はそのまま（stop_timeout の起点を保つ）
      stop_fault_code_ = code;
    } else {
      begin_stop(StopKind::Fault, now);
      stop_fault_code_ = code;
    }
  }

  void finish_stop(uint64_t now) {
    switch (stop_kind_) {
      case StopKind::Hold:
        state_ = v1::state::HOLDING;
        control_target_ = setpoint_;
        break;
      case StopKind::Disarm: enter_disarming(now); break;
      case StopKind::Fault: latch_fault(stop_fault_code_, now); break;
    }
  }

  // ブレーキ給電 OFF → BRAKE_APPLY_WAIT_MS → DISARMED
  void enter_disarming(uint64_t now) {
    state_ = v1::state::DISARMING;
    disarming_started_us_ = now;
    brake_release_cmd_ = false;
    stop_kind_ = StopKind::Hold;
  }

  // FAULT ラッチ: PWM 0・ブレーキ作動・主電源 OFF 指令。最初のコードが残る。FAULT_EVENT を 1 回。
  void latch_fault(uint16_t code, uint64_t now) {
    if (state_ == v1::state::FAULT) return;
    const uint8_t before = state_;
    state_ = v1::state::FAULT;
    fault_code_ = code;
    stop_kind_ = StopKind::Hold;
    brake_release_cmd_ = false;
    duty_ = 0;  // 次の tick を待たず PWM 0
    stall_running_ = false;
    hold_here();
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::FAULT_EVENT;
    m.fault_event.boot_id = id_.boot_id;
    m.fault_event.mcu_time_us = mcu_time_us(now);
    m.fault_event.fault_code = code;
    m.fault_event.mcu_state_before = before;
    m.fault_event.actual_count = actual_count();
    push(m);
  }

  // ------------------------------------------------------------------------
  // tick の中身
  // ------------------------------------------------------------------------

  void update_velocity(uint64_t now) {
    const uint64_t window = ms_to_us(tuning.velocity_window_ms);
    if (now >= vel_anchor_us_ + window) {
      const int64_t dc = raw_ - vel_anchor_raw_;
      const uint64_t dt = now - vel_anchor_us_;
      velocity_ = clamp_count((dc * 1000000) / static_cast<int64_t>(dt));
      vel_anchor_raw_ = raw_;
      vel_anchor_us_ = now;
    }
  }

  void check_timers(uint64_t now) {
    if (state_ == v1::state::ARMING) {
      const uint64_t t = now - arming_started_us_;
      if (!brake_release_cmd_ && t >= ms_to_us(tuning.power_settle_ms)) {
        // 電源が落ち着いた。ソフトリミット外ではブレーキを解除しない（解除直後に SOFT_LIMIT になるパルスを避ける）
        if (!inside_soft_window()) {
          latch_fault(v1::fault::SOFT_LIMIT, now);
          return;
        }
        hold_here();  // 現在位置を保持目標にして位置制御開始
        brake_release_cmd_ = true;
        brake_release_at_us_ = now;
      }
      if (brake_release_cmd_ && now - brake_release_at_us_ >= ms_to_us(tuning.brake_release_wait_ms)) {
        state_ = v1::state::HOLDING;
      }
    } else if (state_ == v1::state::DISARMING) {
      if (now - disarming_started_us_ >= ms_to_us(tuning.brake_apply_wait_ms)) {
        state_ = v1::state::DISARMED;  // ブレーキ作動済み: PWM 0・主電源 OFF
        hold_here();
      }
    }
    if (!has_config_) return;

    // lease: command_timeout_ms の間、有効な SET_POSITION が無い
    bool leased = false;
    switch (state_) {
      case v1::state::ARMING:
      case v1::state::HOLDING:
      case v1::state::ACTIVE: leased = true; break;
      case v1::state::STOPPING: leased = stop_kind_ == StopKind::Hold; break;
      default: break;
    }
    if (leased && now - lease_us_ > ms_to_us(config_.command_timeout_ms)) {
      begin_fault_stop(v1::fault::COMMAND_TIMEOUT, now);
    }
    // 目標の期限: ACTIVE → STOPPING → HOLDING
    if (state_ == v1::state::ACTIVE && now > valid_until_us_) {
      begin_stop(StopKind::Hold, now);
    }
    // 減速は stop_timeout_ms 内に終わる
    if (state_ == v1::state::STOPPING && now - stopping_started_us_ > ms_to_us(config_.stop_timeout_ms)) {
      latch_fault(stop_kind_ == StopKind::Fault ? stop_fault_code_ : v1::fault::STOP_TIMEOUT, now);
    }
  }

  // 速度・加速度制限つきの setpoint（sim_axis.rs の update_setpoint / profile_step と同じ）
  void update_setpoint(double dt) {
    if (!has_config_) return;
    const double lo = static_cast<double>(config_.soft_min_count);
    const double hi = static_cast<double>(config_.soft_max_count);
    if (state_ == v1::state::ACTIVE) {
      double p = setpoint_;
      double v = setpoint_vel_;
      profile_step(control_target_, frame_max_vel_, frame_max_acc_, dt, &p, &v);
      setpoint_ = p < lo ? lo : (p > hi ? hi : p);
      setpoint_vel_ = v;
    } else if (state_ == v1::state::STOPPING) {
      const double dv = static_cast<double>(config_.max_acc_count_s2) * dt;
      double v = setpoint_vel_;
      if (fabs(v) <= dv) {
        v = 0.0;
      } else {
        v -= dv * (v > 0 ? 1.0 : -1.0);
      }
      setpoint_vel_ = v;
      const double p = setpoint_ + v * dt;
      setpoint_ = p < lo ? lo : (p > hi ? hi : p);
    } else {
      setpoint_vel_ = 0.0;  // ARMING / HOLDING / DISARMING は保持
    }
  }

  // 目標へ向かって加速度制限で加減速、速度制限を超えない、到達（通過）で目標に乗せて速度 0。
  static void profile_step(double target, double vmax, double amax, double dt, double* pos, double* vel) {
    const double err = target - *pos;
    if (vmax < 0) vmax = 0;
    if (amax < 0) amax = 0;
    double v_des = 0.0;
    if (err != 0.0) {
      const double brake_v = sqrt(2.0 * amax * fabs(err));
      v_des = (err > 0 ? 1.0 : -1.0) * (vmax < brake_v ? vmax : brake_v);
    }
    double dv = v_des - *vel;
    const double dv_max = amax * dt;
    if (dv > dv_max) dv = dv_max;
    if (dv < -dv_max) dv = -dv_max;
    const double v = *vel + dv;
    const double p = *pos + v * dt;
    const bool settled = (err == 0.0) ? (v == 0.0) : ((target - p) * err <= 0.0);
    if (settled) {
      *pos = target;
      *vel = 0.0;
    } else {
      *pos = p;
      *vel = v;
    }
  }

  void check_motion_faults(uint64_t now) {
    if (!has_config_) return;
    const int64_t fe = static_cast<int64_t>(actual_count()) - static_cast<int64_t>(target_count());
    const int64_t fe_abs = fe < 0 ? -fe : fe;
    const int64_t vel_abs = velocity_ < 0 ? -static_cast<int64_t>(velocity_) : velocity_;
    if (fe_abs > static_cast<int64_t>(config_.following_error_limit_count)) {
      latch_fault(v1::fault::FOLLOWING_ERROR, now);
    } else if (vel_abs > static_cast<int64_t>(config_.overspeed_count_s)) {
      latch_fault(v1::fault::OVERSPEED, now);
    } else if (!inside_soft_window()) {
      latch_fault(v1::fault::SOFT_LIMIT, now);
    }
  }

  // 位置制御の Duty と無パルス判定
  void update_duty(uint64_t now) {
    if (!position_control_on()) {
      duty_ = 0;
      stall_running_ = false;
      in_band_ = false;
      return;
    }
    const int64_t err = static_cast<int64_t>(target_count()) - static_cast<int64_t>(actual_count());
    const int cmd = v1_duty_command(err, setpoint_vel_, in_band_, tuning);  // カウント増の向きが正
    const int mag = cmd < 0 ? -cmd : cmd;
    // カウントを増やしたいなら −sign × duty
    duty_ = -static_cast<int>(sign_) * cmd;

    // ENCODER_FAULT: |duty| ≥ stall_duty が stall_ms 続いてカウントが変わらない（manual_guard.h の無パルス判定と同じ窓）。
    // 窓を回すのは「ブレーキ解除指令中」かつ「軌道が動いている（setpoint_vel ≠ 0）」間だけ（設計 §3）:
    //  - ブレーキ作動指令中（DISARMING）はカウントが変わらないのが正常。残った追従誤差で窓を回すと DISARM が FAULT になる。
    //  - 目標が止まっているときの静止（荷重とのつり合いで最小 Duty では動かない）は、エンコーダの死と区別できない。
    //    この場合の誤差の上限は FOLLOWING_ERROR が守る（エンコーダが生きている前提。エンコーダが死んだまま目標が
    //    止まっている間は検出できず、目標が動けば STALL_MS で出る）。
    if (mag >= tuning.stall_duty && brake_release_cmd_ && setpoint_vel_ != 0.0) {
      if (!stall_running_) {
        stall_running_ = true;
        stall_start_us_ = now;
        stall_last_change_us_ = now;
        stall_last_raw_ = raw_;
      } else {
        if (raw_ != stall_last_raw_) {
          stall_last_raw_ = raw_;
          stall_last_change_us_ = now;
        }
        const uint64_t window = ms_to_us(tuning.stall_ms);
        if (now - stall_start_us_ >= window && now - stall_last_change_us_ >= window) {
          latch_fault(v1::fault::ENCODER_FAULT, now);
          duty_ = 0;
          stall_running_ = false;
        }
      }
    } else {
      stall_running_ = false;
    }
  }

  // ------------------------------------------------------------------------
  // 送信
  // ------------------------------------------------------------------------

  void push_ack(uint32_t request_id, uint8_t acked_type, uint8_t result, uint16_t reason) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::ACK;
    m.ack.request_id = request_id;
    m.ack.acked_type = acked_type;
    m.ack.result = result;
    m.ack.reason = reason;
    push(m);
  }

  // 満杯なら最古の STATUS を 1 つ捨てて入れる。STATUS が無ければ、入れるのが STATUS なら捨て、
  // それ以外なら最古を捨てる（ACK・FAULT_EVENT・AXIS_SELECT・HELLO を優先）。
  void push(const v1::McuMsg& m) {
    if (out_count_ >= kOutboxCap) {
      bool removed = false;
      for (size_t i = 0; i < out_count_; ++i) {
        const size_t idx = (out_head_ + i) % kOutboxCap;
        if (outbox_[idx].type == v1::msg::STATUS) {
          for (size_t j = i; j + 1 < out_count_; ++j) {
            outbox_[(out_head_ + j) % kOutboxCap] = outbox_[(out_head_ + j + 1) % kOutboxCap];
          }
          out_count_ -= 1;
          removed = true;
          break;
        }
      }
      if (!removed) {
        if (m.type == v1::msg::STATUS) return;
        pop();
      }
    }
    outbox_[(out_head_ + out_count_) % kOutboxCap] = m;
    out_count_ += 1;
  }

  void push_status(uint64_t now) {
    v1::McuMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::STATUS;
    m.status = status(now);
    push(m);
  }

  void remember_key() {
    has_last_key_ = true;
    key_state_ = state_;
    key_fault_ = fault_code();
    key_flags_ = flags();
    key_axis_ = selected_axis_;
  }

  // 状態・fault_code・flags・selected_axis が変わったら即時 STATUS
  void emit_status_if_changed() {
    const bool changed = !has_last_key_ || key_state_ != state_ || key_fault_ != fault_code() ||
                         key_flags_ != flags() || key_axis_ != selected_axis_;
    if (changed) {
      remember_key();
      push_status(now_us_);
    }
  }

  // 20 ms ごとの STATUS（固定の刻み、1 回の tick で最大 1 つ）
  void emit_periodic_status(uint64_t now) {
    const uint64_t period = ms_to_us(tuning.status_period_ms ? tuning.status_period_ms : 1);
    if (!has_next_status_) {
      has_next_status_ = true;
      next_status_us_ = now;
    }
    if (now >= next_status_us_) {
      next_status_us_ += ((now - next_status_us_) / period + 1) * period;
      remember_key();
      push_status(now);
    }
  }
};
