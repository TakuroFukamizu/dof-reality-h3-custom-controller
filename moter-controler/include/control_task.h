#pragma once

#include <stdint.h>

#include "auto_move.h"
#include "motor_profile.h"

// 5 ms 周期の制御タスク。エンコーダ読み取り、自動移動の状態機械、PWM 出力をここだけで行う。
// main（UI・シリアル）は入力を渡し、スナップショットを読む。

struct ControlSnapshot {
  int64_t raw_count = 0;
  int32_t position = 0;        // 原点基準。origin_valid が false なら raw と同じ扱い
  bool origin_valid = false;
  int count_per_s = 0;         // 100 ms 窓
  int a_level = 0;
  int b_level = 0;
  int applied_speed = 0;       // 実際に PWM へ書いた値（手動または自動）
  AutoKind kind = AutoKind::Idle;
  AutoPhase phase = AutoPhase::Idle;
  CalStep cal_step = CalStep::None;
  char reason[24] = "";
  MotorProfile profile{};
  bool profile_dirty = false;
  int32_t last_measured_stroke = 0;
  int8_t working_sign = 1;
  uint32_t request_seq = 0;    // 要求を処理するたびに +1
  uint32_t manual_fault_seq = 0;  // 手動速度の無パルス／遅すぎ停止が起きるたびに +1
  char manual_fault_reason[8] = "";  // stall / slow
  char request_error[16] = ""; // 直近の要求が拒否された理由。受理なら空
  uint32_t tick_count = 0;
};

void control_begin(const MotorProfile& initial);
void control_set_relay_live(bool live);
void control_set_brake_released(bool released);
void control_set_manual_speed(int percent);
void control_pulse_stop();
void control_pulse_estop();
void control_request(AutoKind kind);
void control_set_profile(const MotorProfile& p);
void control_clear_profile_dirty();
ControlSnapshot control_snapshot();

#if MC_PROTO_V1
// --- USB プロトコル v1（MC_PROTO_V1）。制御タスクは V1Axis を回し、G2 / G13 / PWM を直接書く ---
struct V1Snapshot {
  uint8_t state = 0;             // v1::state::*
  uint16_t fault_code = 0;       // v1::fault::*
  uint16_t flags = 0;            // STATUS flags
  uint8_t selected_axis = 0;
  bool power_on_cmd = false;
  bool brake_release_cmd = false;
  int duty = 0;                  // 符号付き %
  int32_t actual = 0;
  int32_t target = 0;
  int32_t velocity = 0;          // count/s（100 ms 窓）
  int32_t following_error = 0;
  bool configured = false;
  bool referenced = false;
  bool estop_latched = false;
  bool has_rx = false;
  uint32_t last_rx_age_ms = 0;   // PC から最後に何か受けてからの経過
  uint16_t rx_error_count = 0;
  uint16_t stale_drop_count = 0;
  uint32_t tx_dropped = 0;
  uint32_t tick_count = 0;
  uint32_t max_tick_gap_us = 0;  // 起動からの tick 間隔の最大
  uint64_t device_id = 0;
  uint32_t boot_id = 0;
};

// signs は系統 1..3 の符号（index = motor。[0] は未使用）。selected_axis は NVS の motor。
void control_v1_begin(const int8_t signs[4], uint8_t selected_axis);
void control_v1_select_axis(uint8_t axis);   // DISARMED のときだけ V1Axis が受け付ける
void control_v1_set_estop(bool latched);     // 画面の非常停止（ラッチ / 解除）
V1Snapshot control_v1_snapshot();
#endif
