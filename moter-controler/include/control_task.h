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
