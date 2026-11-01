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
#include "v1_axis.h"  // V1DutyTerms（計器行）
// --- USB プロトコル v1（MC_PROTO_V1）。制御タスクは V1Axis を回し、G2 / G13 / PWM を直接書く ---
struct V1Snapshot {
  uint8_t state = 0;             // v1::state::*
  uint16_t fault_code = 0;       // v1::fault::*
  uint16_t flags = 0;            // STATUS flags
  uint8_t selected_axis = 0;
  bool power_on_cmd = false;
  bool brake_release_cmd = false;
  int duty = 0;                  // 符号付き %
  int stall_boost = 0;           // 停止ブースト（Issue #15 H4）で下限に上乗せしている %（0 = なし）
  V1DutyTerms terms{};           // 位置ループの各項（Issue #25 手順 8 の計器行。permille、カウント増が正）
  int16_t pwm_permille = 0;      // STATUS と同じ pwm_permille（カウント増が正）
  bool encoder_fault_check_disabled = false;  // duty_max がしきい値より低く無パルスの ENCODER_FAULT が出ない（Issue #15 M2）
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
  uint32_t tx_deferred = 0;      // TX リングに空きが無く次の poll に回した回数（Issue #15 M7）
  uint32_t tx_stale_dropped = 0; // 古い／入らない STATUS と期限切れの保留を捨てた数
  uint32_t tick_count = 0;
  uint32_t max_tick_gap_us = 0;  // 起動からの tick 間隔の最大
  uint64_t device_id = 0;
  uint32_t boot_id = 0;
  // 校正（CALIBRATING / CAL_RESULT）
  uint8_t cal_live_step = 0;         // 校正中の段階（v1::cal_step::*。校正中でなければ 0）
  char cal_live_reason[24] = "";     // AutoMachine の直近の理由（"retreat" など）
  bool cal_result_valid = false;     // この boot で CAL_RESULT を送ったことがある
  uint8_t cal_result = 0;            // v1::cal_result::*
  uint8_t cal_step = 0;              // 終了時の段階
  int32_t cal_n_end = 0;
  int32_t cal_neutral = 0;
  int8_t cal_sign = 1;
  char cal_reason[9] = "";           // NUL 終端済み
  bool profile_dirty = false;        // 成功した校正のプロファイルが未保存（loop() が NVS に保存して clear する）
  MotorProfile profile{};
};

// signs は系統 1..3 の符号（index = motor。[0] は未使用）。selected_axis は NVS の motor。
void control_v1_begin(const MotorProfile profiles[4], uint8_t selected_axis);  // profiles[1..3] = NVS の各系統（符号と、校正の速い区間の判定に使う行程）
void control_v1_select_axis(uint8_t axis);   // DISARMED のときだけ V1Axis が受け付ける
void control_v1_set_estop(bool latched);     // 画面の非常停止（ラッチ / 解除）
void control_v1_clear_profile_dirty();       // loop() が NVS に保存した後に呼ぶ
V1Snapshot control_v1_snapshot();
#endif
