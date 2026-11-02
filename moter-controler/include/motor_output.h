#pragma once

#include <stdint.h>

// MD13S への PWM/DIR 出力。書くのは制御タスクだけ。
// 内部分解能は permille（0.1 %、Issue #25 手順 8 b）。±1000 が PWM のフルスケール。
void motor_output_begin();                     // LEDC を設定し PWM 0
void motor_output_write_permille(int permille); // -1000..1000。反転時は PWM 0 → DIR_COAST_MS → DIR 切替
void motor_output_write(int percent);           // -100..100（従来 env の手動速度・自動移動）。×10 して permille の経路へ
int motor_output_applied_permille();            // 直近に書いた値 [permille]
int motor_output_applied();                     // 直近に書いた値 [%]（0 に向けて切り捨て。従来 env の表示用）

// 純粋関数（ホストテスト用）。|permille| → LEDC のカウント。1000 でフルスケール（full_scale）、0 で 0、切り捨て。
inline uint32_t motor_output_pwm_ticks(int permille, uint32_t full_scale) {
  const uint32_t mag = static_cast<uint32_t>(permille < 0 ? -permille : permille);
  return (mag > 1000u ? 1000u : mag) * full_scale / 1000u;
}
// 反転か（どちらも 0 でなく符号が違う）。0 をまたがない同符号の変化・0 への変化・0 からの変化は反転ではない
inline bool motor_output_reverses(int applied_permille, int next_permille) {
  return applied_permille != 0 && next_permille != 0 && (applied_permille < 0) != (next_permille < 0);
}
