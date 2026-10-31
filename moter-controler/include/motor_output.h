#pragma once

// MD13S への PWM/DIR 出力。書くのは制御タスクだけ。
void motor_output_begin();            // LEDC を設定し PWM 0
void motor_output_write(int percent); // -100..100。反転時は PWM 0 → DIR_COAST_MS → DIR 切替
int motor_output_applied();           // 直近に書いた値
