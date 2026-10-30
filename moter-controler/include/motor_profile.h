#pragma once

#include <stdint.h>

// 系統（ケーブル表記 #1〜#3）ごとのプロファイル。NVS に保存する値と同じ形。
// 座標は P 端（速度 + で当たる端）を原点 0 とし、N 端へ向かって増える。
struct MotorProfile {
  uint8_t motor = 1;       // 1..3
  int32_t stroke = 0;      // P 端から N 端までのカウント。未校正は 0
  int32_t neutral = 0;     // 原点からのカウント。未校正は 0
  int8_t sign = 1;         // 計数の符号。+1 なら速度 − でカウント増（#1 の実測）
  bool calibrated = false;
};

inline bool profile_motor_ok(uint8_t motor) { return motor >= 1 && motor <= 3; }

// #1・#2 は前部アクチュエータ（P 端 = 沈み切り = 下限）。#3 は後部ヨー（P 端 = 右端）。
inline const char* p_end_label(uint8_t motor) { return motor == 3 ? "右端" : "下限"; }
inline const char* n_end_label(uint8_t motor) { return motor == 3 ? "左端" : "上限"; }

inline int32_t default_neutral(int32_t stroke) { return stroke / 2; }
