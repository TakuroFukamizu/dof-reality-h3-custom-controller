#pragma once

#include <stdint.h>

#include "motor_profile.h"

// 系統プロファイルの NVS 保存。名前空間 "h3"。書くのは校正完了時と系統切り替え時だけ。
bool profile_store_begin();
uint8_t profile_store_selected_motor();          // 無ければ 1
void profile_store_select_motor(uint8_t motor);  // "motor" を書く
MotorProfile profile_store_load(uint8_t motor);  // 無ければ既定（stroke 0, calibrated false, sign +1）
bool profile_store_save(const MotorProfile& p);  // m{n}_stroke / neutral / cal / sign
