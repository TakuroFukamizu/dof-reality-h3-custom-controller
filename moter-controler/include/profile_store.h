#pragma once

#include <stdint.h>

#include "motor_profile.h"

// 系統プロファイルの NVS 保存。名前空間 "h3"。書くのは校正完了時と系統切り替え時だけ。
// 保存形式は profile_blob.h（1 キー m{n}_prof に 16 byte を 1 回で書く。Issue #15 M6）。旧 4 キー形式は読むだけ。
bool profile_store_begin();
uint8_t profile_store_selected_motor();          // 無ければ 1
void profile_store_select_motor(uint8_t motor);  // "motor" を書く
// 無ければ既定（stroke 0, calibrated false, sign +1）。blob があるのに検査に通らなければ既定を返し *invalid = true
//（呼び出し側は画面・シリアルに出す。校正し直して保存すれば上書きされる）。
MotorProfile profile_store_load(uint8_t motor, bool* invalid = nullptr);
// 失敗したら false（NVS には旧 blob がそのまま残る）。呼び出し側は dirty を残して後で再試行する。
bool profile_store_save(const MotorProfile& p);
