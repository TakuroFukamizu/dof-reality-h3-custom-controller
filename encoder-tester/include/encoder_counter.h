#pragma once

#include <stdint.h>

// 4逓倍 PCNT。A が先に上がる（B が Low のとき A が立ち上がる）向きが SIGN=+1 の正。
// 上下限の加算はドライバの accum_count だけが行う。コールバックで足さない。
// ZERO DISPLAY からは clear_count を呼ばない。clear は初期化とグリッチ再設定だけ。

bool encoder_begin();
bool encoder_get(int32_t* raw);
bool encoder_set_glitch(uint32_t max_glitch_ns);
int encoder_a_level();
int encoder_b_level();
