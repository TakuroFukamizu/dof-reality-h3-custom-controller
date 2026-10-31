#pragma once

#include <stdint.h>

// フィードバックセンサー（2 相直交）を ESP-IDF 4.4 の legacy PCNT で 4 逓倍計数する。
// 16 bit カウンタの ±20000 リミットイベントで ISR が int64 に積む。符号は生（ENCODER_SIGN_DEFAULT=+1 で速度 − のとき増）。
bool encoder_begin();
int64_t encoder_count();
int encoder_a_level();
int encoder_b_level();
