#pragma once

#include <stdint.h>

// 動作確認済み moter-controler と同じ電気設定。
// 未測定の時間・Duty・移動量はここへ入れない。実行時の SET まで UNCONFIGURED。

constexpr int POWER_GPIO = 2;             // Port A 黄。主電源リレー
constexpr int BRAKE_GPIO = 1;             // Port A 白。ブレーキ解除リレー
constexpr int POWER_ON_LEVEL = 1;         // HIGH で給電指令
constexpr int BRAKE_RELEASE_LEVEL = 1;    // HIGH でブレーキ解除指令
constexpr int PWM_GPIO = 9;               // Port B 黄。MD13S PWM
constexpr int DIR_GPIO = 8;               // Port B 白。MD13S DIR
constexpr int DIR_POSITIVE_LEVEL = 0;     // 正方向は DIR Low
constexpr int PWM_FREQUENCY_HZ = 20000;   // MD13S の上限
constexpr int PWM_RESOLUTION_BITS = 10;
constexpr int PWM_FULL_SCALE = (1 << PWM_RESOLUTION_BITS) - 1;  // 1023
constexpr int ENCODER_A_GPIO = 17;        // Port C 黄。センサー白の既定
constexpr int ENCODER_B_GPIO = 18;        // Port C 白。センサー緑の既定
constexpr int ENCODER_SIGN_DEFAULT = 1;

constexpr int UNCONFIGURED = -1;
constexpr int PCNT_HIGH_LIMIT = 20000;
constexpr int PCNT_LOW_LIMIT = -20000;
constexpr int CONTROL_PERIOD_MS = 5;
constexpr int CSV_PERIOD_US = 20000;
constexpr int VELOCITY_WINDOW_US = 100000;
constexpr int UI_PERIOD_MS = 66;
constexpr int SAMPLE_RING = 128;
constexpr int EVENT_RING = 64;
constexpr int TRIAL_SLOTS = 16;
constexpr int VISUAL_SLOTS = 12;

// 50% は 50/255 ではない。10bit フルスケール 1023 に対する百分率。
constexpr int pwm_raw_from_percent(int percent) {
  if (percent <= 0) {
    return 0;
  }
  if (percent >= 100) {
    return PWM_FULL_SCALE;
  }
  return percent * PWM_FULL_SCALE / 100;
}

constexpr int dir_level_for(int direction) {
  if (direction >= 0) {
    return DIR_POSITIVE_LEVEL;
  }
  return DIR_POSITIVE_LEVEL ? 0 : 1;
}

constexpr int relay_off_level(int on_level) { return on_level ? 0 : 1; }

// LEDC: 1..14 bit、1..20000 Hz、80 MHz / freq が 2^bits 以上。
constexpr bool pwm_timing_ok() {
  if (PWM_RESOLUTION_BITS < 1 || PWM_RESOLUTION_BITS > 14) {
    return false;
  }
  if (PWM_FREQUENCY_HZ < 1 || PWM_FREQUENCY_HZ > 20000) {
    return false;
  }
  const uint32_t period = 80000000u / static_cast<uint32_t>(PWM_FREQUENCY_HZ);
  return period >= (1u << PWM_RESOLUTION_BITS);
}

constexpr bool drive_pins_unique() {
  const int pins[] = {POWER_GPIO, BRAKE_GPIO, PWM_GPIO, DIR_GPIO, ENCODER_A_GPIO, ENCODER_B_GPIO};
  for (int i = 0; i < 6; ++i) {
    if (pins[i] < 0 || pins[i] > 48) {
      return false;
    }
    for (int j = i + 1; j < 6; ++j) {
      if (pins[i] == pins[j]) {
        return false;
      }
    }
  }
  return true;
}
