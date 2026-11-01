#include "motor_output.h"

#include <Arduino.h>

#include "config.h"

namespace {

constexpr uint32_t kDutyMax = (1u << PWM_BITS) - 1;
int g_applied = 0;

int oppositeLevel(int level) { return level ? 0 : 1; }

void writePwm(uint32_t duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcWrite(PIN_MD13S_PWM, duty);
#else
  ledcWrite(PWM_CHANNEL, duty);
#endif
}

}  // namespace

void motor_output_begin() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  ledcAttach(PIN_MD13S_PWM, PWM_FREQ_HZ, PWM_BITS);
  const uint32_t actual_hz = PWM_FREQ_HZ;
#else
  const uint32_t actual_hz = ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(PIN_MD13S_PWM, PWM_CHANNEL);
#endif
  writePwm(0);
#if MC_PROTO_V1
  (void)actual_hz;  // v1 では USB CDC はバイナリ専用。テキストを出さない
#else
  Serial.printf("pwm %u Hz requested, timer %u Hz, %u-bit\n", PWM_FREQ_HZ, actual_hz, PWM_BITS);
#endif
}

// 反転するときだけ PWM を落としてから DIR を変える。同じ出力なら戻る。
void motor_output_write(int percent) {
  if (percent == g_applied) {
    return;
  }
  percent = constrain(percent, SPEED_MIN, SPEED_MAX);
  const int new_dir = (percent >= 0) ? DIR_POSITIVE_LEVEL : oppositeLevel(DIR_POSITIVE_LEVEL);
  const int old_dir = (g_applied >= 0) ? DIR_POSITIVE_LEVEL : oppositeLevel(DIR_POSITIVE_LEVEL);
  const bool reversing = percent != 0 && g_applied != 0 && new_dir != old_dir;
  if (reversing || percent == 0) {
    writePwm(0);
  }
  if (reversing) {
    delay(DIR_COAST_MS);
  }
  if (percent != 0) {
    digitalWrite(PIN_MD13S_DIR, new_dir);
    const int magnitude = percent < 0 ? -percent : percent;
    writePwm(static_cast<uint32_t>(magnitude) * kDutyMax / 100u);
  }
  g_applied = percent;
}

int motor_output_applied() { return g_applied; }
