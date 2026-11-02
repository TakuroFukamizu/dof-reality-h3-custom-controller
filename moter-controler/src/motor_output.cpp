#include "motor_output.h"

#include <Arduino.h>

#include "config.h"

namespace {

constexpr uint32_t kDutyMax = (1u << PWM_BITS) - 1;  // 1000 permille でこの値
int g_applied = 0;  // permille

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
void motor_output_write_permille(int permille) {
  if (permille == g_applied) {
    return;
  }
  permille = constrain(permille, PERMILLE_MIN, PERMILLE_MAX);
  const int new_dir = (permille >= 0) ? DIR_POSITIVE_LEVEL : oppositeLevel(DIR_POSITIVE_LEVEL);
  const bool reversing = motor_output_reverses(g_applied, permille);
  if (reversing || permille == 0) {
    writePwm(0);
  }
  if (reversing) {
    delay(DIR_COAST_MS);
  }
  if (permille != 0) {
    digitalWrite(PIN_MD13S_DIR, new_dir);
    writePwm(motor_output_pwm_ticks(permille, kDutyMax));
  }
  g_applied = permille;
}

void motor_output_write(int percent) {
  motor_output_write_permille(constrain(percent, SPEED_MIN, SPEED_MAX) * PERMILLE_PER_PERCENT);
}

int motor_output_applied_permille() { return g_applied; }
int motor_output_applied() { return g_applied / PERMILLE_PER_PERCENT; }
