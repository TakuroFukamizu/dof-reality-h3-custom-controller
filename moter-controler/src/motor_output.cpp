#include "motor_output.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "config.h"
#include "motor_dir_coast.h"

namespace {

constexpr uint32_t kDutyMax = (1u << PWM_BITS) - 1;  // 1000 permille でこの値
int g_applied = 0;  // permille。いまピンに出している値（反転待ちの間は 0）
MotorDirCoast g_coast;  // 反転の順序（PWM 0 → DIR_COAST_MS → DIR → PWM）

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

namespace {

// 状態機械の 1 歩をピンに書く。PWM を 0 にするのは出していたときだけ、DIR は向きが変わるときだけ（PWM 0 の間か、同じ向きのとき）。
// 要求が前回と同じで反転待ちでもなければ何も書かない。
bool apply(int permille, uint64_t now_us) {
  const MotorDirCoast::Out o = g_coast.step(permille, now_us, DIR_COAST_MS * 1000u);
  if (o.permille == 0) {
    if (g_applied != 0) writePwm(0);
  } else {
    if (o.write_dir) {
      digitalWrite(PIN_MD13S_DIR, o.dir > 0 ? DIR_POSITIVE_LEVEL : oppositeLevel(DIR_POSITIVE_LEVEL));
    }
    if (o.permille != g_applied) writePwm(motor_output_pwm_ticks(o.permille, kDutyMax));
  }
  g_applied = o.permille;
  return MotorDirCoast::coasting(permille, o);
}

}  // namespace

// 従来 env（手動速度・自動移動）: 反転待ちの間はこの中で待つ（以前の delay(DIR_COAST_MS) と同じく、戻ったときには新しい向きで出ている）。
// 従来 env には CONTROL_OVERRUN が無く、自動移動の端当て・退避の時間もこの振る舞いで合わせてある。1 ms ずつ見て、最長でも数 ms 余分。
void motor_output_write_permille(int permille) {
  permille = constrain(permille, PERMILLE_MIN, PERMILLE_MAX);
  if (permille == g_applied) {
    return;
  }
  for (uint32_t i = 0; apply(permille, static_cast<uint64_t>(esp_timer_get_time())) && i <= DIR_COAST_MS + 1; ++i) {
    delay(1);
  }
}

// v1（制御タスクが毎 tick 呼ぶ）: 待たない。反転では PWM 0 にして戻り、DIR_COAST_MS 経った後の tick で DIR と PWM を出す。
void motor_output_write_permille_at(int permille, uint64_t now_us) {
  permille = constrain(permille, PERMILLE_MIN, PERMILLE_MAX);
  if (permille == g_applied) {
    return;
  }
  apply(permille, now_us);
}

void motor_output_write(int percent) {
  motor_output_write_permille(constrain(percent, SPEED_MIN, SPEED_MAX) * PERMILLE_PER_PERCENT);
}

int motor_output_applied_permille() { return g_applied; }
int motor_output_applied() { return g_applied / PERMILLE_PER_PERCENT; }
