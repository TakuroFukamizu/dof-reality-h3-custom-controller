#include "encoder_pcnt.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>

#include "config.h"

namespace {

constexpr pcnt_unit_t kUnit = PCNT_UNIT_0;
constexpr int16_t kLimit = 20000;

volatile int64_t g_overflow = 0;
bool g_ready = false;

void IRAM_ATTR on_limit(void*) {
  uint32_t status = 0;
  pcnt_get_event_status(kUnit, &status);
  if (status & PCNT_EVT_H_LIM) {
    g_overflow += kLimit;
  }
  if (status & PCNT_EVT_L_LIM) {
    g_overflow -= kLimit;
  }
}

}  // namespace

bool encoder_begin() {
  // ch0: A のエッジ、B のレベル。立上り DEC・立下り INC、B Low で反転。
  pcnt_config_t ch0 = {};
  ch0.pulse_gpio_num = PIN_ENC_A;
  ch0.ctrl_gpio_num = PIN_ENC_B;
  ch0.channel = PCNT_CHANNEL_0;
  ch0.unit = kUnit;
  ch0.pos_mode = PCNT_COUNT_DEC;
  ch0.neg_mode = PCNT_COUNT_INC;
  ch0.lctrl_mode = PCNT_MODE_REVERSE;
  ch0.hctrl_mode = PCNT_MODE_KEEP;
  ch0.counter_h_lim = kLimit;
  ch0.counter_l_lim = -kLimit;
  if (pcnt_unit_config(&ch0) != ESP_OK) {
    return false;
  }
  // ch1: B のエッジ、A のレベル。立上り INC・立下り DEC、A Low で反転。
  pcnt_config_t ch1 = ch0;
  ch1.pulse_gpio_num = PIN_ENC_B;
  ch1.ctrl_gpio_num = PIN_ENC_A;
  ch1.channel = PCNT_CHANNEL_1;
  ch1.pos_mode = PCNT_COUNT_INC;
  ch1.neg_mode = PCNT_COUNT_DEC;
  if (pcnt_unit_config(&ch1) != ESP_OK) {
    return false;
  }
  pcnt_filter_disable(kUnit);
  gpio_pullup_dis(static_cast<gpio_num_t>(PIN_ENC_A));
  gpio_pulldown_dis(static_cast<gpio_num_t>(PIN_ENC_A));
  gpio_pullup_dis(static_cast<gpio_num_t>(PIN_ENC_B));
  gpio_pulldown_dis(static_cast<gpio_num_t>(PIN_ENC_B));

  pcnt_event_enable(kUnit, PCNT_EVT_H_LIM);
  pcnt_event_enable(kUnit, PCNT_EVT_L_LIM);
  pcnt_counter_pause(kUnit);
  pcnt_counter_clear(kUnit);
  const esp_err_t isr = pcnt_isr_service_install(0);
  if (isr != ESP_OK && isr != ESP_ERR_INVALID_STATE) {
    return false;
  }
  if (pcnt_isr_handler_add(kUnit, on_limit, nullptr) != ESP_OK) {
    return false;
  }
  g_overflow = 0;
  if (pcnt_counter_resume(kUnit) != ESP_OK) {
    return false;
  }
  g_ready = true;
  return true;
}

int64_t encoder_count() {
  if (!g_ready) {
    return 0;
  }
  // overflow → counter → overflow の順に読み、間にリミット ISR が走っていたら読み直す。
  // legacy PCNT はリミット到達でカウンタを 0 に戻すので、カウンタ値だけでは跨ぎを判定できない。
  for (int i = 0; i < 4; ++i) {
    const int64_t before = g_overflow;
    int16_t value = 0;
    pcnt_get_counter_value(kUnit, &value);
    const int64_t after = g_overflow;
    if (before == after) {
      return after + value;
    }
  }
  int16_t value = 0;
  pcnt_get_counter_value(kUnit, &value);
  return g_overflow + value;
}

int encoder_a_level() { return digitalRead(PIN_ENC_A) == HIGH ? 1 : 0; }

int encoder_b_level() { return digitalRead(PIN_ENC_B) == HIGH ? 1 : 0; }
