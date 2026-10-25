#include <M5Unified.h>

#include <driver/gpio.h>
#include <driver/pulse_cnt.h>

#include "config.h"
#include "encoder_counter.h"

namespace {

pcnt_unit_handle_t unit = nullptr;
bool ready = false;

void pulls_off() {
  gpio_pullup_dis(static_cast<gpio_num_t>(ENCODER_A_GPIO));
  gpio_pulldown_dis(static_cast<gpio_num_t>(ENCODER_A_GPIO));
  gpio_pullup_dis(static_cast<gpio_num_t>(ENCODER_B_GPIO));
  gpio_pulldown_dis(static_cast<gpio_num_t>(ENCODER_B_GPIO));
}

bool restart(bool clear) {
  if (pcnt_unit_enable(unit) != ESP_OK) {
    return false;
  }
  if (clear) {
    pcnt_unit_clear_count(unit);
  }
  if (pcnt_unit_start(unit) != ESP_OK) {
    return false;
  }
  ready = true;
  return true;
}

}  // namespace

bool encoder_begin() {
  pcnt_unit_config_t unit_cfg = {};
  unit_cfg.low_limit = PCNT_LOW_LIMIT;
  unit_cfg.high_limit = PCNT_HIGH_LIMIT;
  unit_cfg.flags.accum_count = 1;
  if (pcnt_new_unit(&unit_cfg, &unit) != ESP_OK) {
    return false;
  }
  pcnt_unit_set_glitch_filter(unit, nullptr);

  pcnt_channel_handle_t channel_a = nullptr;
  pcnt_chan_config_t chan_a = {};
  chan_a.edge_gpio_num = ENCODER_A_GPIO;
  chan_a.level_gpio_num = ENCODER_B_GPIO;
  if (pcnt_new_channel(unit, &chan_a, &channel_a) != ESP_OK) {
    return false;
  }
  // 正エッジで減算、負エッジで加算。B が Low のとき反転するので、A 先行は +1 ずつ。
  pcnt_channel_set_edge_action(channel_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                               PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(channel_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pcnt_channel_handle_t channel_b = nullptr;
  pcnt_chan_config_t chan_b = {};
  chan_b.edge_gpio_num = ENCODER_B_GPIO;
  chan_b.level_gpio_num = ENCODER_A_GPIO;
  if (pcnt_new_channel(unit, &chan_b, &channel_b) != ESP_OK) {
    return false;
  }
  pcnt_channel_set_edge_action(channel_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                               PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(channel_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

  pulls_off();
  // watch が無いと limit 割り込みが起きず、accum_count が進まない。
  if (pcnt_unit_add_watch_point(unit, PCNT_HIGH_LIMIT) != ESP_OK) {
    return false;
  }
  if (pcnt_unit_add_watch_point(unit, PCNT_LOW_LIMIT) != ESP_OK) {
    return false;
  }
  return restart(true);
}

bool encoder_get(int32_t* raw) {
  if (raw == nullptr || !ready || unit == nullptr) {
    return false;
  }
  int value = 0;
  if (pcnt_unit_get_count(unit, &value) != ESP_OK) {
    return false;
  }
  *raw = value;
  return true;
}

bool encoder_set_glitch(uint32_t max_glitch_ns) {
  if (unit == nullptr) {
    return false;
  }
  ready = false;
  pcnt_unit_stop(unit);
  pcnt_unit_disable(unit);
  esp_err_t err = ESP_OK;
  if (max_glitch_ns == 0) {
    err = pcnt_unit_set_glitch_filter(unit, nullptr);
  } else {
    pcnt_glitch_filter_config_t filter = {};
    filter.max_glitch_ns = max_glitch_ns;
    err = pcnt_unit_set_glitch_filter(unit, &filter);
  }
  pulls_off();
  const bool running = restart(true);
  return err == ESP_OK && running;
}

int encoder_a_level() { return digitalRead(ENCODER_A_GPIO) == HIGH ? 1 : 0; }

int encoder_b_level() { return digitalRead(ENCODER_B_GPIO) == HIGH ? 1 : 0; }
