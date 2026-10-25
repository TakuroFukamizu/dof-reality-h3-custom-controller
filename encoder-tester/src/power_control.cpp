#include <M5Unified.h>

#include <driver/gpio.h>

#include "config.h"
#include "power_control.h"

namespace {

bool pin_ready = false;

void drive_pin(int pin, int level) {
  gpio_set_level(static_cast<gpio_num_t>(pin), level);
  gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(pin), level);
}

// Arduino-ESP32 3 は pinMode 前の digitalWrite を捨てる。moter-controler と同じ順にする。
void ensure_pin() {
  if (pin_ready) {
    return;
  }
  pinMode(POWER_GPIO, OUTPUT);
  pin_ready = true;
}

}  // namespace

void power_force_off() { drive_pin(POWER_GPIO, relay_off_level(POWER_ON_LEVEL)); }

void power_begin() {
  ensure_pin();
  digitalWrite(POWER_GPIO, relay_off_level(POWER_ON_LEVEL));
}

void power_apply(bool on_cmd) {
  ensure_pin();
  digitalWrite(POWER_GPIO, on_cmd ? POWER_ON_LEVEL : relay_off_level(POWER_ON_LEVEL));
}

int power_gpio_level() { return digitalRead(POWER_GPIO) == HIGH ? 1 : 0; }
