#include <M5Unified.h>

#include <driver/gpio.h>

#include "brake_control.h"
#include "config.h"

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
  pinMode(BRAKE_GPIO, OUTPUT);
  pin_ready = true;
}

}  // namespace

void brake_force_off() { drive_pin(BRAKE_GPIO, relay_off_level(BRAKE_RELEASE_LEVEL)); }

void brake_begin() {
  ensure_pin();
  digitalWrite(BRAKE_GPIO, relay_off_level(BRAKE_RELEASE_LEVEL));
}

void brake_apply(bool release_cmd) {
  ensure_pin();
  digitalWrite(BRAKE_GPIO, release_cmd ? BRAKE_RELEASE_LEVEL : relay_off_level(BRAKE_RELEASE_LEVEL));
}

int brake_gpio_level() { return digitalRead(BRAKE_GPIO) == HIGH ? 1 : 0; }
