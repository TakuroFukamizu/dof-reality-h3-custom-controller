#include <M5Unified.h>

#include <driver/gpio.h>

#include "config.h"
#include "motor_driver.h"

namespace {

int reported_hz = PWM_FREQUENCY_HZ;
bool attached = false;

void drive_pin(int pin, int level) {
  gpio_set_level(static_cast<gpio_num_t>(pin), level);
  gpio_set_direction(static_cast<gpio_num_t>(pin), GPIO_MODE_OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(pin), level);
}

}  // namespace

void motor_force_safe() {
  drive_pin(DIR_GPIO, DIR_POSITIVE_LEVEL);
  drive_pin(PWM_GPIO, 0);
}

bool motor_begin() {
  pinMode(DIR_GPIO, OUTPUT);
  digitalWrite(DIR_GPIO, DIR_POSITIVE_LEVEL);
  attached = ledcAttach(static_cast<uint8_t>(PWM_GPIO), static_cast<uint32_t>(PWM_FREQUENCY_HZ),
                        static_cast<uint8_t>(PWM_RESOLUTION_BITS));
  ledcWrite(static_cast<uint8_t>(PWM_GPIO), 0);
  const uint32_t hz = ledcReadFreq(static_cast<uint8_t>(PWM_GPIO));
  reported_hz = hz > 0 ? static_cast<int>(hz) : PWM_FREQUENCY_HZ;
  return attached;
}

void motor_write(int duty_percent, int direction) {
  digitalWrite(DIR_GPIO, dir_level_for(direction));
  if (attached) {
    ledcWrite(static_cast<uint8_t>(PWM_GPIO), static_cast<uint32_t>(pwm_raw_from_percent(duty_percent)));
  } else {
    digitalWrite(PWM_GPIO, LOW);
  }
}

int motor_pwm_hz() { return reported_hz; }

int motor_dir_level() { return digitalRead(DIR_GPIO) == HIGH ? 1 : 0; }
