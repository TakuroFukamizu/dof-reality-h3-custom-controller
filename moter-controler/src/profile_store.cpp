#include "profile_store.h"

#include <Preferences.h>

#include <stdio.h>

namespace {

Preferences g_prefs;
bool g_open = false;

void key_for(char* out, size_t size, uint8_t motor, const char* field) {
  snprintf(out, size, "m%u_%s", static_cast<unsigned>(motor), field);
}

}  // namespace

bool profile_store_begin() {
  g_open = g_prefs.begin("h3", false);
  return g_open;
}

uint8_t profile_store_selected_motor() {
  if (!g_open) {
    return 1;
  }
  const uint8_t motor = g_prefs.getUChar("motor", 1);
  return profile_motor_ok(motor) ? motor : 1;
}

void profile_store_select_motor(uint8_t motor) {
  if (!g_open || !profile_motor_ok(motor)) {
    return;
  }
  g_prefs.putUChar("motor", motor);
}

MotorProfile profile_store_load(uint8_t motor) {
  MotorProfile p;
  p.motor = profile_motor_ok(motor) ? motor : 1;
  if (!g_open) {
    return p;
  }
  char key[16];
  key_for(key, sizeof(key), p.motor, "stroke");
  p.stroke = g_prefs.getInt(key, 0);
  key_for(key, sizeof(key), p.motor, "neutral");
  p.neutral = g_prefs.getInt(key, 0);
  key_for(key, sizeof(key), p.motor, "cal");
  p.calibrated = g_prefs.getUChar(key, 0) != 0;
  key_for(key, sizeof(key), p.motor, "sign");
  const int8_t sign = g_prefs.getChar(key, 1);
  p.sign = sign == -1 ? -1 : 1;
  return p;
}

bool profile_store_save(const MotorProfile& p) {
  if (!g_open || !profile_motor_ok(p.motor)) {
    return false;
  }
  char key[16];
  key_for(key, sizeof(key), p.motor, "stroke");
  if (g_prefs.putInt(key, p.stroke) == 0) return false;
  key_for(key, sizeof(key), p.motor, "neutral");
  if (g_prefs.putInt(key, p.neutral) == 0) return false;
  key_for(key, sizeof(key), p.motor, "cal");
  if (g_prefs.putUChar(key, p.calibrated ? 1 : 0) == 0) return false;
  key_for(key, sizeof(key), p.motor, "sign");
  if (g_prefs.putChar(key, p.sign) == 0) return false;
  return true;
}
