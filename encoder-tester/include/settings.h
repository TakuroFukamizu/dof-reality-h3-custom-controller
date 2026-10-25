#pragma once

// 操作者が DISARMED 中に入れる上限。初期値は未設定で、推測値は入れない。
// ここにある上下限は入力検査であり、推奨する運転値ではない。

#include "drive_logic.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct OperatorSettings {
  int max_duty_percent;
  int max_run_ms;
  int max_travel_counts;
  int power_settle_ms;
  int brake_release_wait_ms;
  int brake_apply_wait_ms;
  int stop_speed_threshold;
  int stop_confirm_ms;
  int stop_timeout_ms;
  int ready_hold_ms;
  int start_grace_ms;
  int max_count_per_s;
  int ui_timeout_ms;
  int lease_ms;
  int jog_stale_ms;
  int jog_duty;
};

struct SettingsResult {
  bool ok;
  const char* error;
  bool wrote_limit;
  bool wrote_sign;
  bool wrote_glitch;
  int sign;
  int glitch_ns;
};

inline OperatorSettings settings_unconfigured() {
  OperatorSettings settings;
  settings.max_duty_percent = -1;
  settings.max_run_ms = -1;
  settings.max_travel_counts = -1;
  settings.power_settle_ms = -1;
  settings.brake_release_wait_ms = -1;
  settings.brake_apply_wait_ms = -1;
  settings.stop_speed_threshold = -1;
  settings.stop_confirm_ms = -1;
  settings.stop_timeout_ms = -1;
  settings.ready_hold_ms = -1;
  settings.start_grace_ms = -1;
  settings.max_count_per_s = -1;
  settings.ui_timeout_ms = -1;
  settings.lease_ms = -1;
  settings.jog_stale_ms = -1;
  settings.jog_duty = -1;
  return settings;
}

inline bool settings_parse_i32(const char* text, int* out) {
  if (text == nullptr || text[0] == '\0' || out == nullptr) {
    return false;
  }
  char* end = nullptr;
  const long value = strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }
  if (value > 2147483647L || value < -2147483647L) {
    return false;
  }
  *out = static_cast<int>(value);
  return true;
}

inline bool settings_in(int value, int lo, int hi) { return value >= lo && value <= hi; }

inline bool settings_stop_pair_ok(const OperatorSettings& settings) {
  if (settings.stop_confirm_ms < 0 || settings.stop_timeout_ms < 0) {
    return true;
  }
  return settings.stop_timeout_ms > settings.stop_confirm_ms;
}

inline SettingsResult settings_fail(const char* error) {
  SettingsResult result{};
  result.ok = false;
  result.error = error;
  return result;
}

inline bool settings_store(int* field, int value, int lo, int hi, const OperatorSettings& settings,
                           SettingsResult* result) {
  const int previous = *field;
  if (!settings_in(value, lo, hi)) {
    *result = settings_fail("range");
    return false;
  }
  *field = value;
  if (!settings_stop_pair_ok(settings)) {
    *field = previous;
    *result = settings_fail("stop_timeout");
    return false;
  }
  result->ok = true;
  result->error = "";
  result->wrote_limit = true;
  return true;
}

inline SettingsResult settings_apply(OperatorSettings* settings, const char* key, const char* value) {
  if (settings == nullptr || key == nullptr || value == nullptr) {
    return settings_fail("set_arg");
  }
  int number = 0;
  if (!settings_parse_i32(value, &number)) {
    return settings_fail("set_arg");
  }
  SettingsResult result{};
  if (strcmp(key, "MAX_DUTY_PERCENT") == 0) {
    settings_store(&settings->max_duty_percent, number, 1, 100, *settings, &result);
  } else if (strcmp(key, "MAX_RUN_MS") == 0) {
    settings_store(&settings->max_run_ms, number, 1, 3600000, *settings, &result);
  } else if (strcmp(key, "MAX_TRAVEL_COUNTS") == 0) {
    settings_store(&settings->max_travel_counts, number, 1, 1000000000, *settings, &result);
  } else if (strcmp(key, "POWER_SETTLE_MS") == 0) {
    settings_store(&settings->power_settle_ms, number, 0, 600000, *settings, &result);
  } else if (strcmp(key, "BRAKE_RELEASE_WAIT_MS") == 0) {
    settings_store(&settings->brake_release_wait_ms, number, 0, 600000, *settings, &result);
  } else if (strcmp(key, "BRAKE_APPLY_WAIT_MS") == 0) {
    settings_store(&settings->brake_apply_wait_ms, number, 0, 600000, *settings, &result);
  } else if (strcmp(key, "STOP_SPEED_THRESHOLD") == 0) {
    settings_store(&settings->stop_speed_threshold, number, 0, 100000000, *settings, &result);
  } else if (strcmp(key, "STOP_CONFIRM_MS") == 0) {
    settings_store(&settings->stop_confirm_ms, number, 0, 600000, *settings, &result);
  } else if (strcmp(key, "STOP_TIMEOUT_MS") == 0) {
    settings_store(&settings->stop_timeout_ms, number, 1, 600000, *settings, &result);
  } else if (strcmp(key, "READY_HOLD_MS") == 0) {
    settings_store(&settings->ready_hold_ms, number, 1, 3600000, *settings, &result);
  } else if (strcmp(key, "START_GRACE_MS") == 0) {
    settings_store(&settings->start_grace_ms, number, 0, 600000, *settings, &result);
  } else if (strcmp(key, "MAX_COUNT_PER_S") == 0) {
    settings_store(&settings->max_count_per_s, number, 1, 2000000000, *settings, &result);
  } else if (strcmp(key, "UI_TIMEOUT_MS") == 0) {
    settings_store(&settings->ui_timeout_ms, number, 1, 60000, *settings, &result);
  } else if (strcmp(key, "LEASE_MS") == 0) {
    settings_store(&settings->lease_ms, number, 1, 60000, *settings, &result);
  } else if (strcmp(key, "JOG_STALE_MS") == 0) {
    settings_store(&settings->jog_stale_ms, number, 1, 5000, *settings, &result);
  } else if (strcmp(key, "JOG_DUTY") == 0) {
    if (!settings_in(number, 1, 100)) {
      return settings_fail("range");
    }
    settings->jog_duty = number;
    result.ok = true;
    result.error = "";
    result.wrote_limit = true;
  } else if (strcmp(key, "ENCODER_SIGN") == 0) {
    if (number != 1 && number != -1) {
      return settings_fail("range");
    }
    result.ok = true;
    result.error = "";
    result.wrote_sign = true;
    result.sign = number;
  } else if (strcmp(key, "GLITCH_NS") == 0) {
    if (!settings_in(number, 0, 1000000)) {
      return settings_fail("range");
    }
    result.ok = true;
    result.error = "";
    result.wrote_glitch = true;
    result.glitch_ns = number;
  } else {
    return settings_fail("unknown_key");
  }
  return result;
}

inline bool settings_limits_ok(const OperatorSettings& settings) {
  if (!settings_in(settings.max_duty_percent, 1, 100)) return false;
  if (!settings_in(settings.max_run_ms, 1, 3600000)) return false;
  if (!settings_in(settings.max_travel_counts, 1, 1000000000)) return false;
  if (!settings_in(settings.power_settle_ms, 0, 600000)) return false;
  if (!settings_in(settings.brake_release_wait_ms, 0, 600000)) return false;
  if (!settings_in(settings.brake_apply_wait_ms, 0, 600000)) return false;
  if (!settings_in(settings.stop_speed_threshold, 0, 100000000)) return false;
  if (!settings_in(settings.stop_confirm_ms, 0, 600000)) return false;
  if (!settings_in(settings.stop_timeout_ms, 1, 600000)) return false;
  if (!(settings.stop_timeout_ms > settings.stop_confirm_ms)) return false;
  if (!settings_in(settings.ready_hold_ms, 1, 3600000)) return false;
  if (!settings_in(settings.start_grace_ms, 0, 600000)) return false;
  if (!settings_in(settings.max_count_per_s, 1, 2000000000)) return false;
  if (!settings_in(settings.ui_timeout_ms, 1, 60000)) return false;
  if (!settings_in(settings.lease_ms, 1, 60000)) return false;
  if (!settings_in(settings.jog_stale_ms, 1, 5000)) return false;
  return true;
}

inline void settings_append(char* buf, int size, int* used, bool* any, const char* name) {
  if (buf == nullptr || size <= 1 || used == nullptr || any == nullptr || name == nullptr) {
    return;
  }
  const int len = static_cast<int>(strlen(name));
  const int extra = *any ? 1 : 0;
  if (*used + extra + len + 1 >= size) {
    return;
  }
  if (*any) {
    buf[(*used)++] = ',';
  }
  memcpy(buf + *used, name, static_cast<size_t>(len));
  *used += len;
  buf[*used] = '\0';
  *any = true;
}

inline void settings_missing(const OperatorSettings& settings, char* buf, int size) {
  if (buf == nullptr || size <= 0) {
    return;
  }
  buf[0] = '\0';
  int used = 0;
  bool any = false;
  if (settings.max_duty_percent < 0) settings_append(buf, size, &used, &any, "MAX_DUTY_PERCENT");
  if (settings.max_run_ms < 0) settings_append(buf, size, &used, &any, "MAX_RUN_MS");
  if (settings.max_travel_counts < 0) settings_append(buf, size, &used, &any, "MAX_TRAVEL_COUNTS");
  if (settings.power_settle_ms < 0) settings_append(buf, size, &used, &any, "POWER_SETTLE_MS");
  if (settings.brake_release_wait_ms < 0) settings_append(buf, size, &used, &any, "BRAKE_RELEASE_WAIT_MS");
  if (settings.brake_apply_wait_ms < 0) settings_append(buf, size, &used, &any, "BRAKE_APPLY_WAIT_MS");
  if (settings.stop_speed_threshold < 0) settings_append(buf, size, &used, &any, "STOP_SPEED_THRESHOLD");
  if (settings.stop_confirm_ms < 0) settings_append(buf, size, &used, &any, "STOP_CONFIRM_MS");
  if (settings.stop_timeout_ms < 0) settings_append(buf, size, &used, &any, "STOP_TIMEOUT_MS");
  if (settings.ready_hold_ms < 0) settings_append(buf, size, &used, &any, "READY_HOLD_MS");
  if (settings.start_grace_ms < 0) settings_append(buf, size, &used, &any, "START_GRACE_MS");
  if (settings.max_count_per_s < 0) settings_append(buf, size, &used, &any, "MAX_COUNT_PER_S");
  if (settings.ui_timeout_ms < 0) settings_append(buf, size, &used, &any, "UI_TIMEOUT_MS");
  if (settings.lease_ms < 0) settings_append(buf, size, &used, &any, "LEASE_MS");
  if (settings.jog_stale_ms < 0) settings_append(buf, size, &used, &any, "JOG_STALE_MS");
}

inline void settings_to_drive(const OperatorSettings& settings, bool electrical_ok, DriveConfig* out) {
  if (out == nullptr) {
    return;
  }
  *out = DriveConfig{};
  out->electrical_ok = electrical_ok;
  out->limits_ok = settings_limits_ok(settings);
  out->max_duty_percent = settings.max_duty_percent;
  out->max_run_ms = settings.max_run_ms;
  out->max_travel_counts = settings.max_travel_counts;
  out->power_settle_ms = settings.power_settle_ms;
  out->brake_release_wait_ms = settings.brake_release_wait_ms;
  out->brake_apply_wait_ms = settings.brake_apply_wait_ms;
  out->stop_speed_threshold = settings.stop_speed_threshold;
  out->stop_confirm_ms = settings.stop_confirm_ms;
  out->stop_timeout_ms = settings.stop_timeout_ms;
  out->ready_hold_ms = settings.ready_hold_ms;
  out->start_grace_ms = settings.start_grace_ms;
  out->max_count_per_s = settings.max_count_per_s;
}
