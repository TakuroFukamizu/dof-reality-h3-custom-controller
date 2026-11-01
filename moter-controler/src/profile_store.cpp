#include "profile_store.h"

#include <Preferences.h>

#include <stdio.h>

#include "profile_blob.h"

namespace {

Preferences g_prefs;
bool g_open = false;
uint16_t g_seq[4] = {0, 0, 0, 0};  // 系統ごとの保存回数（blob の seq。load で読み、save で +1）

void key_for(char* out, size_t size, uint8_t motor, const char* field) {
  snprintf(out, size, "m%u_%s", static_cast<unsigned>(motor), field);
}

// 旧形式（4 キー）。この版より前に校正した個体を再校正させないために読む。新形式の保存後は読まない。
MotorProfile load_legacy(uint8_t motor) {
  MotorProfile p;
  p.motor = motor;
  char key[16];
  key_for(key, sizeof(key), motor, "stroke");
  p.stroke = g_prefs.getInt(key, 0);
  key_for(key, sizeof(key), motor, "neutral");
  p.neutral = g_prefs.getInt(key, 0);
  key_for(key, sizeof(key), motor, "cal");
  p.calibrated = g_prefs.getUChar(key, 0) != 0;
  key_for(key, sizeof(key), motor, "sign");
  const int8_t sign = g_prefs.getChar(key, 1);
  p.sign = sign == -1 ? -1 : 1;
  return p;
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

MotorProfile profile_store_load(uint8_t motor, bool* invalid) {
  if (invalid) *invalid = false;
  MotorProfile p;
  p.motor = profile_motor_ok(motor) ? motor : 1;
  if (!g_open) {
    return p;
  }
  char key[16];
  key_for(key, sizeof(key), p.motor, "prof");
  const size_t n = g_prefs.getBytesLength(key);
  if (n == 0) {
    return load_legacy(p.motor);  // 新形式の保存がまだ無い
  }
  uint8_t buf[profile_blob::SIZE];
  MotorProfile decoded;
  uint16_t seq = 0;
  if (n == profile_blob::SIZE && g_prefs.getBytes(key, buf, sizeof(buf)) == profile_blob::SIZE &&
      profile_blob::decode(buf, sizeof(buf), &decoded, &seq) && decoded.motor == p.motor) {
    g_seq[p.motor] = seq;
    return decoded;
  }
  // 壊れている（長さ・版・CRC・値域・系統番号の不一致）: 既定値（未校正）。旧キーには戻らない（profile_blob.h）
  if (invalid) *invalid = true;
  return p;
}

bool profile_store_save(const MotorProfile& p) {
  if (!g_open || !profile_motor_ok(p.motor)) {
    return false;
  }
  char key[16];
  key_for(key, sizeof(key), p.motor, "prof");
  uint8_t buf[profile_blob::SIZE];
  const uint16_t seq = static_cast<uint16_t>(g_seq[p.motor] + 1);
  profile_blob::encode(p, seq, buf);
  if (g_prefs.putBytes(key, buf, sizeof(buf)) != sizeof(buf)) {
    return false;  // 失敗。旧 blob（あれば）がそのまま残る。呼び出し側は dirty を残して再試行する
  }
  g_seq[p.motor] = seq;
  return true;
}
