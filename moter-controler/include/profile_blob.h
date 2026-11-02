#pragma once

// 系統プロファイルの NVS 保存形式（Issue #15 M6）。GPIO・NVS を持たない純粋な符号化・復号（ホストテストの対象）。
//
// 旧形式は m{n}_stroke / m{n}_neutral / m{n}_cal / m{n}_sign の 4 キーで、途中のキーで putXxx が失敗すると
// 前のキーだけ新しく後ろが古いまま（例: 行程は新・sign は旧）の「混ざったプロファイル」が残り、再起動後に逆向きに動いた。
// 新形式は 1 キー m{n}_prof に 16 byte の blob を 1 回の putBytes で書く（NVS は 1 エントリを丸ごと書いてから旧エントリを
// 消すので、途中で切れても旧 blob が残る）。読み手は長さ・版・CRC・値域で検査し、通らなければ「壊れている」として
// 既定値（未校正）に落とす（旧キーには戻らない: 旧キーも混ざっている可能性があり、黙って古い sign で動くのが問題だった）。
// 旧キーだけがある個体（この版より前に校正した 3 台）は旧キーから読む（profile_store.cpp）。
//
// 配置（LE）: 0 version / 1 motor / 2 sign(i8) / 3 calibrated / 4..7 stroke(i32) / 8..11 neutral(i32) / 12..13 seq(u16) /
//            14..15 crc16（CRC-16/CCITT-FALSE、v1_codec.h と同じ。0..13 に対して）
// seq は保存のたびに +1（読み手は使わない。どの保存が残ったかを後から見分けるため）。

#include <stddef.h>
#include <stdint.h>

#include "motor_profile.h"
#include "v1_codec.h"  // v1::crc16（純粋、ヘッダのみ）

namespace profile_blob {

constexpr uint8_t VERSION = 1;
constexpr size_t SIZE = 16;

inline void put_i32(uint8_t* p, int32_t v) {
  const uint32_t u = static_cast<uint32_t>(v);
  p[0] = static_cast<uint8_t>(u);
  p[1] = static_cast<uint8_t>(u >> 8);
  p[2] = static_cast<uint8_t>(u >> 16);
  p[3] = static_cast<uint8_t>(u >> 24);
}
inline int32_t get_i32(const uint8_t* p) {
  return static_cast<int32_t>(static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                              (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24));
}

inline void encode(const MotorProfile& p, uint16_t seq, uint8_t out[SIZE]) {
  out[0] = VERSION;
  out[1] = p.motor;
  out[2] = static_cast<uint8_t>(p.sign < 0 ? -1 : 1);
  out[3] = p.calibrated ? 1 : 0;
  put_i32(out + 4, p.stroke);
  put_i32(out + 8, p.neutral);
  out[12] = static_cast<uint8_t>(seq);
  out[13] = static_cast<uint8_t>(seq >> 8);
  const uint16_t crc = v1::crc16(out, 14);
  out[14] = static_cast<uint8_t>(crc);
  out[15] = static_cast<uint8_t>(crc >> 8);
}

// 長さ・版・CRC・値域（motor 1..3、sign ±1、calibrated 0/1、stroke・neutral ≥ 0）のどれかが合わなければ false（out は触らない）。
inline bool decode(const uint8_t* data, size_t len, MotorProfile* out, uint16_t* seq = nullptr) {
  if (data == nullptr || len != SIZE) return false;
  if (data[0] != VERSION) return false;
  const uint16_t crc = static_cast<uint16_t>(data[14] | (data[15] << 8));
  if (v1::crc16(data, 14) != crc) return false;
  if (!profile_motor_ok(data[1])) return false;
  const int8_t sign = static_cast<int8_t>(data[2]);
  if (sign != 1 && sign != -1) return false;
  if (data[3] > 1) return false;
  const int32_t stroke = get_i32(data + 4);
  const int32_t neutral = get_i32(data + 8);
  if (stroke < 0 || neutral < 0) return false;
  if (out) {
    out->motor = data[1];
    out->sign = sign;
    out->calibrated = data[3] != 0;
    out->stroke = stroke;
    out->neutral = neutral;
  }
  if (seq) *seq = static_cast<uint16_t>(data[12] | (data[13] << 8));
  return true;
}

}  // namespace profile_blob
