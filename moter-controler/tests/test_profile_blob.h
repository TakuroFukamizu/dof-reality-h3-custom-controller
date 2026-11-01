// profile_blob.h のホストテスト（Issue #15 M6）。test_host.cpp から include する（CHECK マクロは test_host.cpp のもの）。
#pragma once

#include <cstring>

#include "profile_blob.h"

namespace profile_blob_test {

inline MotorProfile sample() {
  MotorProfile p;
  p.motor = 2;
  p.stroke = 5462;
  p.neutral = 2731;
  p.sign = -1;
  p.calibrated = true;
  return p;
}

inline void test_roundtrip_and_layout() {
  const MotorProfile p = sample();
  uint8_t buf[profile_blob::SIZE];
  profile_blob::encode(p, 7, buf);
  CHECK(buf[0] == profile_blob::VERSION);
  CHECK(buf[1] == 2);
  CHECK(static_cast<int8_t>(buf[2]) == -1);
  CHECK(buf[3] == 1);
  CHECK(buf[4] == (5462 & 0xFF) && buf[5] == (5462 >> 8) && buf[6] == 0 && buf[7] == 0);
  CHECK(buf[12] == 7 && buf[13] == 0);
  MotorProfile q;
  uint16_t seq = 0;
  CHECK(profile_blob::decode(buf, sizeof(buf), &q, &seq));
  CHECK(q.motor == 2 && q.stroke == 5462 && q.neutral == 2731 && q.sign == -1 && q.calibrated);
  CHECK(seq == 7);
  // 未校正の既定値も往復する
  MotorProfile d;
  d.motor = 3;
  profile_blob::encode(d, 0, buf);
  MotorProfile e;
  e.motor = 1;
  CHECK(profile_blob::decode(buf, sizeof(buf), &e));
  CHECK(e.motor == 3 && e.stroke == 0 && e.neutral == 0 && e.sign == 1 && !e.calibrated);
}

// 途中で切れた・ビットが化けた・版が違う・値域外: どれも「壊れている」として拒否し、out を触らない
inline void test_torn_or_corrupt_blob_is_rejected() {
  const MotorProfile p = sample();
  uint8_t buf[profile_blob::SIZE];
  profile_blob::encode(p, 1, buf);
  MotorProfile out;
  out.motor = 9;  // 触られていないことの目印
  // 長さ違い
  CHECK(!profile_blob::decode(buf, profile_blob::SIZE - 1, &out));
  CHECK(!profile_blob::decode(buf, profile_blob::SIZE + 1, &out));
  CHECK(!profile_blob::decode(buf, 0, &out));
  CHECK(!profile_blob::decode(nullptr, profile_blob::SIZE, &out));
  // sign だけ化けた（旧形式で起きた「行程は新・sign は旧」に相当）→ CRC で弾く
  uint8_t t[profile_blob::SIZE];
  memcpy(t, buf, sizeof(t));
  t[2] = static_cast<uint8_t>(1);
  CHECK(!profile_blob::decode(t, sizeof(t), &out));
  // 1 bit 化け
  memcpy(t, buf, sizeof(t));
  t[5] ^= 0x01;
  CHECK(!profile_blob::decode(t, sizeof(t), &out));
  // CRC 自体の化け
  memcpy(t, buf, sizeof(t));
  t[15] ^= 0x80;
  CHECK(!profile_blob::decode(t, sizeof(t), &out));
  // 版違い（CRC は合わせ直す）
  memcpy(t, buf, sizeof(t));
  t[0] = profile_blob::VERSION + 1;
  uint16_t crc = v1::crc16(t, 14);
  t[14] = static_cast<uint8_t>(crc);
  t[15] = static_cast<uint8_t>(crc >> 8);
  CHECK(!profile_blob::decode(t, sizeof(t), &out));
  // 値域外（motor 0 / sign 0 / calibrated 2 / stroke 負）。CRC は合わせ直す
  const struct { size_t at; uint8_t v; } bad[] = {{1, 0}, {1, 4}, {2, 0}, {2, 2}, {3, 2}, {7, 0x80}};
  for (const auto& b : bad) {
    memcpy(t, buf, sizeof(t));
    t[b.at] = b.v;
    crc = v1::crc16(t, 14);
    t[14] = static_cast<uint8_t>(crc);
    t[15] = static_cast<uint8_t>(crc >> 8);
    CHECK(!profile_blob::decode(t, sizeof(t), &out));
  }
  CHECK(out.motor == 9);
  // 全ゼロ（消去直後の NVS 領域）
  memset(t, 0, sizeof(t));
  CHECK(!profile_blob::decode(t, sizeof(t), &out));
}

inline void run_all() {
  test_roundtrip_and_layout();
  test_torn_or_corrupt_blob_is_rejected();
}

}  // namespace profile_blob_test
