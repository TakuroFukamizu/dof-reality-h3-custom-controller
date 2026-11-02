// 計測専用の DIAG（v1_diag_brake.h と V1Axis の統合）のホストテスト。test_host.cpp から include する。
// -DV1_DIAG_BRAKE_TEST=1 の組み立て（run_tests.sh の 4 本目）で中身を検証し、量産の組み立て（既定 0）では
// 「DIAG の種別がコンパイルされておらず、受けても rx_error になるだけ」を検証する。
#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

#include "test_v1_axis.h"
#include "v1_axis.h"
#include "v1_codec.h"

namespace v1diag_test {

using v1axis_test::Harness;
namespace st = v1::state;
namespace fc = v1::fault;
namespace ar = v1::ack_result;
namespace rs = v1::ack_reason;

#if !V1_DIAG_BRAKE_TEST

// 量産の組み立て: DIAG_START（0x40）・DIAG_READ（0x41）は未知の種別。復号できず rx_error を数えるだけで、何も答えない。
inline void test_diag_compiled_out_in_production() {
  CHECK(v1::host_payload_len(0x40) == -1);
  CHECK(v1::host_payload_len(0x41) == -1);
  CHECK(v1::mcu_payload_len(0xC0) == -1);
  CHECK(v1::mcu_payload_len(0xC1) == -1);
  Harness h;
  h.config();
  uint8_t payload[20];
  memset(payload, 0, sizeof(payload));
  payload[0] = 7;   // request_id = 7
  payload[4] = 1;   // mode = SAG
  payload[8] = 150; // t_max_ms
  payload[10] = 60; // d_max_count
  uint8_t frame[64];
  const size_t fn = v1::frame_build(0x40, payload, sizeof(payload), frame, sizeof(frame));
  CHECK(fn > 0);
  uint8_t enc[80];
  const size_t en = v1::cobs_encode(frame, fn, enc, sizeof(enc));
  const uint16_t before = h.ax.rx_error_count();
  const size_t out_before = h.out.size();
  h.ax.handle_frame(enc, en, h.now);
  h.drain();
  CHECK(h.ax.rx_error_count() == before + 1);
  CHECK(h.out.size() == out_before);  // ACK も STATUS も無い
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(!h.ax.brake_release_cmd());
  CHECK(h.ax.duty_permille() == 0);
  // HELLO は DIAG_BRAKE を名乗らない
  const std::vector<v1::McuMsg> r = h.hello(5);
  bool saw = false;
  for (const v1::McuMsg& m : r) {
    if (m.type == v1::msg::HELLO) {
      saw = true;
      CHECK((m.hello.capabilities & (1u << 8)) == 0);
    }
  }
  CHECK(saw);
}

inline void run_all() { test_diag_compiled_out_in_production(); }

#else  // V1_DIAG_BRAKE_TEST

constexpr uint64_t MS = 1000;

// 1 ms 刻みの模擬（制御タスクと同じ呼び方: 5 ms ごとに tick → diag_busy なら diag_step、間の 1 ms は diag_fast のときだけ diag_step）。
// 機構: ブレーキ解除指令中は、PWM 0 なら sag_count_s で沈み、PWM ≠ 0 なら pwm_permille / 10 × count_s_per_percent + sag で動く。
// ブレーキ作動指令の後 grip_ms は直前の速度のまま滑り、その後止まる。stall = エンコーダが数えない。
struct Rig {
  Harness h;
  double pos = 0.0;       // 機構の位置（count、小数）
  double vel = 0.0;       // 直前の速度
  double sag_count_s = 0.0;
  double count_s_per_percent = 100.0;
  uint32_t grip_ms = 30;
  bool stall = false;
  bool was_released = false;
  uint64_t engage_cmd_us = 0;
  uint32_t sub = 0;
  uint16_t adc = 1234;
  uint64_t last_hello_us = 0;
  bool keepalive = false;  // 500 ms ごとに HELLO_REQ を送る（道具と同じ）

  explicit Rig(int32_t start = 0, int8_t sign = 1, uint8_t role = v1::role_flags::POWER_CONTROLLER) : h(1, sign, 0, true) {
    v1::Config c = v1axis_test::base_config();
    c.role_flags = role;
    Harness::accepted(h.config_with(c));
    // 基準を start に取り直す（raw 0 = actual start）
    Harness::accepted(h.set_reference(start));
    pos = static_cast<double>(h.raw);
  }

  void ms1() {
    h.now += MS;
    const bool released = h.ax.brake_release_cmd();
    if (released) {
      vel = h.ax.pwm_permille() / 10.0 * count_s_per_percent + sag_count_s;
    } else {
      if (was_released) engage_cmd_us = h.now - MS;  // 作動指令は前の周期の終わりに出た
      if (engage_cmd_us == 0 || h.now - engage_cmd_us >= static_cast<uint64_t>(grip_ms) * MS) vel = 0.0;
    }
    was_released = released;
    pos += vel / 1000.0;
    if (!stall) h.raw = static_cast<int64_t>(llround(pos));
    if (keepalive && h.now - last_hello_us >= 500 * MS) {
      last_hello_us = h.now;
      h.hello(99);
    }
    sub = (sub + 1) % 5;
    if (sub == 0) {
      h.ax.tick(h.now, h.raw);
      if (h.ax.diag_busy()) h.ax.diag_step(h.now, h.raw, adc);
    } else if (h.ax.diag_fast()) {
      h.ax.diag_step(h.now, h.raw, adc);
    }
    h.drain();
  }
  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; ++i) ms1();
  }
  template <typename Pred>
  bool run_until(Pred pred, uint32_t max_ms) {
    for (uint32_t i = 0; i < max_ms; ++i) {
      if (pred()) return true;
      ms1();
    }
    return pred();
  }

  v1::HostMsg start_msg(uint8_t mode) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::DIAG_START;
    m.diag_start.request_id = h.next_rid();
    m.diag_start.mode = mode;
    return m;
  }
  v1::Ack sag(uint16_t t_max, uint16_t d_max) {
    v1::HostMsg m = start_msg(v1::diag_mode::SAG);
    m.diag_start.t_max_ms = t_max;
    m.diag_start.d_max_count = d_max;
    return h.cmd(m);
  }
  v1::Ack engage(int8_t dir, uint16_t duty, uint16_t d_run, uint16_t v_target, uint16_t delay) {
    v1::HostMsg m = start_msg(v1::diag_mode::ENGAGE);
    m.diag_start.dir = dir;
    m.diag_start.duty_permille = duty;
    m.diag_start.d_max_count = d_run;
    m.diag_start.v_target_count_s = v_target;
    m.diag_start.delay_ms = delay;
    return h.cmd(m);
  }
  v1::Ack power_hold(uint32_t hold_ms) {
    v1::HostMsg m = start_msg(v1::diag_mode::POWER_HOLD);
    m.diag_start.hold_ms = hold_ms;
    return h.cmd(m);
  }
  v1::Ack cancel() { return h.cmd(start_msg(v1::diag_mode::CANCEL)); }

  std::vector<v1::DiagResult> results() const {
    std::vector<v1::DiagResult> r;
    for (const v1::McuMsg& m : h.out) {
      if (m.type == v1::msg::DIAG_RESULT) r.push_back(m.diag_result);
    }
    return r;
  }
  bool done() const { return !results().empty(); }

  // DIAG_READ で記録を全部読む
  std::vector<v1::DiagSample> read_all(uint16_t* total_out = nullptr) {
    std::vector<v1::DiagSample> all;
    uint16_t offset = 0;
    for (int guard = 0; guard < 200; ++guard) {
      v1::HostMsg m;
      memset(&m, 0, sizeof(m));
      m.type = v1::msg::DIAG_READ;
      m.diag_read.request_id = 1000u + static_cast<uint32_t>(guard);
      m.diag_read.offset = offset;
      const std::vector<v1::McuMsg> r = h.send(m);
      const v1::DiagData* d = nullptr;
      for (const v1::McuMsg& x : r) {
        if (x.type == v1::msg::DIAG_DATA) d = &x.diag_data;
      }
      CHECK(d != nullptr);
      if (!d) break;
      CHECK(d->request_id == m.diag_read.request_id);
      CHECK(d->offset == offset);
      if (total_out) *total_out = d->total;
      if (d->n == 0) break;
      for (size_t i = 0; i < d->n; ++i) all.push_back(d->samples[i]);
      offset = static_cast<uint16_t>(offset + d->n);
    }
    return all;
  }
};

// --- 符号化 ---
inline void test_diag_codec_roundtrip() {
  v1::HostMsg m;
  memset(&m, 0, sizeof(m));
  m.type = v1::msg::DIAG_START;
  m.diag_start.request_id = 0x01020304;
  m.diag_start.mode = v1::diag_mode::ENGAGE;
  m.diag_start.dir = -1;
  m.diag_start.duty_permille = 200;
  m.diag_start.t_max_ms = 150;
  m.diag_start.d_max_count = 400;
  m.diag_start.v_target_count_s = 800;
  m.diag_start.delay_ms = 50;
  m.diag_start.hold_ms = 60000;
  uint8_t frame[v1::MAX_FRAME_LEN];
  const size_t fn = v1::encode_host_frame(m, frame, sizeof(frame));
  CHECK(fn == 4 + 20 + 2);
  // Python 側（tools/diag_brake/v1mini.py）のテストと同じバイト列
  const uint8_t expect[] = {0x01, 0x40, 0x14, 0x00, 0x04, 0x03, 0x02, 0x01, 0x02, 0xFF, 0xC8, 0x00, 0x96,
                            0x00, 0x90, 0x01, 0x20, 0x03, 0x32, 0x00, 0x60, 0xEA, 0x00, 0x00};
  CHECK(fn >= sizeof(expect) && memcmp(frame, expect, sizeof(expect)) == 0);
  v1::HostMsg back;
  CHECK(v1::decode_host_frame(frame, fn, &back));
  CHECK(back.type == v1::msg::DIAG_START && back.diag_start.dir == -1 && back.diag_start.d_max_count == 400 &&
        back.diag_start.hold_ms == 60000 && back.diag_start.v_target_count_s == 800);

  v1::McuMsg d;
  memset(&d, 0, sizeof(d));
  d.type = v1::msg::DIAG_DATA;
  d.diag_data.request_id = 9;
  d.diag_data.test_id = 77;
  d.diag_data.offset = 32;
  d.diag_data.total = 551;
  d.diag_data.n = 2;
  d.diag_data.samples[0] = v1::DiagSample{1000, -5, -200, 4095, 3};
  d.diag_data.samples[1] = v1::DiagSample{2000, 123456, 250, 0, 2};
  uint8_t wire[v1::MAX_WIRE_LEN];
  const size_t wn = v1::encode_mcu_wire(d, wire, sizeof(wire));
  CHECK(wn > 0 && wn <= v1::MAX_WIRE_LEN);
  v1::McuMsg db;
  CHECK(v1::decode_mcu_encoded(wire, wn - 1, &db));
  CHECK(db.type == v1::msg::DIAG_DATA && db.diag_data.total == 551 && db.diag_data.n == 2);
  CHECK(db.diag_data.samples[0].count == -5 && db.diag_data.samples[0].pwm_permille == -200 &&
        db.diag_data.samples[0].adc_raw == 4095 && db.diag_data.samples[0].flags == 3);
  CHECK(db.diag_data.samples[1].t_us == 2000 && db.diag_data.samples[1].count == 123456);

  v1::McuMsg r;
  memset(&r, 0, sizeof(r));
  r.type = v1::msg::DIAG_RESULT;
  r.diag_result.test_id = 77;
  r.diag_result.t_engage_us = v1::DIAG_TIME_NONE;
  r.diag_result.disp_at_engage = -42;
  r.diag_result.sign = -1;
  r.diag_result.drive_pwm_permille = -250;
  const size_t rn = v1::encode_mcu_wire(r, wire, sizeof(wire));
  v1::McuMsg rb;
  CHECK(v1::decode_mcu_encoded(wire, rn - 1, &rb));
  CHECK(rb.diag_result.t_engage_us == v1::DIAG_TIME_NONE && rb.diag_result.disp_at_engage == -42 &&
        rb.diag_result.sign == -1 && rb.diag_result.drive_pwm_permille == -250);
}

inline void test_diag_hello_names_capability() {
  Harness h;
  const std::vector<v1::McuMsg> r = h.hello(5);
  bool saw = false;
  for (const v1::McuMsg& m : r) {
    if (m.type == v1::msg::HELLO) {
      saw = true;
      CHECK((m.hello.capabilities & v1::capabilities::DIAG_BRAKE) != 0);
    }
  }
  CHECK(saw);
}

// --- 拒否（上限・状態） ---
inline void test_diag_rejects_params_over_caps() {
  Rig g;
  Harness::rejected(g.sag(0, 60), rs::INVALID_PARAM);
  Harness::rejected(g.sag(501, 60), rs::INVALID_PARAM);
  Harness::rejected(g.sag(150, 0), rs::INVALID_PARAM);
  Harness::rejected(g.sag(150, 201), rs::INVALID_PARAM);
  Harness::rejected(g.engage(0, 200, 400, 800, 50), rs::INVALID_PARAM);    // 向き
  Harness::rejected(g.engage(1, 99, 400, 800, 50), rs::INVALID_PARAM);     // Duty 下限
  Harness::rejected(g.engage(1, 251, 400, 800, 50), rs::INVALID_PARAM);    // Duty 上限 25 %
  Harness::rejected(g.engage(1, 200, 601, 800, 50), rs::INVALID_PARAM);    // D_run 上限
  Harness::rejected(g.engage(1, 200, 0, 800, 50), rs::INVALID_PARAM);
  Harness::rejected(g.engage(1, 200, 400, 0, 50), rs::INVALID_PARAM);
  Harness::rejected(g.engage(1, 200, 400, 3001, 50), rs::INVALID_PARAM);
  Harness::rejected(g.engage(1, 200, 400, 800, 201), rs::INVALID_PARAM);   // 待ちの上限
  Harness::rejected(g.power_hold(0), rs::INVALID_PARAM);
  Harness::rejected(g.power_hold(120001), rs::INVALID_PARAM);
  v1::HostMsg m = g.start_msg(4);
  Harness::rejected(g.h.cmd(m), rs::INVALID_PARAM);
  CHECK(g.h.ax.state() == st::DISARMED);
  // 上限ちょうどは受理
  Harness::accepted(g.sag(500, 200));
  CHECK(g.h.ax.state() == st::DIAG);
}

inline void test_diag_rejects_bad_state() {
  {  // CONFIG なし
    Harness h;
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::DIAG_START;
    m.diag_start.request_id = h.next_rid();
    m.diag_start.mode = v1::diag_mode::SAG;
    m.diag_start.t_max_ms = 150;
    m.diag_start.d_max_count = 60;
    Harness::rejected(h.cmd(m), rs::NOT_CONFIGURED);
  }
  {  // 基準なし
    Harness h(1, 1, 0, false);
    Harness::accepted(h.config());
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::DIAG_START;
    m.diag_start.request_id = h.next_rid();
    m.diag_start.mode = v1::diag_mode::SAG;
    m.diag_start.t_max_ms = 150;
    m.diag_start.d_max_count = 60;
    Harness::rejected(h.cmd(m), rs::NOT_REFERENCED);
  }
  {  // DISARMED 以外（HOLDING）
    Rig g;
    Harness::accepted(g.h.arm());
    Harness::rejected(g.sag(150, 60), rs::BAD_STATE);
  }
  {  // FAULT（ローカル非常停止）
    Rig g;
    g.h.ax.set_local_estop(true, g.h.now);
    g.h.drain();
    Harness::rejected(g.sag(150, 60), rs::FAULT_ACTIVE);
  }
  {  // ソフトリミット外
    Rig g(2800);
    Harness::rejected(g.sag(150, 60), rs::BAD_STATE);
  }
  {  // 余裕なし: soft ±2700、余裕 100 → [−2600, 2600]。SAG は両側に d_max
    Rig g(2550);
    Harness::rejected(g.sag(150, 60), rs::INVALID_PARAM);
    Harness::accepted(g.sag(150, 50));
  }
  {  // ENGAGE: 走行の向きに D_run + 300、反対側に 200
    Rig g(2000);
    Harness::rejected(g.engage(1, 200, 400, 800, 50), rs::INVALID_PARAM);  // 2000 + 700 > 2600
    Harness::accepted(g.engage(-1, 200, 400, 800, 50));                    // 2000 − 700 ≥ −2600、2000 + 200 ≤ 2600
  }
  {  // POWER_HOLD は電源担当軸だけ
    Rig g(0, 1, 0);
    Harness::rejected(g.power_hold(5000), rs::INVALID_PARAM);
  }
}

// --- SAG ---
inline void test_diag_sag_t_max_full_sequence() {
  Rig g;
  g.sag_count_s = -100.0;  // 下向きに 100 count/s
  Harness::accepted(g.sag(150, 60));
  CHECK(g.h.ax.state() == st::DIAG);
  CHECK(g.h.ax.power_on_cmd());          // 電源担当軸: 主電源 ON 指令（ARMING と同じ）
  CHECK(!g.h.ax.brake_release_cmd());    // 電源待ちの間はブレーキ作動のまま
  CHECK(!g.h.ax.diag_fast());
  // 主電源待ち（テストの組み立ては 300 ms）の手前ではまだ解除しない
  g.run(290);
  CHECK(!g.h.ax.brake_release_cmd());
  CHECK(g.h.ax.duty_permille() == 0);
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 20));
  CHECK(g.h.ax.duty_permille() == 0);  // SAG は PWM 0 のまま
  CHECK(g.h.ax.diag_fast());
  CHECK(g.run_until([&] { return g.done(); }, 1000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.result == v1::diag_result::OK);
  CHECK(r.stop_reason == v1::diag_stop::T_MAX);
  CHECK(r.t_drive_us == v1::DIAG_TIME_NONE);
  CHECK(r.t_engage_us >= 150 * MS && r.t_engage_us <= 151 * MS);
  CHECK(r.disp_at_engage <= -14 && r.disp_at_engage >= -16);  // 100 count/s × 150 ms
  CHECK(r.final_disp <= -16 && r.final_disp >= -19);          // 作動後 30 ms 滑る（+3）
  // 100 count/s は静止の判定（20 ms で ±1 = 50 count/s）の 2 倍なので、滑り（30 ms）の終わりの 1 count 手前から静止とみなされうる
  CHECK(r.t_settle_us != v1::DIAG_TIME_NONE && r.t_settle_us >= 5 * MS && r.t_settle_us <= 35 * MS);
  CHECK(r.sample_period_us == 1000);
  CHECK(r.max_sample_gap_us == 1000);  // 1 ms ごとに記録した
  CHECK(r.sample_count >= 548 && r.sample_count <= 553);  // 150 + 400 ms
  CHECK((r.flags & v1::diag_flags::TRUNCATED) == 0);
  CHECK(r.referenced_after == 1);
  CHECK(r.adc_first == 1234);
  CHECK(g.h.ax.state() == st::DISARMED);
  CHECK(!g.h.ax.power_on_cmd());
  CHECK(!g.h.ax.brake_release_cmd());
  CHECK(g.h.ax.duty_permille() == 0);
  CHECK(g.h.ax.referenced());
  CHECK(!g.h.ax.diag_busy());
  // 記録の読み出し: t = 0 の点はブレーキ解除・主電源 ON、作動の点からはブレーキ作動
  uint16_t total = 0;
  const std::vector<v1::DiagSample> s = g.read_all(&total);
  CHECK(s.size() == r.sample_count && total == r.sample_count);
  CHECK(s.front().t_us == 0);
  CHECK((s.front().flags & v1::diag_flags::BRAKE_RELEASE) != 0);
  CHECK((s.front().flags & v1::diag_flags::POWER_ON) != 0);
  CHECK(s.front().count == r.start_count);
  size_t first_engaged = s.size();
  for (size_t i = 0; i < s.size(); ++i) {
    if ((s[i].flags & v1::diag_flags::BRAKE_RELEASE) == 0) {
      first_engaged = i;
      break;
    }
  }
  CHECK(first_engaged < s.size() && s[first_engaged].t_us == r.t_engage_us);
  for (size_t i = 1; i < s.size(); ++i) CHECK(s[i].t_us - s[i - 1].t_us == 1000);
  for (const v1::DiagSample& x : s) CHECK(x.pwm_permille == 0);
  // 範囲外の読み出しは n = 0
  v1::HostMsg m;
  memset(&m, 0, sizeof(m));
  m.type = v1::msg::DIAG_READ;
  m.diag_read.request_id = 5;
  m.diag_read.offset = 5000;
  const std::vector<v1::McuMsg> rr = g.h.send(m);
  CHECK(rr.size() == 1 && rr[0].type == v1::msg::DIAG_DATA && rr[0].diag_data.n == 0 &&
        rr[0].diag_data.total == r.sample_count);
  // 終わった後はもう一度始められる
  Harness::accepted(g.sag(150, 60));
}

inline void test_diag_sag_d_max_stops_early() {
  Rig g;
  g.sag_count_s = 1000.0;
  Harness::accepted(g.sag(500, 60));
  CHECK(g.run_until([&] { return g.done(); }, 2000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::D_MAX);
  CHECK(r.disp_at_engage >= 60 && r.disp_at_engage <= 61);
  CHECK(r.t_engage_us >= 59 * MS && r.t_engage_us <= 61 * MS);
  CHECK(r.v_at_engage_count_s >= 950 && r.v_at_engage_count_s <= 1050);
  CHECK(r.disp_engage_to_settle >= 28 && r.disp_engage_to_settle <= 32);  // 1000 count/s × 30 ms 滑る
}

inline void test_diag_sag_end_margin_uses_lookahead() {
  // hi = 2700 − 100 = 2600。開始 2400（d_max 200 で余裕ちょうど）、上向き 1500 count/s → 先読み 200 ms で 300 先 → すぐ止める
  Rig g(2400);
  g.sag_count_s = 1500.0;
  Harness::accepted(g.sag(500, 200));
  CHECK(g.run_until([&] { return g.done(); }, 2000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::END_MARGIN);
  CHECK(r.disp_at_engage < 30);  // d_max（200）よりずっと手前
  CHECK(g.h.ax.state() == st::DISARMED);
}

// --- ENGAGE ---
inline void test_diag_engage_v_target() {
  Rig g(0, -1);  // 符号 −1: カウントを増やす MD13S の Duty は +
  Harness::accepted(g.engage(1, 200, 400, 800, 50));
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 400));
  CHECK(g.h.ax.duty_permille() == 0);  // 解除から 50 ms は PWM 0
  CHECK(g.run_until([&] { return g.h.ax.duty_permille() != 0; }, 60));
  CHECK(g.h.ax.duty_permille() == 200);       // −sign × (+200)
  CHECK(g.h.ax.pwm_permille() == 200);        // カウント増の向き
  CHECK(g.run_until([&] { return g.done(); }, 2000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.result == v1::diag_result::OK);
  CHECK(r.stop_reason == v1::diag_stop::V_TARGET);
  CHECK(r.t_drive_us >= 50 * MS && r.t_drive_us <= 51 * MS);
  CHECK(r.t_engage_us > r.t_drive_us && r.t_engage_us <= r.t_drive_us + 15 * MS);
  CHECK(r.v_at_engage_count_s >= 800);
  CHECK(r.drive_pwm_permille == 200);
  CHECK(r.sign == -1);
  CHECK(r.disp_engage_to_settle >= 55 && r.disp_engage_to_settle <= 62);  // 2000 count/s × 30 ms
  CHECK(r.t_settle_us >= 28 * MS && r.t_settle_us <= 32 * MS);
  CHECK(r.referenced_after == 1);
  CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);
  // 記録: 走行中は pwm 200、作動の点から 0
  const std::vector<v1::DiagSample> s = g.read_all();
  bool saw_drive = false;
  for (const v1::DiagSample& x : s) {
    if (x.t_us >= r.t_drive_us && x.t_us < r.t_engage_us) {
      CHECK(x.pwm_permille == 200);
      saw_drive = true;
    }
    if (x.t_us >= r.t_engage_us) CHECK(x.pwm_permille == 0 && (x.flags & v1::diag_flags::BRAKE_RELEASE) == 0);
  }
  CHECK(saw_drive);
}

inline void test_diag_engage_d_run_and_direction() {
  Rig g;  // 符号 +1: カウントを減らす MD13S の Duty は +
  Harness::accepted(g.engage(-1, 200, 400, 3000, 0));  // 2000 count/s では 3000 に届かない → D_run
  CHECK(g.run_until([&] { return g.h.ax.duty_permille() != 0; }, 400));
  CHECK(g.h.ax.duty_permille() == 200);   // −(+1) × (−200)
  CHECK(g.h.ax.pwm_permille() == -200);
  CHECK(g.run_until([&] { return g.done(); }, 2000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::D_RUN);
  CHECK(r.t_drive_us == 0);  // 待ち 0 は解除と同時
  CHECK(r.disp_at_engage <= -400 && r.disp_at_engage >= -402);
}

inline void test_diag_engage_run_timeout_fits_buffer() {
  Rig g;
  g.count_s_per_percent = 5.0;  // 20 % で 100 count/s（遅い）: 300 ms 走っても 30 count
  Harness::accepted(g.engage(1, 200, 600, 3000, 200));  // 待ち 200 + 走行 300 + 後 500 = 最長の記録
  CHECK(g.run_until([&] { return g.done(); }, 3000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::RUN_TIMEOUT);
  CHECK(r.t_engage_us - r.t_drive_us >= 300 * MS && r.t_engage_us - r.t_drive_us <= 301 * MS);
  CHECK((r.flags & v1::diag_flags::TRUNCATED) == 0);
  CHECK(r.sample_count <= V1_DIAG_MAX_SAMPLES);
  CHECK(r.sample_count >= 1000);
}

inline void test_diag_engage_no_pulse_latches_encoder_fault() {
  Rig g;
  g.stall = true;
  Harness::accepted(g.engage(1, 200, 400, 800, 0));
  CHECK(g.run_until([&] { return g.h.ax.state() == st::FAULT; }, 1000));
  // ラッチした周期にブレーキ作動・PWM 0
  CHECK(!g.h.ax.brake_release_cmd());
  CHECK(g.h.ax.duty_permille() == 0);
  CHECK(g.h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(!g.done());  // 作動後の記録を続けている
  CHECK(g.run_until([&] { return g.done(); }, 1000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.result == v1::diag_result::FAULT);
  CHECK(r.stop_reason == v1::diag_stop::NO_PULSE);
  CHECK(r.fault_code == fc::ENCODER_FAULT);
  CHECK(r.referenced_after == 0);
  CHECK(r.t_engage_us - r.t_drive_us >= 150 * MS && r.t_engage_us - r.t_drive_us <= 151 * MS);
  CHECK(g.h.ax.state() == st::FAULT);  // 記録が終わっても FAULT のまま
  // 既存の意味: ENCODER_FAULT の解除で基準が消える
  Harness::accepted(g.h.fault_reset());
  CHECK(!g.h.ax.referenced());
}

inline void test_diag_engage_reverse_stops() {
  Rig g;
  g.count_s_per_percent = 5.0;  // 20 % で +100 count/s
  g.sag_count_s = -500.0;       // 荷重に負けて −400 count/s
  Harness::accepted(g.engage(1, 200, 400, 800, 0));
  CHECK(g.run_until([&] { return g.done(); }, 2000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::REVERSE);
  CHECK(r.disp_at_engage <= -20 && r.disp_at_engage >= -22);
  CHECK(r.result == v1::diag_result::OK);
  CHECK(g.h.ax.state() == st::DISARMED && g.h.ax.referenced());
}

inline void test_diag_engage_sag_guard_before_drive() {
  Rig g;
  g.sag_count_s = -1000.0;  // 待ち 200 ms の間に 100 count 沈む
  Harness::accepted(g.engage(1, 200, 400, 800, 200));
  CHECK(g.run_until([&] { return g.done(); }, 2000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::SAG_GUARD);
  CHECK(r.t_drive_us == v1::DIAG_TIME_NONE);
  CHECK(r.disp_at_engage <= -100 && r.disp_at_engage >= -101);
}

// --- 打ち切り ---
inline void test_diag_estop_mid_sag_brakes_same_cycle() {
  Rig g;
  g.sag_count_s = -100.0;
  Harness::accepted(g.sag(500, 200));
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 400));
  g.run(50);
  CHECK(g.h.ax.brake_release_cmd());
  g.h.ax.set_local_estop(true, g.h.now);
  g.h.drain();
  CHECK(g.h.ax.state() == st::FAULT);
  CHECK(g.h.ax.fault_code() == fc::LOCAL_ESTOP);
  CHECK(!g.h.ax.brake_release_cmd());
  CHECK(g.h.ax.duty_permille() == 0);
  CHECK(!g.h.ax.power_on_cmd());
  // 続く記録の周期でも出力は触らない
  g.run(10);
  CHECK(!g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);
  CHECK(g.run_until([&] { return g.done(); }, 1000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.result == v1::diag_result::FAULT);
  CHECK(r.stop_reason == v1::diag_stop::FAULT);
  CHECK(r.fault_code == fc::LOCAL_ESTOP);
  CHECK(r.referenced_after == 1);  // LOCAL_ESTOP は基準を消さない（既存の意味）
  CHECK(r.t_engage_us >= 50 * MS && r.t_engage_us <= 56 * MS);
  // FAULT 中も読み出せる
  const std::vector<v1::DiagSample> s = g.read_all();
  CHECK(s.size() == r.sample_count);
  CHECK(g.h.ax.state() == st::FAULT);
  // 記録が終わるまでは解除しても始められない → 終わった後は解除 → 再開できる
  g.h.ax.set_local_estop(false, g.h.now);
  Harness::accepted(g.h.fault_reset());
  Harness::accepted(g.sag(150, 60));
}

inline void test_diag_host_fault_and_overrun_abort() {
  {
    Rig g;
    Harness::accepted(g.sag(500, 200));
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 400));
    g.h.host_fault();
    CHECK(g.h.ax.state() == st::FAULT && !g.h.ax.brake_release_cmd());
    CHECK(g.run_until([&] { return g.done(); }, 1000));
    CHECK(g.results().back().fault_code == fc::HOST_FAULT);
  }
  {  // 制御周期の逸脱（DIAG は止まっている状態ではないので見る）
    Rig g;
    Harness::accepted(g.sag(500, 200));
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 400));
    g.h.now += 20 * MS;
    g.h.ax.tick(g.h.now, g.h.raw);
    g.h.drain();
    CHECK(g.h.ax.state() == st::FAULT && g.h.ax.fault_code() == fc::CONTROL_OVERRUN);
    CHECK(!g.h.ax.brake_release_cmd());
  }
  {  // 電源待ちの間の異常: 記録なしで即 DIAG_RESULT(FAULT)
    Rig g;
    Harness::accepted(g.sag(150, 60));
    g.run(100);
    g.h.ax.set_local_estop(true, g.h.now);
    g.h.drain();
    CHECK(g.h.ax.state() == st::FAULT && !g.h.ax.power_on_cmd());
    CHECK(g.run_until([&] { return g.done(); }, 10));  // 次の tick で DIAG_RESULT（FAULT_EVENT の後）
    CHECK(g.done() && g.results().back().result == v1::diag_result::FAULT && g.results().back().sample_count == 0);
  }
}

inline void test_diag_cancel_and_disarm() {
  {  // 電源待ちの間の中止: 即 DISARMED、記録なし。ACK の後に DIAG_RESULT
    Rig g;
    Harness::accepted(g.sag(150, 60));
    g.run(100);
    const size_t before = g.h.out.size();
    Harness::accepted(g.cancel());
    CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.power_on_cmd());
    size_t ack_at = 0, res_at = 0;
    for (size_t i = before; i < g.h.out.size(); ++i) {
      if (g.h.out[i].type == v1::msg::ACK && !ack_at) ack_at = i;
      if (g.h.out[i].type == v1::msg::DIAG_RESULT && !res_at) res_at = i;
    }
    CHECK(ack_at && res_at && ack_at < res_at);
    CHECK(g.results().back().result == v1::diag_result::CANCELLED && g.results().back().sample_count == 0);
  }
  {  // 記録中の中止: その周期でブレーキ作動、後の記録を経て CANCELLED
    Rig g;
    g.sag_count_s = -100.0;
    Harness::accepted(g.sag(500, 200));
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 400));
    g.run(40);
    Harness::accepted(g.cancel());
    CHECK(!g.h.ax.brake_release_cmd());
    CHECK(g.h.ax.state() == st::DIAG);  // 後の記録中
    CHECK(g.run_until([&] { return g.done(); }, 1000));
    const v1::DiagResult r = g.results().back();
    CHECK(r.result == v1::diag_result::CANCELLED && r.stop_reason == v1::diag_stop::CANCEL);
    CHECK(r.sample_count > 400);
    CHECK(g.h.ax.state() == st::DISARMED);
  }
  {  // DISARM も中止として扱う
    Rig g;
    Harness::accepted(g.engage(1, 200, 400, 800, 100));
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 400));
    Harness::accepted(g.h.disarm());
    CHECK(!g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);
    CHECK(g.run_until([&] { return g.done(); }, 1000));
    CHECK(g.results().back().result == v1::diag_result::CANCELLED);
  }
}

inline void test_diag_blocks_other_commands() {
  Rig g;
  Harness::accepted(g.sag(500, 200));
  Harness::rejected(g.h.arm(), rs::BAD_STATE);
  Harness::rejected(g.h.calibrate(1), rs::BAD_STATE);
  Harness::rejected(g.h.config(), rs::BAD_STATE);
  Harness::rejected(g.h.set_reference(0), rs::BAD_STATE);
  Harness::rejected(g.sag(150, 60), rs::BAD_STATE);
  CHECK(!g.h.ax.select_axis(2, g.h.now));
  // SET_POSITION は数えずに捨てる
  const uint16_t stale = g.h.ax.stale_drop_count();
  g.h.set_position(100);
  CHECK(g.h.ax.stale_drop_count() == stale);
  CHECK(g.h.ax.state() == st::DIAG);
}

// --- POWER_HOLD ---
inline void test_diag_power_hold() {
  {
    Rig g;
    g.keepalive = true;
    Harness::accepted(g.power_hold(3000));
    CHECK(g.h.ax.state() == st::DIAG && g.h.ax.power_on_cmd());
    g.run(2500);
    CHECK(g.h.ax.power_on_cmd());
    CHECK(!g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);  // ブレーキ作動・PWM 0 のまま
    CHECK(g.run_until([&] { return g.done(); }, 1000));
    CHECK(g.results().back().result == v1::diag_result::OK);
    CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.power_on_cmd());
  }
  {  // 受信が途絶えたら主電源を切る
    Rig g;
    Harness::accepted(g.power_hold(60000));
    CHECK(g.run_until([&] { return g.done(); }, 3000));
    CHECK(g.results().back().result == v1::diag_result::CANCELLED);
    CHECK(g.h.now >= 2000 * MS);
    CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.power_on_cmd());
  }
  {  // 中止
    Rig g;
    g.keepalive = true;
    Harness::accepted(g.power_hold(60000));
    g.run(200);
    Harness::accepted(g.cancel());
    CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.power_on_cmd());
  }
}

// 電源担当でない軸: 主電源は指令しない（他の軸の POWER_HOLD に頼る）が、待ちとブレーキの手順は同じ
inline void test_diag_non_power_axis_does_not_drive_relay() {
  Rig g(0, 1, 0);
  Harness::accepted(g.sag(150, 60));
  g.run(100);
  CHECK(!g.h.ax.power_on_cmd());
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 400));
  CHECK(g.run_until([&] { return g.done(); }, 1000));
  const std::vector<v1::DiagSample> s = g.read_all();
  for (const v1::DiagSample& x : s) CHECK((x.flags & v1::diag_flags::POWER_ON) == 0);
}

inline void run_all() {
  test_diag_codec_roundtrip();
  test_diag_hello_names_capability();
  test_diag_rejects_params_over_caps();
  test_diag_rejects_bad_state();
  test_diag_sag_t_max_full_sequence();
  test_diag_sag_d_max_stops_early();
  test_diag_sag_end_margin_uses_lookahead();
  test_diag_engage_v_target();
  test_diag_engage_d_run_and_direction();
  test_diag_engage_run_timeout_fits_buffer();
  test_diag_engage_no_pulse_latches_encoder_fault();
  test_diag_engage_reverse_stops();
  test_diag_engage_sag_guard_before_drive();
  test_diag_estop_mid_sag_brakes_same_cycle();
  test_diag_host_fault_and_overrun_abort();
  test_diag_cancel_and_disarm();
  test_diag_blocks_other_commands();
  test_diag_power_hold();
  test_diag_non_power_axis_does_not_drive_relay();
}

#endif  // V1_DIAG_BRAKE_TEST

}  // namespace v1diag_test
