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
constexpr uint32_t SETTLE_WAIT_MS = 5600;  // 区間の始まりから最初の解除まで ≥ 5000 ms（V1_DIAG_POWER_SETTLE_MS）+ 余り

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
  bool keepalive = true;   // 500 ms ごとに HELLO_REQ を送る（道具と同じ）。止めると 2 s で区間が閉じる（待機・電源待ちの間）

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
  d.diag_data.session_flags = v1::diag_session::ACTIVE | v1::diag_session::SETTLED;
  d.diag_data.session_ms = 6543;
  d.diag_data.samples[0] = v1::DiagSample{1000, -5, -200, 4095, 3};
  d.diag_data.samples[1] = v1::DiagSample{2000, 123456, 250, 0, 2};
  uint8_t wire[v1::MAX_WIRE_LEN];
  const size_t wn = v1::encode_mcu_wire(d, wire, sizeof(wire));
  CHECK(wn > 0 && wn <= v1::MAX_WIRE_LEN);
  v1::McuMsg db;
  CHECK(v1::decode_mcu_encoded(wire, wn - 1, &db));
  CHECK(db.type == v1::msg::DIAG_DATA && db.diag_data.total == 551 && db.diag_data.n == 2);
  CHECK(db.diag_data.session_flags == (v1::diag_session::ACTIVE | v1::diag_session::SETTLED));
  CHECK(db.diag_data.session_ms == 6543);
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
  r.diag_result.session_ms_at_release = 5005;
  const size_t rn = v1::encode_mcu_wire(r, wire, sizeof(wire));
  v1::McuMsg rb;
  CHECK(v1::decode_mcu_encoded(wire, rn - 1, &rb));
  CHECK(rb.diag_result.t_engage_us == v1::DIAG_TIME_NONE && rb.diag_result.disp_at_engage == -42 &&
        rb.diag_result.sign == -1 && rb.diag_result.drive_pwm_permille == -250 &&
        rb.diag_result.session_ms_at_release == 5005);
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
  {  // 余裕なし: soft ±2700、余裕 100 → [−2600, 2600]。SAG は両側に d_max + 止まるまでの見積もり 225（v = 0、指摘 5）
    Rig g(2320);
    Harness::rejected(g.sag(150, 60), rs::INVALID_PARAM);  // 2320 + 60 + 225 = 2605 > 2600
    Harness::accepted(g.sag(150, 55));                     // 2320 + 55 + 225 = 2600
  }
  {  // 下側も同じ
    Rig g(-2320);
    Harness::rejected(g.sag(150, 60), rs::INVALID_PARAM);
    Harness::accepted(g.sag(150, 55));
  }
  {  // ENGAGE: 走行の向きに D_run + 見積もり（v_target: 0.3·v + 225）、反対側に 100 + 225
    Rig g(2000);
    Harness::rejected(g.engage(1, 200, 400, 800, 50), rs::INVALID_PARAM);  // 2000 + 400 + 240 + 225 > 2600
    Harness::accepted(g.engage(-1, 200, 400, 800, 50));                    // 2000 − 865 ≥ −2600、2000 + 325 ≤ 2600
  }
  {  // ENGAGE の境目: 1735 + 865 = 2600 は受理、1736 は拒否
    Rig g(1736);
    Harness::rejected(g.engage(1, 200, 400, 800, 50), rs::INVALID_PARAM);
    Rig k(1735);
    Harness::accepted(k.engage(1, 200, 400, 800, 50));
  }
  {  // ENGAGE の反対側: 2276 + 325 > 2600 は拒否（走る向きの余裕は足りている）
    Rig g(2276);
    Harness::rejected(g.engage(-1, 200, 100, 300, 0), rs::INVALID_PARAM);
    Rig k(2275);
    Harness::accepted(k.engage(-1, 200, 100, 300, 0));
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
  // 主電源待ち（区間の始まりから ≥ 5 s。ARMING の待ち（テストの組み立ては 300 ms）には従わない）の手前ではまだ解除しない
  g.run(4990);
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
  CHECK(r.session_ms_at_release >= 5000 && r.session_ms_at_release <= 5010);
  CHECK((r.flags & v1::diag_flags::SESSION_KEPT) != 0);
  // 終わった後は待機（DIAG・主電源 ON・ブレーキ作動・PWM 0）。区間は閉じない
  CHECK(g.h.ax.state() == st::DIAG);
  CHECK(g.h.ax.power_on_cmd());
  CHECK(!g.h.ax.brake_release_cmd());
  CHECK(g.h.ax.duty_permille() == 0);
  CHECK(g.h.ax.referenced());
  CHECK(g.h.ax.diag_busy() && !g.h.ax.diag_fast());
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
  // 同じ区間の次の計測は電源待ちなしで始まる
  Harness::accepted(g.sag(150, 60));
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, 10));
  // 区間を閉じる（記録中の中止 → 後の記録 → DISARMED・主電源 OFF）
  Harness::accepted(g.cancel());
  CHECK(g.run_until([&] { return g.h.ax.state() == st::DISARMED; }, 1000));
  CHECK(!g.h.ax.power_on_cmd() && !g.h.ax.diag_busy());
}

inline void test_diag_sag_d_max_stops_early() {
  Rig g;
  g.sag_count_s = 1000.0;
  Harness::accepted(g.sag(500, 60));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::D_MAX);
  CHECK(r.disp_at_engage >= 60 && r.disp_at_engage <= 61);
  CHECK(r.t_engage_us >= 59 * MS && r.t_engage_us <= 61 * MS);
  CHECK(r.v_at_engage_count_s >= 950 && r.v_at_engage_count_s <= 1050);
  CHECK(r.disp_engage_to_settle >= 28 && r.disp_engage_to_settle <= 32);  // 1000 count/s × 30 ms 滑る
}

inline void test_diag_sag_end_margin_uses_lookahead() {
  // hi = 2700 − 100 = 2600。開始 2315（d_max 60 + 225 で余裕ちょうど）、上向き 1500 count/s → 解除直後（速度の 10 ms 窓が
  // 揃う前）の短い履歴の速度でも 0.3 × 1500 + 225 先を見て、すぐ止める（指摘 5: 最初の 10 ms も見る）
  Rig g(2315);
  g.sag_count_s = 1500.0;
  Harness::accepted(g.sag(500, 60));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::END_MARGIN);
  CHECK(r.t_engage_us <= 3 * MS);
  CHECK(r.disp_at_engage <= 5);  // d_max（60）よりずっと手前
  CHECK(g.h.ax.state() == st::DIAG);  // 作動後の滑り（45 count）は端の余裕の内側: 待機に残る
}

// 止まるまでの見積もりは |v|·0.3 s + ½·5000·0.3²（225）。一定速度 200 ms の旧見積もり（+40）では D_MAX まで走る条件で、
// 新しい見積もりは手前で止める
inline void test_diag_end_margin_projection_includes_gravity() {
  Rig g(2270);
  g.sag_count_s = 200.0;
  g.grip_ms = 5;
  Harness::accepted(g.sag(500, 100));  // 2270 + 100 + 225 = 2595 ≤ 2600
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  const v1::DiagResult r = g.results().back();
  // 新: 2270 + d + 0.3 × 200 + 225 > 2600 → d > 45 で END_MARGIN。旧（200 ms 一定速度、+40）なら d_max（100）まで走っていた
  CHECK(r.stop_reason == v1::diag_stop::END_MARGIN);
  CHECK(r.disp_at_engage >= 40 && r.disp_at_engage <= 47);
  CHECK(r.result == v1::diag_result::OK);
}

// --- ENGAGE ---
inline void test_diag_engage_v_target() {
  Rig g(0, -1);  // 符号 −1: カウントを増やす MD13S の Duty は +
  Harness::accepted(g.engage(1, 200, 400, 800, 50));
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
  CHECK(g.h.ax.duty_permille() == 0);  // 解除から 50 ms は PWM 0
  CHECK(g.run_until([&] { return g.h.ax.duty_permille() != 0; }, 60));
  CHECK(g.h.ax.duty_permille() == 200);       // −sign × (+200)
  CHECK(g.h.ax.pwm_permille() == 200);        // カウント増の向き
  CHECK(g.run_until([&] { return g.done(); }, 8000));
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
  CHECK(g.h.ax.state() == st::DIAG && !g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);  // 待機
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
  CHECK(g.run_until([&] { return g.h.ax.duty_permille() != 0; }, SETTLE_WAIT_MS));
  CHECK(g.h.ax.duty_permille() == 200);   // −(+1) × (−200)
  CHECK(g.h.ax.pwm_permille() == -200);
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::D_RUN);
  CHECK(r.t_drive_us == 0);  // 待ち 0 は解除と同時
  CHECK(r.disp_at_engage <= -400 && r.disp_at_engage >= -402);
}

inline void test_diag_engage_run_timeout_fits_buffer() {
  Rig g;
  g.count_s_per_percent = 5.0;  // 20 % で 100 count/s（遅い）: 300 ms 走っても 30 count
  Harness::accepted(g.engage(1, 200, 600, 3000, 200));  // 待ち 200 + 走行 300 + 後 500 = 最長の記録
  CHECK(g.run_until([&] { return g.done(); }, 8000));
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
  CHECK(g.run_until([&] { return g.h.ax.state() == st::FAULT; }, 6200));
  // ラッチした周期にブレーキ作動・PWM 0
  CHECK(!g.h.ax.brake_release_cmd());
  CHECK(g.h.ax.duty_permille() == 0);
  CHECK(g.h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(!g.h.ax.referenced());  // 基準はラッチした周期に消える（FAULT_RESET を待たない）
  CHECK(!g.h.ax.power_on_cmd());
  CHECK(!g.done());  // 作動後の記録を続けている
  CHECK(g.run_until([&] { return g.done(); }, 1000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.result == v1::diag_result::FAULT);
  CHECK(r.stop_reason == v1::diag_stop::NO_PULSE);
  CHECK(r.fault_code == fc::ENCODER_FAULT);
  CHECK(r.referenced_after == 0);
  CHECK(r.t_engage_us - r.t_drive_us >= 150 * MS && r.t_engage_us - r.t_drive_us <= 151 * MS);
  CHECK((r.flags & v1::diag_flags::SESSION_KEPT) == 0);
  CHECK(g.h.ax.state() == st::FAULT);  // 記録が終わっても FAULT のまま
  CHECK(!g.h.ax.diag_busy());
  Harness::accepted(g.h.fault_reset());
  CHECK(!g.h.ax.referenced());
  Harness::rejected(g.sag(150, 60), rs::NOT_REFERENCED);  // 校正し直すまで計測できない
}

// 指摘 1: 走り始めは数えた（≥ 3 count）が、その後エンコーダが止まった。旧判定（走り始めからの変位）では 300 ms 走って OK で終わり
// 基準も残っていた。動きの途絶（1 count も変わらない 150 ms）で止め、ENCODER_FAULT・基準を消す
inline void test_diag_engage_encoder_freezes_after_moving() {
  Rig g;
  g.count_s_per_percent = 5.0;  // 20 % で 100 count/s（v_target・D_run に届かない）
  Harness::accepted(g.engage(1, 200, 600, 3000, 0));
  CHECK(g.run_until([&] { return g.h.ax.duty_permille() != 0; }, SETTLE_WAIT_MS));
  g.run(40);  // 約 4 count 動く
  CHECK(g.h.raw >= 3);
  g.stall = true;  // エンコーダが止まる（機構は動き続ける）
  const uint64_t frozen_at = g.h.now;
  CHECK(g.run_until([&] { return g.h.ax.state() == st::FAULT; }, 400));
  CHECK(g.h.now - frozen_at <= 160 * MS);  // 最後の変化から 150 ms（+ 1 count の間隔）
  CHECK(g.h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(!g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);
  CHECK(!g.h.ax.referenced());
  CHECK(g.run_until([&] { return g.done(); }, 1000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.result == v1::diag_result::FAULT);
  CHECK(r.stop_reason == v1::diag_stop::NO_PULSE);
  CHECK(r.referenced_after == 0);
  CHECK(r.t_engage_us - r.t_drive_us < 250 * MS);  // RUN_TIMEOUT（300 ms）より前
}

// ゆっくりでも 1 count ずつ変わっていれば無パルスにしない（150 ms 窓の内側に変化がある）
inline void test_diag_engage_slow_motion_is_not_no_pulse() {
  Rig g;
  g.count_s_per_percent = 0.5;  // 20 % で 10 count/s（100 ms に 1 count）
  Harness::accepted(g.engage(1, 200, 600, 3000, 0));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  const v1::DiagResult r = g.results().back();
  // 走り始めから 150 ms で 3 count に届かない → 無パルス（±1 のちらつきを除く判定は従来どおり）
  CHECK(r.stop_reason == v1::diag_stop::NO_PULSE);
  Rig k;
  k.count_s_per_percent = 2.0;  // 20 % で 40 count/s（25 ms に 1 count、150 ms で 6 count）
  Harness::accepted(k.engage(1, 200, 600, 3000, 0));
  CHECK(k.run_until([&] { return k.done(); }, 8000));
  CHECK(k.results().back().stop_reason == v1::diag_stop::RUN_TIMEOUT);
  CHECK(k.results().back().result == v1::diag_result::OK);
  CHECK(k.h.ax.referenced());
}

inline void test_diag_engage_reverse_stops() {
  Rig g;
  g.count_s_per_percent = 5.0;  // 20 % で +100 count/s
  g.sag_count_s = -500.0;       // 荷重に負けて −400 count/s
  Harness::accepted(g.engage(1, 200, 400, 800, 0));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::REVERSE);
  CHECK(r.disp_at_engage <= -20 && r.disp_at_engage >= -22);
  CHECK(r.result == v1::diag_result::OK);
  CHECK(g.h.ax.state() == st::DIAG && g.h.ax.referenced());  // 荷重に負けた動きはエンコーダの故障ではない: 基準を残して待機
}

inline void test_diag_engage_sag_guard_before_drive() {
  Rig g;
  g.sag_count_s = -1000.0;  // 待ち 200 ms の間に 100 count 沈む
  Harness::accepted(g.engage(1, 200, 400, 800, 200));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
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
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
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
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
    g.h.host_fault();
    CHECK(g.h.ax.state() == st::FAULT && !g.h.ax.brake_release_cmd());
    CHECK(g.run_until([&] { return g.done(); }, 1000));
    CHECK(g.results().back().fault_code == fc::HOST_FAULT);
  }
  {  // 制御周期の逸脱（DIAG は止まっている状態ではないので見る）
    Rig g;
    Harness::accepted(g.sag(500, 200));
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
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
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
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
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
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
    g.keepalive = false;
    Harness::accepted(g.power_hold(60000));
    CHECK(g.run_until([&] { return g.done(); }, 8000));
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

// DIAG_READ（offset 範囲外）で区間の状態だけを読む
inline v1::DiagData read_session(Rig& g) {
  v1::HostMsg m;
  memset(&m, 0, sizeof(m));
  m.type = v1::msg::DIAG_READ;
  m.diag_read.request_id = 4242;
  m.diag_read.offset = 0xFFFF;
  const std::vector<v1::McuMsg> r = g.h.send(m);
  v1::DiagData d;
  memset(&d, 0, sizeof(d));
  for (const v1::McuMsg& x : r) {
    if (x.type == v1::msg::DIAG_DATA) d = x.diag_data;
  }
  CHECK(d.request_id == 4242 && d.n == 0);
  return d;
}

// 指摘 2: 電源担当軸の POWER_HOLD は、区間の始まりから 5 s で SETTLED を名乗る（PC はこれを見てから他の軸を始める）。
// 待機中の POWER_HOLD は期限を延ばすだけ（区間の始まり・SETTLED はそのまま）
inline void test_diag_power_hold_reports_settled_and_refreshes() {
  Rig g;
  CHECK(read_session(g).session_flags == 0);  // 区間の外
  Harness::accepted(g.power_hold(8000));
  v1::DiagData d = read_session(g);
  CHECK(d.session_flags == (v1::diag_session::ACTIVE | v1::diag_session::POWER_CMD));
  g.run(4990);
  d = read_session(g);
  CHECK((d.session_flags & v1::diag_session::SETTLED) == 0);
  CHECK(d.session_ms >= 4980 && d.session_ms <= 5000);
  g.run(20);
  d = read_session(g);
  CHECK((d.session_flags & v1::diag_session::SETTLED) != 0);
  CHECK((d.session_flags & v1::diag_session::TEST_BUSY) == 0);
  // 期限 8 s の手前で延ばす → 8 s を過ぎても主電源 ON のまま
  g.run(2000);
  Harness::accepted(g.power_hold(8000));
  g.run(3000);
  CHECK(g.h.ax.state() == st::DIAG && g.h.ax.power_on_cmd());
  CHECK(read_session(g).session_ms >= 10000);
  CHECK(g.results().empty());
  CHECK(g.run_until([&] { return g.done(); }, 6000));
  CHECK(g.results().back().mode == v1::diag_mode::POWER_HOLD && g.results().back().result == v1::diag_result::OK);
  CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.power_on_cmd());
  CHECK(read_session(g).session_flags == 0);
}

// 指摘 2: 区間の最初の解除は ≥ 5 s。ARMING の待ち（tuning.power_settle_ms）や cal_power_settle_ms を短くしても縮まない
inline void test_diag_settle_is_at_least_5s() {
  Rig g;
  g.h.ax.tuning.power_settle_ms = 100;
  g.h.ax.tuning.cal_power_settle_ms = 1000;
  Harness::accepted(g.sag(150, 60));
  const uint64_t t_start = g.h.now;
  CHECK(read_session(g).session_flags == (v1::diag_session::ACTIVE | v1::diag_session::POWER_CMD | v1::diag_session::TEST_BUSY));
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
  CHECK(g.h.now - t_start >= 5000 * MS);
  CHECK(g.h.now - t_start <= 5010 * MS);
  CHECK(g.run_until([&] { return g.done(); }, 1000));
  CHECK(g.results().back().session_ms_at_release >= 5000);
}

// 指摘 2: 同じ区間の計測は主電源を入れたまま続く（2 回目以降は電源待ちなし）。受信が 2 s 途絶えたら待機から区間を閉じる
inline void test_diag_session_keeps_power_between_runs() {
  Rig g;
  g.sag_count_s = -50.0;
  Harness::accepted(g.sag(150, 60));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  bool power_dropped = false;
  for (int run = 0; run < 3; ++run) {
    g.h.out.clear();
    Harness::accepted(g.sag(150, 60));
    const uint64_t t_start = g.h.now;
    uint64_t t_release = 0;
    for (uint32_t i = 0; i < 2000 && !g.done(); ++i) {
      g.ms1();
      if (!g.h.ax.power_on_cmd()) power_dropped = true;
      if (g.h.ax.brake_release_cmd() && t_release == 0) t_release = g.h.now;
    }
    CHECK(t_release != 0 && t_release - t_start <= 10 * MS);  // 電源待ちなしで解除
    CHECK(g.done());
    const v1::DiagResult r = g.results().back();
    CHECK(r.result == v1::diag_result::OK && (r.flags & v1::diag_flags::SESSION_KEPT) != 0);
    CHECK(r.session_ms_at_release > 5000);
  }
  CHECK(!power_dropped);
  CHECK(g.h.ax.state() == st::DIAG && g.h.ax.power_on_cmd());
  // ENGAGE も同じ区間で受ける
  Harness::accepted(g.engage(1, 150, 200, 600, 0));
  CHECK(g.run_until([&] { return g.h.ax.duty_permille() != 0; }, 10));
  CHECK(g.run_until([&] { return g.results().back().mode == v1::diag_mode::ENGAGE; }, 2000));
  // 道具が落ちた: 待機で 2 s 受信が無ければ区間を閉じる（主電源 OFF）。POWER_HOLD の結果ではないので DIAG_RESULT は出ない
  g.keepalive = false;
  const size_t results_before = g.results().size();
  const uint64_t silent_from = g.h.ax.last_rx_us();
  CHECK(g.run_until([&] { return g.h.ax.state() == st::DISARMED; }, 3000));
  CHECK(g.h.now - silent_from > 2000 * MS && g.h.now - silent_from <= 2010 * MS);
  CHECK(!g.h.ax.power_on_cmd() && !g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);
  CHECK(g.results().size() == results_before);
  // 閉じた後の計測は新しい区間（また 5 s 待つ）
  g.keepalive = true;
  Harness::accepted(g.sag(150, 60));
  g.run(1000);
  CHECK(!g.h.ax.brake_release_cmd());
}

// 待機の期限（120 s）でも区間を閉じる
inline void test_diag_session_idle_deadline() {
  Rig g;
  Harness::accepted(g.sag(150, 60));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  CHECK(g.h.ax.state() == st::DIAG);
  g.run(V1_DIAG_SESSION_IDLE_MS - 1000);
  CHECK(g.h.ax.state() == st::DIAG && g.h.ax.power_on_cmd());
  CHECK(g.run_until([&] { return g.h.ax.state() == st::DISARMED; }, 2000));
  CHECK(!g.h.ax.power_on_cmd());
}

// 電源待ちの間の受信途絶は区間を閉じる（解除しない）。記録中の途絶では止めない（自分の時計で終わる。ドキュメント §2）
inline void test_diag_rx_silence_during_settle_and_recording() {
  {
    Rig g;
    g.keepalive = false;
    Harness::accepted(g.sag(150, 60));
    CHECK(g.run_until([&] { return g.done(); }, SETTLE_WAIT_MS));
    const v1::DiagResult r = g.results().back();
    CHECK(r.result == v1::diag_result::CANCELLED && r.stop_reason == v1::diag_stop::CANCEL && r.sample_count == 0);
    CHECK(g.h.now <= 2010 * MS);
    CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.power_on_cmd());
  }
  {
    Rig g;
    g.sag_count_s = -50.0;
    Harness::accepted(g.sag(500, 200));
    CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
    g.keepalive = false;  // 解除の直後に道具が落ちた
    CHECK(g.run_until([&] { return g.done(); }, 2000));
    const v1::DiagResult r = g.results().back();
    CHECK(r.result == v1::diag_result::OK && r.stop_reason == v1::diag_stop::T_MAX);  // 記録は最後まで
    CHECK(r.sample_count >= 899);
    CHECK(g.run_until([&] { return g.h.ax.state() == st::DISARMED; }, 2500));  // その後の待機で閉じる
    CHECK(!g.h.ax.power_on_cmd());
  }
}

// 指摘 3: 受理から解除まで（電源待ち 5 s）の間に位置が変わり余裕が無くなった → 解除せず ABORTED / ROOM_LOST（記録なし）
inline void test_diag_room_rechecked_before_release() {
  Rig g(2300);
  Harness::accepted(g.sag(150, 60));  // 2300 + 285 = 2585 ≤ 2600
  g.run(3000);
  g.pos += 40.0;  // 電源待ちの間に（ブレーキ作動のまま）40 count 動いた: 2340 + 285 > 2600
  g.h.raw = llround(g.pos);
  bool released = false;
  for (uint32_t i = 0; i < 3000 && !g.done(); ++i) {
    g.ms1();
    if (g.h.ax.brake_release_cmd()) released = true;
  }
  CHECK(!released);
  CHECK(g.done());
  const v1::DiagResult r = g.results().back();
  CHECK(r.result == v1::diag_result::ABORTED);
  CHECK(r.stop_reason == v1::diag_stop::ROOM_LOST);
  CHECK(r.sample_count == 0);
  CHECK(r.start_count == 2340);
  CHECK(r.t_engage_us == v1::DIAG_TIME_NONE);
  // 何も動かしていないので待機に残る。余裕のある値なら同じ区間で続けられる
  CHECK((r.flags & v1::diag_flags::SESSION_KEPT) != 0);
  CHECK(g.h.ax.state() == st::DIAG && g.h.ax.referenced());
  Harness::rejected(g.sag(150, 60), rs::INVALID_PARAM);  // 受理の時点でも拒否
  Harness::accepted(g.sag(150, 30));                     // 2340 + 30 + 225 = 2595
}

// 指摘 5（作動後）: ブレーキが効かず作動後も滑り続けて端の余裕を越えた → POST_MARGIN・ABORTED、待機せず区間を閉じる
inline void test_diag_post_brake_margin_closes_session() {
  Rig g(2300);
  g.sag_count_s = 1000.0;
  g.grip_ms = 2000;  // 作動しても 2 s 滑る
  Harness::accepted(g.sag(500, 60));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  const v1::DiagResult r = g.results().back();
  CHECK(r.stop_reason == v1::diag_stop::END_MARGIN);  // 見積もりで早めに作動は指令した
  CHECK((r.flags & v1::diag_flags::POST_MARGIN) != 0);
  CHECK((r.flags & v1::diag_flags::SESSION_KEPT) == 0);
  CHECK(r.result == v1::diag_result::ABORTED);
  CHECK(r.start_count + r.final_disp > 2600);
  CHECK(g.h.ax.state() == st::DISARMED && !g.h.ax.power_on_cmd());
}

// 指摘 4: 記録が満杯になったら、その周期にブレーキ作動・PWM 0 にして不成功（ABORTED・TRUNCATED・BUFFER_FULL）で終わる。
// 制御タスクの刻みが 1 ms より細かくなった（≒ 時刻の異常）ときに起こりうる。純粋なクラスを 100 us ごとに回して確かめる
inline void test_diag_buffer_exhaustion_brakes_and_fails() {
  static V1DiagBrake d;  // 記録 16 KB: スタックに置かない
  v1::DiagStart p;
  memset(&p, 0, sizeof(p));
  p.request_id = 55;
  p.mode = v1::diag_mode::ENGAGE;
  p.dir = 1;
  p.duty_permille = 200;
  p.d_max_count = 600;
  p.v_target_count_s = 3000;
  p.delay_ms = 0;
  uint64_t now = 1000;
  d.open_session(now, 5000);
  d.start(p, now, -2600, 2600, 100000);
  CHECK(d.phase() == V1DiagBrake::Phase::Settle);
  now += 5000 * MS;
  d.step(now, 0, 0, true);
  CHECK(d.phase() == V1DiagBrake::Phase::Drive && d.brake_release() && d.pwm_permille() == 200);
  CHECK(d.sample_count() == 1);
  int32_t c = 0;
  size_t steps = 0;
  while (d.phase() == V1DiagBrake::Phase::Drive && steps < 5000) {
    now += 100;  // 100 us ごと（1 ms ごとに 1 count 進む = 1000 count/s）
    steps += 1;
    if (steps % 10 == 0) c += 1;
    d.step(now, c, 0, true);
  }
  CHECK(steps == V1_DIAG_MAX_SAMPLES);  // 1024 点目の次の周期で満杯
  CHECK(!d.brake_release());
  CHECK(d.pwm_permille() == 0);
  v1::DiagResult r;
  CHECK(d.take_result(&r));
  CHECK(r.result == v1::diag_result::ABORTED);
  CHECK(r.stop_reason == v1::diag_stop::BUFFER_FULL);
  CHECK((r.flags & v1::diag_flags::TRUNCATED) != 0);
  CHECK(r.sample_count == V1_DIAG_MAX_SAMPLES);
  CHECK(r.t_engage_us != v1::DIAG_TIME_NONE);
  CHECK(d.phase() == V1DiagBrake::Phase::Hold);  // ブレーキ作動・PWM 0 の待機（PC は不成功を見て止める）
  // 作動後の記録の途中で満杯になっても同じく終わる（ブレーキは作動済み）
  p.request_id = 56;
  p.mode = v1::diag_mode::SAG;
  p.t_max_ms = 50;
  p.d_max_count = 200;
  d.start(p, now, -2600, 2600, 100000);
  d.step(now, c, 0, true);
  CHECK(d.phase() == V1DiagBrake::Phase::Sag);
  steps = 0;
  while (d.busy() && d.phase() != V1DiagBrake::Phase::Hold && steps < 20000) {
    now += 100;
    steps += 1;
    d.step(now, c, 0, true);
  }
  CHECK(d.take_result(&r));
  CHECK(r.result == v1::diag_result::ABORTED && r.stop_reason == v1::diag_stop::T_MAX);
  CHECK((r.flags & v1::diag_flags::TRUNCATED) != 0);
  CHECK(!d.brake_release() && d.pwm_permille() == 0);
}

// 待機中の異常（画面の非常停止）: その場で主電源 OFF・区間を閉じる。解除後の計測は新しい区間（また 5 s 待つ）
inline void test_diag_fault_in_idle_closes_session() {
  Rig g;
  Harness::accepted(g.sag(150, 60));
  CHECK(g.run_until([&] { return g.done(); }, 8000));
  CHECK(g.h.ax.state() == st::DIAG && g.h.ax.power_on_cmd());
  const size_t before = g.results().size();
  g.h.ax.set_local_estop(true, g.h.now);
  g.h.drain();
  CHECK(g.h.ax.state() == st::FAULT && g.h.ax.fault_code() == fc::LOCAL_ESTOP);
  CHECK(!g.h.ax.power_on_cmd() && !g.h.ax.brake_release_cmd() && g.h.ax.duty_permille() == 0);
  CHECK(!g.h.ax.diag_busy());
  g.run(10);
  CHECK(g.results().size() == before);  // 計測の結果は出済み。POWER_HOLD でもないので新しい DIAG_RESULT は無い
  g.h.ax.set_local_estop(false, g.h.now);
  Harness::accepted(g.h.fault_reset());
  CHECK(g.h.ax.referenced());  // LOCAL_ESTOP は基準を消さない
  Harness::accepted(g.sag(150, 60));
  const uint64_t t_start = g.h.now;
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
  CHECK(g.h.now - t_start >= 5000 * MS);
}

// 電源担当でない軸: 主電源は指令しない（他の軸の POWER_HOLD に頼る）が、待ちとブレーキの手順は同じ
inline void test_diag_non_power_axis_does_not_drive_relay() {
  Rig g(0, 1, 0);
  Harness::accepted(g.sag(150, 60));
  g.run(100);
  CHECK(!g.h.ax.power_on_cmd());
  CHECK(g.run_until([&] { return g.h.ax.brake_release_cmd(); }, SETTLE_WAIT_MS));
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
  test_diag_end_margin_projection_includes_gravity();
  test_diag_engage_v_target();
  test_diag_engage_d_run_and_direction();
  test_diag_engage_run_timeout_fits_buffer();
  test_diag_engage_no_pulse_latches_encoder_fault();
  test_diag_engage_encoder_freezes_after_moving();
  test_diag_engage_slow_motion_is_not_no_pulse();
  test_diag_engage_reverse_stops();
  test_diag_engage_sag_guard_before_drive();
  test_diag_estop_mid_sag_brakes_same_cycle();
  test_diag_host_fault_and_overrun_abort();
  test_diag_cancel_and_disarm();
  test_diag_blocks_other_commands();
  test_diag_power_hold();
  test_diag_power_hold_reports_settled_and_refreshes();
  test_diag_settle_is_at_least_5s();
  test_diag_session_keeps_power_between_runs();
  test_diag_session_idle_deadline();
  test_diag_rx_silence_during_settle_and_recording();
  test_diag_room_rechecked_before_release();
  test_diag_post_brake_margin_closes_session();
  test_diag_buffer_exhaustion_brakes_and_fails();
  test_diag_fault_in_idle_closes_session();
  test_diag_non_power_axis_does_not_drive_relay();
}

#endif  // V1_DIAG_BRAKE_TEST

}  // namespace v1diag_test
