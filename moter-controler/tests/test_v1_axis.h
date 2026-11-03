// v1_axis のホストテスト。test_host.cpp から include する（CHECK マクロは test_host.cpp のもの）。
// control-server/tests/sim_axis.rs の主要ケースを C++ に写し、設計 §6 の各項目を確認する。
// 時刻は注入、エンコーダは「setpoint に向かって plant_vel で追従する」簡単なプラントで模擬する。
#pragma once

#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

#include "motor_output.h"
#include "v1_axis.h"

namespace v1axis_test {

namespace st = v1::state;
namespace fc = v1::fault;
namespace ar = v1::ack_result;
namespace rs = v1::ack_reason;
namespace fl = v1::flags;

constexpr uint32_t EPOCH = 3;
constexpr uint64_t MS = 1000;

inline v1::Config base_config(uint32_t epoch = EPOCH) {
  v1::Config c;
  memset(&c, 0, sizeof(c));
  c.session_epoch = epoch;
  c.calibration_hash = 0x12345678;
  c.soft_min_count = -2700;
  c.soft_max_count = 2700;
  c.max_vel_count_s = 4000;
  c.max_acc_count_s2 = 20000;
  c.following_error_limit_count = 200;
  c.overspeed_count_s = 6000;
  c.command_timeout_ms = 100;
  c.stop_timeout_ms = 1500;
  c.role_flags = v1::role_flags::POWER_CONTROLLER;
  return c;
}

struct Harness {
  V1Axis ax;
  uint64_t now = 0;
  uint32_t rid = 100;
  uint32_t seq = 0;
  uint32_t frame = 0;
  int64_t raw = 0;
  bool stall = false;             // プラントが動かない
  uint32_t pos_max_vel = 4000;    // set_position の max_vel_count_s（T のテストで遅い軌道にする）
  double plant_vel = 4000.0;      // count/s
  // 校正（CALIBRATING）の模擬機構: Duty（MD13S の符号付き %）に比例した速度で raw が動き、端 [end_lo, end_hi] で止まる
  // （止まる＝無パルス。AutoMachine の端検出がこれを拾う）。plant_sign = +1 は「Duty + でカウント減」（#1 の実測。
  // P 端 = raw の下限側）。既定の端は stroke 5551 / neutral 2775（CAL_RESULT のテストベクタと同じ値）。
  int8_t plant_sign = 1;
  int64_t end_lo = -1000;
  int64_t end_hi = 4551;
  double cal_count_s_per_percent = 100.0;  // 実測の 1 % ≈ 100〜130 count/s の下側
  std::vector<v1::McuMsg> out;    // これまでの全出力
  std::vector<uint8_t> seen_states;

  explicit Harness(uint8_t axis = 1, int8_t sign = 1, int64_t raw0 = 0, bool reference = true) {
    V1Identity id;
    id.device_id = 0x5131000000000001ull;
    id.boot_id = 2309737967u;
    raw = raw0;
    ax.begin(id, axis, sign, now, raw);
    ax.tick(now, raw);
    drain();
    if (reference) accepted(set_reference(0));
  }

  void drain() {
    v1::McuMsg m;
    while (ax.pop(&m)) {
      out.push_back(m);
      if (m.type == v1::msg::STATUS) {
        if (seen_states.empty() || seen_states.back() != m.status.mcu_state) seen_states.push_back(m.status.mcu_state);
      }
    }
  }

  // 直近の送信を返す（index は drain 後の out の末尾からの位置）
  std::vector<v1::McuMsg> send(const v1::HostMsg& m) {
    const size_t before = out.size();
    ax.handle(m, now);
    drain();
    return std::vector<v1::McuMsg>(out.begin() + static_cast<long>(before), out.end());
  }

  v1::Ack cmd(const v1::HostMsg& m) {
    const std::vector<v1::McuMsg> r = send(m);
    for (const v1::McuMsg& x : r) {
      if (x.type == v1::msg::ACK) return x.ack;
    }
    v1::Ack none;
    memset(&none, 0, sizeof(none));
    none.result = 0xFF;
    return none;
  }

  uint32_t next_rid() { return ++rid; }

  v1::Ack config_with(v1::Config c) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::CONFIG;
    c.request_id = next_rid();
    m.config = c;
    return cmd(m);
  }
  v1::Ack config() { return config_with(base_config()); }
  v1::Ack arm(uint32_t epoch = EPOCH) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::ARM_PREPARE;
    m.arm_prepare.request_id = next_rid();
    m.arm_prepare.session_epoch = epoch;
    return cmd(m);
  }
  v1::Ack start(uint32_t epoch = EPOCH) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::START;
    m.start.request_id = next_rid();
    m.start.session_epoch = epoch;
    return cmd(m);
  }
  v1::Ack stop(uint32_t epoch = EPOCH) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::CONTROLLED_STOP;
    m.controlled_stop.request_id = next_rid();
    m.controlled_stop.session_epoch = epoch;
    m.controlled_stop.reason = 1;
    return cmd(m);
  }
  v1::HostMsg disarm_msg(uint32_t request_id) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::DISARM;
    m.disarm.request_id = request_id;
    return m;
  }
  v1::Ack disarm() { return cmd(disarm_msg(next_rid())); }
  v1::Ack fault_reset() {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::FAULT_RESET;
    m.fault_reset.request_id = next_rid();
    return cmd(m);
  }
  v1::HostMsg host_fault_msg(uint32_t request_id) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::HOST_FAULT;
    m.host_fault.request_id = request_id;
    m.host_fault.fault_code = fc::HOST_FAULT;
    return m;
  }
  v1::Ack host_fault() { return cmd(host_fault_msg(next_rid())); }
  v1::Ack set_reference(int32_t ref) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::SET_REFERENCE;
    m.set_reference.request_id = next_rid();
    m.set_reference.reference_count = ref;
    return cmd(m);
  }
  std::vector<v1::McuMsg> hello(uint32_t request_id = 1) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::HELLO_REQ;
    m.hello_req.request_id = request_id;
    return send(m);
  }
  // max_acc: フレームの max_acc_count_s2（Issue #45 の試験は運用値 2000。既定は従来どおり 20000）
  v1::HostMsg set_position_msg(int32_t target, uint16_t valid_for_ms = 50, uint32_t max_acc = 20000) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::SET_POSITION;
    m.set_position.session_epoch = EPOCH;
    m.set_position.seq = ++seq;
    m.set_position.group_frame_id = ++frame;
    m.set_position.target_count = target;
    m.set_position.max_vel_count_s = pos_max_vel;
    m.set_position.max_acc_count_s2 = max_acc;
    m.set_position.valid_for_ms = valid_for_ms;
    m.set_position.apply_at_us = 0;
    return m;
  }
  void set_position(int32_t target, uint16_t valid_for_ms = 50) { send(set_position_msg(target, valid_for_ms)); }

  // 1 tick（既定 5 ms）。プラントは位置制御中だけ setpoint に追従する。校正中は Duty に応じて端まで動く。
  void step(uint32_t dt_ms = 5) {
    now += dt_ms * MS;
    if (!stall && ax.brake_release_cmd()) {
      if (ax.state() == st::CALIBRATING) {
        const double v = -static_cast<double>(plant_sign) * ax.duty_permille() / 10.0 * cal_count_s_per_percent;  // count/s（Duty は permille）
        raw += llround(v * static_cast<double>(dt_ms) / 1000.0);
        if (raw < end_lo) raw = end_lo;
        if (raw > end_hi) raw = end_hi;
      } else {
        const double max_step = plant_vel * static_cast<double>(dt_ms) / 1000.0;
        double delta = static_cast<double>(ax.target_count()) - static_cast<double>(ax.actual_count());
        if (delta > max_step) delta = max_step;
        if (delta < -max_step) delta = -max_step;
        raw += static_cast<int64_t>(delta);
      }
    }
    ax.tick(now, raw);
    drain();
  }
  // 条件が成り立つまで 5 ms ずつ進める（上限 max_ms）。成り立てば true。
  template <typename Pred>
  bool run_until(Pred pred, uint32_t max_ms) {
    for (uint32_t t = 0; t < max_ms; t += 5) {
      if (pred()) return true;
      step(5);
    }
    return pred();
  }
  v1::HostMsg calibrate_msg(uint8_t mode, uint32_t request_id, uint8_t flags = 0) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::CALIBRATE;
    m.calibrate.request_id = request_id;
    m.calibrate.mode = mode;
    m.calibrate.flags = flags;
    return m;
  }
  v1::Ack calibrate(uint8_t mode = 1, uint8_t flags = 0) { return cmd(calibrate_msg(mode, next_rid(), flags)); }
  // SET_LIMITS（motion-limits 設計 §4.2）。既定値は設計 §3 の新しい初期値
  v1::HostMsg set_limits_msg(uint32_t request_id, uint8_t duty_max = 40, uint8_t duty_min = 3, uint8_t kp = 2,
                             uint16_t ff = 100, uint16_t deadband = 10, uint16_t deadband_out = 20) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::SET_LIMITS;
    m.set_limits.request_id = request_id;
    m.set_limits.duty_max_percent = duty_max;
    m.set_limits.duty_min_percent = duty_min;
    m.set_limits.kp_percent_per_100 = kp;
    m.set_limits.ff_count_s_per_percent = ff;
    m.set_limits.deadband_count = deadband;
    m.set_limits.deadband_out_count = deadband_out;
    return m;
  }
  v1::Ack set_limits(uint8_t duty_max = 40, uint8_t duty_min = 3, uint8_t kp = 2, uint16_t ff = 100,
                     uint16_t deadband = 10, uint16_t deadband_out = 20) {
    return cmd(set_limits_msg(next_rid(), duty_max, duty_min, kp, ff, deadband, deadband_out));
  }
  std::vector<v1::Limits> limits() const {
    std::vector<v1::Limits> r;
    for (const v1::McuMsg& m : out) {
      if (m.type == v1::msg::LIMITS) r.push_back(m.limits);
    }
    return r;
  }
  std::vector<v1::CalResult> cal_results() const {
    std::vector<v1::CalResult> r;
    for (const v1::McuMsg& m : out) {
      if (m.type == v1::msg::CAL_RESULT) r.push_back(m.cal_result);
    }
    return r;
  }
  // CALIBRATE(1) を受理させ、AutoMachine が動き出す（settle + brake wait の後）まで進める。
  void start_calibration_and_reach_running() {
    accepted(calibrate(1));
    CHECK(ax.state() == st::CALIBRATING);
    CHECK(run_until([&] { return ax.duty_permille() != 0; }, 6000));
  }
  void idle(uint32_t ms) {
    while (ms >= 5) {
      step(5);
      ms -= 5;
    }
    if (ms) step(ms);
  }
  // 10 ms ごとに SET_POSITION(target) を送りながら進める
  void drive(uint32_t ms, int32_t target) {
    while (ms > 0) {
      set_position(target);
      const uint32_t d = ms >= 10 ? 10 : ms;
      idle(d);
      ms -= d;
    }
  }

  void reach_holding_from_configured() {
    accepted(arm());
    const int32_t here = ax.actual_count();
    drive(620, here);
    CHECK(ax.state() == st::HOLDING);
  }
  void reach_holding() {
    accepted(config());
    reach_holding_from_configured();
  }
  void reach_active() {
    reach_holding();
    set_position(ax.actual_count());
    accepted(start());
    CHECK(ax.state() == st::ACTIVE);
  }

  std::vector<v1::FaultEvent> fault_events() const {
    std::vector<v1::FaultEvent> r;
    for (const v1::McuMsg& m : out) {
      if (m.type == v1::msg::FAULT_EVENT) r.push_back(m.fault_event);
    }
    return r;
  }
  std::vector<v1::Status> statuses() const {
    std::vector<v1::Status> r;
    for (const v1::McuMsg& m : out) {
      if (m.type == v1::msg::STATUS) r.push_back(m.status);
    }
    return r;
  }
  std::vector<v1::AxisSelect> axis_selects() const {
    std::vector<v1::AxisSelect> r;
    for (const v1::McuMsg& m : out) {
      if (m.type == v1::msg::AXIS_SELECT) r.push_back(m.axis_select);
    }
    return r;
  }
  v1::Status status() const { return ax.status(now); }

  static void accepted(const v1::Ack& a) { CHECK(a.result == ar::ACCEPTED && a.reason == rs::NONE); }
  static void rejected(const v1::Ack& a, uint16_t reason) { CHECK(a.result == ar::REJECTED && a.reason == reason); }
  static void duplicate(const v1::Ack& a) { CHECK(a.result == ar::DUPLICATE && a.reason == rs::NONE); }
};

// ACK より後の STATUS がすべて ok を満たす（ACK の後に少なくとも 1 つある）
inline void status_after_ack(const std::vector<v1::McuMsg>& out, bool (*ok)(const v1::Status&)) {
  bool seen_ack = false;
  int after = 0;
  for (const v1::McuMsg& m : out) {
    if (m.type == v1::msg::ACK) seen_ack = true;
    if (seen_ack && m.type == v1::msg::STATUS) {
      after += 1;
      CHECK(ok(m.status));
    }
  }
  CHECK(seen_ack);
  CHECK(after >= 1);
}

// ---- 起動・HELLO・CLOCK_SYNC --------------------------------------------------

inline void test_boots_disarmed_with_safe_outputs() {
  Harness h(1, 1, 0, false);
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(!h.ax.power_on_cmd());
  CHECK(!h.ax.brake_release_cmd());
  CHECK(h.ax.duty_permille() == 0);
  CHECK(!h.ax.configured());
  CHECK(!h.ax.referenced());
  CHECK(h.statuses().size() >= 1);  // 最初の tick で STATUS
  const v1::Status s = h.status();
  CHECK(s.mcu_state == st::DISARMED && s.flags == 0 && s.selected_axis == 1 && s.boot_id == 2309737967u);
  CHECK(s.current_ma == 0 && (s.flags & fl::CURRENT_VALID) == 0);
}

inline void test_hello_answers_in_every_state() {
  Harness h;
  auto check_hello = [&](uint8_t expect_state, uint32_t expect_hash) {
    const std::vector<v1::McuMsg> r = h.hello(77);
    bool found = false;
    for (const v1::McuMsg& m : r) {
      if (m.type == v1::msg::HELLO) {
        found = true;
        CHECK(m.hello.request_id == 77);
        CHECK(m.hello.protocol_version == 1);
        CHECK(m.hello.selected_axis == 1);
        CHECK(m.hello.mcu_state == expect_state);
        CHECK(m.hello.device_id == 0x5131000000000001ull);
        CHECK(m.hello.boot_id == 2309737967u);
        CHECK(m.hello.fw_major == 0 && m.hello.fw_minor == 1 && m.hello.fw_patch == 0);
        CHECK(m.hello.capabilities == (v1::capabilities::POSITION_MODE | v1::capabilities::LIMITS));
        CHECK((m.hello.capabilities & v1::capabilities::LIMITS) != 0);  // SET_LIMITS / LIMITS 対応（設計 §4.1）
        CHECK(m.hello.calibration_hash == expect_hash);
      }
    }
    CHECK(found);
  };
  check_hello(st::DISARMED, 0);
  Harness::accepted(h.config());
  check_hello(st::DISARMED, 0x12345678);
  Harness::accepted(h.arm());
  check_hello(st::ARMING, 0x12345678);
  h.drive(620, 0);
  check_hello(st::HOLDING, 0x12345678);
  h.set_position(0);
  Harness::accepted(h.start());
  check_hello(st::ACTIVE, 0x12345678);
  Harness::accepted(h.host_fault());
  check_hello(st::FAULT, 0x12345678);
}

inline void test_clock_sync_replies() {
  Harness h;
  h.idle(50);
  v1::HostMsg m;
  memset(&m, 0, sizeof(m));
  m.type = v1::msg::CLOCK_SYNC;
  m.clock_sync.request_id = 9;
  m.clock_sync.pc_time_us = 1000000;
  const std::vector<v1::McuMsg> r = h.send(m);
  bool found = false;
  for (const v1::McuMsg& x : r) {
    if (x.type == v1::msg::CLOCK_SYNC_REPLY) {
      found = true;
      CHECK(x.clock_sync_reply.request_id == 9);
      CHECK(x.clock_sync_reply.pc_time_us == 1000000);
      CHECK(x.clock_sync_reply.mcu_rx_us == 50 * MS);
      CHECK(x.clock_sync_reply.mcu_tx_us == 50 * MS);
    }
  }
  CHECK(found);
}

// ---- CONFIG ------------------------------------------------------------------

inline void test_config_sets_configured_flag_and_immediate_status() {
  Harness h;
  v1::HostMsg m;
  memset(&m, 0, sizeof(m));
  m.type = v1::msg::CONFIG;
  m.config = base_config();
  m.config.request_id = h.next_rid();
  const std::vector<v1::McuMsg> r = h.send(m);
  CHECK(r.size() == 2);
  CHECK(r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED && r[0].ack.acked_type == v1::msg::CONFIG);
  CHECK(r[1].type == v1::msg::STATUS && (r[1].status.flags & fl::CONFIGURED) != 0 &&
        (r[1].status.flags & fl::POWER_CONTROLLER) != 0);
  CHECK(h.ax.configured());
  CHECK(h.ax.calibration_hash() == 0x12345678);
  CHECK(h.ax.state() == st::DISARMED);
}

inline void test_config_invalid_params_rejected() {
  Harness h;
  v1::Config c = base_config();
  c.soft_min_count = 100;
  c.soft_max_count = 100;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  c = base_config();
  c.max_vel_count_s = 0;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  c = base_config();
  c.max_acc_count_s2 = 0;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  c = base_config();
  c.following_error_limit_count = 0;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  c = base_config();
  c.overspeed_count_s = 0;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  c = base_config();
  c.command_timeout_ms = 0;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  c = base_config();
  c.stop_timeout_ms = 0;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  CHECK(!h.ax.configured());
  // role_flags の他ビットは拒否しない
  c = base_config();
  c.role_flags = 0xFF;
  Harness::accepted(h.config_with(c));
}

inline void test_config_only_in_disarmed() {
  Harness h;
  Harness::accepted(h.config());
  Harness::accepted(h.arm());
  Harness::rejected(h.config(), rs::BAD_STATE);
  CHECK(h.ax.state() == st::ARMING);
}

inline void test_request_id_zero_rejected() {
  Harness h;
  Harness::rejected(h.cmd(h.disarm_msg(0)), rs::INVALID_PARAM);
}

// CONFIG の速度・加速度上限は絶対上限（6000 / 40000）で黙ってクランプして保存する（拒否しない。motion-limits 設計 §4.4）
inline void test_config_clamps_vel_and_acc_to_absolute_ceiling() {
  Harness h;
  v1::Config c = base_config();
  c.max_vel_count_s = 99999;
  c.max_acc_count_s2 = 9999999;
  c.soft_min_count = -100000;
  c.soft_max_count = 100000;
  c.following_error_limit_count = 100000;
  c.overspeed_count_s = 9000;
  Harness::accepted(h.config_with(c));
  CHECK(h.ax.config() != nullptr);
  CHECK(h.ax.config()->max_vel_count_s == V1_ABS_MAX_VEL_COUNT_S);
  CHECK(h.ax.config()->max_acc_count_s2 == V1_ABS_MAX_ACC_COUNT_S2);
  CHECK(h.ax.config()->max_vel_count_s == 6000 && h.ax.config()->max_acc_count_s2 == 40000);
  // 絶対上限ちょうど・それ未満はそのまま
  c.max_vel_count_s = 6000;
  c.max_acc_count_s2 = 40000;
  Harness::accepted(h.config_with(c));
  CHECK(h.ax.config()->max_vel_count_s == 6000 && h.ax.config()->max_acc_count_s2 == 40000);
  c.max_vel_count_s = 3000;
  c.max_acc_count_s2 = 15000;
  Harness::accepted(h.config_with(c));
  CHECK(h.ax.config()->max_vel_count_s == 3000 && h.ax.config()->max_acc_count_s2 == 15000);
  // 0 は今までどおり拒否
  c.max_vel_count_s = 0;
  Harness::rejected(h.config_with(c), rs::INVALID_PARAM);
  // 動きにも効く: フレームの上限が大きくても 1 s の移動は 6000 count/s・40000 count/s² の軌道（≈ 5565 count）
  Harness g;
  c = base_config();
  c.max_vel_count_s = 99999;
  c.max_acc_count_s2 = 9999999;
  c.soft_min_count = -100000;
  c.soft_max_count = 100000;
  c.following_error_limit_count = 100000;
  c.overspeed_count_s = 9000;
  Harness::accepted(g.config_with(c));
  g.plant_vel = 10000.0;
  g.reach_holding_from_configured();
  g.set_position(g.ax.actual_count());
  Harness::accepted(g.start());
  for (int i = 0; i < 100; ++i) {
    v1::HostMsg m = g.set_position_msg(100000);
    m.set_position.max_vel_count_s = 99999;
    m.set_position.max_acc_count_s2 = 9999999;
    g.send(m);
    g.idle(10);
  }
  CHECK(g.ax.state() == st::ACTIVE);
  CHECK(g.ax.target_count() > 5300 && g.ax.target_count() < 5800);  // 加速度も速度もクランプされていないと 6000 / 20000 になる
}

// ---- ARM / START / STOP / DISARM ---------------------------------------------

inline void test_arm_prepare_rejections_in_order() {
  {
    Harness h;
    Harness::rejected(h.arm(), rs::NOT_CONFIGURED);
  }
  {
    Harness h(1, 1, 0, false);
    Harness::accepted(h.config());
    Harness::rejected(h.arm(), rs::NOT_REFERENCED);
  }
  {
    Harness h;
    Harness::accepted(h.config());
    Harness::rejected(h.arm(EPOCH + 1), rs::EPOCH_MISMATCH);
    CHECK(h.ax.state() == st::DISARMED);
    // 範囲外（ブレーキを一度も解除しない）
    h.raw = 5000;
    h.step();
    Harness::rejected(h.arm(), rs::BAD_STATE);
    CHECK(h.ax.state() == st::DISARMED);
    for (const v1::Status& s : h.statuses()) CHECK((s.flags & fl::BRAKE_RELEASE_CMD) == 0);
    h.raw = 0;
    h.step();
    h.reach_holding_from_configured();
    Harness::rejected(h.arm(), rs::BAD_STATE);
    Harness::accepted(h.host_fault());
    Harness::rejected(h.arm(), rs::FAULT_ACTIVE);
  }
}

inline void test_arming_sequence_power_then_brake_then_holding() {
  Harness h(1, 1, 321, false);
  Harness::accepted(h.set_reference(321));
  Harness::accepted(h.config());
  const std::vector<v1::McuMsg> r = h.send([&] {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::ARM_PREPARE;
    m.arm_prepare.request_id = h.next_rid();
    m.arm_prepare.session_epoch = EPOCH;
    return m;
  }());
  // ACK → STATUS(ARMING) の順
  CHECK(r.size() == 2 && r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED);
  CHECK(r[1].type == v1::msg::STATUS && r[1].status.mcu_state == st::ARMING);
  CHECK(h.ax.power_on_cmd());
  CHECK(!h.ax.brake_release_cmd());
  CHECK(h.ax.duty_permille() == 0);
  v1::Status s = h.status();
  CHECK((s.flags & fl::POWER_ON_CMD) != 0 && (s.flags & fl::BRAKE_RELEASE_CMD) == 0 && s.pwm_permille == 0);
  // POWER_SETTLE_MS の直前はまだブレーキ作動
  h.drive(295, 321);
  CHECK(h.ax.state() == st::ARMING && !h.ax.brake_release_cmd());
  h.drive(10, 321);
  CHECK(h.ax.state() == st::ARMING && h.ax.brake_release_cmd());
  CHECK(h.ax.target_count() == 321);
  // BRAKE_RELEASE_WAIT_MS の後 HOLDING
  h.drive(290, 321);
  CHECK(h.ax.state() == st::ARMING);
  h.drive(10, 321);
  CHECK(h.ax.state() == st::HOLDING);
  CHECK(h.ax.target_count() == 321);
  CHECK(h.ax.power_on_cmd() && h.ax.brake_release_cmd());
  CHECK(h.seen_states.size() == 3 && h.seen_states[0] == st::DISARMED && h.seen_states[1] == st::ARMING &&
        h.seen_states[2] == st::HOLDING);
}

inline void test_non_power_controller_never_commands_power() {
  Harness h;
  v1::Config c = base_config();
  c.role_flags = 0;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  for (const v1::Status& s : h.statuses()) CHECK((s.flags & (fl::POWER_ON_CMD | fl::POWER_CONTROLLER)) == 0);
  CHECK(!h.ax.power_on_cmd());
  CHECK(h.ax.brake_release_cmd());
}

inline void test_start_rejections_and_transition() {
  Harness h;
  Harness::accepted(h.config());
  Harness::rejected(h.start(), rs::BAD_STATE);
  h.reach_holding_from_configured();
  Harness::rejected(h.start(EPOCH + 1), rs::EPOCH_MISMATCH);
  CHECK(h.ax.state() == st::HOLDING);
  h.set_position(0);
  Harness::accepted(h.start());
  CHECK(h.ax.state() == st::ACTIVE);
  Harness::rejected(h.start(), rs::BAD_STATE);
}

inline void test_start_rejected_without_unexpired_set_position() {
  Harness h;
  h.reach_holding();
  h.idle(60);  // valid_for 50 ms は切れたが lease 100 ms は生きている
  CHECK(h.ax.state() == st::HOLDING);
  Harness::rejected(h.start(), rs::BAD_STATE);
  CHECK(h.ax.state() == st::HOLDING);
  for (uint8_t s : h.seen_states) CHECK(s != st::ACTIVE);
  h.set_position(0);
  Harness::accepted(h.start());
  CHECK(h.ax.state() == st::ACTIVE);
}

inline void test_start_ignores_frames_from_before_arm_prepare() {
  Harness h;
  Harness::accepted(h.config());
  h.set_position(0, 5000);  // DISARMED で受けた長い期限のフレームは START に数えない
  Harness::accepted(h.arm());
  h.idle(55);
  CHECK(h.ax.state() == st::ARMING);  // lease は ARM_PREPARE 受理時刻から
  Harness::rejected(h.start(), rs::BAD_STATE);
}

inline void test_start_keeps_hold_target_until_next_set_position() {
  Harness h;
  h.reach_holding();
  h.set_position(1500);  // HOLDING では目標に使わない
  Harness::accepted(h.start());
  h.idle(50);
  CHECK(h.ax.target_count() == 0);
  CHECK(h.ax.actual_count() == 0);
  h.set_position(1500);
  h.idle(50);
  CHECK(h.ax.target_count() > 0);
}

inline void test_holding_ignores_target_but_refreshes_lease() {
  Harness h;
  h.reach_holding();
  h.drive(500, 1500);
  CHECK(h.ax.state() == st::HOLDING);
  CHECK(h.ax.actual_count() == 0);
  CHECK(h.ax.target_count() == 0);
  CHECK(h.ax.last_applied_seq() == 0);
  CHECK(h.ax.last_rx_seq() == h.seq);
}

inline void test_active_tracks_target_with_limits() {
  Harness h;
  h.reach_active();
  h.drive(1500, 1000);
  CHECK(h.ax.state() == st::ACTIVE);
  CHECK(h.ax.actual_count() == 1000);
  CHECK(h.ax.target_count() == 1000);
  const v1::Status s = h.status();
  CHECK(s.last_applied_seq == h.seq && s.last_applied_frame == h.frame && s.last_rx_seq == h.seq);
  CHECK(s.following_error_count == 0);
}

inline void test_status_target_is_profiled_setpoint_not_commanded() {
  Harness h;
  h.reach_active();
  h.drive(30 + h.ax.tuning.playout_delay_ms, 2000);  // 再生バッファ（Issue #45）なら遅れの分だけ後から動き出す
  const v1::Status s = h.status();
  CHECK(s.mcu_state == st::ACTIVE);
  // 30 ms・20000 count/s² では +2000 に届かない: STATUS の target は軌道上の点
  CHECK(s.target_count > 0 && s.target_count < 100);
  CHECK(s.following_error_count == s.actual_count - s.target_count);
}

inline void test_frame_limits_capped_by_config_and_can_be_lower() {
  // CONFIG より大きいフレーム上限は CONFIG に制限される: 到達時間が CONFIG 上限と同じ
  Harness a;
  a.reach_active();
  for (int i = 0; i < 100; ++i) {
    v1::HostMsg m = a.set_position_msg(2000);
    m.set_position.max_vel_count_s = 100000;
    m.set_position.max_acc_count_s2 = 1000000;
    a.send(m);
    a.idle(10);
  }
  Harness b;
  b.reach_active();
  b.drive(1000, 2000);
  CHECK(a.ax.target_count() == b.ax.target_count());
  // 低い上限は効く: 100 count/s なら 1 s で 100 だけ
  Harness c;
  c.reach_active();
  for (int i = 0; i < 100; ++i) {
    v1::HostMsg m = c.set_position_msg(2000);
    m.set_position.max_vel_count_s = 100;
    c.send(m);
    c.idle(10);
  }
  CHECK(c.ax.target_count() > 50 && c.ax.target_count() <= 101);
}

inline void test_soft_limit_clamps_target() {
  Harness h;
  h.reach_active();
  // profile_step は端ガードが、実測速度 0 でも 1 tick 分の参照速度の d_req だけ手前で止まり続ける。
  // そこへ寄るのに再生バッファの 4 s より長くかかる。
  h.drive(h.ax.tuning.playout_delay_ms > 0 ? 4000 : 8000, 5000);
  CHECK(h.ax.state() == st::ACTIVE);
  // 再生バッファ（Issue #46）は端の reserve 手前。profile_step は端ガードの静止位置
  // （v = max_acc × 5 ms = 100、d_req = 39.25 → 2700 − 39 = 2661）。
  int32_t stop_at = 2700 - V1_SOFT_RESERVE_COUNT;
  if (h.ax.tuning.playout_delay_ms == 0) {
    const double v_tick = 20000.0 * 0.005;
    const double d_req = v_tick * 0.08 + (v_tick * v_tick) / (2.0 * 4000.0) + 30.0;
    stop_at = 2700 - static_cast<int32_t>(std::floor(d_req));
  }
  CHECK(h.ax.target_count() == stop_at);
  CHECK(h.ax.actual_count() == stop_at);
}

inline void test_controlled_stop_decelerates_to_holding() {
  Harness h;
  v1::Config c = base_config();
  // d_req(4000) は約 2350。ソフトリミット 2700 のままだと 200 ms の加速で端ガードが参照速度を 0 にし、減速距離が消える。
  c.soft_min_count = -30000;
  c.soft_max_count = 30000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  CHECK(h.ax.state() == st::ACTIVE);
  h.drive(200, 2500);
  CHECK(h.ax.state() == st::ACTIVE);
  const int32_t at_stop = h.ax.target_count();
  const std::vector<v1::McuMsg> r = h.send([&] {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::CONTROLLED_STOP;
    m.controlled_stop.request_id = h.next_rid();
    m.controlled_stop.session_epoch = EPOCH;
    m.controlled_stop.reason = 1;
    return m;
  }());
  status_after_ack(r, [](const v1::Status& s) { return s.mcu_state == st::STOPPING; });
  // 減速中も lease は要る
  h.drive(300, 2500);
  CHECK(h.ax.state() == st::HOLDING);
  CHECK(h.ax.target_count() > at_stop);  // 減速距離だけ進んで止まる
  CHECK(h.ax.target_count() < 2500);
  CHECK(h.ax.fault_code() == fc::NONE);
  // HOLDING では遅れて届いた目標で動かない
  h.drive(200, 2500);
  CHECK(h.ax.state() == st::HOLDING);
}

inline void test_controlled_stop_epoch_mismatch_and_noop_states() {
  Harness h;
  Harness::rejected(h.stop(), rs::EPOCH_MISMATCH);  // CONFIG 未受理
  Harness::accepted(h.config());
  Harness::rejected(h.stop(EPOCH + 1), rs::EPOCH_MISMATCH);
  Harness::accepted(h.stop());  // DISARMED: no-op
  Harness::accepted(h.disarm());  // DISARMED: no-op
  CHECK(h.ax.state() == st::DISARMED);
  Harness::accepted(h.arm());
  Harness::accepted(h.stop());  // ARMING: no-op
  CHECK(h.ax.state() == st::ARMING);
  h.drive(620, 0);
  CHECK(h.ax.state() == st::HOLDING);
  Harness::accepted(h.stop());  // HOLDING: no-op
  CHECK(h.ax.state() == st::HOLDING);
  Harness::accepted(h.disarm());  // HOLDING → DISARMING（減速なし）
  CHECK(h.ax.state() == st::DISARMING);
  Harness::accepted(h.stop());  // DISARMING: no-op
  Harness::accepted(h.disarm());
  CHECK(h.ax.state() == st::DISARMING);
  h.idle(1000);
  CHECK(h.ax.state() == st::DISARMED);
}

inline void test_valid_for_expiry_stops_to_holding() {
  Harness h;
  h.reach_active();
  h.drive(100, 2500);
  CHECK(h.ax.state() == st::ACTIVE);
  // valid_for 50 ms だけ切らして lease 100 ms は保つ: 60 ms 間隔で送る
  h.idle(60);
  CHECK(h.ax.state() == st::STOPPING || h.ax.state() == st::HOLDING);
  CHECK(h.ax.fault_code() == fc::NONE);
  h.drive(400, 2500);
  CHECK(h.ax.state() == st::HOLDING);  // 遅れた目標で再開しない
  h.set_position(2500);
  Harness::accepted(h.start());
  CHECK(h.ax.state() == st::ACTIVE);
}

inline void test_disarm_from_holding() {
  Harness h;
  h.reach_holding();
  Harness::accepted(h.disarm());
  CHECK(h.ax.state() == st::DISARMING);
  CHECK(!h.ax.brake_release_cmd());
  CHECK(h.ax.power_on_cmd());  // 主電源は DISARMED まで
  h.idle(295);
  CHECK(h.ax.state() == st::DISARMING);
  h.idle(10);
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(!h.ax.power_on_cmd());
  CHECK(h.ax.duty_permille() == 0);
  CHECK(h.ax.fault_code() == fc::NONE);
  CHECK(h.fault_events().empty());  // DISARM の減速は lease の対象外
}

inline void test_disarm_from_active_decelerates_first() {
  Harness h;
  h.reach_active();
  h.drive(200, 2500);
  Harness::accepted(h.disarm());
  CHECK(h.ax.state() == st::STOPPING);
  CHECK(h.ax.brake_release_cmd());
  h.idle(600);  // 減速 200 ms + BRAKE_APPLY_WAIT_MS。SET_POSITION を止めても COMMAND_TIMEOUT にならない
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(h.ax.fault_code() == fc::NONE);
  CHECK(h.fault_events().empty());
}

inline void test_disarm_from_stopping_switches_to_disarm() {
  Harness h;
  h.reach_active();
  h.drive(200, 2500);
  Harness::accepted(h.stop());
  CHECK(h.ax.state() == st::STOPPING);
  Harness::accepted(h.disarm());
  h.idle(600);
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(h.fault_events().empty());
}

inline void test_disarm_from_arming_and_rearm_works() {
  Harness h;
  Harness::accepted(h.config());
  Harness::accepted(h.arm());
  Harness::accepted(h.disarm());
  CHECK(h.ax.state() == st::DISARMING);
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED);
  Harness::accepted(h.disarm());
  h.reach_holding_from_configured();  // CONFIG と基準は残る
}

// ---- SET_POSITION の規則 ----------------------------------------------------

inline void test_set_position_stale_seq_and_epoch_dropped() {
  Harness h;
  Harness::accepted(h.config());
  h.set_position(0);
  CHECK(h.ax.last_rx_seq() == 1);
  v1::HostMsg m = h.set_position_msg(0);
  m.set_position.seq = 1;  // 同じ seq
  h.send(m);
  CHECK(h.ax.stale_drop_count() == 1);
  m = h.set_position_msg(0);
  m.set_position.session_epoch = EPOCH + 1;
  h.send(m);
  CHECK(h.ax.stale_drop_count() == 2);
  CHECK(h.ax.rx_error_count() == 0);
  h.set_position(0);
  CHECK(h.ax.stale_drop_count() == 2);
}

inline void test_set_position_before_config_dropped() {
  Harness h;
  h.set_position(0);
  CHECK(h.ax.stale_drop_count() == 1 && h.ax.last_rx_seq() == 0);
}

inline void test_stale_frames_do_not_refresh_lease() {
  Harness h;
  h.reach_holding();
  const uint32_t good = h.seq;
  for (int i = 0; i < 20; ++i) {
    v1::HostMsg m = h.set_position_msg(0);
    m.set_position.seq = good;  // 古い
    h.send(m);
    h.idle(10);
  }
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::COMMAND_TIMEOUT);
}

inline void test_apply_at_nonzero_dropped_as_rx_error() {
  Harness h;
  h.reach_holding();
  const uint32_t before_seq = h.ax.last_rx_seq();
  for (int i = 0; i < 20; ++i) {
    v1::HostMsg m = h.set_position_msg(0);
    m.set_position.apply_at_us = 123456;
    h.send(m);
    h.idle(10);
  }
  CHECK(h.ax.rx_error_count() == 20);
  CHECK(h.ax.stale_drop_count() == 0);
  CHECK(h.ax.last_rx_seq() == before_seq);  // 受理していない
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::COMMAND_TIMEOUT);  // lease も延びない
}

inline void test_seq_resets_only_on_new_epoch() {
  Harness h;
  Harness::accepted(h.config());
  h.set_position(0);
  h.set_position(0);
  CHECK(h.ax.last_rx_seq() == 2);
  Harness::accepted(h.config());  // 同じ epoch → 保持
  CHECK(h.ax.last_rx_seq() == 2);
  v1::Config c = base_config(EPOCH + 1);
  Harness::accepted(h.config_with(c));
  CHECK(h.ax.last_rx_seq() == 0);
}

// ---- 期限 ---------------------------------------------------------------------

inline void test_command_timeout_in_holding_faults() {
  Harness h;
  h.reach_holding();
  h.set_position(0);
  h.idle(100);
  CHECK(h.ax.state() == st::HOLDING);  // ちょうど timeout は可
  h.idle(5);
  CHECK(h.ax.state() == st::FAULT);
  CHECK(h.ax.fault_code() == fc::COMMAND_TIMEOUT);
  const std::vector<v1::FaultEvent> ev = h.fault_events();
  CHECK(ev.size() == 1);
  CHECK(ev[0].fault_code == fc::COMMAND_TIMEOUT && ev[0].mcu_state_before == st::HOLDING);
  CHECK(ev[0].boot_id == 2309737967u);
  const v1::Status s = h.status();
  CHECK(s.pwm_permille == 0 && (s.flags & (fl::POWER_ON_CMD | fl::BRAKE_RELEASE_CMD)) == 0);
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  // 同じ FAULT に居る間 FAULT_EVENT は増えない
  h.idle(200);
  CHECK(h.fault_events().size() == 1);
}

inline void test_command_timeout_in_arming_counts_from_arm_prepare() {
  Harness h;
  Harness::accepted(h.config());
  Harness::accepted(h.arm());
  h.idle(110);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::COMMAND_TIMEOUT);
  CHECK(!h.ax.power_on_cmd());
}

inline void test_command_timeout_while_moving_decelerates_then_faults() {
  Harness h;
  v1::Config c = base_config();
  c.command_timeout_ms = 30;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.arm());
  for (int i = 0; i < 62; ++i) {
    h.set_position(0);
    h.idle(10);
  }
  CHECK(h.ax.state() == st::HOLDING);
  h.set_position(0);
  Harness::accepted(h.start());
  for (int i = 0; i < 30; ++i) {
    h.set_position(2600, 500);
    h.idle(10);
  }
  CHECK(h.ax.state() == st::ACTIVE);
  h.idle(35);
  CHECK(h.ax.state() == st::STOPPING);
  CHECK(h.ax.fault_code() == fc::COMMAND_TIMEOUT);  // 減速中も fault_code を出す
  CHECK(h.fault_events().empty());  // ラッチ時だけ
  Harness::rejected(h.disarm(), rs::FAULT_ACTIVE);
  h.idle(400);
  CHECK(h.ax.state() == st::FAULT);
  const std::vector<v1::FaultEvent> ev = h.fault_events();
  CHECK(ev.size() == 1 && ev[0].fault_code == fc::COMMAND_TIMEOUT && ev[0].mcu_state_before == st::STOPPING);
}

inline void test_host_fault_during_fault_stop_keeps_pending_code() {
  Harness h;
  v1::Config c = base_config();
  c.command_timeout_ms = 30;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.arm());
  for (int i = 0; i < 62; ++i) {
    h.set_position(0);
    h.idle(10);
  }
  h.set_position(0);
  Harness::accepted(h.start());
  for (int i = 0; i < 30; ++i) {
    h.set_position(2600, 500);
    h.idle(10);
  }
  h.idle(35);
  CHECK(h.ax.state() == st::STOPPING && h.ax.fault_code() == fc::COMMAND_TIMEOUT);
  Harness::accepted(h.host_fault());
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::COMMAND_TIMEOUT);
  CHECK(h.ax.last_host_fault_code() == fc::HOST_FAULT);
}

inline void test_stop_timeout_faults() {
  Harness h;
  v1::Config c = base_config();
  c.max_acc_count_s2 = 100;  // 1 s の加速で約 100 count/s。STOPPING の減速は下限 V1_MIN_DECEL_COUNT_S2（500）で約 200 ms
  c.stop_timeout_ms = 60;    // （PR #49 レビュー P1b で減速度に下限ができたので、止まり切る前に切れるよう短く）
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(0);
  Harness::accepted(h.start());
  for (int i = 0; i < 100; ++i) {
    v1::HostMsg m = h.set_position_msg(2600);
    m.set_position.max_acc_count_s2 = 100000;  // CONFIG に制限される（100）
    h.send(m);
    h.idle(10);
  }
  CHECK(h.ax.state() == st::ACTIVE);
  Harness::accepted(h.stop());
  CHECK(h.ax.state() == st::STOPPING && h.ax.setpoint_vel() > 50.0);
  h.drive(70, 2600);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::STOP_TIMEOUT);
}

// STOPPING の減速は CONFIG の max_acc ではなく最後に適用したフレームの max_acc（motion-limits 設計 §4.4）。
// CONFIG に絶対上限 40000 が入っても、フレームが 2000 なら 2000 count/s² で止まる。
inline void test_stopping_decelerates_with_last_frame_max_acc() {
  auto run = [](uint32_t frame_acc, uint32_t drive_ms, uint32_t* stop_ms) {
    Harness h;
    v1::Config c = base_config();
    c.max_acc_count_s2 = 40000;
    Harness::accepted(h.config_with(c));
    h.reach_holding_from_configured();
    h.set_position(h.ax.actual_count());
    Harness::accepted(h.start());
    for (uint32_t t = 0; t < drive_ms; t += 10) {
      v1::HostMsg m = h.set_position_msg(2600);
      m.set_position.max_vel_count_s = 2000;
      m.set_position.max_acc_count_s2 = frame_acc;
      h.send(m);
      h.idle(10);
    }
    CHECK(h.ax.state() == st::ACTIVE);
    const int32_t at_stop = h.ax.target_count();
    CHECK(at_stop > 500 && at_stop < 1500);  // まだ動いている途中（止める前に到達していない）
    Harness::accepted(h.stop());
    CHECK(h.ax.state() == st::STOPPING);
    uint32_t t = 0;
    while (h.ax.state() == st::STOPPING && t < 3000) {
      h.set_position(2600);  // lease だけ（STOPPING では目標もフレーム上限も適用しない）
      h.idle(10);
      t += 10;
    }
    *stop_ms = t;
    CHECK(h.ax.state() == st::HOLDING);
    CHECK(h.ax.fault_code() == fc::NONE);
    CHECK(h.fault_events().empty());
    CHECK(h.ax.target_count() > at_stop);
  };
  // フレーム 2000 count/s²: 2000 count/s からの減速に約 1 s（CONFIG の 40000 なら 50 ms）
  uint32_t slow_ms = 0;
  run(2000, 1000, &slow_ms);
  CHECK(slow_ms >= 900 && slow_ms <= 1100);
  // フレーム 40000 count/s²: 50 ms で止まる
  uint32_t fast_ms = 0;
  run(40000, 500, &fast_ms);
  CHECK(fast_ms <= 100);
  CHECK(slow_ms > fast_ms * 5);
}

// フレームが max_acc = 0 を運ぶと frame_max_acc_ が 0 になる。STOPPING では CONFIG の値に戻して必ず減速できるようにする
inline void test_stopping_with_zero_frame_acc_falls_back_to_config() {
  Harness h;
  h.reach_active();
  h.drive(200, 2600);  // 20000 count/s² で 4000 count/s に達する
  CHECK(h.ax.state() == st::ACTIVE);
  v1::HostMsg m = h.set_position_msg(2600);
  m.set_position.max_acc_count_s2 = 0;
  h.send(m);
  h.idle(10);
  CHECK(h.ax.state() == st::ACTIVE);
  Harness::accepted(h.stop());
  CHECK(h.ax.state() == st::STOPPING);
  h.drive(400, 2600);  // CONFIG の 20000 count/s² なら 200 ms で止まる
  CHECK(h.ax.state() == st::HOLDING);
  CHECK(h.ax.fault_code() == fc::NONE);
}

// ---- SET_LIMITS / LIMITS（motion-limits 設計 §4.2・§4.3） ------------------------------

inline bool limits_match(const v1::Limits& l, uint8_t duty_max, uint8_t duty_min, uint8_t kp, uint16_t ff,
                         uint16_t deadband, uint16_t deadband_out) {
  return l.boot_id == 2309737967u && l.duty_max_percent == duty_max && l.duty_min_percent == duty_min &&
         l.kp_percent_per_100 == kp && l.ff_count_s_per_percent == ff && l.deadband_count == deadband &&
         l.deadband_out_count == deadband_out && l.abs_duty_max_percent == V1_ABS_DUTY_MAX &&
         l.abs_max_vel_count_s == V1_ABS_MAX_VEL_COUNT_S && l.abs_max_acc_count_s2 == V1_ABS_MAX_ACC_COUNT_S2 &&
         l.abs_duty_max_percent == 60 && l.abs_max_vel_count_s == 6000 && l.abs_max_acc_count_s2 == 40000;
}

inline bool tuning_is_default(const V1Tuning& t) {
  return t.duty_max == V1_DUTY_MAX && t.duty_min == V1_DUTY_MIN && t.kp_percent_per_100 == V1_KP_PERCENT_PER_100 &&
         t.ff_count_s_per_percent == V1_FF_COUNT_S_PER_PERCENT && t.deadband == V1_DEADBAND &&
         t.deadband_out == V1_DEADBAND_OUT;
}

inline void test_set_limits_accepted_in_disarmed_and_reports_limits() {
  Harness h;
  CHECK(tuning_is_default(h.ax.tuning));  // 受理前は config.h の既定（Duty 7〜20 %）
  CHECK(h.ax.tuning.duty_min == 7 && h.ax.tuning.duty_max == 20);
  const std::vector<v1::McuMsg> r = h.send(h.set_limits_msg(h.next_rid(), 40, 3, 2, 100, 10, 20));
  // ACK → LIMITS の順。状態・flags は変わらないので即時 STATUS は無い
  CHECK(r.size() == 2);
  CHECK(r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED && r[0].ack.reason == rs::NONE &&
        r[0].ack.acked_type == v1::msg::SET_LIMITS && r[0].ack.request_id == h.rid);
  CHECK(r[1].type == v1::msg::LIMITS);
  CHECK(limits_match(r[1].limits, 40, 3, 2, 100, 10, 20));
  CHECK(h.ax.tuning.duty_max == 40 && h.ax.tuning.duty_min == 3 && h.ax.tuning.kp_percent_per_100 == 2 &&
        h.ax.tuning.ff_count_s_per_percent == 100 && h.ax.tuning.deadband == 10 && h.ax.tuning.deadband_out == 20);
  CHECK(h.ax.state() == st::DISARMED);
  // 他の調整値は触らない
  CHECK(h.ax.tuning.stall_duty == V1_STALL_DUTY);
  // 別の値で置き換え: LIMITS は毎回 1 つ
  Harness::accepted(h.set_limits(30, 5, 3, 120, 8, 16));
  const std::vector<v1::Limits> ls = h.limits();
  CHECK(ls.size() == 2);
  CHECK(limits_match(ls[1], 30, 5, 3, 120, 8, 16));
  CHECK(h.ax.tuning.duty_max == 30 && h.ax.tuning.duty_min == 5 && h.ax.tuning.kp_percent_per_100 == 3 &&
        h.ax.tuning.ff_count_s_per_percent == 120 && h.ax.tuning.deadband == 8 && h.ax.tuning.deadband_out == 16);
  // duty_min = duty_max、deadband_out = deadband、duty_min 0 は受理
  Harness::accepted(h.set_limits(10, 10, 0, 1, 0, 0));
  CHECK(h.ax.tuning.duty_max == 10 && h.ax.tuning.duty_min == 10 && h.ax.tuning.kp_percent_per_100 == 0 &&
        h.ax.tuning.deadband == 0 && h.ax.tuning.deadband_out == 0);
  Harness::accepted(h.set_limits(40, 0));
  CHECK(h.ax.tuning.duty_min == 0);
  CHECK(h.limits().size() == 4);
  // CONFIG・ARM はそのまま通る（SET_LIMITS は状態を変えない）
  Harness::accepted(h.config());
  h.reach_holding_from_configured();
}

inline void test_set_limits_accepted_in_active_and_takes_effect() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(0);
  Harness::accepted(h.start());
  CHECK(h.ax.state() == st::ACTIVE);
  h.stall = true;
  h.raw = -2500;  // 誤差 2500 × 2 / 100 = 50 % → 既定の上限 20 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -200);
  const std::vector<v1::McuMsg> r = h.send(h.set_limits_msg(h.next_rid(), 40, 3, 2, 100, 10, 20));
  CHECK(r.size() == 2 && r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED &&
        r[0].ack.acked_type == v1::msg::SET_LIMITS);
  CHECK(r[1].type == v1::msg::LIMITS && limits_match(r[1].limits, 40, 3, 2, 100, 10, 20));
  CHECK(h.ax.state() == st::ACTIVE);
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -400);  // 新しい上限がすぐ効く
  h.raw = -100;  // 2 % → 新しい下限 3 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -30);
  // HOLDING / ARMING / STOPPING / DISARMING / CALIBRATING でも受理
  Harness g;
  g.reach_holding();
  Harness::accepted(g.set_limits());
  CHECK(g.ax.state() == st::HOLDING);
  Harness k;
  Harness::accepted(k.config());
  Harness::accepted(k.arm());
  Harness::accepted(k.set_limits());
  CHECK(k.ax.state() == st::ARMING);
  Harness::accepted(k.disarm());
  CHECK(k.ax.state() == st::DISARMING);
  Harness::accepted(k.set_limits(35));
  CHECK(k.ax.state() == st::DISARMING && k.ax.tuning.duty_max == 35);
  k.idle(400);
  Harness::accepted(k.calibrate(1));
  CHECK(k.ax.state() == st::CALIBRATING);
  Harness::accepted(k.set_limits(36));
  CHECK(k.ax.state() == st::CALIBRATING && k.ax.tuning.duty_max == 36);
  CHECK(k.limits().size() == 3);
}

inline void test_set_limits_invalid_params_rejected() {
  Harness h;
  const V1Tuning before = h.ax.tuning;
  auto unchanged = [&] {
    return h.ax.tuning.duty_max == before.duty_max && h.ax.tuning.duty_min == before.duty_min &&
           h.ax.tuning.kp_percent_per_100 == before.kp_percent_per_100 &&
           h.ax.tuning.ff_count_s_per_percent == before.ff_count_s_per_percent &&
           h.ax.tuning.deadband == before.deadband && h.ax.tuning.deadband_out == before.deadband_out;
  };
  // duty_max = 0
  Harness::rejected(h.set_limits(0, 0, 2, 100, 10, 20), rs::INVALID_PARAM);
  CHECK(unchanged());
  // duty_min > duty_max
  Harness::rejected(h.set_limits(40, 41, 2, 100, 10, 20), rs::INVALID_PARAM);
  CHECK(unchanged());
  // ff = 0
  Harness::rejected(h.set_limits(40, 3, 2, 0, 10, 20), rs::INVALID_PARAM);
  CHECK(unchanged());
  // deadband_out < deadband
  Harness::rejected(h.set_limits(40, 3, 2, 100, 20, 19), rs::INVALID_PARAM);
  CHECK(unchanged());
  // 拒否では LIMITS を送らない
  CHECK(h.limits().empty());
  // request_id 0 は共通規則で INVALID_PARAM
  Harness::rejected(h.cmd(h.set_limits_msg(0)), rs::INVALID_PARAM);
  CHECK(unchanged() && h.limits().empty());
  // 拒否された命令の再送は改めて評価する（内容を直せば受理）
  Harness::accepted(h.set_limits(40, 3, 2, 100, 10, 20));
  CHECK(h.limits().size() == 1);
}

inline void test_set_limits_clamps_duty_to_absolute_max() {
  Harness h;
  // duty_max 90 → 有効値 60（絶対上限）。LIMITS はクランプ後の値を返す
  const std::vector<v1::McuMsg> r = h.send(h.set_limits_msg(h.next_rid(), 90, 3, 2, 100, 10, 20));
  CHECK(r.size() == 2 && r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED);
  CHECK(r[1].type == v1::msg::LIMITS);
  CHECK(r[1].limits.duty_max_percent == 60 && r[1].limits.duty_min_percent == 3);
  CHECK(r[1].limits.abs_duty_max_percent == 60);
  CHECK(h.ax.tuning.duty_max == 60 && h.ax.tuning.duty_min == 3);
  // duty_min も絶対上限を超えていればクランプ（検査は受けた値どうしで行う: 70 ≤ 90 は有効）
  Harness::accepted(h.set_limits(90, 70));
  CHECK(h.ax.tuning.duty_max == 60 && h.ax.tuning.duty_min == 60);
  CHECK(h.limits().back().duty_max_percent == 60 && h.limits().back().duty_min_percent == 60);
  // 100 / 100 も受理してクランプ
  Harness::accepted(h.set_limits(100, 100));
  CHECK(h.ax.tuning.duty_max == 60 && h.ax.tuning.duty_min == 60);
  // ちょうど 60 はそのまま
  Harness::accepted(h.set_limits(60, 59));
  CHECK(h.ax.tuning.duty_max == 60 && h.ax.tuning.duty_min == 59);
  // クランプされた上限が Duty に効く
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.set_limits(90, 3, 5));  // kp 5 %/100 count
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw = -2500;  // ソフトリミット内。誤差 2500 × 5 / 100 = 125 % → 60 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.state() == st::HOLDING);
  CHECK(h.ax.duty_permille() == -600);
  CHECK(h.ax.pwm_permille() == 600);
}

inline void test_set_limits_rejected_while_faulted_and_duplicate_not_reexecuted() {
  Harness h;
  h.reach_holding();
  Harness::accepted(h.host_fault());
  CHECK(h.ax.state() == st::FAULT);
  Harness::rejected(h.set_limits(), rs::FAULT_ACTIVE);
  CHECK(tuning_is_default(h.ax.tuning));
  CHECK(h.limits().empty());
  // 減速中の異常（COMMAND_TIMEOUT で STOPPING）も fault_code ≠ NONE なので弾く
  Harness g;
  v1::Config c = base_config();
  c.command_timeout_ms = 30;
  Harness::accepted(g.config_with(c));
  Harness::accepted(g.arm());
  for (int i = 0; i < 62; ++i) {
    g.set_position(0);
    g.idle(10);
  }
  g.set_position(0);
  Harness::accepted(g.start());
  for (int i = 0; i < 30; ++i) {
    g.set_position(2600, 500);
    g.idle(10);
  }
  g.idle(35);
  CHECK(g.ax.state() == st::STOPPING && g.ax.fault_code() == fc::COMMAND_TIMEOUT);
  Harness::rejected(g.set_limits(), rs::FAULT_ACTIVE);
  CHECK(g.limits().empty());
  // 解除後は受理
  Harness::accepted(h.fault_reset());
  Harness::accepted(h.set_limits());
  CHECK(h.limits().size() == 1);
  // 同じ request_id・同じ内容の再送は DUPLICATE で、LIMITS は送らない
  const v1::HostMsg same = h.set_limits_msg(h.rid);
  Harness::duplicate(h.cmd(same));
  CHECK(h.limits().size() == 1);
  // 同じ request_id で内容が違えば新しい命令として実行する
  Harness::accepted(h.cmd(h.set_limits_msg(h.rid, 25)));
  CHECK(h.limits().size() == 2 && h.ax.tuning.duty_max == 25);
}

// ---- 動作中の異常 -------------------------------------------------------------

inline void test_stall_causes_following_error() {
  Harness h;
  h.reach_active();
  h.stall = true;
  h.drive(300, 2000);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::FOLLOWING_ERROR);
  const std::vector<v1::FaultEvent> ev = h.fault_events();
  CHECK(ev.size() == 1 && ev[0].mcu_state_before == st::ACTIVE);
}

inline void test_overspeed_faults() {
  Harness h;
  h.reach_holding();
  // 100 ms で 1000 カウント（10000 count/s）動かされた: 追従誤差 200 を超える前に速度窓で検出するため、
  // 追従誤差上限を大きくした CONFIG で見る
  Harness g;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  c.soft_min_count = -100000;
  c.soft_max_count = 100000;
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.stall = true;
  for (int i = 0; i < 30; ++i) {
    g.raw += 50;  // 10000 count/s
    g.set_position(0);
    g.idle(5);
  }
  CHECK(g.ax.state() == st::FAULT && g.ax.fault_code() == fc::OVERSPEED);
}

inline void test_actual_outside_soft_limits_faults() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw = 2800;  // 手で範囲外へ
  h.set_position(0);
  h.idle(5);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
}

inline void test_arming_never_releases_brake_outside_soft_window() {
  Harness h;
  Harness::accepted(h.config());
  Harness::accepted(h.arm());
  h.stall = true;
  h.raw = 2800;  // settle 中に範囲外へ
  h.drive(295, 0);
  CHECK(h.ax.state() == st::ARMING);
  h.drive(10, 0);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::SOFT_LIMIT);
  for (const v1::Status& s : h.statuses()) CHECK((s.flags & fl::BRAKE_RELEASE_CMD) == 0);
  const std::vector<v1::FaultEvent> ev = h.fault_events();
  CHECK(ev.size() == 1 && ev[0].mcu_state_before == st::ARMING && ev[0].fault_code == fc::SOFT_LIMIT);
}

inline void test_soft_limit_recovery_goes_through_disarmed_and_set_reference() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw = 2800;
  h.set_position(0);
  h.idle(5);
  CHECK(h.ax.state() == st::FAULT);
  Harness::accepted(h.fault_reset());  // 範囲外でも解除する
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(h.ax.configured() && h.ax.referenced());
  Harness::rejected(h.arm(), rs::BAD_STATE);  // 範囲外では再 ARM できない
  Harness::accepted(h.set_reference(0));       // 基準を取り直せば ARM できる
  CHECK(h.ax.actual_count() == 0);
  Harness::accepted(h.arm());
}

inline void test_encoder_fault_on_no_pulses_while_driving() {
  Harness h;
  h.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;  // 追従誤差より先に無パルスで止める
  // 目標 2000 は約 0.6 s で到着して参照速度が 0 になり、600 ms の窓が完了する前にリセットされる。遠くへ置く。
  c.soft_min_count = -30000;
  c.soft_max_count = 30000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(0);
  Harness::accepted(h.start());
  h.stall = true;  // カウントが止まる
  // 軌道が動き出してフィードフォワードが |duty| ≥ しきい値 15 %（max(10, duty_max 20 × 75 %)。Issue #15 M2）に届いてから
  // V1_STALL_MS（600 ms）で異常（max_vel 4000 → 上限 20 % で頭打ち）。FF が 15 % に届くまで約 75 ms なので、500 ms では未達。
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 150);
  h.drive(500, 8000);
  CHECK(h.ax.state() == st::ACTIVE);
  CHECK(h.ax.duty_permille() != 0);
  h.drive(300, 8000);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(h.ax.duty_permille() == 0);
  // FAULT_RESET は通るが POSITION_REFERENCED が消える
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(h.ax.configured() && !h.ax.referenced());
  Harness::rejected(h.arm(), rs::NOT_REFERENCED);
}

inline void test_encoder_fault_not_triggered_while_counts_change() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(0);
  Harness::accepted(h.start());
  h.plant_vel = 200.0;  // 遅いが動いている
  h.drive(1000, 2000);
  CHECK(h.ax.state() == st::ACTIVE);
}

// レビュー指摘の回帰試験: 無パルス判定はブレーキ解除指令中に軌道が動いている間だけ（設計 §3）。
// DISARM でブレーキを作動させた後は、残った追従誤差（デッドバンド超）でカウントが動かなくても ENCODER_FAULT にしない。
inline void test_disarm_with_residual_error_reaches_disarmed_not_encoder_fault() {
  Harness h;
  h.reach_holding();
  h.stall = true;  // ブレーキ作動後はカウントが変わらない
  h.raw -= 25;     // 保持目標から 25 count ずれた残り誤差（不感帯の出る側 20 を超え、|duty| = 7 % が出る）
  h.step();
  CHECK(h.ax.state() == st::HOLDING && h.ax.duty_permille() != 0);
  Harness::accepted(h.disarm());
  CHECK(h.ax.state() == st::DISARMING && !h.ax.brake_release_cmd());
  h.idle(250);  // STALL_MS（200 ms）を超えて DISARMING のまま
  CHECK(h.ax.state() == st::DISARMING);
  h.idle(60);   // BRAKE_APPLY_WAIT_MS（300 ms）で DISARMED
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(h.ax.fault_code() == fc::NONE);
  CHECK(h.fault_events().empty());
  CHECK(h.ax.referenced());
}

// HOLDING で目標が止まっているときに、最小 Duty で機構が動かない（荷重とのつり合い）状態が続いても ENCODER_FAULT にしない。
inline void test_static_hold_error_at_min_duty_is_not_encoder_fault() {
  Harness h;
  h.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;  // 後半の無パルス確認で追従誤差より先に止めない
  c.soft_min_count = -30000;
  c.soft_max_count = 30000;  // 目標 +2000 だと約 0.6 s で参照が止まり、V1_STALL_MS の窓が閉じる前にリセットされる
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;  // 不感帯の出る側（20）を超える静的な誤差 → 下限 7 % の押し当て
  h.drive(400, h.ax.actual_count() + 25);  // 保持目標はそのまま（HOLDING では目標を使わない）。lease を保つ
  CHECK(h.ax.state() == st::HOLDING);
  // 動かないので停止ブースト（Issue #15 H4）が 200 ms ごとに下限を 1 % 上げる: 400 ms で 7 → 8 %（ENCODER_FAULT にはしない）
  CHECK(h.ax.stall_boost_permille() == 10);
  CHECK(h.ax.duty_permille() == 80 || h.ax.duty_permille() == -80);
  CHECK(h.ax.fault_code() == fc::NONE);
  CHECK(h.ax.referenced());
  // 軌道が動き出せば従来どおり無パルスで ENCODER_FAULT（既存の test_encoder_fault_on_no_pulses_while_driving と同じ経路。窓は V1_STALL_MS）
  Harness::accepted(h.start());
  h.drive(800, h.ax.actual_count() + 8000);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
}

// ACTIVE で軌道が目標に到達して止まった後、残り誤差でカウントが動かなくても ENCODER_FAULT にしない。
inline void test_active_settled_with_residual_error_is_not_encoder_fault() {
  Harness h;
  h.reach_active();
  h.drive(300, 100);  // 目標 100 へ到達
  CHECK(h.ax.state() == st::ACTIVE && h.ax.target_count() == 100);
  h.stall = true;
  h.raw -= 25;  // 到達後に 25 count ずれて止まった（不感帯の出る側 20 を超え、|duty| = 7 %。目標は止まっているので無パルス判定なし）
  h.drive(400, 100);
  CHECK(h.ax.state() == st::ACTIVE);
  CHECK(h.ax.duty_permille() != 0);
  CHECK(h.ax.fault_code() == fc::NONE);
}

// Issue #15 H4: 停止ブースト。目標が止まっていて誤差が不感帯の外、下限 Duty でカウントが動かなければ 200 ms ごとに 1 % 上げ、
// duty_max で頭打ち。動き出して不感帯に入れば 0 に戻る。
inline void test_stall_boost_ramps_while_stuck_and_resets_in_band() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  CHECK(h.ax.stall_boost_permille() == 0);
  h.stall = true;
  h.raw -= 25;  // 不感帯の出る側（20）を超える静的な誤差（sign +1 → duty + でカウント減なので、カウントを増やすには duty −）
  h.step();     // 窓の開始
  CHECK(h.ax.duty_permille() == -70 && h.ax.stall_boost_permille() == 0);
  h.drive(200, h.ax.actual_count() + 25);
  CHECK(h.ax.stall_boost_permille() == 10);
  h.step();
  CHECK(h.ax.duty_permille() == -80);
  h.drive(1000, h.ax.actual_count() + 25);
  CHECK(h.ax.stall_boost_permille() == 60);
  h.step();
  CHECK(h.ax.duty_permille() == -130);
  CHECK(h.ax.state() == st::HOLDING && h.ax.fault_code() == fc::NONE);
  // 動き出して不感帯に入れば 0 に戻り、Duty も 0
  h.stall = false;
  h.drive(50, h.ax.target_count());
  CHECK(h.ax.following_error_count() == 0);
  CHECK(h.ax.stall_boost_permille() == 0);
  CHECK(h.ax.duty_permille() == 0);
}

// 上乗せの上限（Codex レビュー HIGH）: 詰まった機構を duty_max で押し続けない。対象になってから V1_STALL_BOOST_MAX_MS（2 s）
// 動かなければ ENCODER_FAULT（出力 0・ブレーキ作動）。FAULT_RESET は通るが基準は消える（protocol.md §4）
inline void test_stall_boost_times_out_to_encoder_fault() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;  // 追従誤差では止めない（25 count の残り誤差は通常の FE 上限より小さい）
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.step();  // 対象になった（t0）
  h.drive(1990, h.ax.actual_count() + 25);
  CHECK(h.ax.state() == st::HOLDING && h.ax.stall_boost_permille() == 90);  // 200 ms × 9 = 1.8 s。2 s の窓はまだ
  h.drive(20, h.ax.actual_count() + 25);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(h.ax.duty_permille() == 0 && !h.ax.brake_release_cmd() && !h.ax.power_on_cmd());
  CHECK(h.ax.stall_boost_permille() == 0);
  const std::vector<v1::FaultEvent> ev = h.fault_events();
  CHECK(ev.size() == 1 && ev[0].fault_code == fc::ENCODER_FAULT && ev[0].mcu_state_before == st::HOLDING);
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED && !h.ax.referenced());
}

// 頭打ち（duty_max）のまま次の窓も動かなければ、2 s を待たずに ENCODER_FAULT
inline void test_stall_boost_at_cap_without_motion_is_encoder_fault() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.set_limits(10, 7, 2, 100, 10, 20));  // 上乗せは最大 3
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.step();
  h.drive(600, h.ax.actual_count() + 25);
  CHECK(h.ax.state() == st::HOLDING && h.ax.stall_boost_permille() == 30);
  h.step();
  CHECK(h.ax.duty_permille() == -100);  // duty_max
  h.drive(200, h.ax.actual_count() + 25);  // 頭打ちのまま 1 窓
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(h.ax.duty_permille() == 0);
}

// 動けば時間の起点も取り直す: ゆっくりでも 3 count 以上/窓 動いていれば 2 s を超えても FAULT にしない
inline void test_stall_boost_timer_restarts_on_motion() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 200;
  h.step();
  for (int i = 0; i < 100; ++i) {  // 3 s。150 ms ごとに 3 count ずつ動く（誤差は 200 → 140）
    h.set_position(h.ax.actual_count());
    h.idle(30);
    if (i % 5 == 4) h.raw += 3;
  }
  CHECK(h.ax.state() == st::HOLDING && h.ax.fault_code() == fc::NONE);
  CHECK(h.ax.stall_boost_permille() == 0);  // 窓が完了する前に動くので上乗せも増えない
}

// duty_max < V1_STALL_DUTY で無パルス判定を無効にしている構成では、上乗せの上限でも FAULT にせず頭打ちのまま
inline void test_stall_boost_cap_without_encoder_fault_when_check_disabled() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.set_limits(8, 7, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_check_disabled());
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.drive(3000, h.ax.actual_count() + 25);
  CHECK(h.ax.state() == st::HOLDING && h.ax.fault_code() == fc::NONE);
  CHECK(h.ax.stall_boost_permille() == 10);
  CHECK(h.ax.duty_permille() == -80);
}

// 対象は |誤差| > deadband_out を明示的に見る（Codex レビュー）: 動いた後（in_band = false）の 10 < |誤差| ≤ 20 の静止誤差は
// 下限 Duty は出るが上乗せしない。不感帯に入ってから（in_band = true）出た 25 は上乗せする
inline void test_stall_boost_requires_error_outside_outer_band() {
  Harness h;
  h.ax.tuning.ki_permille_per_count_s = 0.0;  // E と H4 の境界を見る試験。積分（F）が生の P を動かさないよう切る
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;  // 不感帯の外 → in_band = false、下限 7 %、上乗せ開始
  h.drive(420, h.ax.actual_count() + 25);
  CHECK(h.ax.stall_boost_permille() == 20);
  h.raw += 10;  // 10 count だけ動いて誤差 15: 入る側 10 より大きいので in_band = false のまま。床は |誤差| ≤ 15 で OFF → 生の P 3 permille
  h.drive(1000, h.ax.actual_count() + 15);
  CHECK(h.ax.state() == st::HOLDING && h.ax.fault_code() == fc::NONE);
  CHECK(h.ax.duty_permille() == -3 && !h.ax.duty_terms().floor_on);
  CHECK(h.ax.stall_boost_permille() == 0);  // 20 以下なので上乗せしない（cmd ≠ 0 だけを見ていると 2 s で ENCODER_FAULT になる）
  // 25 に広がれば対象
  h.raw -= 10;
  h.drive(420, h.ax.actual_count() + 25);
  CHECK(h.ax.stall_boost_permille() == 20);
  // 不感帯に入ってから出た履歴（in_band = true → 25 で出る）でも同じ
  Harness g;
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();  // 誤差 0 で in_band = true
  g.stall = true;
  g.raw -= 15;
  g.drive(600, g.ax.actual_count() + 15);
  CHECK(g.ax.duty_permille() == 0 && g.ax.stall_boost_permille() == 0);  // 出る側 20 以下: 不感帯の中
  g.raw -= 10;
  g.drive(420, g.ax.actual_count() + 25);
  CHECK(g.ax.duty_permille() == -90 && g.ax.stall_boost_permille() == 20);
}

// 窓の間に stall_boost_min_move（3）以上動けば窓を取り直す（上乗せは保つ）。それ未満のちらつき（±1）では取り直さない。
inline void test_stall_boost_window_restarts_on_motion_but_ignores_jitter() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 40;
  h.step();
  // ±1 のちらつき: 200 ms 経てば上がる
  for (int i = 0; i < 40; ++i) {
    h.raw += (i % 2 == 0) ? 1 : -1;
    h.set_position(h.ax.actual_count());
    h.step();
  }
  CHECK(h.ax.stall_boost_permille() == 10);
  // 3 count 動いた直後に窓を取り直す: その後 150 ms では上がらない、200 ms で上がる
  h.raw += 3;
  h.step();
  h.drive(150, h.ax.actual_count());
  CHECK(h.ax.stall_boost_permille() == 10);
  h.drive(60, h.ax.actual_count());
  CHECK(h.ax.stall_boost_permille() == 20);
}

// 誤差の符号が反転したら 0 に戻す（反対側へ上乗せを持ち越さない）
inline void test_stall_boost_resets_on_error_sign_flip() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.drive(620, h.ax.actual_count() + 25);
  CHECK(h.ax.stall_boost_permille() == 30);
  h.raw += 50;  // 誤差 +25 → −25
  h.step();     // 反転した tick の Duty から上乗せ無し（反対向きに 10 % のパルスを出さない。Codex レビュー MEDIUM）
  CHECK(h.ax.stall_boost_permille() == 0);
  CHECK(h.ax.duty_permille() == 70);
  // 軌道が動き出す tick も同じ: 上乗せ付きの床を 1 tick 持ち越さない
  Harness g;
  g.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  v1::Config c2 = base_config();
  c2.following_error_limit_count = 100000;
  c2.max_vel_count_s = 200;
  Harness::accepted(g.config_with(c2));
  g.reach_holding_from_configured();
  g.stall = true;
  g.raw -= 25;
  g.drive(620, g.ax.actual_count() + 25);
  CHECK(g.ax.stall_boost_permille() == 30);
  g.set_position(g.ax.actual_count() + 25);
  Harness::accepted(g.start());
  g.set_position(g.ax.actual_count() + 2000);  // 軌道が動き出す
  g.step();
  CHECK(g.ax.state() == st::ACTIVE && g.ax.target_count() > g.ax.actual_count() + 25);
  CHECK(g.ax.stall_boost_permille() == 0);
  CHECK(g.ax.duty_permille() == -70);  // P 0.5 + FF 0.01 → 下限 7（上乗せ 3 を持ち越せば −10）
}

// 軌道が動いている間（FF が効く）とブレーキ作動中（DISARMING）は上乗せしない。duty_max を下げれば上乗せもそこで止まる。
inline void test_stall_boost_not_while_moving_or_brake_applied_and_capped_by_limits() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  c.max_vel_count_s = 200;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;
  h.drive(600, 2000);  // 軌道が動く（200 count/s）。誤差は増えるが上乗せはしない
  CHECK(h.ax.state() == st::ACTIVE && h.ax.target_count() > 100);
  CHECK(h.ax.stall_boost_permille() == 0);
  // 止めて HOLDING に戻し、残り誤差で上乗せが始まる
  Harness::accepted(h.stop());
  h.drive(200, h.ax.target_count());
  CHECK(h.ax.state() == st::HOLDING);
  h.drive(620, h.ax.target_count());
  CHECK(h.ax.stall_boost_permille() >= 30);
  // duty_max 10 / duty_min 7 に下げれば上乗せはその場で 3 に切り詰め、Duty は duty_max
  Harness::accepted(h.set_limits(10, 7, 2, 100, 10, 20));
  h.drive(50, h.ax.target_count());
  CHECK(h.ax.stall_boost_permille() == 30);
  CHECK(h.ax.duty_permille() == 100 || h.ax.duty_permille() == -100);
  // DISARM → ブレーキ作動指令で 0 に戻り（その tick の Duty から）、DISARMING 中に上がらない
  Harness::accepted(h.disarm());
  CHECK(h.ax.state() == st::DISARMING);
  h.step();
  CHECK(h.ax.stall_boost_permille() == 0 && (h.ax.duty_permille() == 70 || h.ax.duty_permille() == -70));
  h.idle(245);
  CHECK(h.ax.state() == st::DISARMING && h.ax.stall_boost_permille() == 0);
  h.idle(60);
  CHECK(h.ax.state() == st::DISARMED && h.ax.fault_code() == fc::NONE);
}

// Issue #25 手順 8 (c)「E」: 床（Duty 下限）はヒステリシス付きの適用状態が ON のときだけ。ON = |誤差| > deadband_out（20）かつ
// 合成と P の向きが一致、OFF = |誤差| ≤ 15 または向きの不一致（即時）。OFF のときは有界の生の FF + P（0 にしない。旧 M1 の 0 置換に代わる）。
inline void test_duty_command_floor_gate_with_hysteresis() {
  V1Tuning t;
  t.kp_percent_per_100 = 6;
  t.duty_min = 9;
  t.duty_max = 20;
  V1DutyState st;
  // Issue の例: err +50（P +30）・vel −200（FF −20）→ 合成 +10 → 向き一致・|誤差| > 20 → 床 ON → 90
  CHECK(v1_duty_command(50, -200.0, st, t) == 90 && st.floor_on);
  // err +50（P +30）・vel −600（FF −60）→ 合成 −30 → 向き不一致 → 即 OFF → 生の −30（旧 M1: 0、それより前: −90 のバースト）
  CHECK(v1_duty_command(50, -600.0, st, t) == -30 && !st.floor_on);
  CHECK(v1_duty_command(-50, 600.0, st, t) == 30 && !st.floor_on);
  // 合成が床以上なら状態に依らずそのまま（減速中に実位置が先行: P −180 + FF +20 = −160）
  CHECK(v1_duty_command(-300, 200.0, st, t) == -160 && st.floor_on);
  // 誤差 0（軌道だけが動いている立ち上がり）: 床 OFF → 生の FF 20（旧: 下限 90）
  CHECK(v1_duty_command(0, 200.0, st, t) == 20 && !st.floor_on);
  // FF だけの追従（小さい誤差）: 生の FF + P。err 5・vel 400 → 40 + 3 = 43
  CHECK(v1_duty_command(5, 400.0, st, t) == 43 && !st.floor_on);
  CHECK(v1_duty_command(-5, -400.0, st, t) == -43 && !st.floor_on);
  // 静止で目標向きの小さい合成: |誤差| > 20 → 床 ON → ±90
  CHECK(v1_duty_command(50, 0.0, st, t) == 90 && st.floor_on);
  CHECK(v1_duty_command(-50, 0.0, st, t) == -90 && st.floor_on);
  // ヒステリシス 15 < |誤差| ≤ 20: ON の履歴なら ON のまま（18 → P 10.8 → 11 < 90 → 90）、15 で OFF（生の 9）、
  // OFF の履歴なら 18 でも OFF（生の 11）、21 で ON
  CHECK(v1_duty_command(18, 0.0, st, t) == 90 && st.floor_on);
  CHECK(v1_duty_command(20, 0.0, st, t) == 90 && st.floor_on);
  CHECK(v1_duty_command(15, 0.0, st, t) == 9 && !st.floor_on);
  CHECK(v1_duty_command(18, 0.0, st, t) == 11 && !st.floor_on);
  CHECK(v1_duty_command(20, 0.0, st, t) == 12 && !st.floor_on);
  CHECK(v1_duty_command(21, 0.0, st, t) == 90 && st.floor_on);
  // 向きの不一致は |誤差| > 20 でも即 OFF。向きが戻れば次の tick で ON
  CHECK(v1_duty_command(50, -600.0, st, t) == -30 && !st.floor_on);
  CHECK(v1_duty_command(50, -200.0, st, t) == 90 && st.floor_on);
  // 合成 0（打ち消し）は向きが無いので OFF、出力 0
  CHECK(v1_duty_command(50, -300.0, st, t) == 0 && !st.floor_on);  // P +30・FF −30
  // 静止の不感帯に入れば床の状態も OFF に戻る。出る側 20 までは 0、21 で床 ON
  CHECK(v1_duty_command(50, 0.0, st, t) == 90 && st.floor_on);
  CHECK(v1_duty_command(5, 0.0, st, t) == 0 && st.in_band && !st.floor_on);
  CHECK(v1_duty_command(18, 0.0, st, t) == 0 && st.in_band && !st.floor_on);
  CHECK(v1_duty_command(21, 0.0, st, t) == 90 && !st.in_band && st.floor_on);
  // 停止ブーストは床が ON のときだけ効く
  CHECK(v1_duty_command(50, 0.0, st, t, 20) == 110 && st.floor_on);
  CHECK(v1_duty_command(50, -600.0, st, t, 20) == -30 && !st.floor_on);
  CHECK(v1_duty_command(0, 200.0, st, t, 20) == 20 && !st.floor_on);
  CHECK(v1_duty_command(50, 0.0, st, t, 500) == 200);  // 床 ＋ 上乗せは上限で頭打ち
  // 生の合成も上限で飽和
  CHECK(v1_duty_command(50, -5000.0, st, t) == -200 && !st.floor_on);
  // 計器: floor_on は状態、cmd は生の値
  V1DutyTerms o;
  CHECK(v1_duty_command(50, -600.0, st, t, 20, &o) == -30);
  CHECK(!o.floor_on && o.p_permille == 30 && o.ff_permille == -60 && o.cmd_permille == -30 && o.boost_permille == 20);
}

// Codex レビュー（PR #28）P2: (1) 向きは P と比べる。kp = 0（SET_LIMITS で受理される）なら P = 0 で床は決して ON にならず、
// FF だけが床まで持ち上がらない。(2) OFF のしきい値は min(15, deadband_out − 1): deadband_out < 15 でも ON と OFF の領域が重ならない。
inline void test_duty_command_floor_requires_p_and_respects_small_deadband_out() {
  V1Tuning t;  // kp 2、ff 100、下限 7、上限 20、deadband 10 / 20
  t.kp_percent_per_100 = 0;
  V1DutyState st;
  // err 21・vel 300: FF 30、P 0 → 床 OFF → 生の 30（旧: 誤差の向きと一致とみなして 70）
  CHECK(v1_duty_command(21, 300.0, st, t) == 30 && !st.floor_on);
  CHECK(v1_duty_command(500, 300.0, st, t) == 30 && !st.floor_on);
  CHECK(v1_duty_command(-500, -300.0, st, t) == -30 && !st.floor_on);
  CHECK(v1_duty_command(500, 0.0, st, t) == 0 && !st.floor_on);  // 静止で kp 0 → 出力 0
  // kp を戻せば従来どおり
  t.kp_percent_per_100 = 2;
  CHECK(v1_duty_command(21, 300.0, st, t) == 70 && st.floor_on);
  // deadband_out 12（deadband 10）: ON は > 12、OFF は ≤ 11（min(15, 12 − 1)）。14 は ON の履歴を保つ、ON 無しなら OFF のまま
  t.deadband_out = 12;
  CHECK(v1_duty_command(14, 0.0, st, t) == 70 && st.floor_on);  // 直前の ON を保つ（12 ≥ 14 ではないので ON の条件は満たさないが OFF にもならない）
  CHECK(v1_duty_command(11, 0.0, st, t) == 2 && !st.floor_on);  // ≤ 11 → OFF → 生の P 2.2 → 2
  CHECK(v1_duty_command(12, 0.0, st, t) == 2 && !st.floor_on);  // OFF の履歴なので OFF のまま
  CHECK(v1_duty_command(13, 0.0, st, t) == 70 && st.floor_on);  // > 12 → ON
  CHECK(v1_duty_command(12, 0.0, st, t) == 70 && st.floor_on);  // 1 count のヒステリシス
  CHECK(v1_duty_command(11, 0.0, st, t) == 2 && !st.floor_on);
  // deadband_out 16: OFF は ≤ 15（既定の floor_off のまま）
  t.deadband_out = 16;
  CHECK(v1_duty_command(17, 0.0, st, t) == 70 && st.floor_on);
  CHECK(v1_duty_command(16, 0.0, st, t) == 70 && st.floor_on);
  CHECK(v1_duty_command(15, 0.0, st, t) == 3 && !st.floor_on);
  // 軸を通して: SET_LIMITS kp 0・deadband_out 12 を受理し、追従中の FF だけ（max_vel 300 → 30 permille）に床が掛からない
  Harness h;
  h.ax.tuning.ki_permille_per_count_s = 0.0;  // kp 0 で FF だけが出ることを見る試験。積分（F）は切る
  h.ax.tuning.assist_ms = 0;  // 模擬プラントは 100 count/s（0.5 count/tick）では動けない＝止まったままなので動き出し補助（T）も切る
  v1::Config c = base_config();
  c.max_vel_count_s = 300;
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.set_limits(20, 7, 0, 100, 10, 12));
  CHECK(h.ax.tuning.kp_percent_per_100 == 0 && h.ax.tuning.deadband_out == 12);
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.plant_vel = 100.0;  // プラントが遅く誤差が増えても（P 0）床は掛からない
  h.drive(600, 5000);
  CHECK(h.ax.state() == st::ACTIVE && h.ax.following_error_count() < -20);
  CHECK(h.ax.pwm_permille() == 30 && !h.ax.duty_terms().floor_on && h.ax.duty_terms().p_permille == 0);
}

// 軸を通した E: 追従中（誤差が小さい）は床ではなく生の FF が出る。静止の残り誤差 15 では床 OFF で生の P、25 で床 ON。
inline void test_axis_tracking_uses_raw_ff_and_floor_follows_hysteresis() {
  Harness h;
  v1::Config c = base_config();
  c.max_vel_count_s = 400;  // FF 4 % = 40 permille（床 70 未満）
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.drive(600, 5000);  // 速度 400 count/s で追従（プラントは毎 tick 追いつく）
  CHECK(h.ax.state() == st::ACTIVE);
  const V1DutyTerms& tm = h.ax.duty_terms();
  CHECK(tm.vel_count_s == 400 && tm.ff_permille == 40 && !tm.floor_on && !tm.in_band);
  CHECK(h.ax.pwm_permille() >= 38 && h.ax.pwm_permille() <= 46);  // 生の FF（＋小さい P）。旧: 下限 70 のバースト
  CHECK(h.ax.pwm_permille() == tm.cmd_permille);
  // 止めて HOLDING。残り誤差 25 → 床 ON（70）、誤差 15 → OFF（生の P 3）、18 → OFF のまま（4）、25 → ON
  Harness::accepted(h.stop());
  h.drive(200, h.ax.target_count());
  CHECK(h.ax.state() == st::HOLDING && h.ax.duty_permille() == 0);
  h.stall = true;
  h.raw -= 25;
  h.step();
  CHECK(h.ax.duty_permille() == -70 && h.ax.duty_terms().floor_on);
  h.raw += 10;  // 誤差 15
  h.step();
  CHECK(h.ax.duty_permille() == -3 && !h.ax.duty_terms().floor_on);
  h.raw -= 3;  // 誤差 18: OFF の履歴なので OFF のまま
  h.step();
  CHECK(h.ax.duty_permille() == -4 && !h.ax.duty_terms().floor_on);
  h.raw -= 7;  // 誤差 25
  h.step();
  CHECK(h.ax.duty_permille() == -70 && h.ax.duty_terms().floor_on);
  h.raw += 7;  // 誤差 18: ON の履歴なので ON のまま
  h.step();
  CHECK(h.ax.duty_permille() == -70 && h.ax.duty_terms().floor_on);
  CHECK(h.ax.fault_code() == fc::NONE);
}

// Issue #15 M2: ENCODER_FAULT のしきい値は max(V1_STALL_DUTY, duty_max × 75 %)。duty_max < 10 なら検出が無効（警告）。
inline void test_encoder_fault_threshold_follows_duty_max() {
  Harness h;
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 150 && !h.ax.encoder_fault_check_disabled());  // 20 × 75 % = 15
  Harness::accepted(h.set_limits(40, 3, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 300);
  Harness::accepted(h.set_limits(12, 3, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 100 && !h.ax.encoder_fault_check_disabled());  // 9 → 下限 10
  Harness::accepted(h.set_limits(8, 3, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 100 && h.ax.encoder_fault_check_disabled());   // 受理はするが検出は出ない
}

// kp 6 で動けない軸: 誤差 170 で 10 % には届くが 15 %（duty_max 20 の 75 %）には届かず、FOLLOWING_ERROR で止まる
// （旧: 10 % + 200 ms で ENCODER_FAULT → 基準喪失）。ゆっくり動く軌道（200 count/s）に対して機構が止まったまま。
inline void test_stuck_axis_with_high_kp_is_following_error_not_encoder_fault() {
  Harness h;
  v1::Config c = base_config();
  c.max_vel_count_s = 200;
  c.following_error_limit_count = 240;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.set_limits(20, 9, 6, 100, 10, 20));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;
  // 旧しきい値 10 % には err ≈ 134（P 8 + FF 2）で届く（t ≈ 0.67 s）→ 旧なら 0.87 s で ENCODER_FAULT
  h.drive(900, 2000);
  CHECK(h.ax.state() == st::ACTIVE);
  CHECK(h.ax.duty_permille() <= -100);
  // 新しきい値 15 % には err ≈ 217（t ≈ 1.08 s）→ 1.28 s。FOLLOWING_ERROR 240 は t = 1.2 s で先に出る
  //（SET_POSITION を送り続けて lease を保つ。止めると COMMAND_TIMEOUT が先に出る）
  h.drive(500, 2000);
  CHECK(h.ax.state() == st::FAULT);
  CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR);
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.referenced());  // FOLLOWING_ERROR では基準を保つ
}

// device_id は 0x5131 + MAC を表記順に並べたもの（画面の下 4 桁 = MAC の末尾 2 オクテット）
inline void test_device_id_packs_mac_in_printed_order() {
  const uint8_t mac[6] = {0x30, 0xED, 0xA0, 0x0B, 0x20, 0x40};
  const uint64_t id = v1_device_id_from_mac(mac);
  CHECK(id == 0x513130EDA00B2040ull);
  CHECK((id & 0xFFFF) == 0x2040);
  CHECK((id >> 48) == 0x5131);
  const uint8_t mac2[6] = {0x1C, 0xDB, 0xD4, 0xBA, 0x8D, 0x58};
  CHECK(v1_device_id_from_mac(mac2) == 0x51311CDBD4BA8D58ull);
}

inline void test_control_overrun_faults_only_when_not_parked() {
  Harness h;
  h.idle(100);
  h.step(50);  // DISARMED での長い間隔は異常にしない
  CHECK(h.ax.state() == st::DISARMED);
  h.reach_holding();
  h.set_position(0);
  h.step(16);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::CONTROL_OVERRUN);
  h.step(50);
  CHECK(h.fault_events().size() == 1);
}

// ---- HOST_FAULT / FAULT_RESET / 異常ゲート ----------------------------------

inline void test_host_fault_latches_immediately_from_active() {
  Harness h;
  h.reach_active();
  h.drive(100, 2000);
  const std::vector<v1::McuMsg> r = h.send(h.host_fault_msg(h.next_rid()));
  // FAULT_EVENT → ACK → STATUS(FAULT) の順（ACK より後の STATUS は遷移後）
  CHECK(r.size() == 3 && r[0].type == v1::msg::FAULT_EVENT && r[1].type == v1::msg::ACK &&
        r[2].type == v1::msg::STATUS && r[2].status.mcu_state == st::FAULT && r[2].status.fault_code == fc::HOST_FAULT);
  // 直後に即時 STATUS（fault_code が変わった）… FAULT_EVENT → ACK → STATUS の順を許す
  bool ack_seen = false;
  bool status_after = false;
  for (const v1::McuMsg& m : h.out) {
    if (m.type == v1::msg::ACK && m.ack.acked_type == v1::msg::HOST_FAULT) ack_seen = true;
  }
  h.step();
  for (size_t i = 0; i < h.out.size(); ++i) {
    if (h.out[i].type == v1::msg::STATUS && i > 0 && h.out[i].status.mcu_state == st::FAULT) status_after = true;
  }
  CHECK(ack_seen && status_after);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::HOST_FAULT);
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  const std::vector<v1::FaultEvent> ev = h.fault_events();
  CHECK(ev.size() == 1 && ev[0].fault_code == fc::HOST_FAULT && ev[0].mcu_state_before == st::ACTIVE);
  // 2 回目の HOST_FAULT は ACCEPTED だが FAULT_EVENT は増えない
  Harness::accepted(h.host_fault());
  CHECK(h.fault_events().size() == 1);
}

inline void test_fault_gate_rejects_commands() {
  Harness h;
  h.reach_holding();
  Harness::accepted(h.host_fault());
  Harness::rejected(h.config(), rs::FAULT_ACTIVE);
  Harness::rejected(h.arm(), rs::FAULT_ACTIVE);
  Harness::rejected(h.start(), rs::FAULT_ACTIVE);
  Harness::rejected(h.stop(), rs::FAULT_ACTIVE);
  Harness::rejected(h.disarm(), rs::FAULT_ACTIVE);
  Harness::rejected(h.set_reference(0), rs::FAULT_ACTIVE);
  CHECK(h.hello().size() >= 1);  // HELLO は常に返す
}

inline void test_fault_reset_rules() {
  Harness h;
  Harness::rejected(h.fault_reset(), rs::BAD_STATE);
  h.reach_holding();
  Harness::rejected(h.fault_reset(), rs::BAD_STATE);
  Harness::accepted(h.host_fault());
  const std::vector<v1::McuMsg> r = h.send([&] {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::FAULT_RESET;
    m.fault_reset.request_id = h.next_rid();
    return m;
  }());
  status_after_ack(r, [](const v1::Status& s) { return s.mcu_state == st::DISARMED && s.fault_code == fc::NONE; });
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(h.ax.configured() && h.ax.referenced());  // CONFIG と基準は残る
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd());
  h.reach_holding_from_configured();  // 自動再 ARM はしないが、PC から ARM できる
}

// ---- HOST_FAULT の共通規則の例外・DUPLICATE ----------------------------------

inline void test_host_fault_with_request_id_zero_and_reused_id_still_latches() {
  Harness h;
  h.reach_holding();
  Harness::accepted(h.cmd(h.host_fault_msg(0)));
  CHECK(h.ax.state() == st::FAULT);
  Harness::accepted(h.fault_reset());
  h.reach_holding_from_configured();
  Harness::accepted(h.cmd(h.host_fault_msg(55)));
  Harness::accepted(h.fault_reset());
  h.reach_holding_from_configured();
  Harness::accepted(h.cmd(h.host_fault_msg(55)));  // 同じ内容でも DUPLICATE にしない
  CHECK(h.ax.state() == st::FAULT);
}

inline void test_duplicate_request_is_not_reexecuted() {
  Harness h;
  h.reach_holding();
  const uint32_t r = h.next_rid();
  Harness::accepted(h.cmd(h.disarm_msg(r)));
  CHECK(h.ax.state() == st::DISARMING);
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED);
  h.reach_holding_from_configured();
  Harness::duplicate(h.cmd(h.disarm_msg(r)));  // 再送は実行しない
  CHECK(h.ax.state() == st::HOLDING);
}

inline void test_same_request_id_with_new_content_is_executed() {
  Harness h;
  const uint32_t r = h.next_rid();
  v1::HostMsg m;
  memset(&m, 0, sizeof(m));
  m.type = v1::msg::CONFIG;
  m.config = base_config();
  m.config.request_id = r;
  Harness::accepted(h.cmd(m));
  m.config.soft_max_count = 2000;  // 同じ request_id、違う内容
  Harness::accepted(h.cmd(m));
  CHECK(h.ax.config()->soft_max_count == 2000);
  Harness::duplicate(h.cmd(m));
  // 種別が違えば別の命令
  Harness::accepted(h.cmd(h.disarm_msg(r)));
}

inline void test_resent_rejected_command_is_reevaluated() {
  Harness h;
  v1::HostMsg m;
  memset(&m, 0, sizeof(m));
  m.type = v1::msg::ARM_PREPARE;
  m.arm_prepare.request_id = h.next_rid();
  m.arm_prepare.session_epoch = EPOCH;
  Harness::rejected(h.cmd(m), rs::NOT_CONFIGURED);
  Harness::accepted(h.config());
  Harness::accepted(h.cmd(m));  // 拒否は記憶しないので改めて評価
  CHECK(h.ax.state() == st::ARMING);
}

inline void test_dedupe_window_is_bounded() {
  Harness h;
  const uint32_t first = h.next_rid();
  Harness::accepted(h.cmd(h.disarm_msg(first)));
  for (size_t i = 0; i < V1Axis::kDedupeDepth - 1; ++i) Harness::accepted(h.cmd(h.disarm_msg(h.next_rid())));
  Harness::duplicate(h.cmd(h.disarm_msg(first)));  // まだ窓内（16 件目）
  Harness::accepted(h.cmd(h.disarm_msg(h.next_rid())));  // 17 件目で最古が落ちる… ただし first は duplicate 判定で
                                                          // 位置を変えないので、ここで落ちるのは first
  Harness::accepted(h.cmd(h.disarm_msg(first)));
}

// ---- SET_REFERENCE -------------------------------------------------------------

inline void test_set_reference_shifts_coordinate_without_sign() {
  // sign = −1 の系統でも生のカウントにそのまま基準を当てる
  Harness h(1, -1, 1000, false);
  CHECK(h.ax.actual_count() == 1000);
  Harness::accepted(h.set_reference(-2770));
  CHECK(h.ax.actual_count() == -2770);
  CHECK(h.ax.referenced());
  CHECK(h.ax.target_count() == -2770);
  h.raw += 100;  // カウント増 → actual も増（符号を掛けない）
  h.step();
  CHECK(h.ax.actual_count() == -2670);
  // sign = +1 でも同じ
  Harness g(1, 1, 500, false);
  Harness::accepted(g.set_reference(0));
  g.raw -= 30;
  g.step();
  CHECK(g.ax.actual_count() == -30);
}

inline void test_set_reference_only_in_disarmed() {
  Harness h;
  Harness::accepted(h.config());
  Harness::accepted(h.arm());
  Harness::rejected(h.set_reference(10), rs::BAD_STATE);
  CHECK(h.ax.actual_count() == 0);
}

// ---- 系統選択 -------------------------------------------------------------------

inline void test_select_axis_in_disarmed_clears_config_and_reference_and_announces_once() {
  Harness h(1);
  Harness::accepted(h.config());
  h.raw = 777;
  h.step();
  Harness::accepted(h.set_reference(777));
  CHECK(h.ax.configured() && h.ax.referenced());
  const size_t before = h.out.size();
  CHECK(h.ax.select_axis(2, h.now));
  h.drain();
  const std::vector<v1::McuMsg> r(h.out.begin() + static_cast<long>(before), h.out.end());
  CHECK(r.size() == 2);
  CHECK(r[0].type == v1::msg::AXIS_SELECT && r[0].axis_select.selected_axis == 2 &&
        r[0].axis_select.prev_selected_axis == 1 && r[0].axis_select.mcu_state == st::DISARMED &&
        r[0].axis_select.boot_id == 2309737967u);
  CHECK(r[1].type == v1::msg::STATUS && r[1].status.selected_axis == 2 &&
        (r[1].status.flags & (fl::CONFIGURED | fl::POSITION_REFERENCED)) == 0);
  CHECK(!h.ax.configured() && !h.ax.referenced());
  CHECK(h.ax.calibration_hash() == 0);
  CHECK(h.ax.actual_count() == 777);  // カウントは触らない
  CHECK(h.ax.selected_axis() == 2);
  h.idle(100);
  CHECK(h.axis_selects().size() == 1);
  for (const v1::McuMsg& m : h.hello()) {
    if (m.type == v1::msg::HELLO) CHECK(m.hello.selected_axis == 2 && m.hello.calibration_hash == 0);
  }
  // 同じ値は何もしない
  CHECK(h.ax.select_axis(2, h.now));
  h.drain();
  CHECK(h.axis_selects().size() == 1);
  // ARM には CONFIG と基準の取り直しが要る
  Harness::rejected(h.arm(), rs::NOT_CONFIGURED);
}

inline void test_select_axis_is_ignored_outside_disarmed() {
  Harness h(1);
  h.reach_holding();
  CHECK(!h.ax.select_axis(3, h.now));
  h.drain();
  CHECK(h.ax.selected_axis() == 1);
  CHECK(h.axis_selects().empty());
  CHECK(h.ax.configured());
  Harness::accepted(h.host_fault());
  CHECK(!h.ax.select_axis(3, h.now));  // FAULT でも切り替えない
  CHECK(h.ax.selected_axis() == 1);
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.select_axis(3, h.now));
  CHECK(h.ax.selected_axis() == 3);
}

// ---- STATUS -------------------------------------------------------------------

inline void test_status_after_an_ack_shows_the_post_command_state() {
  Harness h;
  v1::HostMsg c;
  memset(&c, 0, sizeof(c));
  c.type = v1::msg::CONFIG;
  c.config = base_config();
  c.config.request_id = h.next_rid();
  status_after_ack(h.send(c), [](const v1::Status& s) { return (s.flags & fl::CONFIGURED) != 0; });
  v1::HostMsg a;
  memset(&a, 0, sizeof(a));
  a.type = v1::msg::ARM_PREPARE;
  a.arm_prepare.request_id = h.next_rid();
  a.arm_prepare.session_epoch = EPOCH;
  status_after_ack(h.send(a), [](const v1::Status& s) { return s.mcu_state == st::ARMING; });
  h.drive(620, 0);
  h.set_position(0);
  v1::HostMsg s;
  memset(&s, 0, sizeof(s));
  s.type = v1::msg::START;
  s.start.request_id = h.next_rid();
  s.start.session_epoch = EPOCH;
  status_after_ack(h.send(s), [](const v1::Status& x) { return x.mcu_state == st::ACTIVE; });
  h.drive(20, 300);
  v1::HostMsg cs;
  memset(&cs, 0, sizeof(cs));
  cs.type = v1::msg::CONTROLLED_STOP;
  cs.controlled_stop.request_id = h.next_rid();
  cs.controlled_stop.session_epoch = EPOCH;
  cs.controlled_stop.reason = 1;
  status_after_ack(h.send(cs), [](const v1::Status& x) { return x.mcu_state == st::STOPPING; });
  status_after_ack(h.send(h.host_fault_msg(h.next_rid())),
                   [](const v1::Status& x) { return x.mcu_state == st::FAULT && x.fault_code != fc::NONE; });
  v1::HostMsg fr;
  memset(&fr, 0, sizeof(fr));
  fr.type = v1::msg::FAULT_RESET;
  fr.fault_reset.request_id = h.next_rid();
  status_after_ack(h.send(fr),
                   [](const v1::Status& x) { return x.mcu_state == st::DISARMED && x.fault_code == fc::NONE; });
}

inline void test_periodic_status_every_20ms() {
  Harness h;
  const size_t before = h.statuses().size();
  h.idle(200);  // 5 ms 刻みで 40 tick → 20 ms ごとに 10 個
  const size_t after = h.statuses().size();
  CHECK(after - before == 10);
  // mcu_time_us は起動からの経過
  CHECK(h.statuses().back().mcu_time_us == h.now);
}

inline void test_status_reports_following_error_and_pwm_sign() {
  Harness h;
  h.reach_active();
  h.stall = true;
  h.drive(50, 1000);  // 目標が +へ動く（50 ms で約 25 カウント、デッドバンドの外）が actual は 0 のまま
  const v1::Status s = h.status();
  CHECK(s.target_count > s.actual_count);
  CHECK(s.following_error_count == s.actual_count - s.target_count);
  CHECK(s.following_error_count < 0);
  CHECK(s.pwm_permille > 0);  // 正が count の増える向き
}

// ---- Duty の符号・大きさ ----------------------------------------------------------

inline void test_duty_sign_follows_profile_sign() {
  // sign = +1（速度 − でカウント増）: 目標がカウント増の向きなら Duty は負
  Harness h(1, 1);
  h.reach_holding();
  h.stall = true;
  h.raw = -100;  // actual が目標 0 より下 → カウントを増やしたい
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() < 0);
  CHECK(h.ax.pwm_permille() > 0);
  CHECK(h.ax.duty_permille() == -V1_DUTY_MIN_PERMILLE);  // |err| 100 × 2 / 100 = 2 % → 下限 7 %
  h.raw = 100;
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() > 0 && h.ax.pwm_permille() < 0);
  // sign = −1 では逆
  Harness g(1, -1);
  g.reach_holding();
  g.stall = true;
  g.raw = -100;
  g.set_position(0);
  g.step();
  CHECK(g.ax.duty_permille() > 0);
  CHECK(g.ax.pwm_permille() > 0);
}

// フィードフォワード（docs/multi_axis_20261031.md の実測: 1 % ≈ 100〜130 count/s）。純粋関数を直接検査する。
inline void test_duty_command_feed_forward() {
  V1Tuning t;
  V1DutyState band;
  CHECK(v1_duty_command(0, 600.0, band, t) == 60);   // FF 6 %。誤差 0 なので床は OFF → 生の 60 permille（旧: 下限 7 %）
  CHECK(v1_duty_command(0, 1500.0, band, t) == 150);            // 速度だけで 15 %
  CHECK(v1_duty_command(0, 2500.0, band, t) == V1_DUTY_MAX_PERMILLE);   // 25 % → 上限 20 %
  CHECK(v1_duty_command(300, 600.0, band, t) == 120);           // 6 + 6
  CHECK(v1_duty_command(-300, 600.0, band, t) == 0);           // 誤差が速度と逆向きで打ち消し → 0
  CHECK(v1_duty_command(0, -600.0, band, t) == -60);  // 向きは速度の符号
  CHECK(v1_duty_command(-100, -1000.0, band, t) == -120);       // −2 − 10
  // 目標が動いている間は不感帯を使わない
  band.in_band = true;
  CHECK(v1_duty_command(3, 600.0, band, t) == 61);  // 60 + 0.6 → 61。床は OFF（|誤差| 3 ≤ 15）
  CHECK(!band.in_band);
  // 止まったら不感帯（入る 10 / 出る 20）
  CHECK(v1_duty_command(10, 0.0, band, t) == 0 && band.in_band);
  CHECK(v1_duty_command(20, 0.0, band, t) == 0 && band.in_band);
  CHECK(v1_duty_command(21, 0.0, band, t) == V1_DUTY_MIN_PERMILLE && !band.in_band && band.floor_on);  // 床 ON
  CHECK(v1_duty_command(11, 0.0, band, t) == 2 && !band.in_band && !band.floor_on);  // 不感帯の外だが |誤差| ≤ 15 で床 OFF → 生の P 2.2 → 2
  CHECK(v1_duty_command(10, 0.0, band, t) == 0 && band.in_band);
  // ff = 0 なら比例だけ
  V1Tuning p;
  p.ff_count_s_per_percent = 0;
  V1DutyState b2;
  CHECK(v1_duty_command(0, 600.0, b2, p) == 0);
  CHECK(v1_duty_command(500, 600.0, b2, p) == 100);
}

// Issue #25 手順 8 (a): 位置ループの各項の計器。v1_duty_command の結果は変えず、FF / P / 軌道速度 / 下限の適用 / 不感帯を書く。
inline void test_duty_terms_report_ff_p_floor_and_band() {
  V1Tuning t;  // kp 2、ff 100、下限 7、上限 20
  V1DutyState band;
  V1DutyTerms o;
  // FF 6 % + P 6 % = 12 %。床の状態は ON（|誤差| > 20・向き一致）だが合成が床以上なのでそのまま
  CHECK(v1_duty_command(300, 600.0, band, t, 0, &o) == 120);
  CHECK(o.ff_permille == 60 && o.p_permille == 60 && o.vel_count_s == 600 && o.floor_on && !o.in_band &&
        o.cmd_permille == 120);
  // FF 2 % だけ（誤差 0）→ 床は OFF → 生の 20
  CHECK(v1_duty_command(0, 200.0, band, t, 0, &o) == 20);
  CHECK(o.ff_permille == 20 && o.p_permille == 0 && !o.floor_on && o.cmd_permille == 20);
  // 負の向き: P −1.2 %・FF −2 % → 合成 −3.2 % → 下限 −7 %
  CHECK(v1_duty_command(-60, -200.0, band, t, 0, &o) == -70);
  CHECK(o.ff_permille == -20 && o.p_permille == -12 && o.floor_on && o.cmd_permille == -70 && o.vel_count_s == -200);
  // 誤差と逆向きの合成 → 床 OFF → 生の −50
  CHECK(v1_duty_command(50, -600.0, band, t, 0, &o) == -50);
  CHECK(o.ff_permille == -60 && o.p_permille == 10 && !o.floor_on && o.cmd_permille == -50);
  // 上限で飽和: FF 25 % → 20 %
  CHECK(v1_duty_command(0, 2500.0, band, t, 0, &o) == 200);
  CHECK(o.ff_permille == 250 && o.cmd_permille == 200 && !o.floor_on);
  // 静止の不感帯の中: in_band が立ち、各項は 0
  CHECK(v1_duty_command(5, 0.0, band, t, 0, &o) == 0 && band.in_band);
  CHECK(o.in_band && o.ff_permille == 0 && o.p_permille == 0 && o.cmd_permille == 0 && !o.floor_on);
  // 軌道速度は int16 に飽和
  CHECK(v1_duty_command(0, 40000.0, band, t, 0, &o) == 200);
  CHECK(o.vel_count_s == 32767);
  // 軸経由: 位置制御が止まっていれば全部 0、HOLDING で誤差 25（静止）なら P 0.5 % → 下限 7 %、floor_on
  Harness h;
  CHECK(h.ax.duty_terms().cmd_permille == 0 && !h.ax.duty_terms().floor_on);
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.step();
  const V1DutyTerms& a = h.ax.duty_terms();
  CHECK(a.p_permille == 5 && a.ff_permille == 0 && a.vel_count_s == 0 && a.floor_on && !a.in_band &&
        a.cmd_permille == 70 && a.boost_permille == 0);
  CHECK(h.ax.pwm_permille() == 70);  // sign +1: duty −7 % → カウント増の向きに +70
  // 停止ブーストが 1 % 乗れば boost_permille 10、cmd 80
  h.drive(200, h.ax.actual_count() + 25);
  h.step();
  CHECK(h.ax.duty_terms().boost_permille == 10 && h.ax.duty_terms().cmd_permille == 80 && h.ax.duty_terms().floor_on);
  // 位置制御が切れれば 0 に戻る
  Harness::accepted(h.host_fault());
  CHECK(h.ax.duty_terms().cmd_permille == 0 && h.ax.duty_terms().boost_permille == 0 && !h.ax.duty_terms().floor_on);
}

// Issue #25 手順 8 (b): 位置ループは permille で運ぶ。5.3 % は 53 permille、四捨五入、±duty_max で飽和、向きは変えない。
inline void test_duty_command_permille_resolution_and_saturation() {
  V1Tuning t;  // kp 2、ff 100、下限 7 %、上限 20 %
  t.duty_min = 0;  // 下限なしで生の分解能を見る
  V1DutyState band;
  CHECK(v1_duty_command(265, 0.0, band, t) == 53);    // P 5.3 % → 53 permille（旧: 5 %）
  CHECK(v1_duty_command(-265, 0.0, band, t) == -53);
  CHECK(v1_duty_command(0, 530.0, band, t) == 53);    // FF 5.3 %
  CHECK(v1_duty_command(345, 0.0, band, t) == 69);    // 6.9 %
  CHECK(v1_duty_command(0, 5.0, band, t) == 1);       // 0.05 % = 0.5 permille → 四捨五入で 1
  CHECK(v1_duty_command(0, 4.0, band, t) == 0);       // 0.4 permille → 0
  CHECK(v1_duty_command(0, -5.0, band, t) == -1);     // 0 から遠い側へ
  // 飽和: ±duty_max × 10
  CHECK(v1_duty_command(100000, 0.0, band, t) == V1_DUTY_MAX_PERMILLE);
  CHECK(v1_duty_command(-100000, 0.0, band, t) == -V1_DUTY_MAX_PERMILLE);
  CHECK(v1_duty_command(0, 100000.0, band, t) == 200 && v1_duty_command(0, -100000.0, band, t) == -200);
  t.duty_max = 60;
  CHECK(v1_duty_command(100000, 0.0, band, t) == 600 && v1_duty_command(-100000, 0.0, band, t) == -600);
  // 下限の境界（permille で比較）: 69 → 70 に持ち上げ、70 はそのまま、71 もそのまま
  V1Tuning f;  // 下限 7 % = 70 permille
  CHECK(v1_duty_command(345, 0.0, band, f) == 70);
  CHECK(v1_duty_command(350, 0.0, band, f) == 70);
  CHECK(v1_duty_command(355, 0.0, band, f) == 71);
  CHECK(v1_duty_command(-355, 0.0, band, f) == -71);
  // 上乗せも permille: 下限 70 + 25 = 95（上限を超えれば上限）
  CHECK(v1_duty_command(100, 0.0, band, f, 25) == 95);
  CHECK(v1_duty_command(100, 0.0, band, f, 500) == 200);
  // 軸を通して: SET_LIMITS で下限 0 % → 誤差 265 の P 5.3 % が pwm_permille 53 で出る（sign +1 なので Duty は −53）
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  Harness::accepted(h.set_limits(20, 0, 2, 100, 10, 20));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 265;
  h.step();
  CHECK(h.ax.duty_permille() == -53 && h.ax.pwm_permille() == 53);
  CHECK(h.status().pwm_permille == 53);  // STATUS も実際の permille（旧: 整数 % × 10 = 50）
  h.raw += 530;  // 誤差 −265 → 向きが変わる
  h.set_position(h.ax.target_count());
  h.step();
  CHECK(h.ax.duty_permille() == 53 && h.ax.pwm_permille() == -53);
  // 符号 −1 の系統では Duty の向きが逆
  Harness g(1, -1);
  Harness::accepted(g.config_with(c));
  Harness::accepted(g.set_limits(20, 0, 2, 100, 10, 20));
  g.reach_holding_from_configured();
  g.stall = true;
  g.raw -= 265;
  g.step();
  CHECK(g.ax.duty_permille() == 53 && g.ax.pwm_permille() == 53);
}

// ENCODER_FAULT のしきい値は permille でも整数 % のときと同じ効き目（整数 % で切り捨ててから × 10。duty_max 14 → 10 % → 100）
inline void test_encoder_fault_threshold_permille_matches_percent() {
  Harness h;
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 150);
  Harness::accepted(h.set_limits(14, 3, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 100 && !h.ax.encoder_fault_check_disabled());
  Harness::accepted(h.set_limits(15, 3, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 110);  // 11.25 → 11 %
  Harness::accepted(h.set_limits(60, 3, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 450);
  Harness::accepted(h.set_limits(9, 3, 2, 100, 10, 20));
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 100 && h.ax.encoder_fault_check_disabled());
}

// motor_output の純粋関数: ±1000 permille がフルスケール、53 permille は 54 / 1023、反転の判定は符号が違うときだけ
inline void test_motor_output_permille_mapping() {
  const uint32_t full = (1u << PWM_BITS) - 1;  // 1023
  CHECK(full == 1023);
  CHECK(motor_output_pwm_ticks(1000, full) == 1023 && motor_output_pwm_ticks(-1000, full) == 1023);
  CHECK(motor_output_pwm_ticks(0, full) == 0);
  CHECK(motor_output_pwm_ticks(53, full) == 54 && motor_output_pwm_ticks(-53, full) == 54);  // 53 × 1023 / 1000 = 54.2
  CHECK(motor_output_pwm_ticks(500, full) == 511);
  CHECK(motor_output_pwm_ticks(70, full) == 71);    // 旧 7 %: 7 × 1023 / 100 = 71（同じ）
  CHECK(motor_output_pwm_ticks(200, full) == 204);  // 旧 20 %: 204（同じ）
  CHECK(motor_output_pwm_ticks(1500, full) == 1023);  // 範囲外は飽和
  CHECK(motor_output_pwm_ticks(1, full) == 1);
  CHECK(motor_output_reverses(70, -70) && motor_output_reverses(-1, 1));
  CHECK(!motor_output_reverses(0, -70) && !motor_output_reverses(70, 0) && !motor_output_reverses(70, 80) &&
        !motor_output_reverses(-70, -1) && !motor_output_reverses(0, 0));
  CHECK(PERMILLE_MIN == -1000 && PERMILLE_MAX == 1000 && V1_ABS_DUTY_MAX_PERMILLE == 600 &&
        V1_STALL_DUTY_PERMILLE == 100 && V1_STALL_BOOST_STEP_PERMILLE == 10 && V1_DUTY_MIN_PERMILLE == 70 &&
        V1_DUTY_MAX_PERMILLE == 200);
}

inline void test_duty_deadband_and_clamp() {
  Harness h;
  h.ax.tuning.settled_ms = 0;  // 不感帯のヒステリシスそのものを見る試験。整定判定（S）の 200 ms は test_settled_* で
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw = -5;  // デッドバンド内（入る側 10）
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == 0 && h.ax.pwm_permille() == 0);
  h.raw = -15;  // 出る側 20 までは再駆動しない（ヒステリシス）
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == 0);
  h.raw = -25;  // 出る側を超えた → 下限 7 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -V1_DUTY_MIN_PERMILLE);
  h.raw = -15;  // 駆動中は入る側 10 まで不感帯に入らないが、床は |誤差| ≤ 15 で OFF → 生の P 3 permille（Issue #25 手順 8 c）
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -3);
  h.raw = -10;
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == 0);
  h.raw = -800;  // 800 × 2 / 100 = 16 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -160);
  h.raw = -2500;  // 50 % → 上限 20 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -V1_DUTY_MAX_PERMILLE);
  CHECK(h.ax.pwm_permille() == V1_DUTY_MAX * 10);
  // 位置制御が切れれば 0
  Harness::accepted(h.host_fault());
  CHECK(h.ax.duty_permille() == 0);
}

// ---- LOCAL_ESTOP ------------------------------------------------------------------

inline void test_local_estop_latches_and_blocks_reset_until_released() {
  Harness h;
  h.reach_holding();
  h.ax.set_local_estop(true, h.now);
  h.drain();
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::LOCAL_ESTOP);
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  const std::vector<v1::FaultEvent> ev = h.fault_events();
  CHECK(ev.size() == 1 && ev[0].fault_code == fc::LOCAL_ESTOP && ev[0].mcu_state_before == st::HOLDING);
  CHECK(h.statuses().back().mcu_state == st::FAULT && h.statuses().back().fault_code == fc::LOCAL_ESTOP);
  Harness::rejected(h.fault_reset(), rs::FAULT_CAUSE_PRESENT);
  CHECK(h.ax.state() == st::FAULT);
  h.ax.set_local_estop(false, h.now);
  h.drain();
  CHECK(h.ax.state() == st::FAULT);  // 解除だけでは戻らない
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED);
  CHECK(h.ax.configured() && h.ax.referenced());
  CHECK(h.fault_events().size() == 1);
}

inline void test_local_estop_in_disarmed_and_during_other_fault() {
  Harness h;
  h.ax.set_local_estop(true, h.now);
  h.drain();
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::LOCAL_ESTOP);
  CHECK(h.fault_events().size() == 1 && h.fault_events()[0].mcu_state_before == st::DISARMED);
  h.ax.set_local_estop(false, h.now);
  Harness::accepted(h.fault_reset());
  // 既に FAULT（HOST_FAULT）のときの非常停止: コードは保ち、解除まで FAULT_RESET を拒む
  h.reach_holding();
  Harness::accepted(h.host_fault());
  h.ax.set_local_estop(true, h.now);
  h.drain();
  CHECK(h.ax.fault_code() == fc::HOST_FAULT);
  Harness::rejected(h.fault_reset(), rs::FAULT_CAUSE_PRESENT);
  h.ax.set_local_estop(false, h.now);
  Harness::accepted(h.fault_reset());
  CHECK(h.fault_events().size() == 2);
}

// ---- フレーム入力・outbox -----------------------------------------------------------

inline void test_bad_frames_counted_and_unanswered() {
  Harness h;
  const size_t before = h.out.size();
  const uint8_t junk[] = {0x04, 0x01, 0x07, 0x04, 0x02, 0x07, 0x01, 0x01, 0x03, 0x25, 0x39};  // CRC 違い
  h.ax.handle_frame(junk, sizeof(junk), h.now);
  h.drain();
  CHECK(h.out.size() == before);
  CHECK(h.ax.rx_error_count() == 1);
  // 正しいフレーム（DISARM request_id 7）は ACK
  const uint8_t good[] = {0x04, 0x01, 0x07, 0x04, 0x02, 0x07, 0x01, 0x01, 0x03, 0x25, 0x38};
  h.ax.handle_frame(good, sizeof(good), h.now);
  h.drain();
  CHECK(h.out.size() == before + 1 && h.out.back().type == v1::msg::ACK && h.out.back().ack.request_id == 7);
  h.ax.record_rx_error();
  CHECK(h.ax.rx_error_count() == 2);
  CHECK(h.status().rx_error_count == 2);
}

inline void test_outbox_overflow_drops_status_first() {
  V1Axis ax;
  V1Identity id;
  ax.begin(id, 1, 1, 0, 0);
  // 1 tick で STATUS 1 つ + 多数の HELLO（outbox 容量を超える）
  for (size_t i = 0; i < V1Axis::kOutboxCap + 5; ++i) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::HELLO_REQ;
    m.hello_req.request_id = static_cast<uint32_t>(i + 1);
    ax.handle(m, 0);
  }
  CHECK(ax.pending() == V1Axis::kOutboxCap);
  v1::McuMsg m;
  size_t hellos = 0;
  size_t statuses = 0;
  while (ax.pop(&m)) {
    if (m.type == v1::msg::HELLO) hellos += 1;
    if (m.type == v1::msg::STATUS) statuses += 1;
  }
  CHECK(statuses == 0);  // 最初の即時 STATUS は捨てられる
  CHECK(hellos == V1Axis::kOutboxCap);
  // 捨てた STATUS は次の周期で改めて出る
  ax.tick(20 * MS, 0);
  CHECK(ax.pending() >= 1 && ax.peek()->type == v1::msg::STATUS);
}

// ---- 校正（CALIBRATE / CALIBRATING / CAL_RESULT。protocol.md §1.2） ------------------

inline bool reason_is(const v1::CalResult& r, const char* text) {
  char buf[v1::CAL_REASON_LEN + 1];
  memcpy(buf, r.reason, v1::CAL_REASON_LEN);
  buf[v1::CAL_REASON_LEN] = '\0';
  return strcmp(buf, text) == 0;
}

// out の中で CAL_RESULT が、DISARMED を示す最初の STATUS より前に 1 回だけ出ている（その前に DISARMED の STATUS が無い）。
inline void cal_result_precedes_disarmed_status(const std::vector<v1::McuMsg>& out, size_t from) {
  int results = 0;
  bool seen_result = false;
  bool disarmed_before_result = false;
  bool disarmed_after_result = false;
  for (size_t i = from; i < out.size(); ++i) {
    const v1::McuMsg& m = out[i];
    if (m.type == v1::msg::CAL_RESULT) {
      results += 1;
      seen_result = true;
    } else if (m.type == v1::msg::STATUS && m.status.mcu_state == st::DISARMED) {
      if (seen_result) disarmed_after_result = true;
      else disarmed_before_result = true;
    }
  }
  CHECK(results == 1);
  CHECK(!disarmed_before_result);
  CHECK(disarmed_after_result);
}

inline void test_calibrate_rejections_and_noop_cancel() {
  Harness h;
  // mode が 0・1 以外
  Harness::rejected(h.calibrate(2), rs::INVALID_PARAM);
  Harness::rejected(h.calibrate(255), rs::INVALID_PARAM);
  // request_id 0
  Harness::rejected(h.cmd(h.calibrate_msg(1, 0)), rs::INVALID_PARAM);
  CHECK(h.ax.state() == st::DISARMED);
  // DISARMED で mode 0 は何もせず ACCEPTED（CAL_RESULT も出ない）
  Harness::accepted(h.calibrate(0));
  CHECK(h.ax.state() == st::DISARMED && h.cal_results().empty());
  // DISARMED 以外は BAD_STATE
  Harness::accepted(h.config());
  Harness::accepted(h.arm());
  Harness::rejected(h.calibrate(1), rs::BAD_STATE);
  Harness::accepted(h.calibrate(0));  // ARMING でも mode 0 は no-op ACCEPTED
  CHECK(h.ax.state() == st::ARMING);
  h.drive(620, 0);
  CHECK(h.ax.state() == st::HOLDING);
  Harness::rejected(h.calibrate(1), rs::BAD_STATE);
  Harness::accepted(h.disarm());
  Harness::rejected(h.calibrate(1), rs::BAD_STATE);  // DISARMING
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED);
  // FAULT 中は FAULT_ACTIVE
  Harness::accepted(h.host_fault());
  Harness::rejected(h.calibrate(1), rs::FAULT_ACTIVE);
  Harness::rejected(h.calibrate(0), rs::FAULT_ACTIVE);
  Harness::accepted(h.fault_reset());
  // 受理した CALIBRATE の再送は DUPLICATE（実行しない）
  const uint32_t r = h.next_rid();
  Harness::accepted(h.cmd(h.calibrate_msg(1, r)));
  CHECK(h.ax.state() == st::CALIBRATING);
  Harness::duplicate(h.cmd(h.calibrate_msg(1, r)));
  // 校正中の mode 1 は BAD_STATE
  Harness::rejected(h.calibrate(1), rs::BAD_STATE);
  CHECK(h.ax.state() == st::CALIBRATING);
}

// 成功: 電源担当軸。主電源 ON 指令 → CAL_POWER_SETTLE_MS → ブレーキ解除指令 → BRAKE_RELEASE_WAIT_MS → 端当て → 終了手順
// → CAL_RESULT → STATUS(DISARMED)。P 端 = 0 の基準が立ち、プロファイルが profile_dirty に出る。
inline void test_calibration_success_on_power_axis() {
  Harness h(1, 1, 0, false);  // 基準なし・CONFIG あり（電源担当）
  Harness::accepted(h.config());
  CHECK(!h.ax.referenced());
  const size_t from = h.out.size();
  const std::vector<v1::McuMsg> r = h.send(h.calibrate_msg(1, h.next_rid()));
  // ACK → STATUS(CALIBRATING、主電源 ON 指令、ブレーキ作動、PWM 0、fault NONE、基準なし)
  CHECK(r.size() == 2 && r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED &&
        r[0].ack.acked_type == v1::msg::CALIBRATE);
  CHECK(r[1].type == v1::msg::STATUS && r[1].status.mcu_state == st::CALIBRATING);
  CHECK((r[1].status.flags & fl::POWER_ON_CMD) != 0 && (r[1].status.flags & fl::BRAKE_RELEASE_CMD) == 0 &&
        (r[1].status.flags & fl::POSITION_REFERENCED) == 0 && r[1].status.pwm_permille == 0 &&
        r[1].status.fault_code == fc::NONE);
  CHECK(h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  // HELLO も CALIBRATING
  for (const v1::McuMsg& m : h.hello(5)) {
    if (m.type == v1::msg::HELLO) CHECK(m.hello.mcu_state == st::CALIBRATING);
  }
  // CAL_POWER_SETTLE_MS（5000）の直前はまだブレーキ作動
  h.idle(4995);
  CHECK(h.ax.state() == st::CALIBRATING && h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  h.idle(10);
  CHECK(h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  // BRAKE_RELEASE_WAIT_MS（300）の後に端当てが始まる: P 端向き = Duty +V1_CAL_HOME_DUTY
  h.idle(290);
  CHECK(h.ax.duty_permille() == 0);
  h.idle(10);
  CHECK(h.ax.duty_permille() == v1_permille(V1_CAL_HOME_DUTY));
  CHECK(h.ax.cal_live_step() == v1::cal_step::TO_P);
  const v1::Status running = h.status();
  CHECK(running.mcu_state == st::CALIBRATING && (running.flags & fl::BRAKE_RELEASE_CMD) != 0 &&
        (running.flags & fl::POWER_ON_CMD) != 0 && running.fault_code == fc::NONE);
  CHECK(running.pwm_permille == -V1_CAL_HOME_DUTY * 10);  // sign +1: Duty + はカウント減
  // 端当て → N 端 → 中立 → 終了手順 → DISARMED
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  CHECK(h.ax.fault_code() == fc::NONE && h.fault_events().empty());
  // CAL_RESULT は 1 回、DISARMED の STATUS より前
  cal_result_precedes_disarmed_status(h.out, from);
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1);
  CHECK(res[0].boot_id == 2309737967u);
  CHECK(res[0].result == v1::cal_result::OK && res[0].step == v1::cal_step::TO_NEUTRAL);
  CHECK(res[0].n_end_count == 5551 && res[0].neutral_count == 2775 && res[0].sign == 1);
  CHECK(reason_is(res[0], "ok"));
  CHECK(h.ax.has_cal_result() && h.ax.last_cal_result().n_end_count == 5551);
  // 基準: P 端（raw = end_lo）が 0。いまは中立付近に居る
  CHECK(h.ax.referenced());
  CHECK(h.ax.actual_count() == h.raw - h.end_lo);
  CHECK(h.ax.actual_count() >= 2775 - NEUTRAL_BAND && h.ax.actual_count() <= 2775 + NEUTRAL_BAND);
  CHECK(h.ax.target_count() == h.ax.actual_count());  // DISARMED の保持目標は現在位置
  CHECK((h.status().flags & fl::POSITION_REFERENCED) != 0);
  // プロファイル（制御タスクが NVS に保存する）
  CHECK(h.ax.profile_dirty());
  const MotorProfile& p = h.ax.dirty_profile();
  CHECK(p.motor == 1 && p.stroke == 5551 && p.neutral == 2775 && p.sign == 1 && p.calibrated);
  h.ax.clear_profile_dirty();
  CHECK(!h.ax.profile_dirty());
  CHECK(h.ax.sign() == 1);
  // CONFIG は残っている。基準を取り直さずに ARM できる（PC は SET_REFERENCE を送らない）が、
  // ソフトリミットは P 端 = 0 の座標で作り直す（PC は CAL_RESULT から mechanical ∓ decel_zone を作って CONFIG し直す）
  CHECK(h.ax.configured());
  Harness::rejected(h.arm(), rs::BAD_STATE);  // 古い ±2700 の窓では中立 2775 が範囲外
  v1::Config c = base_config();
  c.soft_min_count = 100;
  c.soft_max_count = 5451;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  CHECK(h.ax.state() == st::HOLDING);
}

// 符号が逆の機構（Duty + でカウント増）: NVS の sign が既定 +1 のままでも、P 端で向きが決まり −1 で保存される。
// CAL_RESULT の座標は符号を掛けない（n_end_count = raw_N − raw_P < 0）。
inline void test_calibration_negative_sign_plant_reports_negative_n_end() {
  Harness h(2, 1, 300, false);
  h.plant_sign = -1;
  h.start_calibration_and_reach_running();
  CHECK(h.ax.duty_permille() == v1_permille(V1_CAL_HOME_DUTY));  // 端当ての Duty は機構の符号に依らず +（P 端向き）
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1 && res[0].result == v1::cal_result::OK);
  CHECK(res[0].n_end_count == -5551 && res[0].neutral_count == -2775 && res[0].sign == -1);
  CHECK(h.ax.sign() == -1);
  CHECK(h.ax.referenced());
  CHECK(h.ax.actual_count() == h.raw - h.end_hi);  // P 端 = raw の上限側 = 0
  CHECK(h.ax.actual_count() <= -2775 + NEUTRAL_BAND && h.ax.actual_count() >= -2775 - NEUTRAL_BAND);
  CHECK(h.ax.profile_dirty());
  CHECK(h.ax.dirty_profile().motor == 2 && h.ax.dirty_profile().sign == -1 && h.ax.dirty_profile().stroke == 5551);
  // 以後の Duty の向きは新しい符号（カウントを増やしたい → Duty +）。窓は N 端側が負
  v1::Config c = base_config();
  c.soft_min_count = -5451;
  c.soft_max_count = -100;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 100;
  h.set_position(h.ax.actual_count());
  h.step();
  CHECK(h.ax.duty_permille() > 0 && h.ax.pwm_permille() > 0);
}

// 電源担当でない軸（CONFIG なしでも受理する）: 主電源 ON 指令は出さず、同じ settle 時間を待ってからブレーキを解除する。
inline void test_calibration_on_non_power_axis_without_config() {
  Harness h(3, 1, 0, false);
  CHECK(!h.ax.configured());
  const size_t from = h.out.size();
  Harness::accepted(h.calibrate(1));
  CHECK(h.ax.state() == st::CALIBRATING && !h.ax.power_on_cmd());
  h.idle(4995);
  CHECK(!h.ax.brake_release_cmd());
  h.idle(10);
  CHECK(h.ax.brake_release_cmd());
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
  for (size_t i = from; i < h.out.size(); ++i) {
    if (h.out[i].type == v1::msg::STATUS) CHECK((h.out[i].status.flags & (fl::POWER_ON_CMD | fl::POWER_CONTROLLER)) == 0);
  }
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1 && res[0].result == v1::cal_result::OK && res[0].n_end_count == 5551);
  CHECK(h.ax.referenced() && h.ax.profile_dirty() && h.ax.dirty_profile().motor == 3);
  CHECK(!h.ax.configured());  // CONFIG は要らないし増えもしない
}

// 失敗: 機構が動かない。ARRIVAL_MIN_TRAVEL 未満で止まる → 退避 → また止まる → FAILED "stall"（step 1）。
// 基準は立たず、FAULT にはならず、プロファイルも出ない。
inline void test_calibration_fails_on_stall_before_min_travel() {
  Harness h(1, 1, 0, true);  // 基準ありで始めても、校正の開始で消える
  Harness::accepted(h.config());
  CHECK(h.ax.referenced());
  h.stall = true;
  const size_t from = h.out.size();
  Harness::accepted(h.calibrate(1));
  CHECK(!h.ax.referenced());
  CHECK(h.run_until([&] { return h.ax.duty_permille() != 0; }, 6000));
  // 開始猶予 300 + 無パルス 200 → 退避（反対向き）→ 猶予 300 + 無パルス 200 → 失敗
  CHECK(h.run_until([&] { return h.ax.duty_permille() == -v1_permille(V1_CAL_HOME_DUTY); }, 1000));
  CHECK(h.ax.state() == st::CALIBRATING && strcmp(h.ax.cal_live_reason(), "retreat") == 0);
  CHECK(h.run_until([&] { return !h.ax.brake_release_cmd(); }, 1000));
  CHECK(h.ax.state() == st::CALIBRATING && h.ax.duty_permille() == 0);  // 終了手順: ブレーキ給電 OFF、PWM 0、主電源はまだ ON
  CHECK(h.ax.power_on_cmd());
  h.idle(295);
  CHECK(h.ax.state() == st::CALIBRATING);
  h.idle(10);
  CHECK(h.ax.state() == st::DISARMED && !h.ax.power_on_cmd());
  cal_result_precedes_disarmed_status(h.out, from);
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1);
  CHECK(res[0].result == v1::cal_result::FAILED && res[0].step == v1::cal_step::TO_P);
  CHECK(reason_is(res[0], "stall"));
  CHECK(res[0].n_end_count == 0 && res[0].neutral_count == 0 && res[0].sign == 1);
  CHECK(!h.ax.referenced() && !h.ax.profile_dirty());
  CHECK(h.ax.fault_code() == fc::NONE && h.fault_events().empty());
  CHECK(h.ax.sign() == 1);
  Harness::rejected(h.arm(), rs::NOT_REFERENCED);  // 原点が信用できないので ARM できない
}

// 中立への移動で失敗（step 3）: N 端まで測れた値は CAL_RESULT に載るが、保存も基準の確立もしない。
inline void test_calibration_failure_at_neutral_reports_values_but_keeps_nothing() {
  Harness h(1, 1, 0, false);
  h.start_calibration_and_reach_running();
  CHECK(h.run_until([&] { return h.ax.cal_live_step() == v1::cal_step::TO_NEUTRAL; }, 30000));
  h.stall = true;  // 中立へ動けない → 下限 Duty を HOME_DUTY まで上げても動かず FAILED
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 30000));
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1);
  CHECK(res[0].result == v1::cal_result::FAILED && res[0].step == v1::cal_step::TO_NEUTRAL);
  CHECK(reason_is(res[0], "stall"));
  CHECK(res[0].n_end_count == 5551 && res[0].neutral_count == 2775);
  CHECK(!h.ax.referenced() && !h.ax.profile_dirty());
  CHECK(h.fault_events().empty());
}

// 中止: CALIBRATE(0)。settle 中（AutoMachine 開始前）と端当て中の両方。終了手順を経て DISARMED、CANCELLED "stop"。
inline void test_calibrate_cancel_before_and_during_motion() {
  {
    Harness h(1, 1, 0, false);
    Harness::accepted(h.config());
    const size_t from = h.out.size();
    Harness::accepted(h.calibrate(1));
    h.idle(1000);  // settle 中
    CHECK(h.ax.state() == st::CALIBRATING && h.ax.power_on_cmd() && !h.ax.brake_release_cmd());
    const std::vector<v1::McuMsg> r = h.send(h.calibrate_msg(0, h.next_rid()));
    CHECK(r.size() >= 1 && r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED);
    CHECK(h.ax.state() == st::CALIBRATING);  // 終了手順の間は CALIBRATING のまま
    h.idle(295);
    CHECK(h.ax.state() == st::CALIBRATING && h.ax.power_on_cmd());
    h.idle(10);
    CHECK(h.ax.state() == st::DISARMED && !h.ax.power_on_cmd());
    cal_result_precedes_disarmed_status(h.out, from);
    const std::vector<v1::CalResult> res = h.cal_results();
    CHECK(res.size() == 1 && res[0].result == v1::cal_result::CANCELLED && res[0].step == v1::cal_step::NONE);
    CHECK(reason_is(res[0], "stop"));
    CHECK(!h.ax.referenced() && !h.ax.profile_dirty() && h.fault_events().empty());
    CHECK(h.raw == 0);  // 動いていない
  }
  {
    Harness h(1, 1, 0, false);
    Harness::accepted(h.config());  // 電源担当（終了手順の間は主電源 ON 指令のまま）
    const size_t from = h.out.size();
    h.start_calibration_and_reach_running();
    h.idle(500);  // P 端へ動いている途中
    CHECK(h.ax.duty_permille() == v1_permille(V1_CAL_HOME_DUTY) && h.raw < 0 && h.raw > h.end_lo);
    const int64_t at_cancel = h.raw;
    const std::vector<v1::McuMsg> r = h.send(h.calibrate_msg(0, h.next_rid()));
    // ACK の時点でブレーキ給電 OFF・PWM 0（ACK の後の STATUS がそれを示す）
    CHECK(r.size() == 2 && r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED);
    CHECK(r[1].type == v1::msg::STATUS && r[1].status.mcu_state == st::CALIBRATING &&
          (r[1].status.flags & fl::BRAKE_RELEASE_CMD) == 0 && r[1].status.pwm_permille == 0);
    CHECK(!h.ax.brake_release_cmd() && h.ax.duty_permille() == 0 && h.ax.power_on_cmd());
    // 終了手順中の 2 回目の中止は何もせず ACCEPTED
    Harness::accepted(h.calibrate(0));
    h.idle(400);
    CHECK(h.ax.state() == st::DISARMED);
    CHECK(h.raw == at_cancel);  // ブレーキ作動後は動かない
    cal_result_precedes_disarmed_status(h.out, from);
    const std::vector<v1::CalResult> res = h.cal_results();
    CHECK(res.size() == 1 && res[0].result == v1::cal_result::CANCELLED && res[0].step == v1::cal_step::TO_P);
    CHECK(reason_is(res[0], "stop"));
    CHECK(!h.ax.referenced() && !h.ax.profile_dirty() && h.fault_events().empty());
    // 改めて校正できる
    h.start_calibration_and_reach_running();
    CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
    CHECK(h.cal_results().size() == 2 && h.cal_results()[1].result == v1::cal_result::OK);
    CHECK(h.ax.referenced());
  }
}

// KEEP_POWER（protocol.md §4 CALIBRATE flags bit0、§1.2）: 電源担当軸は校正が終わっても主電源 ON 指令を保ったまま
// CALIBRATING（POWER_HOLD）に留まり、CALIBRATE(0) で DISARMED。CAL_RESULT は校正終了時の 1 回だけ（step は終了時の段階）。
inline void test_calibration_keep_power_holds_main_power_until_release() {
  Harness h(1, 1, 0, false);
  Harness::accepted(h.config());  // 電源担当
  const size_t from = h.out.size();
  Harness::accepted(h.calibrate(1, v1::cal_flags::KEEP_POWER));
  CHECK(h.ax.state() == st::CALIBRATING && h.ax.power_on_cmd());
  // 校正本体が終わって CAL_RESULT が出る
  CHECK(h.run_until([&] { return h.ax.has_cal_result(); }, 40000));
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1 && res[0].result == v1::cal_result::OK && res[0].step == v1::cal_step::TO_NEUTRAL);
  CHECK(res[0].n_end_count == 5551 && res[0].neutral_count == 2775);
  // POWER_HOLD: CALIBRATING のまま、主電源 ON 指令あり、ブレーキ作動、PWM 0、基準・プロファイルは確定済み
  CHECK(h.ax.state() == st::CALIBRATING && h.ax.cal_live_step() == v1::cal_step::POWER_HOLD);
  CHECK(h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  CHECK(h.ax.referenced() && h.ax.profile_dirty());
  const v1::Status hold = h.status();
  CHECK(hold.mcu_state == st::CALIBRATING && (hold.flags & fl::POWER_ON_CMD) != 0 &&
        (hold.flags & fl::BRAKE_RELEASE_CMD) == 0 && hold.pwm_permille == 0 && hold.fault_code == fc::NONE);
  const int64_t at_hold = h.raw;
  h.idle(10000);  // 期限（120 s）よりずっと短い間は保ち続ける
  CHECK(h.ax.state() == st::CALIBRATING && h.ax.power_on_cmd() && h.raw == at_hold);
  // 保持中の校正開始は BAD_STATE（CALIBRATING）、ARM も BAD_STATE
  Harness::rejected(h.calibrate(1), rs::BAD_STATE);
  Harness::rejected(h.arm(), rs::BAD_STATE);
  // CALIBRATE(0) で主電源 OFF → DISARMED。CAL_RESULT は増えない
  const std::vector<v1::McuMsg> r = h.send(h.calibrate_msg(0, h.next_rid()));
  CHECK(r.size() == 2 && r[0].type == v1::msg::ACK && r[0].ack.result == ar::ACCEPTED);
  CHECK(r[1].type == v1::msg::STATUS && r[1].status.mcu_state == st::DISARMED &&
        (r[1].status.flags & (fl::POWER_ON_CMD | fl::BRAKE_RELEASE_CMD)) == 0);
  CHECK(h.ax.state() == st::DISARMED && !h.ax.power_on_cmd() && h.ax.cal_live_step() == v1::cal_step::NONE);
  CHECK(h.cal_results().size() == 1);
  cal_result_precedes_disarmed_status(h.out, from);
  CHECK(h.ax.referenced() && h.fault_events().empty());
  // フラグ無しの次の校正は保持に入らず DISARMED まで行く（前回の KEEP_POWER が残らない）
  h.start_calibration_and_reach_running();
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
  CHECK(h.cal_results().size() == 2 && !h.ax.power_on_cmd());
}

// POWER_HOLD は CAL_POWER_HOLD_TIMEOUT_MS で自然に終わる（CAL_RESULT は送らない）。DISARM でも終わる。
inline void test_calibration_keep_power_hold_ends_on_timeout_or_disarm() {
  {
    Harness h(1, 1, 0, false);
    Harness::accepted(h.config());
    Harness::accepted(h.calibrate(1, v1::cal_flags::KEEP_POWER));
    CHECK(h.run_until([&] { return h.ax.cal_live_step() == v1::cal_step::POWER_HOLD; }, 40000));
    h.idle(CAL_POWER_HOLD_TIMEOUT_MS - 50);
    CHECK(h.ax.state() == st::CALIBRATING && h.ax.power_on_cmd());
    h.idle(100);
    CHECK(h.ax.state() == st::DISARMED && !h.ax.power_on_cmd());
    CHECK(h.cal_results().size() == 1 && h.ax.referenced() && h.fault_events().empty());
  }
  {
    Harness h(1, 1, 0, false);
    Harness::accepted(h.config());
    Harness::accepted(h.calibrate(1, v1::cal_flags::KEEP_POWER));
    CHECK(h.run_until([&] { return h.ax.cal_live_step() == v1::cal_step::POWER_HOLD; }, 40000));
    Harness::accepted(h.disarm());
    CHECK(h.ax.state() == st::DISARMED && !h.ax.power_on_cmd() && h.cal_results().size() == 1);
  }
}

// PC が校正を途中で止めたら、KEEP_POWER 付きでも POWER_HOLD に入らず終了手順 → DISARMED（protocol.md §4 の mode 0）。
// PC は中止で解放の予約を消すので、ここで保持に入ると期限まで CALIBRATING に留まってしまう。
inline void test_calibration_keep_power_cancel_during_motion_goes_to_disarmed() {
  Harness h(1, 1, 0, false);
  Harness::accepted(h.config());
  Harness::accepted(h.calibrate(1, v1::cal_flags::KEEP_POWER));
  CHECK(h.run_until([&] { return h.ax.duty_permille() != 0; }, 6000));
  h.idle(500);  // P 端へ動いている途中
  Harness::accepted(h.calibrate(0));
  CHECK(h.ax.state() == st::CALIBRATING && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);  // 終了手順
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED && !h.ax.power_on_cmd());
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1 && res[0].result == v1::cal_result::CANCELLED && res[0].step == v1::cal_step::TO_P);
  CHECK(!h.ax.referenced() && h.fault_events().empty());
}

// KEEP_POWER は電源担当軸だけが解釈する。他の軸は校正が終わればそのまま DISARMED（主電源指令は出さない）。
inline void test_calibration_keep_power_ignored_on_non_power_axis() {
  Harness h(3, 1, 0, false);
  const size_t from = h.out.size();
  Harness::accepted(h.calibrate(1, v1::cal_flags::KEEP_POWER));
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
  CHECK(h.cal_results().size() == 1 && h.cal_results()[0].result == v1::cal_result::OK);
  for (size_t i = from; i < h.out.size(); ++i) {
    if (h.out[i].type == v1::msg::STATUS) CHECK((h.out[i].status.flags & fl::POWER_ON_CMD) == 0);
  }
}

// POWER_HOLD 中の FAULT: 出力を落として FAULT。CAL_RESULT は増えない（結果は送信済み）。基準は保つ。
inline void test_fault_during_power_hold_drops_power_without_new_result() {
  Harness h(1, 1, 0, false);
  Harness::accepted(h.config());
  Harness::accepted(h.calibrate(1, v1::cal_flags::KEEP_POWER));
  CHECK(h.run_until([&] { return h.ax.cal_live_step() == v1::cal_step::POWER_HOLD; }, 40000));
  const std::vector<v1::McuMsg> r = h.send(h.host_fault_msg(h.next_rid()));
  CHECK(r.size() == 3 && r[0].type == v1::msg::FAULT_EVENT && r[1].type == v1::msg::ACK && r[2].type == v1::msg::STATUS);
  CHECK(r[0].fault_event.mcu_state_before == st::CALIBRATING);
  CHECK(r[2].status.mcu_state == st::FAULT && (r[2].status.flags & (fl::POWER_ON_CMD | fl::BRAKE_RELEASE_CMD)) == 0);
  CHECK(h.cal_results().size() == 1 && h.ax.referenced());
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED && !h.ax.power_on_cmd());
}

// DISARM も中止（ACCEPTED）。N 端へ向かっている途中なら step 2。
inline void test_disarm_cancels_calibration() {
  Harness h(1, 1, 0, false);
  const size_t from = h.out.size();
  h.start_calibration_and_reach_running();
  CHECK(h.run_until([&] { return h.ax.cal_live_step() == v1::cal_step::TO_N; }, 10000));
  h.idle(300);
  CHECK(h.ax.duty_permille() == -v1_permille(V1_CAL_HOME_DUTY));  // N 端向き（校正中は保存ストロークを信じず低速）
  Harness::accepted(h.disarm());
  CHECK(h.ax.state() == st::CALIBRATING && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED);
  cal_result_precedes_disarmed_status(h.out, from);
  const std::vector<v1::CalResult> res = h.cal_results();
  CHECK(res.size() == 1 && res[0].result == v1::cal_result::CANCELLED && res[0].step == v1::cal_step::TO_N);
  CHECK(reason_is(res[0], "stop"));
  CHECK(!h.ax.referenced());
  Harness::accepted(h.disarm());  // DISARMED では no-op
}

// 校正中の他の命令: ARM_PREPARE / START / CONFIG / SET_REFERENCE / FAULT_RESET → BAD_STATE、CONTROLLED_STOP → no-op ACCEPTED、
// SET_POSITION は捨てる（数えない）、系統の切り替えは無視。
inline void test_calibrating_rejects_or_drops_other_commands() {
  Harness h(1, 1, 0, false);
  Harness::accepted(h.config());
  h.set_position(0);  // last_rx_seq = 1
  CHECK(h.ax.last_rx_seq() == 1);
  h.start_calibration_and_reach_running();
  Harness::rejected(h.arm(), rs::BAD_STATE);
  Harness::rejected(h.start(), rs::BAD_STATE);
  Harness::rejected(h.config(), rs::BAD_STATE);
  Harness::rejected(h.set_reference(0), rs::BAD_STATE);
  Harness::rejected(h.fault_reset(), rs::BAD_STATE);
  Harness::accepted(h.stop());
  CHECK(h.ax.state() == st::CALIBRATING && h.ax.duty_permille() != 0);  // 止まらない
  // SET_POSITION（有効・古い・apply_at ≠ 0）はどれも数えない
  const uint16_t stale = h.ax.stale_drop_count();
  const uint16_t rxerr = h.ax.rx_error_count();
  h.set_position(100);
  v1::HostMsg m = h.set_position_msg(100);
  m.set_position.seq = 1;
  h.send(m);
  m = h.set_position_msg(100);
  m.set_position.apply_at_us = 5;
  h.send(m);
  CHECK(h.ax.last_rx_seq() == 1 && h.ax.stale_drop_count() == stale && h.ax.rx_error_count() == rxerr);
  CHECK(h.ax.last_applied_seq() == 0);
  // 系統の切り替えは無視（AXIS_SELECT も出ない）
  CHECK(!h.ax.select_axis(2, h.now));
  CHECK(h.ax.selected_axis() == 1 && h.axis_selects().empty());
  // lease は見ない（SET_POSITION 無しで COMMAND_TIMEOUT にならない）
  h.idle(500);
  CHECK(h.ax.state() == st::CALIBRATING && h.ax.fault_code() == fc::NONE);
  // 校正が終われば SET_POSITION はまた数える
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
  h.set_position(0);
  CHECK(h.ax.last_rx_seq() == h.seq);
}

// HOST_FAULT 中の校正: 即 FAULT（出力はすべて OFF）。CAL_RESULT(CANCELLED "fault") を FAULT_EVENT より前に 1 回。
// FAULT_RESET 後は DISARMED で基準なし。改めて校正できる。
inline void test_host_fault_during_calibration() {
  Harness h(1, 1, 0, false);
  Harness::accepted(h.config());
  h.start_calibration_and_reach_running();
  h.idle(200);
  CHECK(h.ax.power_on_cmd() && h.ax.brake_release_cmd() && h.ax.duty_permille() != 0);
  const std::vector<v1::McuMsg> r = h.send(h.host_fault_msg(h.next_rid()));
  CHECK(r.size() == 4 && r[0].type == v1::msg::CAL_RESULT && r[1].type == v1::msg::FAULT_EVENT &&
        r[2].type == v1::msg::ACK && r[3].type == v1::msg::STATUS);
  CHECK(r[0].cal_result.result == v1::cal_result::CANCELLED && r[0].cal_result.step == v1::cal_step::TO_P);
  CHECK(reason_is(r[0].cal_result, "fault"));
  CHECK(r[1].fault_event.fault_code == fc::HOST_FAULT && r[1].fault_event.mcu_state_before == st::CALIBRATING);
  CHECK(r[3].status.mcu_state == st::FAULT && r[3].status.fault_code == fc::HOST_FAULT &&
        (r[3].status.flags & (fl::POWER_ON_CMD | fl::BRAKE_RELEASE_CMD)) == 0 && r[3].status.pwm_permille == 0);
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
  CHECK(!h.ax.referenced() && !h.ax.profile_dirty());
  h.idle(1000);
  CHECK(h.ax.state() == st::FAULT && h.cal_results().size() == 1);  // 終了手順は踏まない（CAL_RESULT は増えない）
  Harness::accepted(h.fault_reset());
  CHECK(h.ax.state() == st::DISARMED && !h.ax.referenced() && h.ax.configured());
  // 改めて校正（前回の残りが無い）
  h.start_calibration_and_reach_running();
  CHECK(h.ax.cal_live_step() == v1::cal_step::TO_P);
  CHECK(h.run_until([&] { return h.ax.state() == st::DISARMED; }, 40000));
  CHECK(h.cal_results().size() == 2 && h.cal_results()[1].result == v1::cal_result::OK && h.ax.referenced());
}

// 画面の非常停止中の校正: FAULT(LOCAL_ESTOP)、CAL_RESULT(CANCELLED "estop")。制御周期の逸脱も FAULT。
inline void test_local_estop_and_overrun_during_calibration() {
  {
    Harness h(1, 1, 0, false);
    h.start_calibration_and_reach_running();
    h.ax.set_local_estop(true, h.now);
    h.drain();
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::LOCAL_ESTOP);
    CHECK(!h.ax.brake_release_cmd() && h.ax.duty_permille() == 0);
    const std::vector<v1::CalResult> res = h.cal_results();
    CHECK(res.size() == 1 && res[0].result == v1::cal_result::CANCELLED && reason_is(res[0], "estop"));
    const std::vector<v1::FaultEvent> ev = h.fault_events();
    CHECK(ev.size() == 1 && ev[0].mcu_state_before == st::CALIBRATING);
    Harness::rejected(h.fault_reset(), rs::FAULT_CAUSE_PRESENT);
    h.ax.set_local_estop(false, h.now);
    Harness::accepted(h.fault_reset());
    CHECK(h.ax.state() == st::DISARMED && !h.ax.referenced());
  }
  {
    Harness h(1, 1, 0, false);
    Harness::accepted(h.calibrate(1));
    h.idle(100);
    h.step(16);  // CALIBRATING は parked ではない: 周期の逸脱は異常
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::CONTROL_OVERRUN);
    CHECK(h.cal_results().size() == 1 && reason_is(h.cal_results()[0], "fault"));
  }
}

// 終了手順（ブレーキ作動待ち）の間の FAULT: 結果は確定済みなのでそのまま送る。成功なら基準・プロファイルも保つ。
// 中止 "stop" の結果も "fault" に書き換えない。
inline void test_fault_during_calibration_shutdown_keeps_final_result() {
  {
    Harness h(1, 1, 0, false);
    Harness::accepted(h.config());
    h.start_calibration_and_reach_running();
    // 中立に着いて終了手順に入る（ブレーキ給電 OFF、まだ CALIBRATING）
    CHECK(h.run_until([&] { return h.ax.cal_live_step() == v1::cal_step::TO_NEUTRAL && !h.ax.brake_release_cmd(); },
                      40000));
    CHECK(h.ax.state() == st::CALIBRATING && h.ax.referenced() && h.ax.profile_dirty());
    CHECK(h.cal_results().empty());  // CAL_RESULT はまだ
    h.idle(100);
    const std::vector<v1::McuMsg> r = h.send(h.host_fault_msg(h.next_rid()));
    CHECK(r.size() == 4 && r[0].type == v1::msg::CAL_RESULT && r[1].type == v1::msg::FAULT_EVENT &&
          r[2].type == v1::msg::ACK && r[3].type == v1::msg::STATUS && r[3].status.mcu_state == st::FAULT);
    CHECK(r[0].cal_result.result == v1::cal_result::OK && r[0].cal_result.n_end_count == 5551 &&
          r[0].cal_result.neutral_count == 2775 && reason_is(r[0].cal_result, "ok"));
    CHECK(h.ax.referenced() && h.ax.profile_dirty() && h.ax.dirty_profile().stroke == 5551);
    CHECK((r[3].status.flags & fl::POSITION_REFERENCED) != 0);
    h.idle(1000);
    CHECK(h.ax.state() == st::FAULT && h.cal_results().size() == 1);
    Harness::accepted(h.fault_reset());
    CHECK(h.ax.state() == st::DISARMED && h.ax.referenced());
    CHECK(h.ax.actual_count() == h.raw - h.end_lo);
  }
  {
    Harness h(1, 1, 0, false);
    h.start_calibration_and_reach_running();
    h.idle(300);
    Harness::accepted(h.calibrate(0));  // 中止 → 終了手順
    CHECK(h.ax.state() == st::CALIBRATING && !h.ax.brake_release_cmd());
    h.idle(100);
    h.ax.set_local_estop(true, h.now);
    h.drain();
    CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::LOCAL_ESTOP);
    const std::vector<v1::CalResult> res = h.cal_results();
    CHECK(res.size() == 1 && res[0].result == v1::cal_result::CANCELLED && reason_is(res[0], "stop"));
    CHECK(!h.ax.referenced() && !h.ax.profile_dirty());
  }
}


// ---- Issue #25 手順 9「F」: 有界の積分と保持バイアス --------------------------------------------------------

// 飽和と変化率: 誤差 400（P 80 ≥ 床 70 なので床は持ち上げず、|出力| < duty_max 60 %）で dt 5 ms を回すと
// dI = 0.1 × 400 × 0.005 = 0.2 だが変化率 10 permille/s → 0.05 / tick に制限。1 s で 10、3 s で 30 で頭打ち。
inline void test_integral_rate_limit_and_saturation() {
  V1Tuning t;
  t.duty_max = 60;
  CHECK(t.ki_permille_per_count_s > 0.0);  // 既定 ON（config.h V1_INTEGRAL）
  V1DutyState st;
  for (int i = 0; i < 200; ++i) v1_duty_command(400, 0.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm - 10.0) < 1e-6);
  V1DutyTerms o;
  CHECK(v1_duty_command(400, 0.0, st, t, 0, &o, 0.005) == 90 && o.i_permille == 10 && o.p_permille == 80);
  for (int i = 0; i < 1000; ++i) v1_duty_command(400, 0.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm - 30.0) < 1e-9);  // |I| ≤ 30
  CHECK(v1_duty_command(400, 0.0, st, t) == 110);
  // 小さい誤差では Ki × err が変化率より下: 誤差 20 → 0.1 × 20 × 0.005 = 0.01 / tick（2 permille/s）
  V1DutyState st2;
  for (int i = 0; i < 200; ++i) v1_duty_command(20, 100.0, st2, t, 0, nullptr, 0.005);  // 軌道が動いている（不感帯に入らない）
  CHECK(fabs(st2.i_pm - 2.0) < 1e-6);
  // 負の誤差は負に積む
  V1DutyState st3;
  for (int i = 0; i < 200; ++i) v1_duty_command(-400, 0.0, st3, t, 0, nullptr, 0.005);
  CHECK(fabs(st3.i_pm + 10.0) < 1e-6);
  // dt 0（直接呼ぶ従来のテスト）と ki 0（-DV1_INTEGRAL=0 相当）では積まない
  V1DutyState st4;
  for (int i = 0; i < 200; ++i) v1_duty_command(400, 0.0, st4, t);
  CHECK(st4.i_pm == 0.0);
  t.ki_permille_per_count_s = 0.0;
  for (int i = 0; i < 200; ++i) v1_duty_command(400, 0.0, st4, t, 0, nullptr, 0.005);
  CHECK(st4.i_pm == 0.0);
}

// アンチワインドアップは実出力で見る: duty_max で飽和している向きには積まない。床（duty_min ＋ ブースト）が合成を持ち上げて
// いるときも積まない。どちらも戻す向き（|I| を減らす）は許す。
inline void test_integral_anti_windup_on_applied_output() {
  V1Tuning t;  // kp 2、床 7、上限 20
  V1DutyState st;
  // 誤差 1500 → P 300 > 200 で飽和 → I は 0 のまま
  for (int i = 0; i < 400; ++i) CHECK(v1_duty_command(1500, 0.0, st, t, 0, nullptr, 0.005) == 200);
  CHECK(st.i_pm == 0.0);
  // 誤差 400（P 80、床 70 を持ち上げない、飽和もしない）で 1 s → I = 10
  for (int i = 0; i < 200; ++i) v1_duty_command(400, 0.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm - 10.0) < 1e-6);
  // 再び飽和（同じ向き）: 増えない
  for (int i = 0; i < 200; ++i) v1_duty_command(1500, 0.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm - 10.0) < 1e-6);
  // 反対向きに飽和（誤差 −1500 → −200）: I を戻す向きは許す → 1 s で 10 → 0 → 負に積まれる前に反対の飽和で止まる
  for (int i = 0; i < 200; ++i) v1_duty_command(-1500, 0.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm) < 1e-6);
  for (int i = 0; i < 200; ++i) v1_duty_command(-1500, 0.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm) < 1e-6);  // 0 から負へは「増やす向き」なので飽和中は積まない
  // 床が持ち上げているとき: 誤差 25（P 5）静止 → 床 ON → 出力 70。I は増えない
  V1DutyState st2;
  for (int i = 0; i < 400; ++i) CHECK(v1_duty_command(25, 0.0, st2, t, 0, nullptr, 0.005) == 70);
  CHECK(st2.i_pm == 0.0);
  // ブースト込みの床（70 + 30 = 100）も同じ
  for (int i = 0; i < 400; ++i) CHECK(v1_duty_command(25, 0.0, st2, t, 30, nullptr, 0.005) == 100);
  CHECK(st2.i_pm == 0.0);
  // 床が持ち上げていても、I が反対符号なら戻す向きは積む（I +2 を持ったまま誤差 −25: P −5 + I 2 = −3 → 床 −70 → I は減る）
  st2.i_pm = 2.0;
  for (int i = 0; i < 100; ++i) CHECK(v1_duty_command(-25, 0.0, st2, t, 0, nullptr, 0.005) == -70);
  CHECK(st2.i_pm < 2.0 && st2.i_pm > 0.0);  // 0.1 × 25 × 0.005 = 0.0125 / tick × 100 = 1.25 → 0.75
}

// 床の向き不一致（E の抑制中: 合成が P と逆）では |I| を増やさない。戻す向きは許す。
inline void test_integral_blocked_during_floor_sign_mismatch_but_unwinds() {
  V1Tuning t;  // kp 2、ff 100
  V1DutyState st;
  // 誤差 +50（P +10）・軌道 −600（FF −60）→ 合成 −50 → 不一致。I = 0 → 正に積むのは「増やす」→ 止める
  for (int i = 0; i < 200; ++i) CHECK(v1_duty_command(50, -600.0, st, t, 0, nullptr, 0.005) == -50);
  CHECK(st.i_pm == 0.0);
  // I = −5 を持っていれば、正に積むのは戻す向き → 許す（0.025 / tick × 200 = 5 → 0 に戻り、0 からは「増やす」なので止まる）
  st.i_pm = -5.0;
  for (int i = 0; i < 100; ++i) v1_duty_command(50, -600.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm + 2.5) < 1e-6);
  for (int i = 0; i < 200; ++i) v1_duty_command(50, -600.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm) < 1e-6);
  // 向きが一致していれば普通に積む（誤差 +50・軌道 +600 → 合成 +70、床 ON だが持ち上げない）
  V1DutyState st2;
  for (int i = 0; i < 200; ++i) v1_duty_command(50, 600.0, st2, t, 0, nullptr, 0.005);
  CHECK(fabs(st2.i_pm - 5.0) < 1e-6);  // 0.1 × 50 × 0.005 = 0.025 / tick × 200 = 5（変化率 10/s 未満）
}

// 不感帯の中では I を凍結し、出力は I（保持バイアス）。0 に落とさない。
inline void test_integral_holding_bias_retained_in_band() {
  V1Tuning t;
  t.duty_max = 60;
  V1DutyState st;
  for (int i = 0; i < 400; ++i) v1_duty_command(400, 0.0, st, t, 0, nullptr, 0.005);  // I = 20
  CHECK(fabs(st.i_pm - 20.0) < 1e-6);
  V1DutyTerms o;
  // 静止・誤差 5 → 不感帯に入る → 出力 = I = 20（従来は 0）、床 OFF、積まない
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.005) == 20 && o.in_band && !o.floor_on && o.i_permille == 20);
  for (int i = 0; i < 400; ++i) CHECK(v1_duty_command(-8, 0.0, st, t, 0, nullptr, 0.005) == 20);  // 誤差の符号に依らず凍結
  CHECK(fabs(st.i_pm - 20.0) < 1e-6);
  // 出る側（> 20）で抜ければ普通の P + I（誤差 25 → P 5 + I 20 = 25 → 床 ON → 70。床が持ち上げるので I は増えない）
  CHECK(v1_duty_command(25, 0.0, st, t, 0, &o, 0.005) == 70 && !o.in_band && o.floor_on);
  CHECK(fabs(st.i_pm - 20.0) < 1e-6);
  // ki 0（-DV1_INTEGRAL=0）なら不感帯の中は従来どおり 0
  t.ki_permille_per_count_s = 0.0;
  V1DutyState st2;
  CHECK(v1_duty_command(5, 0.0, st2, t, 0, &o, 0.005) == 0 && o.in_band);
  // I が duty_max を超えることは無い（|I| ≤ 30 < duty_max）が、SET_LIMITS で duty_max 2 % にされれば保持も 20 で頭打ち
  t.ki_permille_per_count_s = V1_KI_PERMILLE_PER_COUNT_S;
  t.duty_max = 2;
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.005) == 20);
  t.duty_max = 1;
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.005) == 10);
}

// 軸を通して: 止まったまま誤差 15（床 OFF・P 3）で I が積まれ、出力に足される。DISARM（DISARMED）/ FAULT / SET_REFERENCE で 0。
inline void test_integral_in_axis_accumulates_and_resets_on_disarm_fault_reference() {
  Harness h;
  h.ax.tuning.settled_ms = 0;  // ここは F だけを見る（S は test_settled_* で）
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  CHECK(h.ax.integral_permille() == 0);
  h.stall = true;
  h.raw -= 25;  // 不感帯の外へ → 床 ON（70）→ I は増えない（床が持ち上げている）
  h.drive(100, h.ax.actual_count() + 25);
  CHECK(h.ax.duty_permille() == -70 && h.ax.integral_permille() == 0);
  h.raw += 10;  // 誤差 15: 床 OFF、生の P 3。ここで I が 0.1 × 15 × dt = 1.5 permille/s で積まれる
  h.drive(2000, h.ax.actual_count() + 15);
  CHECK(h.ax.state() == st::HOLDING);
  CHECK(h.ax.integral_permille() == 3);  // 2 s × 1.5 = 3.0
  CHECK(h.ax.duty_permille() == -6);     // P 3 + I 3（カウント増の向き → MD13S 符号で −）
  CHECK(h.ax.duty_terms().i_permille == 3 && h.ax.duty_terms().p_permille == 3);
  // 不感帯に入れば（誤差 5）出力は I だけ（保持バイアス）、凍結
  h.raw += 10;
  h.drive(200, h.ax.actual_count() + 5);
  CHECK(h.ax.duty_terms().in_band && h.ax.duty_permille() == -3 && h.ax.integral_permille() == 3);
  // DISARM → DISARMED で 0
  Harness::accepted(h.disarm());
  h.idle(400);
  CHECK(h.ax.state() == st::DISARMED && h.ax.integral_permille() == 0 && h.ax.duty_permille() == 0);
  // SET_REFERENCE も（DISARMED でしか通らないので 0 のまま 0）
  Harness::accepted(h.set_reference(0));
  CHECK(h.ax.integral_permille() == 0);
  // FAULT: 積んだ I は latch_fault で即 0
  Harness g;
  g.ax.tuning.settled_ms = 0;
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.stall = true;
  g.raw -= 25;  // いったん不感帯を出てから誤差 15 へ（15 のままでは出る側 20 の内側で凍結）
  g.step();
  g.raw += 10;
  g.drive(2000, g.ax.actual_count() + 15);
  CHECK(g.ax.integral_permille() == 3);
  Harness::accepted(g.host_fault());
  CHECK(g.ax.state() == st::FAULT && g.ax.integral_permille() == 0 && g.ax.duty_permille() == 0);
  g.idle(50);
  CHECK(g.ax.integral_permille() == 0);
}


// ---- Issue #25 手順 10「S」: 整定判定 ----------------------------------------------------------------------

// 不感帯に入るのは「軌道速度 0・|誤差| ≤ 10・|実測速度| ≤ 10」が 200 ms 続いたとき。実測速度は 100 ms 窓なので、
// 25 count 飛んで戻った直後は窓が 250 count/s を示す間（≤ 100 ms）は数えず、その後 200 ms で入る。
inline void test_settled_entry_requires_dwell_with_measured_rest() {
  Harness h;
  CHECK(h.ax.tuning.settled_ms == V1_SETTLED_MS && h.ax.tuning.settled_ms == 200);  // 既定 ON（config.h V1_SETTLED_MEASURED）
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  CHECK(h.ax.duty_terms().in_band);  // ARMING のブレーキ解除から HOLDING まで 300 ms 静止 → 入っている
  h.stall = true;
  h.raw -= 25;  // 出る側を超えて抜ける
  h.step();
  CHECK(!h.ax.duty_terms().in_band && h.ax.duty_permille() == -70);
  h.raw += 25;  // 誤差 0 に戻る。実測速度の窓（100 ms）が更新されるまで 250 count/s を示し、その後 200 ms 待つ
  h.step();
  CHECK(!h.ax.duty_terms().in_band && h.ax.duty_permille() == 0);  // 入っていないが P = 0 なので出力 0
  h.drive(150, h.ax.actual_count());
  CHECK(!h.ax.duty_terms().in_band);  // 150 ms ではまだ
  h.drive(200, h.ax.actual_count());
  CHECK(h.ax.duty_terms().in_band);  // 遅くとも 100 + 200 = 300 ms で入る
  // 中では誤差 15 でも出力は I（0）のまま（出る側 20 まで）
  h.raw -= 15;
  h.step();
  CHECK(h.ax.duty_terms().in_band && h.ax.duty_permille() == 0);
  // 軌道が動けば即抜ける
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = false;
  h.drive(50, h.ax.actual_count() + 500);
  CHECK(!h.ax.duty_terms().in_band);
}

// 実測速度が 10 count/s を超えている間（1 count / 25 ms = 40 count/s）は |誤差| ≤ 10 でも入らず、生の P を出す。止まれば
// （窓が 0 を示してから）200 ms で入る。settled_ms = 0（-DV1_SETTLED_MEASURED=0）なら従来どおり即入る。
inline void test_settled_blocked_by_measured_motion_and_legacy_immediate() {
  Harness h;
  h.ax.tuning.ki_permille_per_count_s = 0.0;  // 生の P を見やすく
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.step();
  h.raw += 25;
  h.step();
  // 25 ms ごとに 1 count ずつずれる（誤差 −1 … −8、|誤差| ≤ 10）: 実測 40 count/s → 入らない → 生の P（誤差 −8 → −1.6 → −2 permille → MD13S +2）
  for (int i = 0; i < 8; ++i) {
    h.raw += 1;
    h.drive(25, h.ax.actual_count() - (i + 1));  // 目標は元の位置（actual が +1 ずつ増えているので −(i+1)）
  }
  CHECK(!h.ax.duty_terms().in_band);
  CHECK(h.ax.following_error_count() == 8 && h.ax.duty_permille() == 2);
  // 止まる: 窓が 0 になってから 200 ms 以内に入る（上限 300 ms）
  h.drive(300, h.ax.target_count());
  CHECK(h.ax.duty_terms().in_band && h.ax.duty_permille() == 0);
  // settled_ms = 0: 即入る
  Harness g;
  g.ax.tuning.settled_ms = 0;
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.stall = true;
  g.raw -= 25;
  g.step();
  CHECK(!g.ax.duty_terms().in_band);
  g.raw += 25;
  g.step();
  CHECK(g.ax.duty_terms().in_band);
}

// S が ON でも停止ブースト（H4）の 2 s の上限 → ENCODER_FAULT はそのまま（|誤差| > 20 で止まったまま）。
inline void test_settled_on_keeps_stationary_stall_fault() {
  Harness h;
  CHECK(h.ax.tuning.settled_ms > 0 && h.ax.tuning.ki_permille_per_count_s > 0.0);  // 既定（全部 ON）
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.step();
  h.drive(1990, h.ax.actual_count() + 25);
  CHECK(h.ax.state() == st::HOLDING);
  h.drive(20, h.ax.actual_count() + 25);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(h.ax.integral_permille() == 0 && h.ax.duty_permille() == 0);
}


// ---- Issue #25 手順 9「T」: 追従中の動き出し補助 ----------------------------------------------------------------

// 軌道 110 count/s で動いているのにカウントが動かず |誤差| が増え続けて 150 ms → |誤差| ≈ 17（床の 20 未満）でも床 7 % を掛ける。
// カウントが 3 以上動けば即解除。
inline void test_track_assist_triggers_after_150ms_and_releases_on_motion() {
  Harness h;
  h.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  CHECK(h.ax.tuning.assist_ms == V1_ASSIST_MS && h.ax.tuning.assist_ms == 150);  // 既定 ON（config.h V1_TRACK_ASSIST）
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;
  h.pos_max_vel = 110;
  const int32_t start = h.ax.actual_count();
  h.drive(140, start + 5000);
  // 140 ms: |誤差| ≈ 15、FF 11 + P 3 → 生の 14（床 OFF、補助もまだ）
  CHECK(!h.ax.duty_terms().assist_on && !h.ax.duty_terms().floor_on);
  CHECK(h.ax.duty_permille() < 0 && h.ax.duty_permille() > -30);
  h.drive(30, start + 5000);
  // 170 ms: 補助 ON → 床 70（|誤差| ≈ 18 < 20 なので E の床ではない）
  const int32_t fe = -h.ax.following_error_count();
  CHECK(fe > 10 && fe < 20);
  CHECK(h.ax.duty_terms().assist_on && !h.ax.duty_terms().floor_on && h.ax.duty_permille() == -70);
  CHECK(h.ax.state() == st::ACTIVE);
  // 動き出す（3 count 以上）→ 次の tick で解除、生の FF + P に戻る
  h.stall = false;
  h.plant_vel = 2000.0;
  h.drive(10, start + 5000);
  CHECK(!h.ax.duty_terms().assist_on);
  CHECK(h.ax.duty_permille() > -70 && h.ax.duty_permille() < 0);
}

// 軌道が 100 count/s 以下なら対象外（80 count/s: 150 ms 後も生の出力。誤差が 20 を超えれば E の床）。assist_ms = 0
// （-DV1_TRACK_ASSIST=0）なら 110 count/s でも掛けない。
inline void test_track_assist_requires_trajectory_speed_and_can_be_disabled() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;
  h.pos_max_vel = 80;
  const int32_t start = h.ax.actual_count();
  h.drive(200, start + 5000);  // |誤差| ≈ 16
  CHECK(!h.ax.duty_terms().assist_on && !h.ax.duty_terms().floor_on && h.ax.duty_permille() > -30);
  h.drive(100, start + 5000);  // |誤差| ≈ 24 → E の床
  CHECK(!h.ax.duty_terms().assist_on && h.ax.duty_terms().floor_on && h.ax.duty_permille() == -70);
  Harness g;
  g.ax.tuning.assist_ms = 0;
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.set_position(g.ax.actual_count());
  Harness::accepted(g.start());
  g.stall = true;
  g.pos_max_vel = 110;
  g.drive(170, g.ax.actual_count() + 5000);
  CHECK(!g.ax.duty_terms().assist_on && !g.ax.duty_terms().floor_on && g.ax.duty_permille() > -30);
}

// 補助は保護を妨げない: 軌道 600 count/s で止まったままなら FOLLOWING_ERROR（400）は従来どおり約 0.7 s で出る。
// kp 6 で FF + P が 15 %（duty_max 20 の 75 %）に届けば V1_STALL_MS（600 ms）の無パルスで ENCODER_FAULT。
inline void test_track_assist_does_not_block_following_error_or_moving_stall_fault() {
  Harness h;
  v1::Config c = base_config();
  c.following_error_limit_count = 400;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;
  h.pos_max_vel = 600;
  const int32_t start = h.ax.actual_count();
  const uint64_t t0 = h.now;
  h.drive(200, start + 5000);
  CHECK(h.ax.state() == st::ACTIVE && h.ax.duty_terms().assist_on);  // 補助は対象（|誤差| > 20 なので E の床も ON）
  CHECK(h.ax.duty_permille() <= -70 && h.ax.duty_permille() > -150);  // FF 60 + P（誤差 ≈ 110 → 22）。15 % 未満: 無パルス判定は回らない
  for (int i = 0; i < 80 && h.ax.state() != st::FAULT; ++i) h.drive(10, start + 5000);  // SET_POSITION を送り続ける（lease を切らさない）
  CHECK(h.ax.state() == st::FAULT && h.now - t0 <= 1000 * MS);  // 400 / 600 ≈ 0.67 s ＋ 加速ぶん
  CHECK(h.ax.fault_code() == fc::FOLLOWING_ERROR);
  CHECK(h.ax.duty_permille() == 0 && !h.ax.duty_terms().assist_on && h.ax.integral_permille() == 0);
  // 移動中の無パルス（ENCODER_FAULT）: kp 6・duty_min 9・duty_max 20 → 誤差 150 で FF 60 + P 90 = 150 ≥ 150 → 200 ms で FAULT
  Harness g;
  g.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  v1::Config c2 = base_config();
  c2.following_error_limit_count = 100000;
  Harness::accepted(g.config_with(c2));
  Harness::accepted(g.set_limits(20, 9, 6, 100, 10, 20));
  g.reach_holding_from_configured();
  g.set_position(g.ax.actual_count());
  Harness::accepted(g.start());
  g.stall = true;
  g.pos_max_vel = 600;
  const int32_t gstart = g.ax.actual_count();
  const uint64_t gt0 = g.now;
  g.drive(160, gstart + 5000);
  CHECK(g.ax.state() == st::ACTIVE && g.ax.duty_terms().assist_on && g.ax.duty_permille() <= -90);
  // 誤差 150 まで ≈ 0.25 s ＋ V1_STALL_MS 600 ms ≈ 0.85 s。窓が 200 ms のときの 70 回では届かない。
  for (int i = 0; i < 120 && g.ax.state() != st::FAULT; ++i) g.drive(10, gstart + 5000);
  CHECK(g.ax.state() == st::FAULT && g.now - gt0 <= 1200 * MS);
  CHECK(g.ax.fault_code() == fc::ENCODER_FAULT);
}


// 床の向きと ON/OFF は FF + P で決め、I は入れない（Codex/Fable レビュー）: 保持で学んだ I = +30 を持ったまま目標が下に
// 25 count 動く（誤差 −25、P −5）と、I 込みの合成は +25 だが床は −70（ON、誤差の向き）。I が床を消して停止ブースト対象のまま
// 2 s で ENCODER_FAULT になる経路を塞ぐ。I は戻す向き（負）なので減っていく。
inline void test_integral_does_not_flip_floor_direction_when_stationary() {
  V1Tuning t;  // kp 2、床 7
  V1DutyState st;
  st.i_pm = 30.0;
  V1DutyTerms o;
  CHECK(v1_duty_command(-25, 0.0, st, t, 0, &o, 0.005) == -70 && o.floor_on && o.i_permille == 30);
  CHECK(st.i_pm < 30.0 && st.i_pm > 29.9);  // 戻す向き: 0.1 × 25 × 0.005 = 0.0125 減る
  for (int i = 0; i < 400; ++i) CHECK(v1_duty_command(-25, 0.0, st, t, 0, nullptr, 0.005) == -70);
  CHECK(st.i_pm < 25.0 + 1e-6 && st.i_pm > 24.9);  // 2 s で 5 減る
  // I と同じ向きの合成は従来どおり: 誤差 +25（P +5）・I +30 → +35 < 床 70 → 70
  CHECK(v1_duty_command(25, 0.0, st, t, 0, &o, 0.0) == 70 && o.floor_on);
  // 床 OFF（|誤差| ≤ 15）なら I 込みの生の合成（P −3 + I 25 = +22）がそのまま出る
  CHECK(v1_duty_command(-15, 0.0, st, t, 0, &o, 0.0) == 22 && !o.floor_on);
  // 軸を通して: 保持で I を持ったまま目標が逆へ 25 動いても停止ブーストが効き、2 s で ENCODER_FAULT にならず動けば抜ける
  Harness h;
  h.ax.tuning.settled_ms = 0;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;
  h.step();
  h.raw += 10;  // 誤差 15 で 2 s → I = 3
  h.drive(2000, h.ax.actual_count() + 15);
  CHECK(h.ax.integral_permille() == 3);
  h.raw += 40;  // 誤差 −25（I は +3 のまま）→ 床 +70（MD13S −→ +）、停止ブーストが上がる
  h.drive(100, h.ax.actual_count() - 25);
  CHECK(h.ax.duty_permille() == 70 && h.ax.duty_terms().floor_on);
  h.drive(400, h.ax.actual_count() - 25);
  CHECK(h.ax.state() == st::HOLDING && h.ax.stall_boost_permille() == 20);
}


// ---- Issue #25 r6: 方向別 FF ＋ PI、走行中の床なし、保持 2/4、動き出しパルス（config.h V1_FF_DIRECTIONAL） ----------------
// ホストテストは -DV1_FF_DIRECTIONAL=0 で組む（run_tests.sh）ので、既定の V1Tuning は従来の経路。r6 は V1Tuning(true) で選ぶ。

inline V1Tuning r6_tuning(int kp = 6, int duty_max = 20) {
  V1Tuning t(true);
  t.kp_percent_per_100 = kp;
  t.duty_min = 9;
  t.duty_max = duty_max;
  return t;
}

// 方向別 FF の値（実測 11-02: 上 46 + 0.10 v、下 −7 + 0.107 v [permille]）とテーパ（|v| < 30 で切片を w = |v|/30 倍）。
// 旧 ff_count_s_per_percent は無視する。
inline void test_r6_ff_directional_values_and_taper() {
  CHECK(V1Tuning{}.ff_directional == (V1_FF_DIRECTIONAL != 0));  // 既定はコンパイルスイッチ（テストは 0 で組む）
  CHECK(!V1Tuning{}.ff_directional && fabs(V1Tuning{}.ki_permille_per_count_s - (V1_INTEGRAL ? V1_KI_PERMILLE_PER_COUNT_S : 0.0)) < 1e-12);
  V1Tuning t = r6_tuning();
  CHECK(t.ff_directional && fabs(t.ki_permille_per_count_s - 0.2) < 1e-12);  // Ki 0.02 %/(count·s)
  CHECK(t.ff_a_up_permille == 46 && t.ff_a_down_permille == -7 && t.ff_taper_count_s == 30);
  CHECK(fabs(t.ff_b_up_permille_per_count_s - 0.10) < 1e-12 && fabs(t.ff_b_down_permille_per_count_s - 0.107) < 1e-12);
  CHECK(v1_ff_directional_pm(0.0, t) == 0.0);
  CHECK(fabs(v1_ff_directional_pm(300.0, t) - 76.0) < 1e-9);     // 46 + 30
  CHECK(fabs(v1_ff_directional_pm(-300.0, t) + 39.1) < 1e-9);    // −7 − 32.1
  CHECK(fabs(v1_ff_directional_pm(600.0, t) - 106.0) < 1e-9);    // 46 + 60
  CHECK(fabs(v1_ff_directional_pm(-600.0, t) + 71.2) < 1e-9);    // −7 − 64.2
  CHECK(fabs(v1_ff_directional_pm(30.0, t) - 49.0) < 1e-9);      // w = 1 ちょうど
  CHECK(fabs(v1_ff_directional_pm(-30.0, t) + 10.21) < 1e-9);
  CHECK(fabs(v1_ff_directional_pm(15.0, t) - 24.5) < 1e-9);      // w = 0.5: 23 + 1.5
  CHECK(fabs(v1_ff_directional_pm(-15.0, t) + 5.105) < 1e-9);    // −3.5 − 1.605
  CHECK(fabs(v1_ff_directional_pm(3.0, t) - 4.9) < 1e-9);        // w = 0.1: 4.6 + 0.3
  // v1_duty_command 経由（誤差 0、I 0）: FF がそのまま出る（床なし、四捨五入 1 回）
  V1DutyState st;
  V1DutyTerms o;
  CHECK(v1_duty_command(0, 300.0, st, t, 0, &o) == 76 && o.ff_permille == 76 && o.mode == V1DutyMode::MOVE && !o.floor_on);
  CHECK(v1_duty_command(0, -300.0, st, t, 0, &o) == -39 && o.ff_permille == -39);
  CHECK(v1_duty_command(0, 600.0, st, t, 0, &o) == 106);
  CHECK(v1_duty_command(0, -600.0, st, t, 0, &o) == -71);
  CHECK(v1_duty_command(0, 15.0, st, t, 0, &o) == 25);   // 24.5 → 25（0 から遠い側）
  CHECK(v1_duty_command(0, -15.0, st, t, 0, &o) == -5);
  // 旧 ff は無視（1 にしても同じ）。boost・assist=false も無関係
  t.ff_count_s_per_percent = 1;
  CHECK(v1_duty_command(0, 300.0, st, t, 50, &o) == 76 && o.boost_permille == 50);
  // 係数を変えれば追従する（コンサルの更新に備える）
  t.ff_a_up_permille = 60;
  t.ff_b_up_permille_per_count_s = 0.03;
  CHECK(v1_duty_command(0, 300.0, st, t, 0, &o) == 69);  // 60 + 9
}

// 走行中に床は無い: 1〜5 % の生の FF + P + I がそのまま出る。duty_min 9 は関係しない。
inline void test_r6_no_floor_while_moving() {
  V1Tuning t = r6_tuning();  // kp 6、duty_min 9、duty_max 20
  V1DutyState st;
  V1DutyTerms o;
  CHECK(v1_duty_command(0, 15.0, st, t, 0, &o) == 25 && !o.floor_on && o.mode == V1DutyMode::MOVE);  // 2.5 %
  CHECK(v1_duty_command(3, 0.0, st, t, 0, &o, 0.0, false) == 2 && o.p_permille == 2 && !o.in_band);  // P 1.8 → 2（0.2 %）
  CHECK(v1_duty_command(10, 0.0, st, t, 0, &o, 0.0, false) == 6);   // 誤差 10 → 0.6 %（旧: 不感帯で 0 か床 9 %）
  CHECK(v1_duty_command(25, 0.0, st, t, 0, &o) == 15 && !o.floor_on);  // 誤差 25 → 1.5 %（旧: 床 ON で 90）
  CHECK(v1_duty_command(-25, 0.0, st, t, 0, &o) == -15);
  // 軌道と逆向きの誤差（先行）: 合成は生のまま（符号不一致の抑制なし）。誤差 −50・軌道 +300 → FF 76 + P −30 = 46
  CHECK(v1_duty_command(-50, 300.0, st, t, 0, &o) == 46);
  // 上限は duty_max で 1 回だけ飽和
  CHECK(v1_duty_command(300, 600.0, st, t, 0, &o) == 200);   // 106 + 180 → 200
  CHECK(v1_duty_command(-300, -600.0, st, t, 0, &o) == -200);
}

// PI: Ki 0.2 permille/(count·s)、|I| ≤ 30、|dI/dt| ≤ 10/s。アンチワインドアップは「飽和した向きへ積み増さない」だけ。
// 符号不一致の抑制は無い（先行して逆向きに補正するのは正当）。戻す向きは飽和中も許す。
inline void test_r6_pi_anti_windup_at_duty_max() {
  V1Tuning t = r6_tuning();  // kp 6、duty_max 20
  V1DutyState st;
  // 誤差 400 静止: P 240 → 200 で飽和。I は 2 s 経っても 0
  for (int i = 0; i < 400; ++i) CHECK(v1_duty_command(400, 0.0, st, t, 0, nullptr, 0.005) == 200);
  CHECK(st.i_pm == 0.0);
  // 誤差 30: P 18（飽和しない）→ 0.2 × 30 × 0.005 = 0.03 / tick、1 s で 6 → 出力 24
  for (int i = 0; i < 200; ++i) v1_duty_command(30, 0.0, st, t, 0, nullptr, 0.005);
  CHECK(fabs(st.i_pm - 6.0) < 1e-9);
  V1DutyTerms o;
  CHECK(v1_duty_command(30, 0.0, st, t, 0, &o) == 24 && o.i_permille == 6 && o.p_permille == 18);
  // 変化率の上限: 誤差 400 で duty_max 60（飽和しない）→ 0.4 / tick が 0.05 / tick に → 1 s で 10。上限 30
  V1Tuning t60 = r6_tuning(6, 60);
  V1DutyState st2;
  for (int i = 0; i < 200; ++i) v1_duty_command(400, 0.0, st2, t60, 0, nullptr, 0.005);
  CHECK(fabs(st2.i_pm - 10.0) < 1e-9);
  for (int i = 0; i < 1000; ++i) v1_duty_command(400, 0.0, st2, t60, 0, nullptr, 0.005);
  CHECK(fabs(st2.i_pm - 30.0) < 1e-9);
  // 戻す向きは飽和中も許す: I +30 のまま誤差 −400（飽和 −200）→ I は 1 s で 10 減る
  for (int i = 0; i < 200; ++i) CHECK(v1_duty_command(-400, 0.0, st2, t, 0, nullptr, 0.005) == -200);
  CHECK(fabs(st2.i_pm - 20.0) < 1e-9);
  // 0 を越えて負（飽和した向きへ積み増す）には行かない
  for (int i = 0; i < 1000; ++i) v1_duty_command(-400, 0.0, st2, t, 0, nullptr, 0.005);
  CHECK(fabs(st2.i_pm) < 1e-9);
  // 符号不一致でも積む: 誤差 +50・軌道 −600（FF −71 + P 30 = −41）→ I は正に育つ（旧 E の抑制は無い）
  V1DutyState st3;
  for (int i = 0; i < 200; ++i) CHECK(v1_duty_command(50, -600.0, st3, t, 0, nullptr, 0.005) < 0);
  CHECK(fabs(st3.i_pm - 10.0) < 1e-9);  // 0.2 × 50 × 0.005 = 0.05 / tick（変化率の上限ちょうど）× 200
  // I は小数のまま持ち、出力は 1 回だけ四捨五入: I 0.6 + P 0 → 1
  V1DutyState st4;
  st4.i_pm = 0.6;
  CHECK(v1_duty_command(0, 0.0, st4, t, 0, &o, 0.0, false) == 1 && o.i_permille == 1);
}

// 保持: 軌道速度 0・|誤差| ≤ 2・settle_ok で入る（I と出力を 0）。|誤差| ≤ 4 の間とどまり、> 4 か軌道が動けば出る。
// settle_ok = false（整定の 200 ms がまだ）なら |誤差| ≤ 2 でも入らず生の P + I。
inline void test_r6_hold_entry_exit_and_zeroing() {
  V1Tuning t = r6_tuning();
  V1DutyState st;
  st.i_pm = 12.0;
  V1DutyTerms o;
  CHECK(v1_duty_command(2, 0.0, st, t, 0, &o, 0.005, false) == 13 && !o.in_band && o.mode == V1DutyMode::MOVE);  // P 1.2 + I 12
  CHECK(v1_duty_command(3, 0.0, st, t, 0, &o, 0.005, true) != 0 && !o.in_band);  // 3 > 2: 入らない
  CHECK(v1_duty_command(2, 0.0, st, t, 0, &o, 0.005, true) == 0 && o.in_band && o.mode == V1DutyMode::HOLD);
  CHECK(st.i_pm == 0.0 && o.i_permille == 0 && !o.floor_on);
  CHECK(v1_duty_command(4, 0.0, st, t, 0, &o, 0.005) == 0 && o.in_band);   // 出口 4 まではとどまる（settle_ok は見ない）
  CHECK(v1_duty_command(-4, 0.0, st, t, 0, &o, 0.005, false) == 0 && o.in_band);
  CHECK(st.i_pm == 0.0);  // 保持中は積まない
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.005) == 3 && !o.in_band && o.mode == V1DutyMode::MOVE);  // 5 > 4: 出る、P 3
  CHECK(v1_duty_command(2, 0.0, st, t, 0, &o, 0.005) == 0 && o.in_band);  // 再び入る
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.005) == 56 && !o.in_band);  // 軌道が動けば出る（FF 46 + 10）
  // 従来の deadband / deadband_out（10 / 20）は r6 では使わない
  CHECK(v1_duty_command(8, 0.0, st, t, 0, &o, 0.005) == 5 && !o.in_band);
}

// 動き出しパルス（v1_duty_command 側）: breakaway = true のとき、誤差の向きに方向別の動き出し Duty と |合成|（同じ向きのとき）の
// 大きい方。duty_max 以内。I は凍結。
inline void test_r6_breakaway_pulse_output() {
  V1Tuning t = r6_tuning();
  V1DutyState st;
  V1DutyTerms o;
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.005, true, true) == 90 && o.mode == V1DutyMode::BRK && !o.floor_on);
  CHECK(st.i_pm == 0.0);  // 凍結
  CHECK(v1_duty_command(-5, 0.0, st, t, 0, &o, 0.005, true, true) == -60 && o.mode == V1DutyMode::BRK);
  CHECK(v1_duty_command(300, 0.0, st, t, 0, &o, 0.0, true, true) == 180);  // 合成 180 > 90 → 合成
  CHECK(v1_duty_command(0, 0.0, st, t, 0, &o, 0.0, false, true) == 0 && o.mode == V1DutyMode::MOVE);  // 向きが無い
  st.i_pm = -20.0;  // 古い I で合成が逆（P 3 − 20 = −17）でも、パルスは誤差の向き（上 +90。目標から離れる側へ蹴らない）
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.0, true, true) == 90);
  st.i_pm = 20.0;   // 合成が同じ向きで大きければ合成（P 3 + 20 = 23 < 90 → 90。誤差 300: 180 + 20 = 200 → 上限 200）
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.0, true, true) == 90);
  CHECK(v1_duty_command(300, 0.0, st, t, 0, &o, 0.0, true, true) == 200);
  st.i_pm = 0.0;
  t.duty_max = 5;
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.0, true, true) == 50);  // duty_max で頭打ち
  t.breakaway_up_permille = 120;
  t.duty_max = 20;
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.0, true, true) == 120);
}

// 軸を通して: 保持に入るのは「軌道完了・|誤差| ≤ 2・|実測速度| ≤ 10 が 200 ms」。入れば出力 0・I 0。|誤差| 3〜4 ではとどまり、5 で出る。
inline void test_r6_axis_hold_requires_dwell_and_holds_to_4() {
  Harness h;
  h.ax.tuning = r6_tuning(2);
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD && h.ax.duty_permille() == 0 && h.ax.duty_terms().in_band);
  h.stall = true;
  h.raw += 3;  // 誤差 −3
  h.drive(50, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD && h.ax.duty_permille() == 0);
  h.raw += 2;  // 誤差 −5 → 出る。P = −1 permille（kp 2）→ duty +1（sign 1）
  h.drive(50, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::MOVE && h.ax.duty_permille() == 1 && !h.ax.duty_terms().in_band);
  // 戻る（誤差 −2）: 実測速度（100 ms 窓、最長 200 ms 古い）が ≤ 10 になってから 200 ms → 入るまで最長 400 ms。50 ms では入らない
  h.raw -= 3;
  h.drive(50, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::MOVE && !h.ax.duty_terms().in_band);  // P −0.4 → 0 だが保持ではない
  h.drive(450, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD && h.ax.duty_permille() == 0 && h.ax.integral_permille() == 0);
  CHECK(h.ax.state() == st::HOLDING);
}

// 軸を通して: 静止で |誤差| 10 のまま進まない → 100 ms 後に動き出しパルス（上 90 → duty −90）を 100 ms、進まなければ 100 ms 空けて
// 次のパルス。5 回で打ち切り。パルスは静止 2 s の監視の時計を戻さず、2 s で ENCODER_FAULT。
inline void test_r6_breakaway_pulses_then_stall_watchdog_at_2s() {
  Harness h;
  h.ax.tuning = r6_tuning(2);
  h.ax.tuning.vel_kd_permille_per_count_s = 0.0;  // カウントを 1 刻みで跳ばすので v_fast が跳ねる。r6/r8 の仕組みを見る試験なので減衰（r9）は切る
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 10;  // 誤差 +10（上へ）
  h.step();     // 対象になる（窓の開始・監視の起点）
  const uint64_t t0 = h.now;
  CHECK(h.ax.duty_terms().mode == V1DutyMode::MOVE && h.ax.duty_permille() == -2);  // 生の P 2
  h.drive(90, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::MOVE && !h.ax.breakaway_active());
  h.drive(20, h.ax.target_count());  // 110 ms: パルス ON
  CHECK(h.ax.duty_terms().mode == V1DutyMode::BRK && h.ax.duty_permille() == -90 && h.ax.breakaway_active());
  h.drive(80, h.ax.target_count());  // 190 ms: まだパルス
  CHECK(h.ax.duty_terms().mode == V1DutyMode::BRK);
  h.drive(30, h.ax.target_count());  // 220 ms: 終わって 1 回目を数えた。生の P に戻る
  CHECK(h.ax.duty_terms().mode == V1DutyMode::MOVE && h.ax.duty_permille() == -2 && h.ax.breakaway_pulses() == 1);
  // 5 回まで（1 回目は上で観測済み）。1.1 s 時点で 5 回、以後は打たない
  int brk_ticks = 0;
  while (h.now - t0 < 1900 * MS) {
    h.drive(10, h.ax.target_count());
    if (h.ax.duty_terms().mode == V1DutyMode::BRK) ++brk_ticks;
  }
  CHECK(h.ax.breakaway_pulses() == 5 && !h.ax.breakaway_active());
  CHECK(brk_ticks >= 36 && brk_ticks <= 44);  // 残り 4 回 × 100 ms = 400 ms ≈ 40 回（10 ms 刻みの観測）
  CHECK(h.ax.state() == st::HOLDING && h.ax.duty_permille() <= -2 && h.ax.duty_permille() >= -5);  // 生の P 2 ＋ 育った I（≤ 3）
  h.drive(120, h.ax.target_count());  // 2 s
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(h.ax.duty_permille() == 0 && h.ax.integral_permille() == 0 && h.ax.breakaway_pulses() == 0);
  CHECK(h.now - t0 <= 2100 * MS);
}

// パルス中に 3 count 進めば次の tick で解除され、数は 0 に戻る。下向きの要求は −60（duty +60）。
// パルスを切った構成（breakaway_window_ms = 0）でも 2 s の監視はそのまま。
inline void test_r6_breakaway_releases_on_progress_and_down_direction() {
  Harness h;
  h.ax.tuning = r6_tuning(2);
  h.ax.tuning.vel_kd_permille_per_count_s = 0.0;  // カウントを 1 刻みで跳ばすので v_fast が跳ねる。r6/r8 の仕組みを見る試験なので減衰（r9）は切る
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw += 10;  // 誤差 −10（下へ）
  h.step();
  h.drive(110, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::BRK && h.ax.duty_permille() == 60);
  h.raw -= 3;  // 下へ 3 進んだ
  h.drive(10, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::MOVE && !h.ax.breakaway_active() && h.ax.breakaway_pulses() == 0);
  CHECK(h.ax.duty_permille() > 0 && h.ax.duty_permille() < 10);  // 生の P（誤差 −7 → −1.4 → −1 → duty +1）
  // 自重で逆へ動くのは「進み」ではない: 上の要求（誤差 +10）でカウントが下がっても窓は取り直されずパルスが出る
  Harness g;
  g.ax.tuning = r6_tuning(2);
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.stall = true;
  g.raw -= 10;
  g.step();
  g.raw -= 5;  // 逆向きに 5
  g.drive(110, g.ax.target_count());
  CHECK(g.ax.duty_terms().mode == V1DutyMode::BRK && g.ax.duty_permille() == -90);
  // パルス無効でも監視は 2 s
  Harness k;
  k.ax.tuning = r6_tuning(2);
  k.ax.tuning.breakaway_window_ms = 0;
  Harness::accepted(k.config_with(c));
  k.reach_holding_from_configured();
  k.stall = true;
  k.raw -= 10;
  k.step();
  const uint64_t t0 = k.now;
  k.drive(1900, k.ax.target_count());
  CHECK(k.ax.state() == st::HOLDING && k.ax.duty_terms().mode == V1DutyMode::MOVE && k.ax.breakaway_pulses() == 0);
  k.drive(120, k.ax.target_count());
  CHECK(k.ax.state() == st::FAULT && k.ax.fault_code() == fc::ENCODER_FAULT && k.now - t0 <= 2100 * MS);
}

// 追従（プラントが付いてくる）では床もパルスも出ず、出力は FF（300 count/s 上り → 76）± 小さい P + I。到着後に保持。
inline void test_r6_axis_tracking_outputs_raw_ff_without_floor_or_pulse() {
  Harness h;
  h.ax.tuning = r6_tuning(6);
  h.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  v1::Config c = base_config();
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.pos_max_vel = 300;
  const int32_t start = h.ax.actual_count();
  bool any_brk = false, any_floor = false;
  int min_mag = 1000, max_mag = 0;
  for (int i = 0; i < 100; ++i) {  // 1 s: 加速 15 ms の後 300 count/s で約 290 count
    h.drive(10, start + 2000);
    if (h.ax.duty_terms().mode == V1DutyMode::BRK) any_brk = true;
    if (h.ax.duty_terms().floor_on) any_floor = true;
    if (i >= 5) {
      const int mag = -h.ax.duty_permille();  // 上り（カウント増）は sign 1 で負
      if (mag < min_mag) min_mag = mag;
      if (mag > max_mag) max_mag = mag;
    }
  }
  CHECK(!any_brk && !any_floor && h.ax.state() == st::ACTIVE);
  CHECK(min_mag >= 70 && max_mag <= 85);  // FF 76（旧: 床 90 か生の 30）
  CHECK(h.ax.actual_count() - start > 250);
  // 下り: FF −39 → duty +39 前後
  h.drive(1000, start);
  CHECK(h.ax.duty_permille() >= 33 && h.ax.duty_permille() <= 46 && h.ax.duty_terms().mode == V1DutyMode::MOVE);
  // 到着して止まれば保持（出力 0）
  h.drive(1500, start);
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD && h.ax.duty_permille() == 0);
}

// 保護は変わらない: 600 count/s で止まったままなら FOLLOWING_ERROR 400 は約 0.7 s。kp 6 / duty_max 20 なら FF 106 + P が 15 %
// に届いた所から V1_STALL_MS（600 ms）の無パルスで ENCODER_FAULT（動き出しパルス 90 は 15 % 未満で判定に入らない）。
inline void test_r6_following_error_and_moving_stall_fault_unchanged() {
  Harness h;
  h.ax.tuning = r6_tuning(2, 40);  // duty_max 40 → 無パルスのしきい値 30 %。FF 106 + P 80 = 186 はそれ未満なので FOLLOWING_ERROR 側
  v1::Config c = base_config();
  c.following_error_limit_count = 400;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;
  h.pos_max_vel = 600;
  const int32_t start = h.ax.actual_count();
  const uint64_t t0 = h.now;
  for (int i = 0; i < 100 && h.ax.state() != st::FAULT; ++i) h.drive(10, start + 5000);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::FOLLOWING_ERROR && h.now - t0 <= 1000 * MS);
  CHECK(h.ax.duty_permille() == 0 && h.ax.integral_permille() == 0);
  Harness g;
  g.ax.tuning = r6_tuning(6);
  g.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  v1::Config c2 = base_config();
  c2.following_error_limit_count = 100000;
  Harness::accepted(g.config_with(c2));
  Harness::accepted(g.set_limits(20, 9, 6, 100, 10, 20));  // ff 100 は受理されるが無視
  CHECK(g.ax.tuning.ff_count_s_per_percent == 100 && g.ax.tuning.ff_directional);
  g.reach_holding_from_configured();
  g.set_position(g.ax.actual_count());
  Harness::accepted(g.start());
  g.stall = true;
  g.pos_max_vel = 600;
  const int32_t gstart = g.ax.actual_count();
  const uint64_t gt0 = g.now;
  g.drive(60, gstart + 5000);
  CHECK(g.ax.duty_permille() <= -106 && g.ax.state() == st::ACTIVE);  // FF 106 ＋ P
  // しきい値まで ≈ 0.14 s ＋ V1_STALL_MS 600 ms。窓 200 ms の上限 600 ms では足りない。
  for (int i = 0; i < 120 && g.ax.state() != st::FAULT; ++i) g.drive(10, gstart + 5000);
  CHECK(g.ax.state() == st::FAULT && g.ax.fault_code() == fc::ENCODER_FAULT && g.now - gt0 <= 1100 * MS);
}

// PR #33 Codex P1: 遅い軌道（25 count/s）に 8 count 遅れて付いていく（100 ms に 2〜3 count）だけでは動き出しパルスを打たない。
// 軌道が速い（300 count/s）のに軸が止まったまま |誤差| が増え続ければ 200 ms の窓の後に打つ（合成が 90 を超えていればその値）。
inline void test_r6_breakaway_not_fired_by_slow_lag_but_fires_on_moving_stall() {
  Harness h;
  h.ax.tuning = r6_tuning(6);
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;  // プラントはテストが手で動かす
  h.pos_max_vel = 25;
  const int32_t start = h.ax.actual_count();
  int brk_ticks = 0;
  uint64_t next_move = h.now + 320 * MS;  // 8 count 遅れてから 40 ms に 1 count（25 count/s）で付いていく
  for (int i = 0; i < 200; ++i) {  // 2 s
    h.drive(10, start + 2000);
    if (h.now >= next_move) {
      h.raw += 1;
      next_move += 40 * MS;
    }
    if (h.ax.duty_terms().mode == V1DutyMode::BRK) ++brk_ticks;
  }
  CHECK(h.ax.state() == st::ACTIVE && brk_ticks == 0 && h.ax.breakaway_pulses() == 0);
  const int64_t lag = static_cast<int64_t>(h.ax.target_count()) - h.ax.actual_count();
  CHECK(lag >= 5 && lag <= 12);  // 遅れは保たれている（対象外なのは遅れが小さいからではない）
  // 速い軌道で止まったまま: 200 ms の窓の後にパルス
  Harness g;
  g.ax.tuning = r6_tuning(6);
  g.ax.tuning.playout_delay_ms = 0;  // 従来の軌道（profile_step。目標へ即座に向かう）の時刻で書いた試験（Issue #45）
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.set_position(g.ax.actual_count());
  Harness::accepted(g.start());
  g.stall = true;
  g.pos_max_vel = 300;
  const int32_t gstart = g.ax.actual_count();
  g.drive(150, gstart + 2000);  // 加速 15 ms の後 300 count/s。150 ms では窓（200 ms）の途中
  CHECK(g.ax.state() == st::ACTIVE && g.ax.duty_terms().mode == V1DutyMode::MOVE && !g.ax.breakaway_active());
  g.drive(100, gstart + 2000);  // 250 ms
  CHECK(g.ax.duty_terms().mode == V1DutyMode::BRK && g.ax.breakaway_active() && g.ax.duty_permille() <= -90);
}

// つなぎ（Issue #44）: 静止 2 s の監視は、どちら向きでも 3 count 動けば時計を戻す（エンコーダが生きている証拠）。乗員に押されて
// 目標から離れ続ける間は ENCODER_FAULT にしない（PR #33 の Codex P2 で向きを限定したら、手をかけただけで FAULT になった）。
// 押さえ込まれて全く動かなければ従来どおり 2 s で ENCODER_FAULT（根本対策は #44 の r10）。
inline void test_r6_stall_watchdog_ignores_motion_away_from_target() {
  Harness h;
  h.ax.tuning = r6_tuning(2);
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 10;  // 誤差 +10（上へ）
  h.step();
  const uint64_t t0 = h.now;
  while (h.now - t0 < 1900 * MS) {
    h.drive(150, h.ax.target_count());
    h.raw -= 3;  // 150 ms ごとに目標から 3 count 離れる（従来の向きを問わない判定なら時計が戻っていた）
  }
  CHECK(h.ax.state() == st::HOLDING);
  h.drive(1000, h.ax.target_count());  // 押されて離れた後、動きが止まってから 2 s 経つまでは FAULT にしない
  CHECK(h.ax.state() == st::HOLDING);
  h.drive(1200, h.ax.target_count());  // 押さえ込まれて動かないまま 2 s → 従来どおり ENCODER_FAULT
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
  // 誤差の向きへ 3 count 進めば時計は戻る
  Harness g;
  g.ax.tuning = r6_tuning(2);
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.stall = true;
  g.raw -= 30;
  g.step();
  const uint64_t gt0 = g.now;
  while (g.now - gt0 < 2500 * MS) {
    g.drive(150, g.ax.target_count());
    g.raw += 3;
  }
  CHECK(g.ax.state() == st::HOLDING && g.ax.fault_code() == 0);
}

// r7: 静止目標で |誤差| = 3（保持に入れず、r6 ではパルスも監視も対象外だった帯）→ 100 ms 後に動き出しパルス、進めば保持へ。
// 進まなければ 5 回で打ち切り、2 s で ENCODER_FAULT（監視も > 2 から）。
inline void test_r7_stationary_error_3_gets_breakaway_and_watchdog() {
  Harness h;
  h.ax.tuning = r6_tuning(6);
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 10;  // 誤差 +10 で保持から出る（保持中は |誤差| > 4 まで保持のまま: ヒステリシス）
  h.step();
  h.raw += 7;   // 誤差 +3 まで戻って止まる（r6 ではここで P ＋ I が流れたまま止まった）
  h.step();
  h.drive(90, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::MOVE && !h.ax.breakaway_active());
  h.drive(20, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::BRK && h.ax.duty_permille() == -90);
  h.raw += 3;  // 進んだ → 誤差 0
  h.drive(10, h.ax.target_count());
  CHECK(!h.ax.breakaway_active() && h.ax.breakaway_pulses() == 0);
  h.drive(450, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD && h.ax.duty_permille() == 0);
  Harness g;
  g.ax.tuning = r6_tuning(6);
  Harness::accepted(g.config_with(c));
  g.reach_holding_from_configured();
  g.stall = true;
  g.raw -= 10;
  g.step();
  g.raw += 7;
  g.step();
  const uint64_t t0 = g.now;
  g.drive(1900, g.ax.target_count());
  CHECK(g.ax.state() == st::HOLDING && g.ax.breakaway_pulses() == 5);
  g.drive(200, g.ax.target_count());
  CHECK(g.ax.state() == st::FAULT && g.ax.fault_code() == fc::ENCODER_FAULT && g.now - t0 <= 2300 * MS);
}

// r8: 軌道速度 50〜150 count/s の帯では「引っかかった時だけ短く押す」。
// (A) 100 count/s で軸が止まったまま → 100 ms の窓の後に 50 ms のパルス、その後 200 ms は押さない（次は ~350 ms 後）。
// (B) 1 count 進めばパルスは次の tick で終わり、休みに入る。(C) 普通に付いてきている（10 ms に 1 count）なら押さない。
inline void test_r8_band_stuck_assist_short_pulses_with_cooldown() {
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  auto active = [&](Harness& h) {
    h.ax.tuning = r6_tuning(30);
    Harness::accepted(h.config_with(c));
    h.reach_holding_from_configured();
    h.set_position(h.ax.actual_count());
    Harness::accepted(h.start());
    h.pos_max_vel = 100;
  };
  // (A)
  Harness a;
  active(a);
  a.ax.tuning.kp_percent_per_100 = 10;  // kp 30 では止まったまま約 0.5 s で P が 15 % を超え無パルス保護が先に働く（それは正しい）
  a.stall = true;
  const int32_t sa = a.ax.actual_count();
  int onsets = 0, run = 0, max_run = 0;
  bool prev = false;
  std::vector<uint64_t> onset_t;
  const uint64_t t0 = a.now;
  for (int i = 0; i < 80; ++i) {  // 0.8 s
    a.drive(10, sa + 2000);
    const bool brk = a.ax.duty_terms().mode == V1DutyMode::BRK;
    if (brk && !prev) { ++onsets; onset_t.push_back(a.now - t0); run = 0; }
    if (brk) { ++run; if (run > max_run) max_run = run; }
    prev = brk;
  }
  CHECK(a.ax.state() == st::ACTIVE);
  CHECK(onsets >= 2 && onsets <= 4);
  CHECK(max_run <= 6);  // 50 ms（10 ms 刻みの観測で 5〜6）
  CHECK(onset_t.size() >= 2 && onset_t[0] >= 100 * MS && onset_t[0] <= 200 * MS);
  CHECK(onset_t.size() >= 2 && onset_t[1] - onset_t[0] >= 330 * MS && onset_t[1] - onset_t[0] <= 400 * MS);
  // (B)
  Harness b;
  active(b);
  b.stall = true;
  const int32_t sb = b.ax.actual_count();
  for (int i = 0; i < 30 && b.ax.duty_terms().mode != V1DutyMode::BRK; ++i) b.drive(10, sb + 2000);
  CHECK(b.ax.duty_terms().mode == V1DutyMode::BRK && b.ax.duty_permille() <= -90);
  b.raw += 1;  // 上へ 1 count
  b.drive(10, sb + 2000);
  CHECK(b.ax.duty_terms().mode != V1DutyMode::BRK && b.ax.breakaway_pulses() == 0);
  bool brk_in_cooldown = false;
  for (int i = 0; i < 18; ++i) {  // 休み 200 ms の間（止まったままでも）押さない
    b.drive(10, sb + 2000);
    if (b.ax.duty_terms().mode == V1DutyMode::BRK) brk_in_cooldown = true;
  }
  CHECK(!brk_in_cooldown);
  // (C)
  Harness d;
  active(d);
  d.stall = true;
  const int32_t sd = d.ax.actual_count();
  bool any_brk = false;
  d.drive(50, sd + 2000);  // 5 count ほど遅れる（> 2）
  for (int i = 0; i < 100; ++i) {
    d.raw += 1;  // 10 ms に 1 count = 100 count/s で付いていく
    d.drive(10, sd + 2000);
    if (d.ax.duty_terms().mode == V1DutyMode::BRK) any_brk = true;
  }
  CHECK(!any_brk && d.ax.state() == st::ACTIVE);
}

// 2026-11-02 16:10 の #3: 保持（出力 0）に入った後に 1〜2 count ずれて |誤差| 3〜4 のまま静止しても、静止の監視で ENCODER_FAULT にしない
inline void test_hold_drift_within_band_is_not_encoder_fault() {
  Harness h;
  h.ax.tuning = r6_tuning(30);
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.drive(600, h.ax.target_count());
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD && h.ax.duty_permille() == 0);
  h.stall = true;
  h.raw -= 3;  // 保持の中で 3 count ずれた（保持から出るのは > 4）
  h.drive(3000, h.ax.target_count());
  CHECK(h.ax.state() == st::HOLDING && h.ax.fault_code() == 0);
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD && h.ax.duty_permille() == 0);
}

// SET_LIMITS は従来どおり受理・LIMITS で返す。r6 で効くのは kp と duty_max。ff / duty_min / deadband は出力に影響しない。
inline void test_r6_set_limits_ff_and_floor_fields_are_ignored() {
  V1Tuning a = r6_tuning(6, 20);
  V1Tuning b = a;
  b.ff_count_s_per_percent = 50;
  b.duty_min = 3;
  b.deadband = 1;
  b.deadband_out = 2;
  V1DutyState sa, sb;
  for (int err = -60; err <= 60; err += 7) {
    for (double v : {0.0, 20.0, -200.0, 600.0}) {
      CHECK(v1_duty_command(err, v, sa, a, 0, nullptr, 0.005, false) == v1_duty_command(err, v, sb, b, 0, nullptr, 0.005, false));
    }
  }
  b.kp_percent_per_100 = 12;
  CHECK(v1_duty_command(50, 0.0, sa, a, 0, nullptr, 0.0, false) != v1_duty_command(50, 0.0, sb, b, 0, nullptr, 0.0, false));
  b.kp_percent_per_100 = 6;
  b.duty_max = 5;
  CHECK(v1_duty_command(0, 600.0, sa, a, 0, nullptr, 0.0, false) == 106 && v1_duty_command(0, 600.0, sb, b, 0, nullptr, 0.0, false) == 50);
}

// ---- Issue #35 r9: 速度誤差の減衰項（config.h V1_VEL_DAMPING）。40 ms 窓の速い速度推定 v_fast、u_d = clamp(Kd × (v_traj − v_fast), ±10) ----
// ホストテストは -DV1_VEL_DAMPING=0 で組む（run_tests.sh。既存の r6〜r8 のテストは r8 と同じ出力で検証する）。r9 は kd を明示的に入れて検証する。

inline V1Tuning r9_tuning(int kp = 30, int duty_max = 20) {
  V1Tuning t = r6_tuning(kp, duty_max);
  t.vel_kd_permille_per_count_s = V1_VEL_KD_PERMILLE_PER_COUNT_S;
  return t;
}

// 既定値とコンパイルスイッチの配線。従来の経路（V1Tuning(false)）では kd は常に 0。
inline void test_r9_tuning_defaults_and_switch() {
  CHECK(fabs(V1_VEL_KD_PERMILLE_PER_COUNT_S - 0.05) < 1e-12 && V1_VEL_D_MAX_PERMILLE == 10 && V1_V_FAST_WINDOW_MS == 40);
  CHECK(fabs(V1Tuning(true).vel_kd_permille_per_count_s - (V1_VEL_DAMPING ? 0.05 : 0.0)) < 1e-12);  // テストは 0 で組む
  CHECK(V1Tuning(false).vel_kd_permille_per_count_s == 0.0);
  CHECK(V1Tuning(true).vel_d_max_permille == 10 && V1Tuning(true).v_fast_window_ms == 40);
  const V1Tuning t = r9_tuning();
  CHECK(t.ff_directional && t.kp_percent_per_100 == 30 && fabs(t.vel_kd_permille_per_count_s - 0.05) < 1e-12);
}

// u_d の符号と上限（直接呼ぶ。誤差 0・I 0 なので出力 = FF + u_d）。上り 100 count/s の FF = 46 + 10 = 56。
inline void test_r9_damping_term_sign_and_clamp() {
  const V1Tuning t = r9_tuning();
  V1DutyState st;
  V1DutyTerms o;
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.0, true, false, true, 100.0) == 56 && o.d_permille == 0);  // 速度どおり
  CHECK(o.v_fast_valid && o.v_fast_count_s == 100 && o.mode == V1DutyMode::MOVE);
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.0, true, false, true, 50.0) == 59 && o.d_permille == 3);    // 遅い: +2.5 → 58.5 → 59
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.0, true, false, true, 150.0) == 54 && o.d_permille == -3);  // 速い: −2.5 → 53.5 → 54
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.0, true, false, true, 75.0) == 57);   // 量子化 25 count/s ぶん = 1.25 permille
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.0, true, false, true, -200.0) == 66 && o.d_permille == 10);  // +15 → 上限 +10
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.0, true, false, true, 400.0) == 46 && o.d_permille == -10);  // −15 → 下限 −10
  // 下り: FF = −7 − 10.7 = −17.7。v_fast −50（遅い）→ u_d = 0.05 × (−100 + 50) = −2.5 → −20.2 → −20
  CHECK(v1_duty_command(0, -100.0, st, t, 0, &o, 0.0, true, false, true, -50.0) == -20 && o.d_permille == -3);
  CHECK(v1_duty_command(0, -100.0, st, t, 0, &o, 0.0, true, false, true, -150.0) == -15);  // 速すぎ: +2.5 → −15.2 → −15
  // 静止目標の補正中（保持の外）も効く: 軌道 0 で +100 count/s で動いていれば −5（P の行き過ぎを抑える向き）
  CHECK(v1_duty_command(0, 0.0, st, t, 0, &o, 0.0, false, false, true, 100.0) == -5 && o.d_permille == -5);
  CHECK(v1_duty_command(10, 0.0, st, t, 0, &o, 0.0, false, false, true, 100.0) == 25);  // P 30 − 5
  // 推定なし（has_v_fast = false）・kd 0・従来の経路では u_d は出ない
  CHECK(v1_duty_command(0, 100.0, st, t, 0, &o, 0.0, true, false, false, -200.0) == 56 && o.d_permille == 0 && !o.v_fast_valid);
  V1Tuning t0 = t;
  t0.vel_kd_permille_per_count_s = 0.0;
  CHECK(v1_duty_command(0, 100.0, st, t0, 0, &o, 0.0, true, false, true, -200.0) == 56 && o.d_permille == 0);
  V1Tuning legacy(false);
  legacy.vel_kd_permille_per_count_s = 0.05;
  V1DutyState sl1, sl2;
  CHECK(v1_duty_command(25, 300.0, sl1, legacy, 0, &o, 0.0, true, false, true, -500.0) ==
        v1_duty_command(25, 300.0, sl2, legacy, 0, nullptr, 0.0, true, false, false, 0.0));
  CHECK(o.d_permille == 0);
  // 上限は調整値で変えられる
  V1Tuning t5 = t;
  t5.vel_d_max_permille = 5;
  CHECK(v1_duty_command(0, 100.0, st, t5, 0, &o, 0.0, true, false, true, -200.0) == 61 && o.d_permille == 5);
}

// 保持（HOLD）の中では u_d を足さない: 出力 0 のまま（v_fast がいくら大きくても）。出れば次の tick から効く。
inline void test_r9_damping_zero_in_hold() {
  const V1Tuning t = r9_tuning();
  V1DutyState st;
  V1DutyTerms o;
  CHECK(v1_duty_command(2, 0.0, st, t, 0, &o, 0.005, true, false, true, 300.0) == 0 && o.mode == V1DutyMode::HOLD);
  CHECK(o.d_permille == 0 && o.in_band && st.i_pm == 0.0);
  CHECK(v1_duty_command(4, 0.0, st, t, 0, &o, 0.005, false, false, true, -300.0) == 0 && o.d_permille == 0 && o.in_band);
  CHECK(o.v_fast_valid && o.v_fast_count_s == -300);  // 計器として v_fast は見える
  CHECK(v1_duty_command(5, 0.0, st, t, 0, &o, 0.005, false, false, true, -300.0) == 25 && o.mode == V1DutyMode::MOVE);  // P 15 + 10
}

// 飽和とアンチワインドアップは u_d 込みの合成で判定する。動き出しパルスの振る舞いは変えない。
inline void test_r9_damping_saturation_and_breakaway_interplay() {
  const V1Tuning t = r9_tuning();  // kp 30（誤差 1 = 3 permille）、duty_max 20
  V1DutyTerms o;
  // u_d が飽和から引き戻す: 誤差 50・上り 100 → P 150 + FF 56 = 206（u_d なしなら飽和で I を積まない）
  V1DutyState sa;
  for (int i = 0; i < 200; ++i) v1_duty_command(50, 100.0, sa, t, 0, &o, 0.005, true, false, true, 100.0);
  CHECK(o.cmd_permille == 200 && sa.i_pm == 0.0);
  V1DutyState sb;  // v_fast 300（速すぎ）→ u_d −10 → 196: 飽和しないので I が積まれ、上限 200 で頭打ち
  for (int i = 0; i < 200; ++i) v1_duty_command(50, 100.0, sb, t, 0, &o, 0.005, true, false, true, 300.0);
  CHECK(o.d_permille == -10 && sb.i_pm > 0.0 && o.cmd_permille == 200);
  // u_d が飽和へ押し込む: 誤差 45 → 135 + 56 = 191（飽和しない）→ v_fast −100（遅い）で u_d +10 → 201 → 200 で飽和、I は積まない
  V1DutyState sc;
  CHECK(v1_duty_command(45, 100.0, sc, t, 0, &o, 0.005, true, false, true, -100.0) == 200 && o.d_permille == 10);
  CHECK(sc.i_pm == 0.0);
  V1DutyState sd;  // u_d なしなら 191 で I が積まれる
  CHECK(v1_duty_command(45, 100.0, sd, t, 0, &o, 0.005, true, false, true, 100.0) == 191 && sd.i_pm > 0.0);
  // 下向きも同じ: 誤差 −60 → −180 − 17.7 = −197.7、v_fast 0（遅い）→ u_d −5 → −202.7 → −200
  V1DutyState se;
  CHECK(v1_duty_command(-60, -100.0, se, t, 0, &o, 0.005, true, false, true, 0.0) == -200 && se.i_pm == 0.0);
  // 動き出しパルスは従来どおり: 誤差の向きに max(|合成|, 90 / 60)
  V1DutyState sf;
  CHECK(v1_duty_command(5, 0.0, sf, t, 0, &o, 0.005, true, true, true, -100.0) == 90 && o.mode == V1DutyMode::BRK);  // 15 + 5 < 90
  CHECK(v1_duty_command(-5, 0.0, sf, t, 0, &o, 0.005, true, true, true, 100.0) == -60 && o.mode == V1DutyMode::BRK);
  CHECK(sf.i_pm == 0.0);  // パルス中は凍結
  CHECK(v1_duty_command(40, 0.0, sf, t, 0, &o, 0.0, true, true, true, -300.0) == 130);  // 合成 120 + 10 = 130 > 90 → 合成
}

// 軸を通した v_fast: 位置制御中（HOLDING）に生カウントを手で動かして推定値を見る。
inline void r9_holding(Harness& h) {
  h.ax.tuning = r9_tuning(2);
  h.ax.tuning.overrun_ms = 1000;  // 時間の飛びのテストで CONTROL_OVERRUN にしない
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;  // プラントはテストが手で動かす
}
// 1 tick 進める（毎 tick SET_POSITION で lease を保つ。目標は動かさない）
inline void r9_tick(Harness& h, int64_t draw, uint32_t dt_ms = 5) {
  h.set_position(h.ax.target_count());
  h.raw += draw;
  h.step(dt_ms);
}

inline void test_r9_v_fast_constant_speed_and_quantization() {
  // 一定速度: 5 ms に 1 count = 200 count/s → 40 ms 窓で 8 count / 0.040 s = 200 ちょうど
  Harness h;
  r9_holding(h);
  for (int i = 0; i < 20; ++i) r9_tick(h, 1);
  CHECK(h.ax.v_fast_valid() && fabs(h.ax.v_fast_count_s() - 200.0) < 1e-9);
  CHECK(h.ax.duty_terms().v_fast_valid && h.ax.duty_terms().v_fast_count_s == 200);
  // 下向き 5 ms に 2 count = −400
  for (int i = 0; i < 10; ++i) r9_tick(h, -2);
  CHECK(fabs(h.ax.v_fast_count_s() + 400.0) < 1e-9);
  // 速度が変わったら 40 ms で追いつく（100 ms 窓より速い）: 止めて 35 ms ではまだ動きが残り、40 ms で 0
  for (int i = 0; i < 7; ++i) r9_tick(h, 0);
  CHECK(h.ax.v_fast_count_s() < 0.0);
  r9_tick(h, 0);
  CHECK(h.ax.v_fast_valid() && h.ax.v_fast_count_s() == 0.0);
  // 量子化: 15 ms に 1 count（66.7 count/s）→ 40 ms 窓には 2 か 3 count → 50 か 75 だけ。平均は 66.7 に近い
  Harness g;
  r9_holding(g);
  double sum = 0.0;
  int n = 0;
  bool only_steps = true;
  for (int i = 0; i < 300; ++i) {  // 1.5 s（目標から離れ続けるので 2 s で静止の監視が ENCODER_FAULT にする。その前まで）
    r9_tick(g, (i % 3) == 0 ? 1 : 0);
    if (i >= 30) {
      const double v = g.ax.v_fast_count_s();
      if (fabs(v - 50.0) > 1e-9 && fabs(v - 75.0) > 1e-9) only_steps = false;
      sum += v;
      ++n;
    }
  }
  CHECK(only_steps && g.ax.v_fast_valid() && g.ax.state() == st::HOLDING);
  CHECK(fabs(sum / n - 200.0 / 3.0) < 1.0);
}

inline void test_r9_v_fast_gap_startup_and_reset() {
  Harness h;
  r9_holding(h);
  for (int i = 0; i < 20; ++i) r9_tick(h, 1);
  // 時間の飛び 60 ms（≤ 2 × 窓）: 実際の経過で割る。60 ms に 12 count → 200 count/s
  r9_tick(h, 12, 60);
  CHECK(h.ax.state() == st::HOLDING && h.ax.v_fast_valid() && fabs(h.ax.v_fast_count_s() - 200.0) < 1e-9);
  // 飛び 100 ms（> 2 × 窓）: 古いサンプルを捨てる → 推定なし（減衰項 0）。20 ms 分たまれば最も古いサンプルで割って戻る
  r9_tick(h, 20, 100);
  CHECK(h.ax.state() == st::HOLDING && !h.ax.v_fast_valid() && h.ax.v_fast_count_s() == 0.0);
  CHECK(h.ax.duty_terms().d_permille == 0 && !h.ax.duty_terms().v_fast_valid);
  for (int i = 0; i < 3; ++i) r9_tick(h, 1);  // 15 ms
  CHECK(!h.ax.v_fast_valid());
  r9_tick(h, 1);  // 20 ms: 4 count / 0.020 s
  CHECK(h.ax.v_fast_valid() && fabs(h.ax.v_fast_count_s() - 200.0) < 1e-9);
  for (int i = 0; i < 10; ++i) r9_tick(h, 1);
  CHECK(fabs(h.ax.v_fast_count_s() - 200.0) < 1e-9);
  // FAULT（積分と同じ契機）で捨てる。FAULT の間も推定なし
  Harness::accepted(h.host_fault());
  CHECK(h.ax.state() == st::FAULT && !h.ax.v_fast_valid() && h.ax.duty_terms().d_permille == 0);
  h.raw += 5;
  h.step();
  CHECK(!h.ax.v_fast_valid());
  // 位置制御の開始（ARMING のブレーキ解除）から: 最初の tick は推定なし、20 ms で戻る
  Harness g;
  g.ax.tuning = r9_tuning(2);
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(g.config_with(c));
  Harness::accepted(g.arm());
  g.stall = true;
  int ticks_on = 0;
  bool first_invalid = false;
  int valid_at = -1;
  for (int i = 0; i < 200 && valid_at < 0; ++i) {
    g.set_position(g.ax.target_count());
    g.raw += 1;
    g.step();
    CHECK(g.ax.brake_release_cmd() || !g.ax.v_fast_valid());  // 位置制御の外では推定しない
    if (g.ax.brake_release_cmd()) {
      if (ticks_on == 0) first_invalid = !g.ax.v_fast_valid();
      if (g.ax.v_fast_valid()) valid_at = ticks_on;
      ++ticks_on;
    }
  }
  CHECK(first_invalid && valid_at == 4);  // 5 サンプル目（20 ms 前のサンプルがある）
  CHECK(fabs(g.ax.v_fast_count_s() - 200.0) < 1e-9);
}

// 軸を通して: 保持中は v_fast があっても出力 0。保護（FOLLOWING_ERROR・走行中の無パルス・静止 2 s の監視）は r8 と同じ判定で出る。
inline void test_r9_axis_hold_zero_and_protections_unchanged() {
  // 保持中に 1 count のちらつき（v_fast ±25）が来ても出力 0
  Harness h;
  r9_holding(h);
  h.ax.tuning.overrun_ms = V1_OVERRUN_MS;
  CHECK(h.ax.duty_terms().mode == V1DutyMode::HOLD);
  for (int i = 0; i < 40; ++i) {
    r9_tick(h, (i % 2) == 0 ? 1 : -1);
    CHECK(h.ax.duty_permille() == 0 && h.ax.duty_terms().d_permille == 0 && h.ax.duty_terms().mode == V1DutyMode::HOLD);
  }
  // 走行中に止まった軸（v_fast 0、軌道 600）: u_d +10 が足される。FOLLOWING_ERROR 400 は r8 と同じ時刻
  auto stuck_run = [](double kd, uint64_t* fault_after_ms, uint16_t* code, int* d_seen) {
    Harness g;
    g.ax.tuning = r6_tuning(2, 40);
    g.ax.tuning.vel_kd_permille_per_count_s = kd;
    v1::Config c = base_config();
    c.following_error_limit_count = 400;
    Harness::accepted(g.config_with(c));
    g.reach_holding_from_configured();
    g.set_position(g.ax.actual_count());
    Harness::accepted(g.start());
    g.stall = true;
    g.pos_max_vel = 600;
    const int32_t s0 = g.ax.actual_count();
    const uint64_t t0 = g.now;
    *d_seen = 0;
    for (int i = 0; i < 150 && g.ax.state() != st::FAULT; ++i) {
      g.drive(10, s0 + 5000);
      if (g.ax.duty_terms().d_permille > *d_seen) *d_seen = g.ax.duty_terms().d_permille;
    }
    *fault_after_ms = (g.now - t0) / MS;
    *code = g.ax.fault_code();
    CHECK(g.ax.duty_permille() == 0 && !g.ax.v_fast_valid());
  };
  uint64_t t_r8 = 0, t_r9 = 0;
  uint16_t c_r8 = 0, c_r9 = 0;
  int d_r8 = 0, d_r9 = 0;
  stuck_run(0.0, &t_r8, &c_r8, &d_r8);
  stuck_run(V1_VEL_KD_PERMILLE_PER_COUNT_S, &t_r9, &c_r9, &d_r9);
  CHECK(c_r8 == fc::FOLLOWING_ERROR && c_r9 == fc::FOLLOWING_ERROR && t_r8 == t_r9 && t_r9 <= 1000);
  CHECK(d_r8 == 0 && d_r9 == 10);
  // 走行中の無パルス（kp 6 / duty_max 20 → しきい値 15 %）: r9 は u_d +10 のぶん早くしきい値に届くことがあるが、判定の窓 V1_STALL_MS は同じ
  auto stuck_enc = [](double kd, uint64_t* fault_after_ms, uint16_t* code) {
    Harness g;
    g.ax.tuning = r6_tuning(6);
    g.ax.tuning.vel_kd_permille_per_count_s = kd;
    v1::Config c = base_config();
    c.following_error_limit_count = 100000;
    Harness::accepted(g.config_with(c));
    g.reach_holding_from_configured();
    g.set_position(g.ax.actual_count());
    Harness::accepted(g.start());
    g.stall = true;
    g.pos_max_vel = 600;
    const int32_t s0 = g.ax.actual_count();
    const uint64_t t0 = g.now;
    for (int i = 0; i < 160 && g.ax.state() != st::FAULT; ++i) g.drive(10, s0 + 5000);
    *fault_after_ms = (g.now - t0) / MS;
    *code = g.ax.fault_code();
  };
  stuck_enc(0.0, &t_r8, &c_r8);
  stuck_enc(V1_VEL_KD_PERMILLE_PER_COUNT_S, &t_r9, &c_r9);
  CHECK(c_r8 == fc::ENCODER_FAULT && c_r9 == fc::ENCODER_FAULT && t_r9 <= t_r8 && t_r9 + 100 >= t_r8 && t_r8 <= 1200);
  // 静止 2 s の監視: 静止目標で誤差 10 のまま進まない → 2 s で ENCODER_FAULT（v_fast 0・軌道 0 なので u_d 0）
  Harness k;
  r9_holding(k);
  k.ax.tuning.overrun_ms = V1_OVERRUN_MS;
  k.raw -= 10;
  k.step();
  const uint64_t t0 = k.now;
  k.drive(1900, k.ax.target_count());
  CHECK(k.ax.state() == st::HOLDING);
  k.drive(150, k.ax.target_count());
  CHECK(k.ax.state() == st::FAULT && k.ax.fault_code() == fc::ENCODER_FAULT && k.now - t0 <= 2100 * MS);
}

// 遅れ・慣性のあるプラントで、減衰項が遅い区間の速度の波を減らす（決定的なシミュレーション）。
// プラント: 出力（pwm_permille、カウント増が正）は 3 tick（15 ms）遅れて効き、定常速度は FF の逆（上り v = (u − a(x)) / 0.10、
// 下り v = (u + 7) / 0.107、その間は 0）。a(x) = 46 + cog × sin(2π x / period) は位置で変わる摩擦（30 count 周期なら 100 count/s で
// 3.3 Hz の外乱。cog 5 permille ≈ ±50 count/s で、実機の ±60 count/s の波と同じ桁）。速度は時定数 tau の一次遅れ。位置は小数で持ち、
// ファームには切り捨てた整数カウントを渡す。kp 30、軌道 100 count/s の上り。P ループの共振（kp 30・遅れ 15 ms・tau 50 ms で約 3 Hz）を
// 外乱が揺らすので、減衰が足りないと速度の波が大きくなる。実機の再現ではなく「減衰項が共振の山を下げる」ことの確認。
struct R9PlantResult {
  double vel_err_rms = 0.0;   // プラントの真の速度 − 軌道速度
  double vfw_err_rms = 0.0;   // ファームの 100 ms 窓の速度 − 軌道速度（実機の指標 v_fw に相当）
  double fe_rms = 0.0;        // 追従誤差
  int d_max_abs = 0;          // |u_d| の最大
  int reversals = 0;          // 真の速度の符号反転（上り一定の軌道なので 0 のはず）
  bool active = false;
};
inline R9PlantResult r9_plant_run(double kd, double cog, double period, double tau_s) {
  Harness h;
  h.ax.tuning = r6_tuning(30);
  h.ax.tuning.vel_kd_permille_per_count_s = kd;
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(h.ax.actual_count());
  Harness::accepted(h.start());
  h.stall = true;
  h.pos_max_vel = 100;
  const double pi = 3.14159265358979323846;
  const double dt = 0.005, tau = tau_s;
  double pos = static_cast<double>(h.raw);
  double v = 0.0;
  int16_t delay[3] = {0, 0, 0};
  const int32_t s0 = h.ax.actual_count();
  R9PlantResult r;
  double se = 0.0, sfw = 0.0, sfe = 0.0;
  int n = 0;
  int prev_sign = 0;
  for (int i = 0; i < 800; ++i) {  // 4 s
    if ((i % 2) == 0) h.set_position(s0 + 2000);
    const int16_t u_now = h.ax.pwm_permille();
    const double u = delay[0];
    delay[0] = delay[1];
    delay[1] = delay[2];
    delay[2] = u_now;
    const double a_up = 46.0 + cog * sin(2.0 * pi * pos / period);
    double v_ss = 0.0;
    if (u > a_up) {
      v_ss = (u - a_up) / 0.10;
    } else if (u < -7.0) {
      v_ss = (u + 7.0) / 0.107;
    }
    v += (v_ss - v) * dt / tau;
    pos += v * dt;
    h.raw = static_cast<int64_t>(floor(pos));
    h.step(5);
    if (i >= 200) {  // 1 s 以降（加速と初期の過渡を除く）
      const double sv = h.ax.duty_terms().vel_count_s;
      se += (v - sv) * (v - sv);
      const double vfw = static_cast<double>(h.ax.velocity_count_s());
      sfw += (vfw - sv) * (vfw - sv);
      const double fe = static_cast<double>(h.ax.following_error_count());
      sfe += fe * fe;
      ++n;
      const int d = h.ax.duty_terms().d_permille;
      if ((d < 0 ? -d : d) > r.d_max_abs) r.d_max_abs = d < 0 ? -d : d;
      const int sg = v > 1.0 ? 1 : (v < -1.0 ? -1 : 0);
      if (sg != 0) {
        if (prev_sign != 0 && sg != prev_sign) ++r.reversals;
        prev_sign = sg;
      }
    }
  }
  r.vel_err_rms = sqrt(se / n);
  r.vfw_err_rms = sqrt(sfw / n);
  r.fe_rms = sqrt(sfe / n);
  r.active = h.ax.state() == st::ACTIVE && h.ax.duty_terms().vel_count_s == 100;
  return r;
}

inline void test_r9_damping_reduces_velocity_ripple_in_delayed_plant() {
  // 2 つの条件（摩擦の揺れ ±5 permille・30 count 周期・時定数 50 ms ／ ±8 permille・40 count 周期・時定数 80 ms）で比べる
  struct Case { double cog, period, tau; };
  for (const Case& k : {Case{5.0, 30.0, 0.050}, Case{8.0, 40.0, 0.080}}) {
    const R9PlantResult r8 = r9_plant_run(0.0, k.cog, k.period, k.tau);
    const R9PlantResult r9 = r9_plant_run(V1_VEL_KD_PERMILLE_PER_COUNT_S, k.cog, k.period, k.tau);
    if (std::getenv("R9_PLANT_PRINT")) {
      std::printf("r9 plant (cog %.0f / %.0f count / tau %.0f ms): r8 vel_err %.1f vfw_err %.1f fe %.2f rev %d / "
                  "r9 vel_err %.1f vfw_err %.1f fe %.2f rev %d d_max %d\n",
                  k.cog, k.period, k.tau * 1000.0, r8.vel_err_rms, r8.vfw_err_rms, r8.fe_rms, r8.reversals, r9.vel_err_rms,
                  r9.vfw_err_rms, r9.fe_rms, r9.reversals, r9.d_max_abs);
    }
    CHECK(r8.active && r9.active);
    CHECK(r8.vel_err_rms > 30.0);                       // 減衰なしでは波が立つ（テストの前提）
    CHECK(r9.vel_err_rms < 0.75 * r8.vel_err_rms);      // 25 % 以上減る（Issue #35 の合格基準と同じ比）
    CHECK(r9.vfw_err_rms < 0.75 * r8.vfw_err_rms);      // ファームの 100 ms 窓の速度（v_fw）で見ても
    CHECK(r9.fe_rms <= r8.fe_rms * 1.10);               // 追従誤差は悪くしない
    CHECK(r8.reversals == 0 && r9.reversals == 0);      // 軌道にない往復を増やさない
    CHECK(r9.d_max_abs > 0 && r9.d_max_abs <= 10);
  }
}

// ---- Issue #45: 再生バッファ（playout）の参照 -------------------------------------------------------------
// 実機の受信を模擬する: サーバーの planner が frame_ms ごとに整数の目標を作り（planner.rs と同じく丸める）、USB で遅れて届く
//（基本 5 ms ＋ 決定的な ±5 ms のぶれ。任意で 100 ms ごとに 40 ms 途切れ、途切れの間のフレームは明けにまとめて届く）。
// 制御タスク（control_task.cpp）と同じく、5 ms の tick の直前に、その時刻までに届いたフレームを受信時刻つきで渡す
//（handle(m, tick の時刻, 受信時刻)）。フレームは運用値 max_vel 600 / max_acc 2000、valid_for 100 ms（bringup.toml）。
// 見るもの: 参照速度 setpoint_vel（FF・r9 の減衰・保持の判定が使う値そのもの）と FF 項。

struct PlayoutScenario {
  double vel = 50.0;            // 目標の軌道の速度 [count/s]（負なら下向き）
  uint32_t frame_ms = 10;       // フレームの間隔
  bool jitter = false;          // ±5 ms のぶれ
  bool blackout = false;        // 100 ms ごとに 40 ms 途切れてまとめて届く
  uint32_t drop_every = 0;      // N フレームごとに 1 つ落とす（seq は進む）
  uint32_t ramp_ms = 3000;      // 軌道が動く時間（その後は止まった目標を送り続ける）
  uint32_t hold_ms = 1000;
  uint32_t max_acc = 2000;
  uint32_t max_vel = 600;
  bool reverse_half = false;    // 前半 +vel、後半 −vel（反転）
  uint32_t delay_ms = V1_PLAYOUT_DELAY_MS;  // 0 = 従来の profile_step
  uint32_t warm_ms = 300;       // 定常区間の始まり（ランプの開始から。立ち上がりを除く）
  uint32_t pre_ms = 700;        // ランプの前に止まった目標を同じ間隔で送る時間（サーバーは HOLDING 中から一定周期で送り続ける）
  uint32_t frame_ms2 = 0;       // ≠ 0 なら switch_ms（ランプの開始から）以降のフレーム間隔（周期の変化。PR #49 レビュー P2a）
  uint32_t switch_ms = 0;
  uint32_t stop_stream_ms = 0;  // ≠ 0 ならランプの開始からこの時刻以降は送らない（途絶。P1a）
  uint16_t valid_for_ms = 100;
};

struct PlayoutResult {
  int snaps = 0;            // 定常区間で 速度 ≠ 0 → ちょうど 0 になった回数（人工的な停止）
  int zero_ticks = 0;       // 定常区間で速度がちょうど 0 の tick
  double max_dv = 0.0;      // 全区間の |Δv_ref| / tick の最大 [count/s]
  int ff_p99 = 0;           // 定常区間の |ΔFF| / tick の p99 [permille]
  int ff_max = 0;
  double mean_v = 0.0;      // 定常区間の平均の参照速度
  int sign_changes = 0;     // ランプの間（減速の前）に参照速度の向きが変わった回数（|v| ≤ 2 count/s は向きなし。0 を挟んでも 1 回）
  double overshoot = 0.0;   // ランプの後、最後の目標を最後の動きの向きへ越えた量の最大 [count]
  bool left_active = false; // 途中で ACTIVE を外れた（期限切れ・FAULT）
  bool final_zero = false;  // 最後に参照速度がちょうど 0
  int32_t final_target = 0; // 最後に送った目標
  int32_t final_ref = 0;    // 最後の参照位置（target_count）
  bool hold_mode = false;   // 最後に保持（HOLD）に入っている
  double ref_min = 0.0, ref_max = 0.0;  // 参照位置の最小・最大
  int32_t act_min = 0, act_max = 0;     // 実位置の最小・最大
  uint32_t reseeds = 0;     // 周期の変化で時刻の付け直しを取り直した回数
  int starved_ticks = 0;    // ACTIVE 中に「途切れ」（starved）と判定した tick（送信を打ち切る前まで）
  double max_vel_err = 0.0; // 定常区間の |参照速度 − 軌道の速度| の最大
  double loss_excursion = 0.0;  // 送信を打ち切った後、参照が最後に送った目標を動きの向きへ越えた量の最大 [count]
  int32_t last_sent = 0;    // 最後に送った目標
};

// サーバーの planner と同じく max_acc で加減速する軌道（1 ms 刻みで積分）。速度の指令は pre_ms まで 0、その後 ramp_ms の間 vel
//（reverse_half なら後半は −vel）、その後 0。位置はソフトリミットの内側に clamp して送る（サーバーと同じ）。
inline std::vector<double> playout_traj_table(const PlayoutScenario& s, uint32_t total_ms) {
  std::vector<double> x(total_ms + 1, 0.0);
  const v1::Config c = base_config();
  const double a = static_cast<double>(s.max_acc);
  double p = 0.0, v = 0.0;
  for (uint32_t t = 0; t <= total_ms; ++t) {
    double vc = 0.0;
    if (t >= s.pre_ms && t < s.pre_ms + s.ramp_ms) {
      vc = s.vel;
      if (s.reverse_half && t >= s.pre_ms + s.ramp_ms / 2) vc = -s.vel;
    }
    const double dv_max = a * 0.001;
    double dv = vc - v;
    if (dv > dv_max) dv = dv_max;
    if (dv < -dv_max) dv = -dv_max;
    v += dv;
    p += v * 0.001;
    if (p < c.soft_min_count) { p = c.soft_min_count; v = 0.0; }
    if (p > c.soft_max_count) { p = c.soft_max_count; v = 0.0; }
    x[t] = p;
  }
  return x;
}

inline PlayoutResult run_playout_scenario(const PlayoutScenario& s) {
  PlayoutResult r;
  Harness h;
  h.ax.tuning = V1Tuning(true);
  h.ax.tuning.vel_kd_permille_per_count_s = 0.0;  // FF の落ちをそのまま見る（r9 の減衰も同じ参照速度を使う）
  h.ax.tuning.playout_delay_ms = s.delay_ms;
  h.pos_max_vel = s.max_vel;
  h.reach_active();
  const uint64_t t0 = h.now;
  const uint32_t total_ms = s.pre_ms + s.ramp_ms + s.hold_ms;
  struct F {
    uint64_t del;
    v1::HostMsg m;
  };
  std::vector<F> frames;
  const std::vector<double> traj = playout_traj_table(s, total_ms);
  uint64_t prev_del = t0;
  uint32_t tg_ms = 0;
  for (uint32_t k = 0; tg_ms <= total_ms; ++k, tg_ms += (s.frame_ms2 && tg_ms >= s.pre_ms + s.switch_ms) ? s.frame_ms2 : s.frame_ms) {
    if (s.stop_stream_ms && tg_ms >= s.pre_ms + s.stop_stream_ms) break;
    const int32_t target = static_cast<int32_t>(llround(traj[tg_ms]));
    const v1::HostMsg m = h.set_position_msg(target, s.valid_for_ms, s.max_acc);  // 落とすフレームも seq は進める
    r.final_target = target;
    r.last_sent = target;
    if (s.drop_every && k % s.drop_every == s.drop_every - 1) continue;
    int64_t lat_ms = 5;
    if (s.jitter) lat_ms += static_cast<int64_t>((k * 37u) % 11u) - 5;  // 0〜10 ms
    uint64_t del = t0 + (static_cast<uint64_t>(tg_ms) + static_cast<uint64_t>(lat_ms)) * MS;
    if (s.blackout) {
      const uint64_t ph = (del - t0) % (100 * MS);
      if (ph >= 50 * MS && ph < 90 * MS) del += 90 * MS - ph;  // 途切れの間に届くはずだったものは明けにまとめて
    }
    if (del < prev_del) del = prev_del;  // USB は順序を保つ
    prev_del = del;
    frames.push_back(F{del, m});
  }
  size_t fi = 0;
  double v_prev = h.ax.setpoint_vel();
  int ff_prev = h.ax.duty_terms().ff_permille;
  int last_sign = 0;
  std::vector<int> dff;
  double sum_v = 0.0;
  int n_v = 0;
  r.ref_min = r.ref_max = h.ax.setpoint_pos();
  r.act_min = r.act_max = h.ax.actual_count();
  const uint64_t end = t0 + static_cast<uint64_t>(total_ms) * MS;
  while (h.now < end) {
    const uint64_t tick = h.now + 5 * MS;
    while (fi < frames.size() && frames[fi].del <= tick) {
      h.ax.handle(frames[fi].m, tick, frames[fi].del);
      h.drain();
      ++fi;
    }
    h.step(5);
    if (h.ax.state() != st::ACTIVE) r.left_active = true;
    const bool streaming = fi < frames.size();
    if (h.ax.state() == st::ACTIVE && streaming && h.ax.playout_fit().starved) ++r.starved_ticks;
    if (!streaming) {
      const double dir = s.vel > 0.0 ? 1.0 : -1.0;
      const double ex = dir * (h.ax.setpoint_pos() - static_cast<double>(r.last_sent));
      if (ex > r.loss_excursion) r.loss_excursion = ex;
    }
    const double v = h.ax.setpoint_vel();
    const int ff = h.ax.duty_terms().ff_permille;
    const double t_ms = static_cast<double>(h.now - t0) / 1000.0 - s.pre_ms;  // ランプの開始から
    const double dv = fabs(v - v_prev);
    if (dv > r.max_dv) r.max_dv = dv;
    const int sgn = v > 2.0 ? 1 : (v < -2.0 ? -1 : 0);
    if (sgn != 0 && t_ms <= s.ramp_ms) {
      if (last_sign != 0 && sgn != last_sign) ++r.sign_changes;
      last_sign = sgn;
    }
    if (t_ms > s.ramp_ms) {
      const double dir = (s.reverse_half ? -s.vel : s.vel) > 0.0 ? 1.0 : -1.0;
      const double os = dir * (h.ax.setpoint_pos() - static_cast<double>(r.final_target));
      if (os > r.overshoot) r.overshoot = os;
    }
    if (h.ax.setpoint_pos() < r.ref_min) r.ref_min = h.ax.setpoint_pos();
    if (h.ax.setpoint_pos() > r.ref_max) r.ref_max = h.ax.setpoint_pos();
    if (h.ax.actual_count() < r.act_min) r.act_min = h.ax.actual_count();
    if (h.ax.actual_count() > r.act_max) r.act_max = h.ax.actual_count();
    bool steady = t_ms >= s.warm_ms && t_ms <= s.ramp_ms;  // ランプの終わりの減速は遅れ（40 ms）の後なので入らない
    if (s.reverse_half && fabs(t_ms - s.ramp_ms / 2.0 - s.delay_ms) < 250.0) steady = false;  // 反転の前後は除く
    if (steady) {
      if (v == 0.0) {
        ++r.zero_ticks;
        if (v_prev != 0.0) ++r.snaps;
      }
      const int d = ff - ff_prev;
      dff.push_back(d < 0 ? -d : d);
      sum_v += v;
      ++n_v;
      const double ve = fabs(v - s.vel);
      if (ve > r.max_vel_err) r.max_vel_err = ve;
    }
    v_prev = v;
    ff_prev = ff;
  }
  if (!dff.empty()) {
    std::vector<int> sorted = dff;
    std::sort(sorted.begin(), sorted.end());
    r.ff_p99 = sorted[static_cast<size_t>(0.99 * static_cast<double>(sorted.size() - 1))];
    r.ff_max = sorted.back();
  }
  r.mean_v = n_v ? sum_v / n_v : 0.0;
  r.final_zero = h.ax.setpoint_vel() == 0.0;
  r.final_ref = h.ax.target_count();
  r.hold_mode = h.ax.duty_terms().mode == V1DutyMode::HOLD;
  r.reseeds = h.ax.playout_reseeds();
  if (getenv("V1_PLAYOUT_REPORT")) {
    std::printf("playout delay=%u vel=%+.0f frame=%u jit=%d bo=%d drop=%u rev=%d: snaps=%d zero=%d max_dv=%.2f "
                "ff_p99=%d ff_max=%d mean_v=%.1f sign=%d os=%.2f rs=%u starve=%d verr=%.1f loss=%.2f left=%d final(v0=%d ref=%d tgt=%d hold=%d) ref[%.1f,%.1f]\n",
                s.delay_ms, s.vel, s.frame_ms, s.jitter, s.blackout, s.drop_every, s.reverse_half, r.snaps,
                r.zero_ticks, r.max_dv, r.ff_p99, r.ff_max, r.mean_v, r.sign_changes, r.overshoot, r.reseeds, r.starved_ticks, r.max_vel_err, r.loss_excursion, r.left_active, r.final_zero,
                r.final_ref, r.final_target, r.hold_mode, r.ref_min, r.ref_max);
  }
  return r;
}

// 従来の profile_step（playout_delay_ms = 0）が Issue #45 の欠陥を再現すること（この試験は欠陥の記録。従来の経路を残す間は通る）:
// ゆっくりの定常ランプで参照速度が加速度制限を迂回して 0 に落ち、FF が 1 tick で数十 permille 抜ける。
inline void test_playout_legacy_profile_reproduces_issue45_snaps() {
  for (double vel : {25.0, 50.0, -50.0}) {
    PlayoutScenario s;
    s.vel = vel;
    s.jitter = true;
    s.blackout = true;
    s.delay_ms = 0;
    const PlayoutResult r = run_playout_scenario(s);
    CHECK(!r.left_active);
    CHECK(r.snaps >= 10);           // 2.7 s の定常区間で人工的な停止が何度も
    CHECK(r.max_dv > 10.0 + 1e-6);  // 加速度制限（2000 × 5 ms = 10 count/s）を迂回した 0 への落ち
    if (vel > 0) CHECK(r.ff_p99 >= 30);  // 上向きは切片 4.6 % がテーパごと抜ける
  }
}

// 再生バッファ（既定）: ±25 / 50 / 100 count/s、10 / 20 ms のフレーム、ぶれ・途切れの有無で、定常区間に人工的な停止が無く、
// |Δv_ref| ≤ max_acc × 5 ms（10 count/s）、平均速度は軌道どおり、最後は止まった目標にちょうど止まって保持に入る。
inline void test_playout_ramps_have_no_artificial_stops() {
  for (double vel : {25.0, -25.0, 50.0, -50.0, 100.0, -100.0}) {
    for (uint32_t frame_ms : {10u, 20u}) {
      for (int noisy = 0; noisy < 2; ++noisy) {
        PlayoutScenario s;
        s.vel = vel;
        s.frame_ms = frame_ms;
        s.jitter = noisy != 0;
        s.blackout = noisy != 0;
        const PlayoutResult r = run_playout_scenario(s);
        CHECK(!r.left_active);
        CHECK(r.snaps == 0);
        CHECK(r.zero_ticks == 0);
        CHECK(r.max_dv <= 10.0 + 1e-3);
        CHECK(fabs(r.mean_v - vel) <= 0.1 * fabs(vel));
        CHECK(r.sign_changes == 0);
        // 止まるときの行き過ぎ: 途切れが無ければ 1 count 未満。止まる所に 40 ms の途切れが重なると、届いていない区間を直線で外挿する
        // ので 100 count/s で約 2.6 count 越えて戻る（遅れ 40 ms と途切れ 40 ms が同じなので。PR に残差として記載）
        CHECK(r.overshoot <= (noisy ? 3.0 : 1.0));
        CHECK(r.ff_p99 <= 10);
        CHECK(r.final_zero);
        CHECK(r.final_ref == r.final_target);
        CHECK(r.hold_mode);
      }
    }
  }
}

// 反転（+v → −v）: 向きは 1 回だけ変わり、|Δv_ref| は加速度制限の中、反転の前後以外で止まらない
inline void test_playout_reversal_is_smooth() {
  for (double vel : {50.0, 100.0, -50.0}) {
    PlayoutScenario s;
    s.vel = vel;
    s.reverse_half = true;
    s.jitter = true;
    s.blackout = true;
    const PlayoutResult r = run_playout_scenario(s);
    CHECK(!r.left_active);
    CHECK(r.sign_changes == 1);
    CHECK(r.overshoot <= 3.0);
    CHECK(r.snaps == 0);
    CHECK(r.max_dv <= 10.0 + 1e-3);
    CHECK(r.final_zero);
    CHECK(r.final_ref == r.final_target);
  }
}

// フレームの欠落（3 つに 1 つ落ちる。seq は進む）でも止まらない
inline void test_playout_dropped_frames() {
  for (double vel : {25.0, 50.0, -100.0}) {
    PlayoutScenario s;
    s.vel = vel;
    s.jitter = true;
    s.drop_every = 3;
    const PlayoutResult r = run_playout_scenario(s);
    CHECK(!r.left_active);
    CHECK(r.snaps == 0);
    CHECK(r.max_dv <= 10.0 + 1e-3);
    CHECK(fabs(r.mean_v - vel) <= 0.1 * fabs(vel));
    CHECK(r.final_zero);
    CHECK(r.final_ref == r.final_target);
  }
}

// Issue #46: 全速（600 count/s、max_acc 2000）でソフトリミットに向かっても、参照は端の reserve（100）手前で行き過ぎずに止まり、
// SOFT_LIMIT にならない（目標はサーバーと同じくソフトリミットの値に clamp されて届く）
// フレームの max_acc が a_brake より小さい（500）ときも、その減速度で reserve の手前に止まる
inline void test_playout_soft_limit_approach_stops_inside_reserve() {
  for (int k = 0; k < 4; ++k) {
    const double vel = (k % 2 == 0) ? 600.0 : -600.0;
    PlayoutScenario s;
    s.vel = vel;
    s.max_acc = k < 2 ? 2000 : 500;
    s.ramp_ms = 5500;  // 0 → ±2700 に 4.5 s
    s.hold_ms = 1000;
    s.jitter = true;
    const PlayoutResult r = run_playout_scenario(s);
    const v1::Config c = base_config();
    CHECK(!r.left_active);
    CHECK(r.max_dv <= s.max_acc * 0.005 + 1e-3);
    CHECK(r.final_zero);  // PR #49 レビュー P1c: 端の手前で止まった参照の速度はちょうど 0（保持・PR #48 の静止の監視が効く）
    if (s.max_acc == 2000) CHECK(r.hold_mode);
    if (vel > 0) {
      CHECK(r.ref_max <= c.soft_max_count - V1_SOFT_RESERVE_COUNT + 0.5);
      CHECK(r.final_ref >= c.soft_max_count - V1_SOFT_RESERVE_COUNT - 1);  // 手前すぎない
      CHECK(r.act_max <= c.soft_max_count - V1_SOFT_RESERVE_COUNT + 1);
    } else {
      CHECK(r.ref_min >= c.soft_min_count + V1_SOFT_RESERVE_COUNT - 0.5);
      CHECK(r.final_ref <= c.soft_min_count + V1_SOFT_RESERVE_COUNT + 1);
      CHECK(r.act_min >= c.soft_min_count + V1_SOFT_RESERVE_COUNT - 1);
    }
  }
}

// V1PlayoutClock / V1PlayoutBuffer 単体: まとめて届いたフレームの時刻を seq の直線（下側の包絡）に付け直す。
// 窓の目標がすべて同じなら傾きはちょうど 0.0・位置はその値。ランプは位置・速度とも一致。
inline void test_playout_clock_and_fit_unit() {
  V1PlayoutClock clk;
  V1PlayoutBuffer buf;
  // 10 ms 周期、5 ms 遅れ。seq 40〜43 は 40 ms 途切れて seq 44 と同時（t = 445 ms）に届く
  for (uint32_t k = 1; k <= 60; ++k) {
    uint64_t rx = (k * 10 + 5) * MS;
    if (k >= 40 && k <= 43) rx = 445 * MS;
    clk.push(k, rx);
    buf.push(k, rx, static_cast<int32_t>(2 * k));  // 200 count/s
  }
  CHECK(!clk.valid());  // 当てはめは refit_if_dirty() まで遅らせる（1 tick に 1 回）
  clk.refit_if_dirty();
  CHECK(clk.valid());
  CHECK(fabs(clk.period_us() - 10000.0) < 300.0);
  CHECK(llabs(static_cast<long long>(clk.time_of(41, 445 * MS)) - static_cast<long long>(415 * MS)) <= 2000);
  const V1PlayoutFit f = buf.fit(420 * MS, 80 * MS, 40 * MS, clk);
  CHECK(f.has && !f.starved);
  CHECK(fabs(f.vel - 200.0) < 10.0);
  CHECK(fabs(f.pos - 2.0 * 41.5) < 1.0);
  // 受信時刻のままだと（時刻の付け直しなし）まとめ届きで傾きが崩れる
  V1PlayoutClock none;
  const V1PlayoutFit g = buf.fit(420 * MS, 80 * MS, 40 * MS, none);
  CHECK(fabs(g.vel - 200.0) > 30.0);
  // 止まった目標: 傾きちょうど 0、位置はその値
  V1PlayoutBuffer flat;
  for (uint32_t k = 1; k <= 20; ++k) flat.push(k, (k * 10) * MS, 123);
  const V1PlayoutFit h = flat.fit(150 * MS, 80 * MS, 40 * MS, none);
  CHECK(h.has && h.vel == 0.0 && h.pos == 123.0);
  // tp がまだ最初のフレームより前 → has なし。最新より extrap を越えて先 → starved
  CHECK(!flat.fit(5 * MS, 80 * MS, 40 * MS, none).has);
  CHECK(flat.fit(260 * MS, 80 * MS, 40 * MS, none).starved);
  CHECK(!flat.fit(230 * MS, 80 * MS, 40 * MS, none).starved);
  // seq が戻る（epoch が変わった）・500 ms 以上空く → 取り直し
  clk.push(5, 1000 * MS);
  CHECK(!clk.valid());
}

// PR #49 レビュー P1a: フレームが途絶しても、参照は最後に受けた目標を V1_PLAYOUT_LOSS_MARGIN_COUNT（5）＋ 量子化以上越えない。
// 600 count/s のランプの途中で送信を打ち切る: 外挿 → 減速、期限切れ（valid_for 60 → STOPPING）・lease 切れ（valid_for 100 =
// command_timeout → COMMAND_TIMEOUT の減速停止）のどちらの経路でも。遅い軌道（50 count/s）も
inline void test_playout_stream_loss_excursion_is_bounded() {
  for (double vel : {600.0, -600.0, 50.0}) {
    for (uint16_t valid_for : {static_cast<uint16_t>(60), static_cast<uint16_t>(100)}) {
      PlayoutScenario s;
      s.vel = vel;
      s.ramp_ms = 3000;
      s.hold_ms = 1500;
      s.jitter = true;
      s.valid_for_ms = valid_for;
      s.stop_stream_ms = 1500;  // 600 count/s の定常（約 0.8 s 走った所）で切る
      const PlayoutResult r = run_playout_scenario(s);
      CHECK(r.left_active);  // 期限切れ・lease 切れで ACTIVE を出る
      CHECK(r.loss_excursion <= V1_PLAYOUT_LOSS_MARGIN_COUNT + 1.0);
      CHECK(r.max_dv <= 10.0 + 1e-3);
    }
  }
}

// PR #49 レビュー P1b: フレームの max_acc が 0（やごく小さい値）でも参照は止まれる（減速は V1_MIN_DECEL_COUNT_S2 を下限）。
// 動いている参照に max_acc 0 のフレームで止まった目標を送る → 速度はちょうど 0 に。途絶 → 止まる。STOPPING も止まり切る
inline void test_playout_zero_max_acc_still_decelerates() {
  // 止まった目標
  {
    Harness h;
    h.ax.tuning = V1Tuning(true);
    h.pos_max_vel = 600;
    h.reach_active();
    const int32_t from = h.ax.actual_count();
    for (int i = 1; i <= 60; ++i) {
      h.send(h.set_position_msg(from + i, 100, 2000));  // 100 count/s
      h.idle(10);
    }
    CHECK(h.ax.setpoint_vel() > 50.0);
    for (int i = 0; i < 100; ++i) {
      h.send(h.set_position_msg(from + 60, 100, 0));  // max_acc 0
      h.idle(10);
    }
    CHECK(h.ax.state() == st::ACTIVE && h.ax.setpoint_vel() == 0.0);
    // 減速度が 2000 → 500（下限）に下がった分だけ最後の目標を越えて止まる。max_acc 0 の間は加速できないのでそこに留まる
    CHECK(std::abs(h.ax.target_count() - (from + 60)) <= 12);
  }
  // max_acc 0 のまま途絶 → 外挿の後に減速、期限で STOPPING（CONFIG の max_acc に戻る）
  {
    Harness h;
    h.ax.tuning = V1Tuning(true);
    h.pos_max_vel = 600;
    h.reach_active();
    const int32_t from = h.ax.actual_count();
    for (int i = 1; i <= 60; ++i) {
      h.send(h.set_position_msg(from + i, 100, 2000));
      h.idle(10);
    }
    for (int i = 61; i <= 63; ++i) {
      h.send(h.set_position_msg(from + i, 60, 0));
      h.idle(10);
    }
    CHECK(h.run_until([&] { return h.ax.setpoint_vel() == 0.0; }, 1000));
    CHECK(h.ax.target_count() - (from + 63) <= 12);
  }
  // ごく小さい max_acc（1）で CONTROLLED_STOP → 下限 500 で止まり切る（STOP_TIMEOUT にならない）
  {
    Harness h;
    h.ax.tuning = V1Tuning(true);
    h.pos_max_vel = 600;
    h.reach_active();
    const int32_t from = h.ax.actual_count();
    for (int i = 1; i <= 60; ++i) {
      h.send(h.set_position_msg(from + i, 100, i < 58 ? 2000 : 1));
      h.idle(10);
    }
    CHECK(h.ax.setpoint_vel() > 50.0);
    Harness::accepted(h.stop());
    for (int i = 0; i < 40; ++i) {
      h.send(h.set_position_msg(from + 60, 100, 1));
      h.idle(10);
    }
    CHECK(h.ax.state() == st::HOLDING && h.ax.setpoint_vel() == 0.0);
  }
}

// PR #49 レビュー P2a: サーバーのフレーム周期が途中で変わっても（100 → 50 Hz、50 → 100 Hz）、届いている限り「途切れ」とは
// 判定せず、時刻の付け直しを取り直して速度の誤差は有界、人工的な停止なし
inline void test_playout_cadence_change_reseeds_clock() {
  for (int dir = 0; dir < 2; ++dir) {
    for (int noisy = 0; noisy < 2; ++noisy) {
      PlayoutScenario s;
      s.vel = 100.0;
      s.frame_ms = dir == 0 ? 10 : 20;
      s.frame_ms2 = dir == 0 ? 20 : 10;
      s.switch_ms = 1500;
      s.pre_ms = 700;
      s.jitter = noisy != 0;
      const PlayoutResult r = run_playout_scenario(s);
      CHECK(!r.left_active);
      CHECK(r.starved_ticks == 0);
      CHECK(r.snaps == 0 && r.zero_ticks == 0);
      CHECK(r.max_dv <= 10.0 + 1e-3);
      // 取り直すまでの間（100 → 50 Hz で約 80 ms）は新しいフレームの時刻が詰まって傾きが大きく出る: 100 count/s に対し
      // 最大約 57 count/s の誤差（加速度制限の中で戻る）。50 → 100 Hz は約 30。実運用では planner の周期を変えるには
      // サーバーの再起動 → CONFIG が要り、CONFIG で時刻の付け直しは取り直す（P2b）
      CHECK(r.max_vel_err <= 60.0);
      if (dir == 0) CHECK(r.reseeds >= 1);
      CHECK(r.final_zero && r.final_ref == r.final_target && r.hold_mode);
    }
  }
}

// 再生バッファでも期限・STOP・FAULT・epoch・再 START の扱いは従来どおり（バッファにフレームが残っていても鮮度は延びない）
inline void test_playout_expiry_stop_fault_epoch_unchanged() {
  auto make = [](Harness& h) {
    h.ax.tuning = V1Tuning(true);
    h.pos_max_vel = 600;
    CHECK(h.ax.tuning.playout_delay_ms == V1_PLAYOUT_DELAY_MS || V1_PLAYOUT == 0);
    h.ax.tuning.playout_delay_ms = 40;
    h.reach_active();
  };
  // ランプ（100 count/s、10 ms ごと、max_acc 2000、valid_for 60 ms。lease 100 ms より先に期限が来る）を n フレーム送る
  auto ramp = [](Harness& h, int n, int32_t from) {
    for (int i = 1; i <= n; ++i) {
      h.send(h.set_position_msg(from + i, 60, 2000));
      h.idle(10);
    }
  };
  // 期限: 送るのをやめると、最後に取り出した時刻 + valid_for（60 ms）を過ぎた tick で STOPPING、max_acc で減速して HOLDING。
  // 最後のフレームの受信時刻が 50 ms 前でも期限は取り出した時刻から（受信時刻は再生の時刻印だけ）
  {
    Harness h;
    make(h);
    const int32_t from = h.ax.actual_count();
    ramp(h, 50, from);
    CHECK(h.ax.state() == st::ACTIVE && h.ax.setpoint_vel() > 50.0);
    const uint64_t t_last = h.now;
    h.ax.handle(h.set_position_msg(from + 51, 60, 2000), t_last, t_last - 50 * MS);
    h.drain();
    CHECK(h.run_until([&] { return h.ax.state() != st::ACTIVE; }, 200));
    CHECK(h.now > t_last + 60 * MS && h.now <= t_last + 65 * MS);
    CHECK(h.ax.state() == st::STOPPING && h.ax.fault_code() == fc::NONE);
    double v_prev = h.ax.setpoint_vel();
    bool bounded = true;
    while (h.ax.state() == st::STOPPING && h.now < t_last + 1000 * MS) {
      h.send(h.set_position_msg(from + 100, 60, 2000));  // lease だけ保つ（STOPPING では目標に使わない）
      h.step();
      if (fabs(h.ax.setpoint_vel() - v_prev) > 10.0 + 1e-3) bounded = false;
      v_prev = h.ax.setpoint_vel();
    }
    CHECK(bounded);
    CHECK(h.ax.state() == st::HOLDING && h.ax.setpoint_vel() == 0.0);
    const int32_t held = h.ax.target_count();
    // 期限までは最新フレームの先を直線で外挿してよい（extrap 40 ms ≈ 4 count）ので、止まる所は最後の目標 + 外挿 + 減速の距離
    //（100² / (2 × 2000) = 2.5 count）以内
    CHECK(held >= from + 45 && held <= from + 51 + 4 + 3);
    h.drive(200, held);  // HOLDING ではバッファの残り（受けた先のランプ）を再生しない
    CHECK(h.ax.target_count() == held);
    // 再 START: 前の ACTIVE の残りは再生せず、新しいフレームの位置に留まる
    h.set_position(held);
    Harness::accepted(h.start());
    for (int i = 0; i < 30; ++i) {
      h.send(h.set_position_msg(held, 100, 2000));
      h.idle(10);
    }
    CHECK(h.ax.state() == st::ACTIVE && h.ax.target_count() == held && h.ax.setpoint_vel() == 0.0);
  }
  // CONTROLLED_STOP: ランプの途中で即 STOPPING、減速は加速度制限の中、HOLDING の後は動かない
  {
    Harness h;
    make(h);
    const int32_t from = h.ax.actual_count();
    ramp(h, 50, from);
    Harness::accepted(h.stop());
    CHECK(h.ax.state() == st::STOPPING);
    double v_prev = h.ax.setpoint_vel();
    bool bounded = true;
    for (int i = 0; i < 60; ++i) {
      h.send(h.set_position_msg(from + 51 + i, 100, 2000));  // 送り続けても STOPPING の減速は変わらない
      h.idle(5);
      if (fabs(h.ax.setpoint_vel() - v_prev) > 10.0 + 1e-3) bounded = false;
      v_prev = h.ax.setpoint_vel();
    }
    CHECK(bounded);
    CHECK(h.ax.state() == st::HOLDING && h.ax.setpoint_vel() == 0.0);
  }
  // HOST_FAULT: ランプの途中で即 FAULT、出力 0、参照の速度 0
  {
    Harness h;
    make(h);
    ramp(h, 30, h.ax.actual_count());
    Harness::accepted(h.host_fault());
    CHECK(h.ax.state() == st::FAULT && h.ax.duty_permille() == 0 && h.ax.setpoint_vel() == 0.0);
  }
  // epoch の違うフレームは stale として捨て、参照は動かない（バッファにも入らない）
  {
    Harness h;
    make(h);
    const int32_t here = h.ax.actual_count();
    const uint16_t stale0 = h.ax.stale_drop_count();
    for (int i = 0; i < 20; ++i) {
      v1::HostMsg m = h.set_position_msg(here + 500, 100, 2000);
      m.set_position.session_epoch = EPOCH + 1;
      h.send(m);
      h.send(h.set_position_msg(here, 100, 2000));  // lease は正しいフレームで保つ
      h.idle(10);
    }
    CHECK(h.ax.stale_drop_count() == stale0 + 20);
    CHECK(h.ax.state() == st::ACTIVE && h.ax.target_count() == here && h.ax.setpoint_vel() == 0.0);
  }
}

// ---- 端ガード（実測位置・速度） ----------------------------------------------
// 既存の追従プラントは参照に張り付くので、遅れと減速度の上限を持つプラントで端に寄せる。
// PWM は pwm_permille（正がカウント増）。出力は delay_ms 遅れて効く。外向き PWM は approach 速度まで加速し、
// 内向き PWM は a_plant で減速する。Issue #46 は切る（指定速度のまま端に入れるため）。

struct EgSim {
  Harness h;
  double x = 0;
  double v = 0;
  double a_plant = 6000;
  double v_cap = 1200;
  int dir = 1;
  std::vector<int> delay;
  int32_t lim_hi = 30000;
  int32_t lim_lo = -30000;
  bool guard_seen = false;
  bool stopping_guard = false;
  bool outward_pwm = false;  // ガード中に外向き PWM を出した
  int32_t outward_most = 0;
  uint16_t fault = fc::NONE;
};

inline void eg_prepare(EgSim& s, double speed, double a_plant, int delay_ms, int dir, int32_t lim = 0,
                       uint32_t command_timeout_ms = 5000, uint32_t stop_timeout_ms = 8000) {
  if (lim <= 0) {
    // 全速に達してから危険域に入る位置。20 s の試験で届くよう、端を必要最小限に置く
    const double accel = speed * speed / (2.0 * a_plant) + 300.0;
    const double d_req = speed * 0.08 + speed * speed / (2.0 * 4000.0) + 30.0;
    lim = static_cast<int32_t>(ceil(accel + speed * 0.8 + d_req + 500.0));
  }
  s.lim_hi = lim;
  s.lim_lo = -lim;
  v1::Config c = base_config();
  c.soft_min_count = s.lim_lo;
  c.soft_max_count = s.lim_hi;
  c.max_vel_count_s = 6000;
  c.max_acc_count_s2 = 40000;
  c.following_error_limit_count = 1000000;
  c.overspeed_count_s = 100000;
  c.command_timeout_ms = command_timeout_ms;
  c.stop_timeout_ms = stop_timeout_ms;
  Harness::accepted(s.h.config_with(c));
  s.h.reach_holding_from_configured();
  s.h.ax.tuning.kp_percent_per_100 = 200;
  s.h.ax.tuning.deadband = 0;
  s.h.ax.tuning.deadband_out = 0;
  s.h.ax.tuning.duty_min = 0;
  s.h.ax.tuning.duty_max = 100;
  s.h.ax.tuning.ki_permille_per_count_s = 0;
  s.h.ax.tuning.stall_boost_ms = 0;
  s.h.ax.tuning.assist_ms = 0;
  s.h.ax.tuning.stall_ms = 60000;
  s.h.ax.tuning.settled_ms = 0;
  s.h.ax.tuning.soft_brake_acc_count_s2 = 0;
  s.h.ax.tuning.playout_delay_ms = 0;
  s.h.pos_max_vel = static_cast<uint32_t>(speed);
  s.h.set_position(0, 200);
  Harness::accepted(s.h.start());
  s.a_plant = a_plant;
  s.v_cap = speed;
  s.dir = dir;
  s.x = 0;
  s.v = 0;
  s.outward_most = 0;
  s.delay.assign(static_cast<size_t>(delay_ms / 5), 0);
}

inline int32_t eg_target(const EgSim& s) { return s.dir > 0 ? s.lim_hi : s.lim_lo; }

inline void eg_step(EgSim& s, int32_t target, uint32_t max_acc = 20000) {
  const int applied = s.delay.empty() ? 0 : s.delay.front();
  if (!s.delay.empty()) s.delay.erase(s.delay.begin());
  const double dt = 0.005;
  // 正の PWM はカウント増。|PWM| がしきい値を超えたら ±a_plant で加減速（遅れの分だけ古い指令）。
  // 外向きの速さだけ approach 速度で頭打ち。内向きは制動が通り抜ける
  if (applied > 15) s.v += s.a_plant * dt;
  else if (applied < -15) s.v -= s.a_plant * dt;
  if (s.dir > 0 && s.v > s.v_cap) s.v = s.v_cap;
  if (s.dir < 0 && s.v < -s.v_cap) s.v = -s.v_cap;
  s.x += s.v * dt;
  s.h.now += 5 * MS;
  s.h.raw = llround(s.x);
  if (s.dir > 0 && s.h.raw > s.outward_most) s.outward_most = static_cast<int32_t>(s.h.raw);
  if (s.dir < 0 && s.h.raw < s.outward_most) s.outward_most = static_cast<int32_t>(s.h.raw);
  s.h.send(s.h.set_position_msg(target, 200, max_acc));
  s.h.ax.tick(s.h.now, s.h.raw);
  s.h.drain();
  s.delay.push_back(s.h.ax.pwm_permille());
  if (s.h.ax.end_guard_active()) {
    s.guard_seen = true;
    if (s.h.ax.state() == st::STOPPING) s.stopping_guard = true;
    const int pwm = s.h.ax.pwm_permille();
    if ((s.dir > 0 && pwm > 0) || (s.dir < 0 && pwm < 0)) s.outward_pwm = true;
  }
  if (s.h.ax.state() == st::FAULT) s.fault = s.h.ax.fault_code();
}

inline bool eg_inside(const EgSim& s) {
  const int32_t a = s.h.ax.actual_count();
  return a >= s.lim_lo && a <= s.lim_hi;
}

// 物理的に止まれる組み合わせは端の内側で END_GUARD が立ち、止まれない組み合わせは SOFT_LIMIT のまま。
inline void test_end_guard_approach_matrix() {
  const double speeds[] = {1200, 2500, 4000};
  const double accels[] = {6000, 3000};
  const int delays[] = {20, 40};
  CHECK(v1::flags::END_GUARD == (1u << 7));
  for (double speed : speeds) {
    for (double a_plant : accels) {
      for (int delay_ms : delays) {
        const double d_req = speed * 0.08 + speed * speed / (2.0 * 4000.0) + 30.0;
        const double d_stop = speed * (delay_ms / 1000.0 + 0.005) + speed * speed / (2.0 * a_plant);
        const bool can_stop = d_stop < d_req;
        for (int dir : {+1, -1}) {
          EgSim s;
          eg_prepare(s, speed, a_plant, delay_ms, dir);
          CHECK(s.h.ax.tuning.end_guard_lat_ms == V1_END_GUARD_LAT_MS);
          CHECK(s.h.ax.tuning.end_guard_acc_count_s2 == V1_END_GUARD_ACC_COUNT_S2);
          CHECK(s.h.ax.tuning.end_guard_margin_count == V1_END_GUARD_MARGIN_COUNT);
          bool stopped = false;
          bool flag_while_guard = false;
          for (int ms = 0; ms < 20000; ms += 5) {
            eg_step(s, eg_target(s));
            if (s.h.ax.end_guard_active()) {
              if ((s.h.ax.flags() & fl::END_GUARD) != 0) flag_while_guard = true;
              if (dir > 0) {
                CHECK(s.h.ax.setpoint_pos() <= static_cast<double>(s.h.ax.actual_count()) + 1e-6);
                CHECK(s.h.ax.setpoint_vel() <= 0.0);
              } else {
                CHECK(s.h.ax.setpoint_pos() >= static_cast<double>(s.h.ax.actual_count()) - 1e-6);
                CHECK(s.h.ax.setpoint_vel() >= 0.0);
              }
            }
            if (s.fault != fc::NONE) break;
            // 100 ms 窓の velocity_ は止まった直後も高いままなので、プラントの速さで「止まった」を見る
            const double pv = s.v < 0 ? -s.v : s.v;
            if (s.guard_seen && pv < 30.0 && eg_inside(s)) {
              stopped = true;
              break;
            }
          }
          if (!can_stop) {
            if (s.fault != fc::SOFT_LIMIT || !s.guard_seen) {
              std::printf("end-guard expected SOFT_LIMIT dir=%d v=%g a=%g delay=%d fault=%u guard=%d x=%d lim=%d\n",
                          dir, speed, a_plant, delay_ms, s.fault, s.guard_seen ? 1 : 0, s.h.ax.actual_count(),
                          s.lim_hi);
            }
            CHECK(s.fault == fc::SOFT_LIMIT);
            CHECK(s.guard_seen);  // ガードが先に掛かっても、越えたら SOFT_LIMIT は残る
            CHECK(!eg_inside(s));
          } else {
            if (!stopped || !flag_while_guard || s.outward_pwm || s.fault != fc::NONE) {
              std::printf(
                  "end-guard expected stop dir=%d v=%g a=%g delay=%d stopped=%d flag=%d pwm_out=%d fault=%u x=%d lim=%d pv=%g\n",
                  dir, speed, a_plant, delay_ms, stopped ? 1 : 0, flag_while_guard ? 1 : 0, s.outward_pwm ? 1 : 0, s.fault,
                  s.h.ax.actual_count(), s.lim_hi, s.v);
            }
            CHECK(stopped);
            CHECK(flag_while_guard);
            CHECK(!s.outward_pwm);
            CHECK(s.fault == fc::NONE);
            CHECK(eg_inside(s));
          }
        }
      }
    }
  }
}

// ガード中の外向きフレームは参照を進めない。内側フレームは再生遅れ 40 ms を待たず、その tick で内側の速度になる。
inline void test_end_guard_outward_clamped_and_inward_immediate() {
  EgSim s;
  eg_prepare(s, 1200, 6000, 20, +1);
  bool armed = false;
  for (int ms = 0; ms < 20000 && !armed; ms += 5) {
    eg_step(s, eg_target(s));
    if (s.h.ax.end_guard_active() && s.h.ax.state() == st::ACTIVE) armed = true;
  }
  CHECK(armed);
  CHECK((s.h.ax.flags() & fl::END_GUARD) != 0);
  const double sp = s.h.ax.setpoint_pos();
  const int32_t x = s.h.ax.actual_count();
  CHECK(sp <= static_cast<double>(x) + 1e-6);
  CHECK(s.h.ax.setpoint_vel() <= 0.0);
  eg_step(s, s.lim_hi);  // さらに外（ソフトリミット）を要求
  CHECK(s.h.ax.end_guard_active());
  CHECK(s.h.ax.setpoint_pos() <= sp + 1e-6);
  CHECK(s.h.ax.setpoint_vel() <= 0.0);
  CHECK(s.h.ax.pwm_permille() <= 0);
  // 再生バッファに切り替えても、内側の目標はその tick で追従する（遅れ 40 ms を待たない）
  s.h.ax.tuning.playout_delay_ms = 40;
  eg_step(s, 0, 20000);
  CHECK(!s.h.ax.end_guard_active());
  CHECK((s.h.ax.flags() & fl::END_GUARD) == 0);
  CHECK(s.h.ax.setpoint_vel() < 0.0);
  CHECK(s.h.ax.state() == st::ACTIVE);
}

// STOPPING でも掛かる。減速をフレームの 500 count/s² にすると、ガードが無ければ端を越える距離で止める。
inline void test_end_guard_in_stopping() {
  EgSim s;
  eg_prepare(s, 1200, 6000, 40, +1, 8000);
  bool stopped_cmd = false;
  for (int ms = 0; ms < 20000; ms += 5) {
    const int32_t x = static_cast<int32_t>(llround(s.x));
    const int32_t dist = s.lim_hi - x;
    const int32_t vm = s.h.ax.velocity_count_s();
    if (!stopped_cmd && s.h.ax.state() == st::ACTIVE && !s.h.ax.end_guard_active() && vm > 800 && dist < 1000 &&
        dist > 500) {
      Harness::accepted(s.h.stop());
      stopped_cmd = true;
    }
    eg_step(s, s.lim_hi, 500);
    if (s.fault != fc::NONE) break;
    if (stopped_cmd && s.h.ax.state() == st::HOLDING) break;
  }
  if (!stopped_cmd || !s.stopping_guard || s.fault != fc::NONE) {
    std::printf("end-guard stopping cmd=%d guard=%d fault=%u x=%d state=%u\n", stopped_cmd ? 1 : 0,
                s.stopping_guard ? 1 : 0, s.fault, s.h.ax.actual_count(), s.h.ax.state());
  }
  CHECK(stopped_cmd);
  CHECK(s.stopping_guard);
  CHECK(s.fault == fc::NONE);
  CHECK(eg_inside(s));
  CHECK(s.h.ax.actual_count() < s.lim_hi);
}

// ストローク中央の正弦は端に届かない。ビットは立たず、ガード無効の軸と参照・実位置・Duty が一致する。
inline void test_end_guard_midstroke_sine_unchanged() {
  auto run = [](bool enable, std::vector<int32_t>* act, std::vector<int32_t>* tgt, std::vector<int>* duty,
                std::vector<double>* sv, bool* bit) {
    Harness h;
    h.reach_active();
    if (!enable) h.ax.tuning.end_guard_acc_count_s2 = 0;
    *bit = false;
    for (int ms = 0; ms < 2000; ms += 5) {
      if (ms % 10 == 0) {
        const double ang = 6.283185307179586 * static_cast<double>(ms) / 2000.0;
        h.set_position(static_cast<int32_t>(llround(400.0 * sin(ang))), 50);
      }
      h.step(5);
      if (h.ax.end_guard_active() || (h.ax.flags() & fl::END_GUARD) != 0) *bit = true;
      act->push_back(h.ax.actual_count());
      tgt->push_back(h.ax.target_count());
      duty->push_back(h.ax.duty_permille());
      sv->push_back(h.ax.setpoint_vel());
    }
  };
  std::vector<int32_t> act_a, act_b, tgt_a, tgt_b;
  std::vector<int> duty_a, duty_b;
  std::vector<double> sv_a, sv_b;
  bool bit_a = false, bit_b = false;
  run(true, &act_a, &tgt_a, &duty_a, &sv_a, &bit_a);
  run(false, &act_b, &tgt_b, &duty_b, &sv_b, &bit_b);
  CHECK(!bit_a);
  CHECK(!bit_b);
  CHECK(act_a.size() == act_b.size());
  bool same = act_a.size() == act_b.size();
  for (size_t i = 0; same && i < act_a.size(); ++i) {
    if (act_a[i] != act_b[i] || tgt_a[i] != tgt_b[i] || duty_a[i] != duty_b[i] || fabs(sv_a[i] - sv_b[i]) > 1e-9) {
      same = false;
      std::printf("end-guard midstroke differs at %zu act %d/%d tgt %d/%d duty %d/%d\n", i, act_a[i], act_b[i],
                  tgt_a[i], tgt_b[i], duty_a[i], duty_b[i]);
    }
  }
  CHECK(same);
}

// プラントは動かさず、いまの速度のまま惰性で進める（PWM を加速度に戻さない）。期限の試験用。
inline void eg_coast(EgSim& s) {
  s.x += s.v * 0.005;
  s.h.now += 5 * MS;
  s.h.raw = llround(s.x);
  s.h.ax.tick(s.h.now, s.h.raw);
  s.h.drain();
}

// 実測 100・凍結参照 150・目標 120（lo 側）。目標は実測より内側だが参照より外。参照を外へ動かさない。
// フレームの max_vel / max_acc が 0 のときは CONFIG の速度・加速度で埋めない。
inline void test_end_guard_target_between_actual_and_ref_does_not_move_outward() {
  Harness h;
  h.ax.tuning.playout_delay_ms = 0;
  h.ax.tuning.deadband = 0;
  h.ax.tuning.deadband_out = 0;
  h.ax.tuning.ki_permille_per_count_s = 0;
  h.ax.tuning.stall_ms = 60000;
  h.ax.tuning.stall_boost_ms = 0;
  h.ax.tuning.assist_ms = 0;
  h.ax.tuning.soft_brake_acc_count_s2 = 0;
  v1::Config c = base_config();
  c.soft_min_count = 0;
  c.soft_max_count = 5000;
  c.max_vel_count_s = 6000;
  c.max_acc_count_s2 = 40000;
  c.following_error_limit_count = 1000000;
  c.overspeed_count_s = 100000;
  c.command_timeout_ms = 2000;
  c.stop_timeout_ms = 2000;
  Harness::accepted(h.config_with(c));
  h.raw = 150;
  h.step(5);  // 軸の生カウントを 150 にしてから基準を合わせる
  Harness::accepted(h.set_reference(150));
  CHECK(h.ax.actual_count() == 150);
  h.reach_holding_from_configured();
  h.set_position(150, 200);
  Harness::accepted(h.start());
  h.stall = true;
  h.drive(50, 150);
  CHECK(fabs(h.ax.setpoint_pos() - 150.0) < 1e-6);
  h.raw -= 50;  // 実測 100。100 ms 窓が −500 count/s になるまで目標は 150 のまま
  bool guarded = false;
  double frozen = 0;
  for (int i = 0; i < 40 && !guarded; ++i) {
    h.set_position(150, 200);
    h.step(5);
    if (h.ax.end_guard_active() && h.ax.end_guard_side() < 0) {
      guarded = true;
      frozen = h.ax.setpoint_pos();
    }
  }
  if (!guarded || h.ax.actual_count() != 100 || frozen < 150.0 - 1e-6) {
    std::printf("end-guard between setup guard=%d x=%d v=%d sp=%g sv=%g side=%d\n", guarded ? 1 : 0,
                h.ax.actual_count(), h.ax.velocity_count_s(), h.ax.setpoint_pos(), h.ax.setpoint_vel(),
                h.ax.end_guard_side());
  }
  CHECK(guarded);
  CHECK(h.ax.actual_count() == 100);
  CHECK(h.ax.velocity_count_s() <= -400);
  CHECK(frozen >= 150.0 - 1e-6);
  h.pos_max_vel = 4000;
  for (int i = 0; i < 8; ++i) {
    h.send(h.set_position_msg(120, 200, 20000));
    h.step(5);
    if (!h.ax.end_guard_active() || h.ax.setpoint_vel() < -1e-9 || h.ax.setpoint_pos() < frozen - 1e-6) {
      std::printf("end-guard between retarget i=%d side=%d sp=%g sv=%g x=%d\n", i, h.ax.end_guard_side(),
                  h.ax.setpoint_pos(), h.ax.setpoint_vel(), h.ax.actual_count());
    }
    CHECK(h.ax.end_guard_active());
    CHECK(h.ax.setpoint_vel() >= -1e-9);
    CHECK(h.ax.setpoint_pos() >= frozen - 1e-6);
    CHECK(h.ax.setpoint_pos() >= static_cast<double>(h.ax.actual_count()) - 1e-6);
  }
  const double held = h.ax.setpoint_pos();
  h.pos_max_vel = 0;
  for (int i = 0; i < 5; ++i) {
    h.send(h.set_position_msg(400, 200, 0));  // 内側だがフレームの速度・加速度は 0
    h.step(5);
    if (fabs(h.ax.setpoint_vel()) >= 1e-6 || fabs(h.ax.setpoint_pos() - held) >= 1e-6) {
      std::printf("end-guard zero frame i=%d sp=%g sv=%g held=%g\n", i, h.ax.setpoint_pos(), h.ax.setpoint_vel(),
                  held);
    }
    CHECK(fabs(h.ax.setpoint_vel()) < 1e-6);
    CHECK(fabs(h.ax.setpoint_pos() - held) < 1e-6);
  }
  CHECK(h.ax.state() == st::ACTIVE && h.ax.fault_code() == fc::NONE);
}

// 端ガード中にエンコーダが死ぬと、参照速度 0 のままでも既存の無パルス窓で ENCODER_FAULT。
// 実測速度が落ちてガードが解け、STOPPING が HOLDING で終わらない。
inline void test_end_guard_dead_encoder_faults_within_stall_window() {
  EgSim s;
  eg_prepare(s, 1200, 6000, 20, +1, 4000);
  s.h.ax.tuning.stall_ms = V1_STALL_MS;
  s.h.ax.tuning.duty_max = 20;
  bool armed = false;
  for (int ms = 0; ms < 20000 && !armed; ms += 5) {
    eg_step(s, eg_target(s));
    if (s.h.ax.end_guard_active() && s.h.ax.state() == st::ACTIVE && s.v > 400 && s.h.ax.velocity_count_s() > 200)
      armed = true;
  }
  if (!armed) {
    std::printf("end-guard dead-encoder setup x=%d v=%g vm=%d state=%u guard=%d fault=%u\n", s.h.ax.actual_count(),
                s.v, s.h.ax.velocity_count_s(), s.h.ax.state(), s.h.ax.end_guard_active() ? 1 : 0, s.fault);
  }
  CHECK(armed);
  const int32_t jumped = s.h.ax.actual_count() + 40;
  CHECK(jumped < s.lim_hi);
  s.h.raw = jumped;
  s.x = jumped;
  s.v = 0;
  s.h.stall = true;
  Harness::accepted(s.h.stop());
  bool holding = false;
  uint32_t elapsed = 0;
  for (; elapsed < 1000 && s.h.ax.state() != st::FAULT; elapsed += 5) {
    s.h.send(s.h.set_position_msg(s.lim_hi, 200, 20000));
    s.h.step(5);
    if (s.h.ax.state() == st::HOLDING || s.h.ax.state() == st::DISARMING || s.h.ax.state() == st::DISARMED)
      holding = true;
  }
  if (holding || s.h.ax.fault_code() != fc::ENCODER_FAULT || elapsed > V1_STALL_MS + 80) {
    std::printf("end-guard dead-encoder holding=%d fault=%u elapsed=%u state=%u duty=%d x=%d sp=%g\n",
                holding ? 1 : 0, s.h.ax.fault_code(), elapsed, s.h.ax.state(), s.h.ax.duty_permille(),
                s.h.ax.actual_count(), s.h.ax.setpoint_pos());
  }
  CHECK(!holding);
  CHECK(s.h.ax.state() == st::FAULT && s.h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(elapsed <= V1_STALL_MS + 80);
  CHECK(elapsed + 5 >= V1_STALL_MS);  // 窓を短くしていない
  CHECK(s.h.ax.duty_permille() == 0 && s.h.ax.pwm_permille() == 0);
  CHECK(!s.h.ax.brake_release_cmd());
}

// 速度窓がまだ 0 の起動で、端の d_req（参照速度を推定の上限）以内なら外へ参照を進めない。
inline void test_end_guard_start_near_limit_before_velocity_window() {
  auto run = [](int dir) {
    Harness h;
    h.ax.tuning.playout_delay_ms = 0;
    h.ax.tuning.soft_brake_acc_count_s2 = 0;
    h.ax.tuning.stall_ms = 60000;
    h.ax.tuning.deadband = 0;
    h.ax.tuning.deadband_out = 0;
    v1::Config c = base_config();
    c.soft_min_count = -40;
    c.soft_max_count = 40;
    c.max_vel_count_s = 6000;
    c.max_acc_count_s2 = 40000;
    c.following_error_limit_count = 1000000;
    c.overspeed_count_s = 100000;
    c.command_timeout_ms = 1000;
    c.stop_timeout_ms = 2000;
    Harness::accepted(h.config_with(c));
    h.reach_holding_from_configured();
    h.set_position(0, 200);
    Harness::accepted(h.start());
    h.stall = true;
    h.pos_max_vel = 4000;
    const int32_t target = dir > 0 ? 40 : -40;
    double outward = 0;
    bool flag = false;
    bool vel_moved = false;
    for (int ms = 0; ms < 80; ms += 5) {
      h.send(h.set_position_msg(target, 200, 40000));
      h.step(5);
      if (h.ax.velocity_count_s() != 0) vel_moved = true;
      if (h.ax.end_guard_active()) flag = true;
      const double sp = h.ax.setpoint_pos();
      if (dir > 0) {
        if (sp > outward) outward = sp;
        CHECK(h.ax.setpoint_vel() <= 1e-9);
      } else {
        if (sp < outward) outward = sp;
        CHECK(h.ax.setpoint_vel() >= -1e-9);
      }
    }
    if (!flag || vel_moved || h.ax.actual_count() != 0 || (dir > 0 ? outward >= 5.0 : outward <= -5.0)) {
      std::printf("end-guard start dir=%d flag=%d vel_moved=%d x=%d outward=%g sv=%g side=%d state=%u\n", dir,
                  flag ? 1 : 0, vel_moved ? 1 : 0, h.ax.actual_count(), outward, h.ax.setpoint_vel(),
                  h.ax.end_guard_side(), h.ax.state());
    }
    CHECK(flag);
    CHECK((h.ax.flags() & fl::END_GUARD) != 0);
    CHECK(!vel_moved);
    CHECK(h.ax.actual_count() == 0);
    CHECK(h.ax.state() == st::ACTIVE && h.ax.fault_code() == fc::NONE);
    if (dir > 0) CHECK(outward < 5.0);
    else CHECK(outward > -5.0);
  };
  run(+1);
  run(-1);
}

// ガードで参照速度が 0 でも、COMMAND_TIMEOUT / DISARM は即ラッチせず stop_timeout_ms の期限で止まる。
inline void test_end_guard_command_timeout_and_disarm_hit_stop_deadline() {
  auto reach = [](EgSim& s) {
    bool reached = false;
    for (int ms = 0; ms < 20000 && !reached; ms += 5) {
      eg_step(s, eg_target(s));
      const int32_t dist = s.lim_hi - s.h.ax.actual_count();
      if (s.fault != fc::NONE) break;
      // 掛かり始めは端まで d_req（約 300）。惰性 130 ms × 1200 count/s が収まる余裕だけ残す
      if (s.h.ax.end_guard_active() && s.h.ax.state() == st::ACTIVE && s.v > 500 && s.h.ax.velocity_count_s() > 200 &&
          dist > 200)
        reached = true;
    }
    if (!reached) {
      std::printf("end-guard deadline setup x=%d lim=%d v=%g vm=%d state=%u guard=%d fault=%u\n",
                  s.h.ax.actual_count(), s.lim_hi, s.v, s.h.ax.velocity_count_s(), s.h.ax.state(),
                  s.h.ax.end_guard_active() ? 1 : 0, s.fault);
    }
    CHECK(reached);
    return reached;
  };
  auto coast_deadline = [](EgSim& s, uint64_t t_stop, uint16_t expect) {
    bool at70 = false;
    bool at80 = false;
    bool early = false;
    bool holding = false;
    uint64_t t_fault = 0;
    while (s.h.now < t_stop + 200 * MS && t_fault == 0) {
      eg_coast(s);
      const uint64_t el = s.h.now - t_stop;
      if (s.h.ax.state() == st::HOLDING || s.h.ax.state() == st::DISARMING || s.h.ax.state() == st::DISARMED)
        holding = true;
      if (s.h.ax.state() == st::FAULT) {
        t_fault = s.h.now;
        if (el <= 80 * MS) early = true;
      } else {
        if (el == 70 * MS) at70 = s.h.ax.state() == st::STOPPING;
        if (el == 80 * MS) at80 = s.h.ax.state() == st::STOPPING;
      }
    }
    if (!at70 || !at80 || early || holding || t_fault == 0 || s.h.ax.fault_code() != expect ||
        t_fault > t_stop + 85 * MS) {
      std::printf(
          "end-guard deadline expect=%u fault=%u el=%llu at70=%d at80=%d early=%d hold=%d state=%u x=%d v=%g\n",
          expect, s.h.ax.fault_code(), t_fault ? (unsigned long long)((t_fault - t_stop) / MS) : 0ull, at70 ? 1 : 0,
          at80 ? 1 : 0, early ? 1 : 0, holding ? 1 : 0, s.h.ax.state(), s.h.ax.actual_count(), s.v);
    }
    CHECK(at70);
    CHECK(at80);
    CHECK(!early && !holding);
    CHECK(t_fault > t_stop + 80 * MS && t_fault <= t_stop + 85 * MS);
    CHECK(s.h.ax.fault_code() == expect);
    CHECK(s.h.ax.duty_permille() == 0 && s.h.ax.pwm_permille() == 0);
    CHECK(!s.h.ax.brake_release_cmd());
  };

  EgSim timeout;
  eg_prepare(timeout, 1200, 6000, 20, +1, 8000, 40, 80);
  if (reach(timeout)) {
    uint64_t t_stop = 0;
    for (int i = 0; i < 30 && t_stop == 0; ++i) {
      eg_coast(timeout);
      if (timeout.h.ax.state() == st::STOPPING) t_stop = timeout.h.now;
      if (timeout.h.ax.state() == st::FAULT) break;
    }
    if (t_stop == 0) {
      std::printf("end-guard timeout did not enter STOPPING state=%u fault=%u\n", timeout.h.ax.state(),
                  timeout.h.ax.fault_code());
    }
    CHECK(t_stop != 0);
    if (t_stop != 0) coast_deadline(timeout, t_stop, fc::COMMAND_TIMEOUT);
  }

  EgSim disarm;
  eg_prepare(disarm, 1200, 6000, 20, +1, 8000, 5000, 80);
  if (reach(disarm)) {
    Harness::accepted(disarm.h.disarm());
    CHECK(disarm.h.ax.state() == st::STOPPING);
    coast_deadline(disarm, disarm.h.now, fc::STOP_TIMEOUT);
  }
}

// 自重（外向きの一定加速度）と積分を入れたまま、下端へ指令を出し続ける。端の内側で止まるか SOFT_LIMIT。静かに越えない。
inline void test_end_guard_gravity_bias_stops_or_soft_limit() {
  EgSim s;
  eg_prepare(s, 600, 8000, 30, -1, 1800);
  s.h.ax.tuning.ki_permille_per_count_s = V1_KI_PERMILLE_PER_COUNT_S;
  s.h.ax.tuning.i_max_permille = V1_I_MAX_PERMILLE;
  s.h.ax.tuning.i_rate_permille_per_s = V1_I_RATE_PERMILLE_PER_S;
  s.h.ax.tuning.stall_ms = 60000;
  CHECK(s.h.ax.tuning.ki_permille_per_count_s > 0.0);
  const double bias = 2000.0;  // count/s²。カウント減（lo 側）へ
  bool silent = false;
  bool guard = false;
  for (int ms = 0; ms < 6000; ms += 5) {
    const int applied = s.delay.empty() ? 0 : s.delay.front();
    if (!s.delay.empty()) s.delay.erase(s.delay.begin());
    const double dt = 0.005;
    if (applied > 15) s.v += s.a_plant * dt;
    else if (applied < -15) s.v -= s.a_plant * dt;
    s.v -= bias * dt;
    if (s.v > s.v_cap) s.v = s.v_cap;
    if (s.v < -s.v_cap) s.v = -s.v_cap;
    s.x += s.v * dt;
    s.h.now += 5 * MS;
    s.h.raw = llround(s.x);
    s.h.send(s.h.set_position_msg(s.lim_lo, 200, 20000));
    s.h.ax.tick(s.h.now, s.h.raw);
    s.h.drain();
    s.delay.push_back(s.h.ax.pwm_permille());
    if (s.h.ax.end_guard_active()) guard = true;
    if (s.h.ax.actual_count() < s.lim_lo && s.h.ax.fault_code() != fc::SOFT_LIMIT) silent = true;
    if (s.h.ax.state() == st::FAULT) break;
    const double pv = s.v < 0.0 ? -s.v : s.v;
    if (guard && pv < 30.0 && s.h.ax.actual_count() >= s.lim_lo && s.h.ax.fault_code() == fc::NONE) break;
  }
  const bool inside = s.h.ax.fault_code() == fc::NONE && s.h.ax.actual_count() >= s.lim_lo &&
                      (s.v < 0.0 ? -s.v : s.v) < 30.0;
  const bool soft = s.h.ax.fault_code() == fc::SOFT_LIMIT;
  if (!guard || silent || (!inside && !soft)) {
    std::printf("end-guard gravity guard=%d silent=%d inside=%d fault=%u x=%d lim=%d v=%g state=%u\n", guard ? 1 : 0,
                silent ? 1 : 0, inside ? 1 : 0, s.h.ax.fault_code(), s.h.ax.actual_count(), s.lim_lo, s.v,
                s.h.ax.state());
  }
  CHECK(guard);
  CHECK(!silent);
  CHECK(inside || soft);
}

inline void run_all() {
  test_boots_disarmed_with_safe_outputs();
  test_hello_answers_in_every_state();
  test_clock_sync_replies();
  test_config_sets_configured_flag_and_immediate_status();
  test_config_invalid_params_rejected();
  test_config_only_in_disarmed();
  test_request_id_zero_rejected();
  test_config_clamps_vel_and_acc_to_absolute_ceiling();
  test_arm_prepare_rejections_in_order();
  test_arming_sequence_power_then_brake_then_holding();
  test_non_power_controller_never_commands_power();
  test_start_rejections_and_transition();
  test_start_rejected_without_unexpired_set_position();
  test_start_ignores_frames_from_before_arm_prepare();
  test_start_keeps_hold_target_until_next_set_position();
  test_holding_ignores_target_but_refreshes_lease();
  test_active_tracks_target_with_limits();
  test_status_target_is_profiled_setpoint_not_commanded();
  test_frame_limits_capped_by_config_and_can_be_lower();
  test_soft_limit_clamps_target();
  test_controlled_stop_decelerates_to_holding();
  test_controlled_stop_epoch_mismatch_and_noop_states();
  test_valid_for_expiry_stops_to_holding();
  test_disarm_from_holding();
  test_disarm_from_active_decelerates_first();
  test_disarm_from_stopping_switches_to_disarm();
  test_disarm_from_arming_and_rearm_works();
  test_set_position_stale_seq_and_epoch_dropped();
  test_set_position_before_config_dropped();
  test_stale_frames_do_not_refresh_lease();
  test_apply_at_nonzero_dropped_as_rx_error();
  test_seq_resets_only_on_new_epoch();
  test_command_timeout_in_holding_faults();
  test_command_timeout_in_arming_counts_from_arm_prepare();
  test_command_timeout_while_moving_decelerates_then_faults();
  test_host_fault_during_fault_stop_keeps_pending_code();
  test_stop_timeout_faults();
  test_stopping_decelerates_with_last_frame_max_acc();
  test_stopping_with_zero_frame_acc_falls_back_to_config();
  test_set_limits_accepted_in_disarmed_and_reports_limits();
  test_set_limits_accepted_in_active_and_takes_effect();
  test_set_limits_invalid_params_rejected();
  test_set_limits_clamps_duty_to_absolute_max();
  test_set_limits_rejected_while_faulted_and_duplicate_not_reexecuted();
  test_stall_causes_following_error();
  test_overspeed_faults();
  test_actual_outside_soft_limits_faults();
  test_arming_never_releases_brake_outside_soft_window();
  test_soft_limit_recovery_goes_through_disarmed_and_set_reference();
  test_encoder_fault_on_no_pulses_while_driving();
  test_encoder_fault_not_triggered_while_counts_change();
  test_disarm_with_residual_error_reaches_disarmed_not_encoder_fault();
  test_static_hold_error_at_min_duty_is_not_encoder_fault();
  test_active_settled_with_residual_error_is_not_encoder_fault();
  test_stall_boost_ramps_while_stuck_and_resets_in_band();
  test_stall_boost_times_out_to_encoder_fault();
  test_stall_boost_at_cap_without_motion_is_encoder_fault();
  test_stall_boost_timer_restarts_on_motion();
  test_stall_boost_cap_without_encoder_fault_when_check_disabled();
  test_stall_boost_requires_error_outside_outer_band();
  test_stall_boost_window_restarts_on_motion_but_ignores_jitter();
  test_stall_boost_resets_on_error_sign_flip();
  test_stall_boost_not_while_moving_or_brake_applied_and_capped_by_limits();
  test_duty_command_floor_gate_with_hysteresis();
  test_axis_tracking_uses_raw_ff_and_floor_follows_hysteresis();
  test_duty_command_floor_requires_p_and_respects_small_deadband_out();
  test_encoder_fault_threshold_follows_duty_max();
  test_stuck_axis_with_high_kp_is_following_error_not_encoder_fault();
  test_integral_rate_limit_and_saturation();
  test_integral_anti_windup_on_applied_output();
  test_integral_blocked_during_floor_sign_mismatch_but_unwinds();
  test_integral_holding_bias_retained_in_band();
  test_integral_in_axis_accumulates_and_resets_on_disarm_fault_reference();
  test_integral_does_not_flip_floor_direction_when_stationary();
  test_settled_entry_requires_dwell_with_measured_rest();
  test_settled_blocked_by_measured_motion_and_legacy_immediate();
  test_settled_on_keeps_stationary_stall_fault();
  test_track_assist_triggers_after_150ms_and_releases_on_motion();
  test_track_assist_requires_trajectory_speed_and_can_be_disabled();
  test_track_assist_does_not_block_following_error_or_moving_stall_fault();
  test_r6_ff_directional_values_and_taper();
  test_r6_no_floor_while_moving();
  test_r6_pi_anti_windup_at_duty_max();
  test_r6_hold_entry_exit_and_zeroing();
  test_r6_breakaway_pulse_output();
  test_r6_axis_hold_requires_dwell_and_holds_to_4();
  test_r6_breakaway_pulses_then_stall_watchdog_at_2s();
  test_r6_breakaway_releases_on_progress_and_down_direction();
  test_r6_axis_tracking_outputs_raw_ff_without_floor_or_pulse();
  test_r6_following_error_and_moving_stall_fault_unchanged();
  test_r6_set_limits_ff_and_floor_fields_are_ignored();
  test_r6_breakaway_not_fired_by_slow_lag_but_fires_on_moving_stall();
  test_r6_stall_watchdog_ignores_motion_away_from_target();
  test_r7_stationary_error_3_gets_breakaway_and_watchdog();
  test_hold_drift_within_band_is_not_encoder_fault();
  test_r8_band_stuck_assist_short_pulses_with_cooldown();
  test_r9_tuning_defaults_and_switch();
  test_r9_damping_term_sign_and_clamp();
  test_r9_damping_zero_in_hold();
  test_r9_damping_saturation_and_breakaway_interplay();
  test_r9_v_fast_constant_speed_and_quantization();
  test_r9_v_fast_gap_startup_and_reset();
  test_r9_axis_hold_zero_and_protections_unchanged();
  test_r9_damping_reduces_velocity_ripple_in_delayed_plant();
  test_playout_clock_and_fit_unit();
  test_playout_legacy_profile_reproduces_issue45_snaps();
  test_playout_ramps_have_no_artificial_stops();
  test_playout_reversal_is_smooth();
  test_playout_dropped_frames();
  test_playout_soft_limit_approach_stops_inside_reserve();
  test_playout_expiry_stop_fault_epoch_unchanged();
  test_playout_stream_loss_excursion_is_bounded();
  test_playout_zero_max_acc_still_decelerates();
  test_playout_cadence_change_reseeds_clock();
  test_device_id_packs_mac_in_printed_order();
  test_control_overrun_faults_only_when_not_parked();
  test_host_fault_latches_immediately_from_active();
  test_fault_gate_rejects_commands();
  test_fault_reset_rules();
  test_host_fault_with_request_id_zero_and_reused_id_still_latches();
  test_duplicate_request_is_not_reexecuted();
  test_same_request_id_with_new_content_is_executed();
  test_resent_rejected_command_is_reevaluated();
  test_dedupe_window_is_bounded();
  test_set_reference_shifts_coordinate_without_sign();
  test_set_reference_only_in_disarmed();
  test_select_axis_in_disarmed_clears_config_and_reference_and_announces_once();
  test_select_axis_is_ignored_outside_disarmed();
  test_status_after_an_ack_shows_the_post_command_state();
  test_periodic_status_every_20ms();
  test_status_reports_following_error_and_pwm_sign();
  test_duty_sign_follows_profile_sign();
  test_duty_deadband_and_clamp();
  test_duty_command_feed_forward();
  test_duty_terms_report_ff_p_floor_and_band();
  test_duty_command_permille_resolution_and_saturation();
  test_encoder_fault_threshold_permille_matches_percent();
  test_motor_output_permille_mapping();
  test_local_estop_latches_and_blocks_reset_until_released();
  test_local_estop_in_disarmed_and_during_other_fault();
  test_bad_frames_counted_and_unanswered();
  test_outbox_overflow_drops_status_first();
  test_calibrate_rejections_and_noop_cancel();
  test_calibration_success_on_power_axis();
  test_calibration_negative_sign_plant_reports_negative_n_end();
  test_calibration_on_non_power_axis_without_config();
  test_calibration_fails_on_stall_before_min_travel();
  test_calibration_failure_at_neutral_reports_values_but_keeps_nothing();
  test_calibrate_cancel_before_and_during_motion();
  test_disarm_cancels_calibration();
  test_calibration_keep_power_holds_main_power_until_release();
  test_calibration_keep_power_hold_ends_on_timeout_or_disarm();
  test_calibration_keep_power_cancel_during_motion_goes_to_disarmed();
  test_calibration_keep_power_ignored_on_non_power_axis();
  test_fault_during_power_hold_drops_power_without_new_result();
  test_calibrating_rejects_or_drops_other_commands();
  test_host_fault_during_calibration();
  test_local_estop_and_overrun_during_calibration();
  test_fault_during_calibration_shutdown_keeps_final_result();
  test_end_guard_approach_matrix();
  test_end_guard_outward_clamped_and_inward_immediate();
  test_end_guard_in_stopping();
  test_end_guard_midstroke_sine_unchanged();
  test_end_guard_target_between_actual_and_ref_does_not_move_outward();
  test_end_guard_dead_encoder_faults_within_stall_window();
  test_end_guard_start_near_limit_before_velocity_window();
  test_end_guard_command_timeout_and_disarm_hit_stop_deadline();
  test_end_guard_gravity_bias_stops_or_soft_limit();
}

}  // namespace v1axis_test
