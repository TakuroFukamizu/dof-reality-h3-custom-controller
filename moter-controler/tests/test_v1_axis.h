// v1_axis のホストテスト。test_host.cpp から include する（CHECK マクロは test_host.cpp のもの）。
// control-server/tests/sim_axis.rs の主要ケースを C++ に写し、設計 §6 の各項目を確認する。
// 時刻は注入、エンコーダは「setpoint に向かって plant_vel で追従する」簡単なプラントで模擬する。
#pragma once

#include <cstdio>
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
  v1::HostMsg set_position_msg(int32_t target, uint16_t valid_for_ms = 50) {
    v1::HostMsg m;
    memset(&m, 0, sizeof(m));
    m.type = v1::msg::SET_POSITION;
    m.set_position.session_epoch = EPOCH;
    m.set_position.seq = ++seq;
    m.set_position.group_frame_id = ++frame;
    m.set_position.target_count = target;
    m.set_position.max_vel_count_s = 4000;
    m.set_position.max_acc_count_s2 = 20000;
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
  h.drive(30, 2000);
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
  h.drive(2000, 5000);
  CHECK(h.ax.state() == st::ACTIVE);
  CHECK(h.ax.target_count() == 2700);
  CHECK(h.ax.actual_count() == 2700);
}

inline void test_controlled_stop_decelerates_to_holding() {
  Harness h;
  h.reach_active();
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
  c.max_acc_count_s2 = 100;  // 4000 count/s からの減速に 40 s
  c.stop_timeout_ms = 300;
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
  CHECK(h.ax.state() == st::STOPPING);
  h.drive(310, 2600);
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
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;  // 追従誤差より先に無パルスで止める
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.set_position(0);
  Harness::accepted(h.start());
  h.stall = true;  // カウントが止まる
  // 軌道が動き出してフィードフォワードが |duty| ≥ しきい値 15 %（max(10, duty_max 20 × 75 %)。Issue #15 M2）に届いてから
  // STALL_MS（200 ms）で異常（max_vel 4000 → 上限 20 % で頭打ち）
  CHECK(h.ax.encoder_fault_duty_threshold_permille() == 150);
  h.drive(150, 2000);
  CHECK(h.ax.state() == st::ACTIVE);
  CHECK(h.ax.duty_permille() != 0);
  h.drive(150, 2000);
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
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;  // 後半の無パルス確認で追従誤差より先に止めない
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
  // 軌道が動き出せば従来どおり無パルスで ENCODER_FAULT（既存の test_encoder_fault_on_no_pulses_while_driving と同じ経路）
  Harness::accepted(h.start());
  h.drive(300, h.ax.actual_count() + 2000);
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
  v1::Config c = base_config();
  c.following_error_limit_count = 100000;
  Harness::accepted(h.config_with(c));
  h.reach_holding_from_configured();
  h.stall = true;
  h.raw -= 25;  // 不感帯の外 → in_band = false、下限 7 %、上乗せ開始
  h.drive(420, h.ax.actual_count() + 25);
  CHECK(h.ax.stall_boost_permille() == 20);
  h.raw += 10;  // 10 count だけ動いて誤差 15: 入る側 10 より大きいので in_band = false のまま → 下限 7 % は出続ける
  h.drive(1000, h.ax.actual_count() + 15);
  CHECK(h.ax.state() == st::HOLDING && h.ax.fault_code() == fc::NONE);
  CHECK(h.ax.duty_permille() == -70);
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

// Issue #15 M1: Duty 下限は「目標へ向かう向き」のときだけ。FF と P の合成が下限未満で誤差と逆向きなら 0（惰行）。
inline void test_duty_command_floor_only_toward_target() {
  V1Tuning t;
  t.kp_percent_per_100 = 6;
  t.duty_min = 9;
  t.duty_max = 20;
  t.floor_sign_gate = true;
  bool band = false;
  // Issue の例: err +50（P +3 %）・vel −200（FF −2 %）→ 合成 +1 % → 目標向き → 下限 9
  CHECK(v1_duty_command(50, -200.0, band, t) == 90);
  // err +50（P +3）・vel −600（FF −6）→ 合成 −3 % → 目標から離れる向き → 0（旧: −9 のバースト）
  CHECK(v1_duty_command(50, -600.0, band, t) == 0);
  CHECK(v1_duty_command(-50, 600.0, band, t) == 0);
  // 合成が下限以上なら向きに関わらずそのまま（減速中に実位置が先行: P −18 + FF +2 = −16）
  CHECK(v1_duty_command(-300, 200.0, band, t) == -160);
  // 誤差 0 は向きの一致とみなし下限を掛ける（軌道だけが動いている立ち上がり）
  CHECK(v1_duty_command(0, 200.0, band, t) == 90);
  // 目標向きの小さい合成は下限（従来どおり）
  CHECK(v1_duty_command(50, 0.0, band, t) == 90);
  CHECK(v1_duty_command(-50, 0.0, band, t) == -90);
  // gate を切れば旧動作（合成値に下限を掛ける）
  t.floor_sign_gate = false;
  CHECK(v1_duty_command(50, -600.0, band, t) == -90);
  CHECK(v1_duty_command(-50, 600.0, band, t) == 90);
  CHECK(v1_duty_command(-300, 200.0, band, t) == -160);
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
  bool band = false;
  CHECK(v1_duty_command(0, 600.0, band, t) == V1_DUTY_MIN_PERMILLE);   // 6 % → 下限 7 %
  CHECK(v1_duty_command(0, 1500.0, band, t) == 150);            // 速度だけで 15 %
  CHECK(v1_duty_command(0, 2500.0, band, t) == V1_DUTY_MAX_PERMILLE);   // 25 % → 上限 20 %
  CHECK(v1_duty_command(300, 600.0, band, t) == 120);           // 6 + 6
  CHECK(v1_duty_command(-300, 600.0, band, t) == 0);           // 誤差が速度と逆向きで打ち消し → 0
  CHECK(v1_duty_command(0, -600.0, band, t) == -V1_DUTY_MIN_PERMILLE);  // 向きは速度の符号
  CHECK(v1_duty_command(-100, -1000.0, band, t) == -120);       // −2 − 10
  // 目標が動いている間は不感帯を使わない
  band = true;
  CHECK(v1_duty_command(3, 600.0, band, t) == V1_DUTY_MIN_PERMILLE);
  CHECK(!band);
  // 止まったら不感帯（入る 10 / 出る 20）
  CHECK(v1_duty_command(10, 0.0, band, t) == 0 && band);
  CHECK(v1_duty_command(20, 0.0, band, t) == 0 && band);
  CHECK(v1_duty_command(21, 0.0, band, t) == V1_DUTY_MIN_PERMILLE && !band);
  CHECK(v1_duty_command(11, 0.0, band, t) == V1_DUTY_MIN_PERMILLE && !band);
  CHECK(v1_duty_command(10, 0.0, band, t) == 0 && band);
  // ff = 0 なら比例だけ
  V1Tuning p;
  p.ff_count_s_per_percent = 0;
  bool b2 = false;
  CHECK(v1_duty_command(0, 600.0, b2, p) == 0);
  CHECK(v1_duty_command(500, 600.0, b2, p) == 100);
}

// Issue #25 手順 8 (a): 位置ループの各項の計器。v1_duty_command の結果は変えず、FF / P / 軌道速度 / 下限の適用 / 不感帯を書く。
inline void test_duty_terms_report_ff_p_floor_and_band() {
  V1Tuning t;  // kp 2、ff 100、下限 7、上限 20
  bool band = false;
  V1DutyTerms o;
  // FF 6 % + P 6 % = 12 %。下限は掛からない
  CHECK(v1_duty_command(300, 600.0, band, t, 0, &o) == 120);
  CHECK(o.ff_permille == 60 && o.p_permille == 60 && o.vel_count_s == 600 && !o.floor_on && !o.in_band &&
        o.cmd_permille == 120);
  // FF 2 % だけ → 下限 7 %（floor_on）
  CHECK(v1_duty_command(0, 200.0, band, t, 0, &o) == 70);
  CHECK(o.ff_permille == 20 && o.p_permille == 0 && o.floor_on && o.cmd_permille == 70);
  // 負の向き: P −1.2 %・FF −2 % → 合成 −3.2 % → 下限 −7 %
  CHECK(v1_duty_command(-60, -200.0, band, t, 0, &o) == -70);
  CHECK(o.ff_permille == -20 && o.p_permille == -12 && o.floor_on && o.cmd_permille == -70 && o.vel_count_s == -200);
  // M1: 誤差と逆向きの合成が下限未満 → 0。floor_on は立たない（下限を掛けていない）
  CHECK(v1_duty_command(50, -600.0, band, t, 0, &o) == 0);
  CHECK(o.ff_permille == -60 && o.p_permille == 10 && !o.floor_on && o.cmd_permille == 0);
  // 上限で飽和: FF 25 % → 20 %
  CHECK(v1_duty_command(0, 2500.0, band, t, 0, &o) == 200);
  CHECK(o.ff_permille == 250 && o.cmd_permille == 200 && !o.floor_on);
  // 静止の不感帯の中: in_band が立ち、各項は 0
  CHECK(v1_duty_command(5, 0.0, band, t, 0, &o) == 0 && band);
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
  bool band = false;
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
  h.raw = -15;  // 駆動中は入る側 10 まで続ける
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_permille() == -V1_DUTY_MIN_PERMILLE);
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
  test_duty_command_floor_only_toward_target();
  test_encoder_fault_threshold_follows_duty_max();
  test_stuck_axis_with_high_kp_is_following_error_not_encoder_fault();
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
}

}  // namespace v1axis_test
