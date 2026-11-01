// v1_axis のホストテスト。test_host.cpp から include する（CHECK マクロは test_host.cpp のもの）。
// control-server/tests/sim_axis.rs の主要ケースを C++ に写し、設計 §6 の各項目を確認する。
// 時刻は注入、エンコーダは「setpoint に向かって plant_vel で追従する」簡単なプラントで模擬する。
#pragma once

#include <cstdio>
#include <cstring>
#include <vector>

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

  // 1 tick（既定 5 ms）。プラントは位置制御中だけ setpoint に追従する。
  void step(uint32_t dt_ms = 5) {
    now += dt_ms * MS;
    if (!stall && ax.brake_release_cmd()) {
      const double max_step = plant_vel * static_cast<double>(dt_ms) / 1000.0;
      double delta = static_cast<double>(ax.target_count()) - static_cast<double>(ax.actual_count());
      if (delta > max_step) delta = max_step;
      if (delta < -max_step) delta = -max_step;
      raw += static_cast<int64_t>(delta);
    }
    ax.tick(now, raw);
    drain();
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
  CHECK(h.ax.duty_percent() == 0);
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
        CHECK(m.hello.capabilities == v1::capabilities::POSITION_MODE);
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
  CHECK(h.ax.duty_percent() == 0);
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
  CHECK(h.ax.duty_percent() == 0);
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
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_percent() == 0);
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
  // 軌道が動き出した tick からフィードフォワードで |duty| ≥ 10 %（max_vel 4000 → 上限 20 %）。そこから STALL_MS（200 ms）で異常
  h.drive(150, 2000);
  CHECK(h.ax.state() == st::ACTIVE);
  CHECK(h.ax.duty_percent() != 0);
  h.drive(100, 2000);
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::ENCODER_FAULT);
  CHECK(h.ax.duty_percent() == 0);
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
  CHECK(h.ax.state() == st::HOLDING && h.ax.duty_percent() != 0);
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
  CHECK(h.ax.duty_percent() == 7 || h.ax.duty_percent() == -7);
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
  CHECK(h.ax.duty_percent() != 0);
  CHECK(h.ax.fault_code() == fc::NONE);
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
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_percent() == 0);
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
  CHECK(h.ax.duty_percent() < 0);
  CHECK(h.ax.pwm_permille() > 0);
  CHECK(h.ax.duty_percent() == -V1_DUTY_MIN);  // |err| 100 × 2 / 100 = 2 % → 下限 7 %
  h.raw = 100;
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_percent() > 0 && h.ax.pwm_permille() < 0);
  // sign = −1 では逆
  Harness g(1, -1);
  g.reach_holding();
  g.stall = true;
  g.raw = -100;
  g.set_position(0);
  g.step();
  CHECK(g.ax.duty_percent() > 0);
  CHECK(g.ax.pwm_permille() > 0);
}

// フィードフォワード（docs/multi_axis_20261031.md の実測: 1 % ≈ 100〜130 count/s）。純粋関数を直接検査する。
inline void test_duty_command_feed_forward() {
  V1Tuning t;
  bool band = false;
  CHECK(v1_duty_command(0, 600.0, band, t) == V1_DUTY_MIN);   // 6 % → 下限 7 %
  CHECK(v1_duty_command(0, 1500.0, band, t) == 15);            // 速度だけで 15 %
  CHECK(v1_duty_command(0, 2500.0, band, t) == V1_DUTY_MAX);   // 25 % → 上限 20 %
  CHECK(v1_duty_command(300, 600.0, band, t) == 12);           // 6 + 6
  CHECK(v1_duty_command(-300, 600.0, band, t) == 0);           // 誤差が速度と逆向きで打ち消し → 0
  CHECK(v1_duty_command(0, -600.0, band, t) == -V1_DUTY_MIN);  // 向きは速度の符号
  CHECK(v1_duty_command(-100, -1000.0, band, t) == -12);       // −2 − 10
  // 目標が動いている間は不感帯を使わない
  band = true;
  CHECK(v1_duty_command(3, 600.0, band, t) == V1_DUTY_MIN);
  CHECK(!band);
  // 止まったら不感帯（入る 10 / 出る 20）
  CHECK(v1_duty_command(10, 0.0, band, t) == 0 && band);
  CHECK(v1_duty_command(20, 0.0, band, t) == 0 && band);
  CHECK(v1_duty_command(21, 0.0, band, t) == V1_DUTY_MIN && !band);
  CHECK(v1_duty_command(11, 0.0, band, t) == V1_DUTY_MIN && !band);
  CHECK(v1_duty_command(10, 0.0, band, t) == 0 && band);
  // ff = 0 なら比例だけ
  V1Tuning p;
  p.ff_count_s_per_percent = 0;
  bool b2 = false;
  CHECK(v1_duty_command(0, 600.0, b2, p) == 0);
  CHECK(v1_duty_command(500, 600.0, b2, p) == 10);
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
  CHECK(h.ax.duty_percent() == 0 && h.ax.pwm_permille() == 0);
  h.raw = -15;  // 出る側 20 までは再駆動しない（ヒステリシス）
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_percent() == 0);
  h.raw = -25;  // 出る側を超えた → 下限 7 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_percent() == -V1_DUTY_MIN);
  h.raw = -15;  // 駆動中は入る側 10 まで続ける
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_percent() == -V1_DUTY_MIN);
  h.raw = -10;
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_percent() == 0);
  h.raw = -800;  // 800 × 2 / 100 = 16 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_percent() == -16);
  h.raw = -2500;  // 50 % → 上限 20 %
  h.set_position(0);
  h.step();
  CHECK(h.ax.duty_percent() == -V1_DUTY_MAX);
  CHECK(h.ax.pwm_permille() == V1_DUTY_MAX * 10);
  // 位置制御が切れれば 0
  Harness::accepted(h.host_fault());
  CHECK(h.ax.duty_percent() == 0);
}

// ---- LOCAL_ESTOP ------------------------------------------------------------------

inline void test_local_estop_latches_and_blocks_reset_until_released() {
  Harness h;
  h.reach_holding();
  h.ax.set_local_estop(true, h.now);
  h.drain();
  CHECK(h.ax.state() == st::FAULT && h.ax.fault_code() == fc::LOCAL_ESTOP);
  CHECK(!h.ax.power_on_cmd() && !h.ax.brake_release_cmd() && h.ax.duty_percent() == 0);
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

inline void run_all() {
  test_boots_disarmed_with_safe_outputs();
  test_hello_answers_in_every_state();
  test_clock_sync_replies();
  test_config_sets_configured_flag_and_immediate_status();
  test_config_invalid_params_rejected();
  test_config_only_in_disarmed();
  test_request_id_zero_rejected();
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
  test_local_estop_latches_and_blocks_reset_until_released();
  test_local_estop_in_disarmed_and_during_other_fault();
  test_bad_frames_counted_and_unanswered();
  test_outbox_overflow_drops_status_first();
}

}  // namespace v1axis_test
