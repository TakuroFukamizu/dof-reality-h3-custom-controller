// moter-controler のホストテスト。GPIO・時計・NVS に依存しない純粋ロジックだけを対象にする。
#include <cstdio>
#include <string>

#include "auto_move.h"
#include "motor_profile.h"

namespace {

int g_pass = 0;
int g_fail = 0;

#define CHECK(cond)                                                      \
  do {                                                                   \
    if (cond) {                                                          \
      ++g_pass;                                                          \
    } else {                                                             \
      ++g_fail;                                                          \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
    }                                                                    \
  } while (0)

void test_motor_profile() {
  CHECK(std::string(p_end_label(1)) == "下限");
  CHECK(std::string(p_end_label(2)) == "下限");
  CHECK(std::string(p_end_label(3)) == "右端");
  CHECK(std::string(n_end_label(2)) == "上限");
  CHECK(std::string(n_end_label(3)) == "左端");
  CHECK(default_neutral(5556) == 2778);
  CHECK(profile_motor_ok(0) == false);
  CHECK(profile_motor_ok(1) == true);
  CHECK(profile_motor_ok(3) == true);
  CHECK(profile_motor_ok(4) == false);
  MotorProfile p;
  CHECK(p.sign == 1);
  CHECK(!p.calibrated);
  CHECK(p.stroke == 0);
  CHECK(p.motor == 1);
}


// ---- auto_move ----------------------------------------------------------

struct Sim {
  AutoMachine m;
  AutoInputs in{};
  int64_t raw = 0;
  Sim() {
    in.now_ms = 1000;
    in.relay_live = true;
    in.brake_released = true;
    in.raw_count = 0;
  }
  void step(uint32_t dt_ms, int64_t draw) {
    in.now_ms += dt_ms;
    raw += draw;
    in.raw_count = raw;
    m.tick(in);
    in.stop = false;
    in.estop = false;
  }
  void steps(int n, uint32_t dt_ms, int64_t draw) {
    for (int i = 0; i < n; ++i) step(dt_ms, draw);
  }
  bool request(AutoKind k, const char** err = nullptr) {
    const char* e = "";
    const bool ok = m.request(k, in, &e);
    if (err) *err = e;
    return ok;
  }
  // P 端へ当てて原点を取る定番の列。raw は −600 動いてから止まる。
  void home_p() {
    request(AutoKind::HomeP);
    steps(50, 20, -12);
    steps(6, 50, 0);
  }
};

void test_home_p_arrives() {
  Sim s;
  CHECK(s.request(AutoKind::HomeP));
  CHECK(s.m.duty_signed == HOME_DUTY);
  s.steps(50, 20, -12);
  CHECK(s.m.phase == AutoPhase::Moving);
  s.steps(6, 50, 0);
  CHECK(s.m.phase == AutoPhase::Done);
  CHECK(s.m.origin_valid);
  CHECK(s.m.position(s.raw) == 0);
  CHECK(s.m.duty_signed == 0);
  CHECK(std::string(s.m.reason) == "ok");
}

void test_home_p_stall_without_travel_fails() {
  Sim s;
  s.request(AutoKind::HomeP);
  s.steps(12, 50, 0);
  CHECK(s.m.phase == AutoPhase::Failed);
  CHECK(std::string(s.m.reason) == "stall");
  CHECK(!s.m.origin_valid);
  CHECK(s.m.duty_signed == 0);
}

Sim calibrated_sim();

void test_home_p_slow_counts_fails() {
  // 片相断線などで、カウントが止まらずに遅く進み続ける場合。到達として記録してはいけない。
  Sim s;
  s.request(AutoKind::HomeP);
  s.steps(30, 20, -1);  // 50 count/s を 600 ms
  CHECK(s.m.phase == AutoPhase::Failed);
  CHECK(std::string(s.m.reason) == "slow");
  CHECK(!s.m.origin_valid);
  CHECK(s.m.duty_signed == 0);
}

void test_home_p_slow_after_real_travel_is_arrival() {
  // 端で押し込みながらゆっくり止まる場合は、十分動いたあとなので到達でよい。
  Sim s;
  s.request(AutoKind::HomeP);
  s.steps(50, 20, -12);  // 600 count/s で 1 s
  s.steps(15, 20, -1);   // 50 count/s に落ちる
  CHECK(s.m.phase == AutoPhase::Done);
  CHECK(s.m.origin_valid);
}

void test_neutral_slow_counts_fails() {
  Sim s = calibrated_sim();
  s.request(AutoKind::Neutral);
  s.steps(30, 20, 1);  // 50 count/s
  CHECK(s.m.phase == AutoPhase::Failed);
  CHECK(std::string(s.m.reason) == "slow");
}

void test_home_p_near_end_short_travel_is_arrival() {
  Sim s;
  s.home_p();
  CHECK(s.m.phase == AutoPhase::Done);
  s.step(20, 100);  // 待機中に N 端側へ 100
  CHECK(s.m.position(s.raw) == 100);
  CHECK(s.request(AutoKind::HomeP));
  CHECK(s.m.duty_signed == HOME_DUTY);  // 端まで 100 なので低速
  s.step(20, -100);
  s.steps(6, 50, 0);
  CHECK(s.m.phase == AutoPhase::Done);
  CHECK(s.m.position(s.raw) == 0);
}

void test_sign_autodetect() {
  Sim s;
  s.request(AutoKind::HomeP);
  s.steps(50, 20, 12);  // 速度 + なのに raw が増える系統
  s.steps(6, 50, 0);
  CHECK(s.m.phase == AutoPhase::Done);
  CHECK(s.m.working_sign == -1);
  s.step(20, -50);
  CHECK(s.m.position(s.raw) == 50);
}

void test_home_n_measures_stroke_and_cal_saves() {
  Sim s;
  CHECK(s.request(AutoKind::Cal));
  CHECK(s.m.kind == AutoKind::Cal && s.m.cal_step == CalStep::ToP);
  CHECK(s.m.duty_signed == HOME_DUTY);
  s.steps(50, 20, -12);
  s.steps(6, 50, 0);
  CHECK(s.m.origin_valid);
  CHECK(s.m.cal_step == CalStep::ToN);
  CHECK(s.m.phase == AutoPhase::Moving);
  CHECK(s.m.duty_signed == -HOME_DUTY);  // ストローク未知なので低速
  s.steps(463, 20, 12);  // +5556
  s.steps(6, 50, 0);
  CHECK(s.m.profile.stroke == 5556);
  CHECK(s.m.profile.neutral == 2778);
  CHECK(s.m.profile.calibrated);
  CHECK(s.m.profile.sign == 1);
  CHECK(s.m.profile_dirty);
  CHECK(s.m.last_measured_stroke == 5556);
  CHECK(s.m.cal_step == CalStep::ToNeutral);
  CHECK(s.m.phase == AutoPhase::Moving);
  CHECK(s.m.duty_signed == AUTO_DUTY_MAX);  // 中立は P 端側 2778 下
  s.step(20, -2778);
  CHECK(s.m.phase == AutoPhase::Dwell);
  CHECK(s.m.duty_signed == 0);
  s.steps(4, 50, 0);
  CHECK(s.m.phase == AutoPhase::Done);
  CHECK(s.m.kind == AutoKind::Cal);
}

void test_home_n_rejected_without_origin() {
  Sim s;
  const char* err = "";
  CHECK(!s.request(AutoKind::HomeN, &err));
  CHECK(std::string(err) == "no_origin");
}

void test_neutral_rejected_uncalibrated() {
  Sim s;
  s.home_p();
  const char* err = "";
  CHECK(!s.request(AutoKind::Neutral, &err));
  CHECK(std::string(err) == "not_calibrated");
}

Sim calibrated_sim() {
  Sim s;
  s.m.profile.stroke = 5556;
  s.m.profile.neutral = 2778;
  s.m.profile.calibrated = true;
  s.m.origin_valid = true;
  s.m.origin_raw = 0;
  return s;
}

void test_neutral_p_control() {
  Sim s = calibrated_sim();
  CHECK(s.request(AutoKind::Neutral));
  CHECK(s.m.duty_signed == -AUTO_DUTY_MAX);  // 2778 上へ。N 端向きは −
  s.step(20, 2700);
  CHECK(s.m.duty_signed == -NEUTRAL_DUTY_MIN);  // err 78 → 0.78 % → 下限
  s.step(20, 100);  // 2800、err −22
  CHECK(s.m.duty_signed == NEUTRAL_DUTY_MIN);
  s.step(20, -22);  // 2778
  CHECK(s.m.phase == AutoPhase::Dwell);
  CHECK(s.m.duty_signed == 0);
  s.step(50, 12);  // 2790、バンド外
  CHECK(s.m.phase == AutoPhase::Moving);
  CHECK(s.m.duty_signed == NEUTRAL_DUTY_MIN);
  s.step(20, -15);  // 2775
  CHECK(s.m.phase == AutoPhase::Dwell);
  s.steps(4, 50, 0);
  CHECK(s.m.phase == AutoPhase::Done);
  CHECK(std::string(s.m.reason) == "ok");
}

void test_slow_zone() {
  Sim s = calibrated_sim();
  s.step(20, 1000);
  CHECK(s.request(AutoKind::HomeP));
  CHECK(s.m.duty_signed == AUTO_DUTY_MAX);
  s.step(20, -700);  // position 300
  CHECK(s.m.duty_signed == HOME_DUTY);
}

void test_cancel_reasons() {
  {
    Sim s; s.request(AutoKind::HomeP); s.steps(3, 20, -12);
    s.in.stop = true; s.step(20, -12);
    CHECK(s.m.phase == AutoPhase::Cancelled); CHECK(std::string(s.m.reason) == "stop"); CHECK(s.m.duty_signed == 0);
  }
  {
    Sim s; s.request(AutoKind::HomeP); s.steps(3, 20, -12);
    s.in.relay_live = false; s.step(20, -12);
    CHECK(s.m.phase == AutoPhase::Cancelled); CHECK(std::string(s.m.reason) == "relay");
  }
  {
    Sim s; s.request(AutoKind::HomeP); s.steps(3, 20, -12);
    s.in.brake_released = false; s.step(20, -12);
    CHECK(s.m.phase == AutoPhase::Cancelled); CHECK(std::string(s.m.reason) == "brake");
  }
  {
    Sim s; s.request(AutoKind::HomeP); s.steps(3, 20, -12);
    s.in.manual_speed_nonzero = true; s.step(20, -12);
    CHECK(s.m.phase == AutoPhase::Cancelled); CHECK(std::string(s.m.reason) == "manual");
  }
  {
    Sim s; s.home_p(); CHECK(s.m.origin_valid);
    s.request(AutoKind::HomeN); s.steps(3, 20, 12);
    s.in.estop = true; s.step(20, 12);
    CHECK(s.m.phase == AutoPhase::Cancelled); CHECK(std::string(s.m.reason) == "estop");
    CHECK(!s.m.origin_valid);
  }
}

void test_travel_and_timeout() {
  {
    Sim s; s.request(AutoKind::HomeP);
    s.steps(1001, 20, -12);  // −12012
    CHECK(s.m.phase == AutoPhase::Failed); CHECK(std::string(s.m.reason) == "travel"); CHECK(s.m.duty_signed == 0);
  }
  {
    Sim s; s.request(AutoKind::HomeP);
    s.steps(2251, 20, -4);  // 45020 ms、200 count/s で遅すぎず、移動 9004 < 12000
    CHECK(s.m.phase == AutoPhase::Failed); CHECK(std::string(s.m.reason) == "timeout");
  }
}

void test_can_start_conditions() {
  const char* err = "";
  {
    Sim s; s.in.relay_live = false;
    CHECK(!s.request(AutoKind::HomeP, &err)); CHECK(std::string(err) == "not_live");
  }
  {
    Sim s; s.in.brake_released = false;
    CHECK(!s.request(AutoKind::HomeP, &err)); CHECK(std::string(err) == "brake");
  }
  {
    Sim s; s.in.manual_speed_nonzero = true;
    CHECK(!s.request(AutoKind::HomeP, &err)); CHECK(std::string(err) == "manual");
  }
  {
    Sim s; s.request(AutoKind::HomeP); s.steps(3, 20, -12);
    CHECK(!s.request(AutoKind::HomeP, &err)); CHECK(std::string(err) == "busy");
  }
}

void test_set_profile_invalidates_origin() {
  Sim s; s.home_p(); CHECK(s.m.origin_valid);
  MotorProfile p; p.motor = 2;
  s.m.set_profile(p);
  CHECK(!s.m.origin_valid);
  CHECK(s.m.profile.motor == 2);
  CHECK(s.m.working_sign == p.sign);
}

}  // namespace

int main() {
  test_motor_profile();
  test_home_p_arrives();
  test_home_p_stall_without_travel_fails();
  test_home_p_near_end_short_travel_is_arrival();
  test_home_p_slow_counts_fails();
  test_home_p_slow_after_real_travel_is_arrival();
  test_neutral_slow_counts_fails();
  test_sign_autodetect();
  test_home_n_measures_stroke_and_cal_saves();
  test_home_n_rejected_without_origin();
  test_neutral_rejected_uncalibrated();
  test_neutral_p_control();
  test_slow_zone();
  test_cancel_reasons();
  test_travel_and_timeout();
  test_can_start_conditions();
  test_set_profile_invalidates_origin();
  std::printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
