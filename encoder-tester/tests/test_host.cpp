#include "command_parse.h"
#include "config.h"
#include "drive_logic.h"
#include "measure_math.h"
#include "relay_trace.h"
#include "settings.h"
#include "visual_session.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace {

int g_failed = 0;
int g_passed = 0;

void check(bool ok, const char* file, int line, const char* text) {
  if (ok) {
    ++g_passed;
    return;
  }
  std::printf("FAIL %s:%d %s\n", file, line, text);
  ++g_failed;
}

#define CHECK(cond) check(static_cast<bool>(cond), __FILE__, __LINE__, #cond)

bool near(double left, double right) {
  const double delta = left - right;
  return (delta < 0 ? -delta : delta) < 1e-6;
}

DriveSensors sense(uint64_t now_ms, int64_t count = 0, int count_per_s = 0) {
  DriveSensors sensors{};
  sensors.now_ms = now_ms;
  sensors.count = count;
  sensors.count_per_s = count_per_s;
  sensors.ui_fresh = true;
  sensors.usb_authority = false;
  sensors.lease_fresh = true;
  return sensors;
}

void fill_test_settings(OperatorSettings* settings) {
  *settings = settings_unconfigured();
  const char* rows[][2] = {
      {"MAX_DUTY_PERCENT", "30"},       {"MAX_RUN_MS", "1000"},
      {"MAX_TRAVEL_COUNTS", "5000"},    {"POWER_SETTLE_MS", "100"},
      {"BRAKE_RELEASE_WAIT_MS", "50"},  {"BRAKE_APPLY_WAIT_MS", "40"},
      {"STOP_SPEED_THRESHOLD", "5"},    {"STOP_CONFIRM_MS", "30"},
      {"STOP_TIMEOUT_MS", "200"},       {"READY_HOLD_MS", "500"},
      {"START_GRACE_MS", "80"},         {"MAX_COUNT_PER_S", "10000"},
      {"UI_TIMEOUT_MS", "500"},         {"LEASE_MS", "1000"},
      {"JOG_STALE_MS", "150"},
  };
  for (unsigned i = 0; i < sizeof(rows) / sizeof(rows[0]); ++i) {
    const SettingsResult result = settings_apply(settings, rows[i][0], rows[i][1]);
    CHECK(result.ok);
  }
}

DriveConfig good_config() {
  OperatorSettings settings;
  fill_test_settings(&settings);
  DriveConfig config{};
  settings_to_drive(settings, true, &config);
  return config;
}

void arm_to_ready(DriveMachine* machine, const DriveConfig& config, uint64_t* now) {
  DriveCommand command{};
  command.arm = true;
  machine->tick(config, command, sense(*now));
  command = DriveCommand{};
  *now += static_cast<uint64_t>(config.power_settle_ms);
  machine->tick(config, command, sense(*now));
  *now += static_cast<uint64_t>(config.brake_release_wait_ms);
  machine->tick(config, command, sense(*now));
}

void test_settings_and_units() {
  CHECK(pwm_raw_from_percent(0) == 0);
  CHECK(pwm_raw_from_percent(1) == 10);
  CHECK(pwm_raw_from_percent(50) == 511);
  CHECK(pwm_raw_from_percent(100) == 1023);
  CHECK(pwm_timing_ok());
  CHECK(drive_pins_unique());
  CHECK(dir_level_for(1) == 0);
  CHECK(dir_level_for(-1) == 1);

  OperatorSettings settings = settings_unconfigured();
  CHECK(!settings_limits_ok(settings));
  char missing[400];
  settings_missing(settings, missing, sizeof(missing));
  CHECK(std::strstr(missing, "MAX_DUTY_PERCENT") != nullptr);
  CHECK(std::strstr(missing, "JOG_STALE_MS") != nullptr);

  const SettingsResult low = settings_apply(&settings, "MAX_DUTY_PERCENT", "0");
  CHECK(!low.ok);
  CHECK(settings.max_duty_percent == -1);
  const SettingsResult high = settings_apply(&settings, "MAX_DUTY_PERCENT", "101");
  CHECK(!high.ok);
  CHECK(settings_apply(&settings, "STOP_CONFIRM_MS", "30").ok);
  const SettingsResult tight = settings_apply(&settings, "STOP_TIMEOUT_MS", "30");
  CHECK(!tight.ok);
  CHECK(settings.stop_timeout_ms == -1);
  CHECK(settings.stop_confirm_ms == 30);
  CHECK(std::strcmp(settings_apply(&settings, "NO_SUCH", "1").error, "unknown_key") == 0);
  CHECK(settings_apply(&settings, "ENCODER_SIGN", "-1").wrote_sign);
  CHECK(settings_apply(&settings, "ENCODER_SIGN", "-1").sign == -1);
  CHECK(settings_apply(&settings, "GLITCH_NS", "0").wrote_glitch);
  CHECK(!settings_apply(&settings, "JOG_DUTY", "0").ok);

  fill_test_settings(&settings);
  CHECK(settings_limits_ok(settings));
  settings_missing(settings, missing, sizeof(missing));
  CHECK(missing[0] == '\0');
  CHECK(settings.jog_duty == -1);
}

void test_drive() {
  const DriveConfig config = good_config();
  CHECK(config.limits_ok);
  CHECK(config.electrical_ok);

  DriveMachine locked;
  locked.reset();
  DriveConfig unset = config;
  unset.limits_ok = false;
  DriveCommand arm{};
  arm.arm = true;
  locked.tick(unset, arm, sense(0));
  CHECK(locked.st.state == DriveState::Disarmed);
  CHECK(std::strcmp(locked.reject, "config") == 0);
  CHECK(!locked.st.power_on_cmd);
  CHECK(!locked.st.brake_release_cmd);
  CHECK(locked.st.applied_duty_percent == 0);

  DriveMachine machine;
  machine.reset();
  DriveCommand run{};
  run.run = true;
  run.run_duty = 10;
  run.run_dir = 1;
  run.run_duration_ms = 100;
  machine.tick(config, run, sense(0));
  CHECK(machine.st.state == DriveState::Disarmed);
  CHECK(std::strcmp(machine.reject, "not_ready") == 0);

  uint64_t now = 0;
  arm_to_ready(&machine, config, &now);
  CHECK(machine.st.state == DriveState::Ready);
  CHECK(machine.st.power_on_cmd);
  CHECK(machine.st.brake_release_cmd);
  CHECK(machine.st.applied_duty_percent == 0);

  DriveCommand both{};
  both.jog_pos = true;
  both.jog_neg = true;
  both.jog_duty = 10;
  machine.tick(config, both, sense(now));
  CHECK(machine.st.state == DriveState::Ready);
  CHECK(std::strcmp(machine.reject, "both_jog") == 0);

  DriveCommand bad{};
  bad.jog_pos = true;
  bad.jog_duty = 0;
  machine.tick(config, bad, sense(now));
  CHECK(std::strcmp(machine.reject, "duty") == 0);
  bad.jog_duty = -3;
  machine.tick(config, bad, sense(now));
  CHECK(std::strcmp(machine.reject, "duty") == 0);
  bad.jog_duty = 31;
  machine.tick(config, bad, sense(now));
  CHECK(std::strcmp(machine.reject, "duty") == 0);
  CHECK(machine.st.trial_id == 0);

  DriveCommand bad_run{};
  bad_run.run = true;
  bad_run.run_duty = 10;
  bad_run.run_dir = 0;
  bad_run.run_duration_ms = 100;
  machine.tick(config, bad_run, sense(now));
  CHECK(std::strcmp(machine.reject, "run_arg") == 0);
  bad_run.run_dir = 1;
  bad_run.run_duration_ms = 0;
  machine.tick(config, bad_run, sense(now));
  CHECK(std::strcmp(machine.reject, "run_arg") == 0);
  bad_run.run_duration_ms = 1001;
  machine.tick(config, bad_run, sense(now));
  CHECK(std::strcmp(machine.reject, "run_arg") == 0);
  CHECK(machine.st.state == DriveState::Ready);

  const int64_t origin_before = machine.st.run_origin_count;
  DriveCommand jog{};
  jog.jog_pos = true;
  jog.jog_duty = 10;
  machine.tick(config, jog, sense(now, 1000));
  CHECK(machine.st.state == DriveState::Running);
  CHECK(machine.st.direction == 1);
  CHECK(machine.st.applied_duty_percent == 10);
  CHECK(machine.st.trial_id == 1);
  CHECK(machine.st.run_origin_count == 1000);
  CHECK(origin_before != 1000 || machine.st.run_origin_count == 1000);
  jog = DriveCommand{};
  machine.tick(config, jog, sense(now + 10, 1100));
  CHECK(machine.st.state == DriveState::Stopping);
  CHECK(machine.st.stop_reason == StopReason::JogRelease);
  CHECK(machine.st.direction == 1);
  CHECK(machine.st.applied_duty_percent == 0);
  CHECK(machine.st.count_at_pwm_off == 1100);

  machine.tick(config, jog, sense(now + 10, 1120, 0));
  CHECK(machine.st.state == DriveState::Stopping);
  CHECK(machine.st.slow_timing);
  machine.tick(config, jog, sense(now + 40, 1130, 0));
  CHECK(machine.st.state == DriveState::BrakeApply);
  CHECK(machine.st.coast_valid);
  CHECK(machine.st.coast_counts == 30);
  CHECK(!machine.st.brake_release_cmd);
  CHECK(machine.st.power_on_cmd);
  machine.tick(config, jog, sense(now + 80, 1130, 0));
  CHECK(machine.st.state == DriveState::PowerOff);
  machine.tick(config, jog, sense(now + 81, 1130, 0));
  CHECK(machine.st.state == DriveState::Disarmed);
  CHECK(!machine.st.power_on_cmd);
  CHECK(!machine.st.brake_release_cmd);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  DriveCommand timed{};
  timed.run = true;
  timed.run_duty = 10;
  timed.run_dir = 1;
  timed.run_duration_ms = 100;
  machine.tick(config, timed, sense(now, 50));
  CHECK(machine.st.state == DriveState::Running);
  machine.tick(config, DriveCommand{}, sense(now + 50, 50));
  CHECK(machine.st.state == DriveState::Running);
  machine.tick(config, DriveCommand{}, sense(now + 100, 50));
  CHECK(machine.st.stop_reason == StopReason::Duration);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  timed.run_duration_ms = 500;
  machine.tick(config, timed, sense(now, 7));
  machine.tick(config, DriveCommand{}, sense(now + 80, 7));
  CHECK(machine.st.state == DriveState::Stopping);
  CHECK(machine.st.stop_reason == StopReason::EncoderStall);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  timed.run_duration_ms = 800;
  machine.tick(config, timed, sense(now, 0));
  const int64_t display_offset = 400;
  const int64_t raw_count = 5000;
  CHECK(displayed_count(raw_count, display_offset) == 4600);
  machine.tick(config, DriveCommand{}, sense(now + 10, raw_count, 10));
  CHECK(machine.st.stop_reason == StopReason::Travel);
  CHECK(machine.st.run_origin_count == 0);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  jog.jog_pos = true;
  jog.jog_duty = 10;
  machine.tick(config, jog, sense(now, 0));
  DriveCommand reverse{};
  reverse.jog_neg = true;
  machine.tick(config, reverse, sense(now + 5, 10));
  CHECK(machine.st.stop_reason == StopReason::ReverseRequest);
  CHECK(machine.st.direction == 1);
  CHECK(machine.st.applied_duty_percent == 0);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  machine.tick(config, jog, sense(now, 0));
  machine.tick(config, both, sense(now + 5, 10));
  CHECK(machine.st.stop_reason == StopReason::BothJog);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  machine.tick(config, jog, sense(now, 0, 0));
  machine.tick(config, jog, sense(now + 5, 20, 10001));
  CHECK(machine.st.state == DriveState::Fault);
  CHECK(machine.st.stop_reason == StopReason::Overspeed);
  CHECK(!machine.st.power_on_cmd);
  CHECK(!machine.st.brake_release_cmd);
  CHECK(machine.st.applied_duty_percent == 0);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  DriveSensors leased = sense(now, 0);
  leased.usb_authority = true;
  leased.lease_fresh = false;
  machine.tick(config, DriveCommand{}, leased);
  CHECK(machine.st.stop_reason == StopReason::Lease);
  CHECK(machine.st.state == DriveState::Stopping);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  DriveSensors stale_ui = sense(now);
  stale_ui.ui_fresh = false;
  machine.tick(config, DriveCommand{}, stale_ui);
  CHECK(machine.st.stop_reason == StopReason::UiTimeout);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  machine.tick(config, DriveCommand{}, sense(now + 500));
  CHECK(machine.st.stop_reason == StopReason::ReadyTimeout);

  DriveMachine early;
  early.reset();
  arm = DriveCommand{};
  arm.arm = true;
  early.tick(config, arm, sense(0));
  CHECK(early.st.state == DriveState::PowerWait);
  CHECK(early.st.power_on_cmd);
  CHECK(!early.st.brake_release_cmd);
  DriveCommand stop{};
  stop.stop = true;
  early.tick(config, stop, sense(10));
  CHECK(early.st.state == DriveState::PowerOff);
  CHECK(!early.st.brake_release_cmd);
  early.tick(config, DriveCommand{}, sense(11));
  CHECK(early.st.state == DriveState::Disarmed);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  machine.tick(config, jog, sense(now, 0, 100));
  DriveCommand estop{};
  estop.estop = true;
  machine.tick(config, estop, sense(now + 5, 10, 100));
  CHECK(machine.st.state == DriveState::Fault);
  CHECK(!machine.st.power_on_cmd);
  const int trial = machine.st.trial_id;
  DriveCommand reset{};
  reset.fault_reset = true;
  reset.arm = true;
  machine.tick(config, reset, sense(now + 6));
  CHECK(machine.st.state == DriveState::Disarmed);
  CHECK(machine.st.trial_id == trial);
  CHECK(!machine.st.power_on_cmd);
  arm.arm = true;
  machine.tick(config, arm, sense(now + 7));
  CHECK(machine.st.state == DriveState::PowerWait);

  now = 0;
  machine.reset();
  arm_to_ready(&machine, config, &now);
  machine.tick(config, jog, sense(now, 0, 100));
  machine.tick(config, stop, sense(now + 1, 10, 100));
  CHECK(machine.st.state == DriveState::Stopping);
  machine.tick(config, DriveCommand{}, sense(now + 201, 80, 100));
  CHECK(machine.st.state == DriveState::Fault);
  CHECK(machine.st.stop_reason == StopReason::StopTimeout);

  CHECK(jog_is_fresh(200, 100, 150));
  CHECK(!jog_is_fresh(251, 100, 150));
  CHECK(!jog_is_fresh(200, 0, 150));
}

void test_math_and_visual() {
  const CalResult angle = cal_from_angle(1200, 90.0);
  CHECK(angle.ok);
  CHECK(near(angle.counts_per_degree, 1200.0 / 90.0));
  CHECK(near(angle.counts_per_measured_rev_x4, 4800.0));
  CHECK(!cal_from_angle(1200, 0.0).ok);
  CHECK(!cal_from_angle(1200, -10.0).ok);
  CHECK(!cal_from_angle(1200, std::numeric_limits<double>::quiet_NaN()).ok);
  CHECK(!cal_from_angle(0, 90.0).ok);
  CHECK(std::strcmp(cal_from_angle(0, 90.0).error, "zero_count") == 0);

  const CalResult revs = cal_from_revolutions(9600, 2.0);
  CHECK(revs.ok);
  CHECK(near(revs.counts_per_measured_rev_x4, 4800.0));
  CHECK(!cal_from_revolutions(9600, 0.0).ok);

  double ppr = 0;
  CHECK(ppr_from_encoder_shaft("encoder", 4800.0, &ppr));
  CHECK(near(ppr, 1200.0));
  CHECK(!ppr_from_encoder_shaft("output", 4800.0, &ppr));
  CHECK(!ppr_from_encoder_shaft("motor", 4800.0, &ppr));
  CHECK(!ppr_from_encoder_shaft(nullptr, 4800.0, &ppr));

  CountExtender extender;
  CHECK(extender.sample(2147483640) == 2147483640LL);
  const int32_t wrapped =
      static_cast<int32_t>(static_cast<uint32_t>(2147483640) + 20u);
  CHECK(extender.sample(wrapped) == 2147483640LL + 20);
  extender.reset(-2147483640);
  const int32_t wrapped_down =
      static_cast<int32_t>(static_cast<uint32_t>(static_cast<int32_t>(-2147483640)) - 20u);
  CHECK(extender.sample(wrapped_down) == -2147483640LL - 20);

  VelocityWindow window;
  window.push(0, 0);
  CHECK(!window.window_ready);
  window.push(480, 100000);
  CHECK(window.window_ready);
  CHECK(window.count_per_s == 4800);

  StopStability stability;
  CHECK(!stability.update(false, 0, true, 5, 30, 0));
  CHECK(!stability.update(true, 0, false, 5, 30, 0));
  CHECK(!stability.update(true, 0, true, 5, 30, 0));
  CHECK(stability.update(true, 0, true, 5, 30, 30));

  const char* error = "";
  VisualSession session;
  CHECK(!session.configure("platform", "mark", 1, 1, 1000, 1000, &error));
  CHECK(!session.configure("output", "mark", 1, 0, 1000, 1000, &error));
  CHECK(session.configure("output", "fixed mark", 1, 1, 5000, 20000, &error));
  CHECK(!session.capture_reference(false, true, 1000, 10, 3, &error));
  CHECK(!session.capture_reference(true, false, 1000, 10, 3, &error));
  CHECK(session.capture_reference(true, true, 1000, 10, 3, &error));
  CHECK(session.state == VisualState::ReferenceCaptured);

  DriveMachine safety;
  safety.reset();
  safety.st.run_origin_count = 42;
  safety.st.trial_id = 7;
  CHECK(safety.st.run_origin_count == 42);
  CHECK(safety.st.trial_id == 7);
  CHECK(session.start_count == 1000);

  session.note_motion(1, 20, 2000, 30);
  session.note_motion(-1, 20, 1500, 40);
  CHECK(session.reversal);
  CHECK(session.finish(true, true, 5800, 50, 9, &error));
  CHECK(session.state == VisualState::Result);
  CHECK(session.last.valid);
  CHECK(std::strcmp(session.last.method, "visual_stop_align") == 0);
  CHECK(session.last.signed_delta_x4 == 4800);
  CHECK(near(session.last.counts_per_measured_shaft_rev_x4, 4800.0));
  CHECK(session.last.reversal_detected);

  CHECK(session.configure("output", "fixed mark", -1, 1, 5000, 20000, &error));
  CHECK(session.capture_reference(true, true, 5800, 60, 1, &error));
  CHECK(session.finish(true, true, 1000, 70, 2, &error));
  CHECK(session.last.signed_delta_x4 == -4800);
  CHECK(near(session.last.counts_per_measured_shaft_rev_x4, 4800.0));

  CHECK(session.configure("encoder", "shaft mark", 1, 2, 8000, 20000, &error));
  CHECK(session.capture_reference(true, true, 1000, 80, 1, &error));
  CHECK(session.finish(true, true, 10600, 90, 4, &error));
  CHECK(session.last.signed_delta_x4 == 9600);
  CHECK(near(session.last.counts_per_measured_shaft_rev_x4, 4800.0));
  CHECK(ppr_from_encoder_shaft(session.last.shaft, session.last.counts_per_measured_shaft_rev_x4, &ppr));

  CHECK(session.configure("output", "mark", 1, 1, 5000, 20000, &error));
  CHECK(session.capture_reference(true, true, 1000, 100, 1, &error));
  CHECK(!session.finish(true, true, 1000, 110, 1, &error));
  CHECK(std::strcmp(error, "no_movement") == 0);
  CHECK(session.state == VisualState::ReferenceCaptured);

  session.note_motion(1, 6000, 1100, 120);
  CHECK(session.state == VisualState::Invalid);
  CHECK(!session.last.valid);
  CHECK(std::strcmp(session.last.invalid_reason, "session_limit") == 0);
  CHECK(!session.finish(true, true, 2000, 130, 2, &error));

  CHECK(session.configure("output", "mark", 1, 1, 5000, 100, &error));
  CHECK(session.capture_reference(true, true, 1000, 140, 1, &error));
  session.note_motion(1, 10, 1201, 150);
  CHECK(session.state == VisualState::Invalid);

  FlyingMark flying;
  CHECK(!flying.start(false, 10, 1, &error));
  CHECK(flying.start(true, 1000, 1, &error));
  flying.note_stopped(1400, "output");
  CHECK(!flying.has_result);
  CHECK(flying.end_mark(true, 5800, 2, &error));
  flying.note_stopped(5900, "output");
  CHECK(flying.has_result);
  CHECK(std::strcmp(flying.last.method, "visual_flying_mark") == 0);
  CHECK(flying.last.signed_delta_x4 == 4800);
  CHECK(flying.stopped_count == 5900);
  CHECK(flying.last.end_count_x4 == 5800);
  CHECK(flying.last.valid);

  FlyingMark empty;
  CHECK(empty.start(true, 10, 1, &error));
  CHECK(empty.end_mark(true, 10, 2, &error));
  empty.note_stopped(25, "encoder");
  CHECK(empty.has_result);
  CHECK(!empty.last.valid);
  CHECK(std::strcmp(empty.last.invalid_reason, "no_movement") == 0);
}

int step_quad(int a, int b, int prev_a, int prev_b, int count) {
  if (a != prev_a) {
    int action = (a == 1) ? -1 : 1;
    if (b == 0) {
      action = -action;
    }
    count += action;
  }
  if (b != prev_b) {
    int action = (b == 1) ? 1 : -1;
    if (a == 0) {
      action = -action;
    }
    count += action;
  }
  return count;
}

int cycle_from(const int (*steps)[2], int count) {
  int prev_a = steps[0][0];
  int prev_b = steps[0][1];
  for (int i = 1; i < 5; ++i) {
    count = step_quad(steps[i][0], steps[i][1], prev_a, prev_b, count);
    prev_a = steps[i][0];
    prev_b = steps[i][1];
  }
  return count;
}

TraceSample trace_row(uint8_t state, int64_t count, int pwm) {
  TraceSample row{};
  row.count_total = count;
  row.pwm_raw = static_cast<int16_t>(pwm);
  row.state = state;
  return row;
}

void test_relay_trace() {
  TraceSession<8> trace;
  TraceSample row = trace_row(0, 0, 0);
  CHECK(!trace.on_tick(0, false, true, false, 1000, row));
  CHECK(trace.count == 0);

  row = trace_row(1, 2, 0);
  row.power_cmd = 1;
  CHECK(trace.on_tick(1, true, false, true, 2000, row));
  trace.on_post(2001, 1, row);
  CHECK(trace.count == 2);
  CHECK(trace.samples[0].mark == static_cast<uint8_t>(TraceMark::Tick));
  CHECK(trace.samples[0].delta == 0);
  CHECK(trace.samples[0].count_total == 2);
  CHECK(trace.samples[1].mark == static_cast<uint8_t>(TraceMark::Post));
  CHECK(trace.samples[1].delta == 0);
  CHECK(trace.samples[0].pwm_raw == 0);

  row = trace_row(1, -258, 0);
  CHECK(!trace.on_tick(1, true, false, false, 3000, row));
  CHECK(trace.samples[2].delta == -260);
  CHECK(trace.samples[2].pwm_raw == 0);

  row = trace_row(0, -258, 0);
  CHECK(!trace.on_tick(0, false, true, false, 4000, row));
  CHECK(trace.recording);
  CHECK(!trace.finished);
  const int before_tail = trace.count;
  row = trace_row(0, -258, 0);
  CHECK(!trace.on_tick(0, false, true, false, 4000 + 2000000ULL, row));
  CHECK(trace.finished);
  CHECK(!trace.recording);
  CHECK(trace.count == before_tail + 1);
  const int frozen = trace.count;
  row = trace_row(1, -300, 5);
  CHECK(!trace.on_tick(1, true, false, true, 9000000, row));
  CHECK(trace.count == frozen);
  CHECK(trace.samples[frozen - 1].pwm_raw == 0);

  TraceSession<2> full;
  row = trace_row(1, 1, 0);
  CHECK(full.on_tick(1, true, false, true, 10, row));
  full.on_post(11, 1, row);
  CHECK(full.count == 2);
  CHECK(!full.on_tick(1, true, false, false, 20, row));
  CHECK(full.overflow);
  CHECK(full.finished);
  CHECK(full.count == 2);
}

void test_quadrature_and_commands() {
  const int a_leading[5][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
  const int b_leading[5][2] = {{0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}};
  CHECK(cycle_from(a_leading, 0) == 4);
  CHECK(cycle_from(b_leading, 0) == -4);

  ParsedCommand run = parse_command("RUN duty_pct=5 direction=positive duration_ms=100");
  CHECK(run.ok);
  CHECK(run.kind == CmdKind::Run);
  CHECK(run.duty_pct == 5);
  CHECK(run.direction == 1);
  CHECK(run.duration_ms == 100);
  ParsedCommand reverse = parse_command("run duty_pct=12 direction=negative duration_ms=40");
  CHECK(reverse.ok);
  CHECK(reverse.direction == -1);

  ParsedCommand visual = parse_command(
      "VISUAL CONFIG shaft=output reference=\"tape mark\" direction=positive revolutions=1 "
      "max_drive_ms=5000 max_travel_counts=20000");
  CHECK(visual.ok);
  CHECK(std::strcmp(visual.shaft, "output") == 0);
  CHECK(std::strcmp(visual.reference, "tape mark") == 0);
  CHECK(visual.revolutions == 1);
  CHECK(visual.max_drive_ms == 5000);
  CHECK(!parse_command("VISUAL CONFIG shaft=output revolutions=1 direction=positive").ok);

  ParsedCommand set = parse_command("SET max_duty_percent 20");
  CHECK(set.ok);
  CHECK(std::strcmp(set.key, "MAX_DUTY_PERCENT") == 0);
  CHECK(std::strcmp(set.value, "20") == 0);
  CHECK(parse_command("POWER ON").kind == CmdKind::PowerOn);
  CHECK(parse_command("BRAKE RELEASE").kind == CmdKind::BrakeRelease);
  CHECK(parse_command("FAULT RESET").kind == CmdKind::FaultReset);
  CHECK(parse_command("ZERO DISPLAY").kind == CmdKind::ZeroDisplay);
  CHECK(parse_command("AUTHORITY USB").kind == CmdKind::AuthorityUsb);
  CHECK(parse_command("STREAM OFF").kind == CmdKind::StreamOff);
  CHECK(parse_command("MARK END").kind == CmdKind::MarkEnd);
  const ParsedCommand angle = parse_command("CAL ANGLE 90");
  CHECK(angle.kind == CmdKind::CalAngle);
  CHECK(near(angle.number, 90.0));
  const ParsedCommand nan = parse_command("CAL ANGLE nan");
  CHECK(nan.ok);
  CHECK(!std::isfinite(nan.number));
  CHECK(parse_command("NOPE").kind == CmdKind::Unknown);
  CHECK(!parse_command("").ok);
}

}  // namespace

int main() {
  test_settings_and_units();
  test_drive();
  test_math_and_visual();
  test_relay_trace();
  test_quadrature_and_commands();
  if (g_failed != 0) {
    std::printf("%d failed, %d passed\n", g_failed, g_passed);
    return 1;
  }
  std::printf("%d passed\n", g_passed);
  return 0;
}
