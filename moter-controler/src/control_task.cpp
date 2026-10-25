#include "control_task.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

#include "config.h"
#include "encoder_pcnt.h"
#include "motor_output.h"

namespace {

struct Inputs {
  bool relay_live = false;
  bool brake_released = false;
  int manual_speed = 0;
  bool stop_pulse = false;
  bool estop_pulse = false;
  bool request_pulse = false;
  AutoKind request = AutoKind::Idle;
  bool profile_pulse = false;
  MotorProfile profile{};
  bool clear_dirty_pulse = false;
};

constexpr uint32_t kPeriodMs = 5;
constexpr uint32_t kVelocityWindowMs = 100;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Inputs g_in;
ControlSnapshot g_snap;
MotorProfile g_initial;

void copy_text(char* dest, size_t size, const char* text) {
  strncpy(dest, text, size - 1);
  dest[size - 1] = '\0';
}

void task_main(void*) {
  AutoMachine machine;
  machine.set_profile(g_initial);
  int64_t vel_anchor_count = 0;
  uint32_t vel_anchor_ms = millis();
  int count_per_s = 0;
  uint32_t request_seq = 0;
  char request_error[16] = "";
  uint32_t ticks = 0;

  for (;;) {
    Inputs in;
    portENTER_CRITICAL(&g_mux);
    in = g_in;
    g_in.stop_pulse = false;
    g_in.estop_pulse = false;
    g_in.request_pulse = false;
    g_in.profile_pulse = false;
    g_in.clear_dirty_pulse = false;
    portEXIT_CRITICAL(&g_mux);

    if (in.profile_pulse) {
      machine.set_profile(in.profile);
    }
    if (in.clear_dirty_pulse) {
      machine.profile_dirty = false;
    }

    const uint32_t now = millis();
    const int64_t raw = encoder_count();
    AutoInputs ai;
    ai.now_ms = now;
    ai.raw_count = raw;
    ai.relay_live = in.relay_live;
    ai.brake_released = in.brake_released;
    ai.manual_speed_nonzero = in.manual_speed != 0;
    ai.stop = in.stop_pulse;
    ai.estop = in.estop_pulse;

    if (in.estop_pulse) {
      machine.invalidate_origin();  // 自動移動中でなくても非常停止で原点は無効
    }
    if (in.request_pulse) {
      const char* err = "";
      request_seq += 1;
      if (machine.request(in.request, ai, &err)) {
        request_error[0] = '\0';
      } else {
        copy_text(request_error, sizeof(request_error), err);
      }
    }
    machine.tick(ai);

    const int out = machine.active() ? machine.duty_signed : (in.relay_live ? in.manual_speed : 0);
    motor_output_write(out);

    if (now - vel_anchor_ms >= kVelocityWindowMs) {
      const int64_t dc = raw - vel_anchor_count;
      count_per_s = static_cast<int>((dc * 1000) / static_cast<int64_t>(now - vel_anchor_ms));
      vel_anchor_count = raw;
      vel_anchor_ms = now;
    }

    ControlSnapshot snap;
    snap.raw_count = raw;
    snap.origin_valid = machine.origin_valid;
    snap.position = machine.origin_valid ? machine.position(raw) : static_cast<int32_t>(raw * machine.working_sign);
    snap.count_per_s = count_per_s;
    snap.a_level = encoder_a_level();
    snap.b_level = encoder_b_level();
    snap.applied_speed = motor_output_applied();
    snap.kind = machine.kind;
    snap.phase = machine.phase;
    snap.cal_step = machine.cal_step;
    copy_text(snap.reason, sizeof(snap.reason), machine.reason);
    snap.profile = machine.profile;
    snap.profile_dirty = machine.profile_dirty;
    snap.last_measured_stroke = machine.last_measured_stroke;
    snap.working_sign = machine.working_sign;
    snap.request_seq = request_seq;
    copy_text(snap.request_error, sizeof(snap.request_error), request_error);
    ticks += 1;
    snap.tick_count = ticks;

    portENTER_CRITICAL(&g_mux);
    g_snap = snap;
    portEXIT_CRITICAL(&g_mux);

    vTaskDelay(pdMS_TO_TICKS(kPeriodMs));
  }
}

}  // namespace

void control_begin(const MotorProfile& initial) {
  g_initial = initial;
  g_snap.profile = initial;
  g_snap.working_sign = initial.sign;
  xTaskCreatePinnedToCore(task_main, "control", 8192, nullptr, 5, nullptr, 1);
}

void control_set_relay_live(bool live) {
  portENTER_CRITICAL(&g_mux);
  g_in.relay_live = live;
  portEXIT_CRITICAL(&g_mux);
}

void control_set_brake_released(bool released) {
  portENTER_CRITICAL(&g_mux);
  g_in.brake_released = released;
  portEXIT_CRITICAL(&g_mux);
}

void control_set_manual_speed(int percent) {
  portENTER_CRITICAL(&g_mux);
  g_in.manual_speed = percent;
  portEXIT_CRITICAL(&g_mux);
}

void control_pulse_stop() {
  portENTER_CRITICAL(&g_mux);
  g_in.stop_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_pulse_estop() {
  portENTER_CRITICAL(&g_mux);
  g_in.estop_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_request(AutoKind kind) {
  portENTER_CRITICAL(&g_mux);
  g_in.request = kind;
  g_in.request_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_set_profile(const MotorProfile& p) {
  portENTER_CRITICAL(&g_mux);
  g_in.profile = p;
  g_in.profile_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

void control_clear_profile_dirty() {
  portENTER_CRITICAL(&g_mux);
  g_in.clear_dirty_pulse = true;
  portEXIT_CRITICAL(&g_mux);
}

ControlSnapshot control_snapshot() {
  ControlSnapshot snap;
  portENTER_CRITICAL(&g_mux);
  snap = g_snap;
  portEXIT_CRITICAL(&g_mux);
  return snap;
}
