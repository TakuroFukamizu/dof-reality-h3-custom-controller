#pragma once

#include "drive_logic.h"
#include "measure_math.h"

#include <stdint.h>

struct TrialRecord {
  bool used;
  int id;
  int duty_percent;
  int direction;
  int requested_duration_ms;
  uint64_t start_us;
  uint64_t pwm_off_us;
  uint64_t stop_us;
  int64_t start_count;
  int64_t pwm_off_count;
  int64_t stop_count;
  int64_t coast_counts;
  bool coast_valid;
  bool completed;
  bool faulted;
  int peak_count_per_s;
  StopReason reason;
};

struct Measurement {
  CountExtender extender;
  VelocityWindow velocity;
  StopStability stability;
  int sign = 1;
  int64_t display_offset = 0;
  TrialRecord trials[16]{};
  bool in_run = false;
  int run_duty = 0;
  int run_dir = 1;
  int run_duration_ms = 0;
  uint64_t run_start_us = 0;
  int64_t run_start_count = 0;
  int peak = 0;
  int64_t pwm_off_count = 0;
  uint64_t pwm_off_us = 0;
  bool have_pwm_off = false;
  int active_id = 0;

  int64_t sample(int32_t raw);
  void set_sign(int next);
  void reset_counter(int32_t raw);
  void zero_display(int64_t signed_count);
  void begin_run(int id, int duty, int direction, int duration_ms, int64_t signed_count, uint64_t t_us);
  void note_running(int count_per_s);
  void end_pwm(uint64_t t_us, int64_t signed_count, StopReason reason);
  void complete_stop(uint64_t t_us, int64_t signed_count);
  void fault_run(StopReason reason);
  bool latest_completed_delta(int64_t* delta) const;
  const TrialRecord* find_id(int id) const;
};
