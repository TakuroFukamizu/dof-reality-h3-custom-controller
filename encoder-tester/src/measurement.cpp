#include "measurement.h"

#include <string.h>

int64_t Measurement::sample(int32_t raw) {
  const int64_t total = extender.sample(raw);
  return static_cast<int64_t>(sign) * total;
}

void Measurement::set_sign(int next) {
  if (next != 1 && next != -1) {
    return;
  }
  sign = next;
  velocity.reset();
}

void Measurement::reset_counter(int32_t raw) {
  extender.reset(raw);
  velocity.reset();
  stability.reset();
  display_offset = static_cast<int64_t>(sign) * static_cast<int64_t>(raw);
}

void Measurement::zero_display(int64_t signed_count) { display_offset = signed_count; }

void Measurement::begin_run(int id, int duty, int direction, int duration_ms, int64_t signed_count,
                            uint64_t t_us) {
  in_run = true;
  active_id = id;
  run_duty = duty;
  run_dir = direction;
  run_duration_ms = duration_ms;
  run_start_us = t_us;
  run_start_count = signed_count;
  peak = 0;
  have_pwm_off = false;
  pwm_off_count = signed_count;
  pwm_off_us = t_us;
}

void Measurement::note_running(int count_per_s) {
  const int abs_cps = count_per_s < 0 ? -count_per_s : count_per_s;
  if (abs_cps > peak) {
    peak = abs_cps;
  }
}

void Measurement::end_pwm(uint64_t t_us, int64_t signed_count, StopReason reason) {
  if (!in_run) {
    return;
  }
  have_pwm_off = true;
  pwm_off_us = t_us;
  pwm_off_count = signed_count;
  TrialRecord* slot = &trials[active_id % 16];
  *slot = TrialRecord{};
  slot->used = true;
  slot->id = active_id;
  slot->duty_percent = run_duty;
  slot->direction = run_dir;
  slot->requested_duration_ms = run_duration_ms;
  slot->start_us = run_start_us;
  slot->start_count = run_start_count;
  slot->pwm_off_us = t_us;
  slot->pwm_off_count = signed_count;
  slot->peak_count_per_s = peak;
  slot->reason = reason;
  in_run = false;
}

void Measurement::complete_stop(uint64_t t_us, int64_t signed_count) {
  if (active_id <= 0) {
    return;
  }
  TrialRecord* slot = &trials[active_id % 16];
  if (!slot->used || slot->id != active_id) {
    return;
  }
  slot->stop_us = t_us;
  slot->stop_count = signed_count;
  slot->coast_counts = signed_count - slot->pwm_off_count;
  slot->coast_valid = true;
  slot->completed = true;
  slot->faulted = false;
}

void Measurement::fault_run(StopReason reason) {
  if (active_id <= 0) {
    return;
  }
  TrialRecord* slot = &trials[active_id % 16];
  if (!slot->used || slot->id != active_id) {
    if (in_run) {
      end_pwm(run_start_us, run_start_count, reason);
      slot = &trials[active_id % 16];
    } else {
      return;
    }
  }
  slot->faulted = true;
  slot->completed = false;
  slot->reason = reason;
  in_run = false;
}

bool Measurement::latest_completed_delta(int64_t* delta) const {
  if (delta == nullptr) {
    return false;
  }
  int best_id = -1;
  const TrialRecord* best = nullptr;
  for (int i = 0; i < 16; ++i) {
    if (!trials[i].used || !trials[i].completed || trials[i].faulted) {
      continue;
    }
    if (trials[i].id >= best_id) {
      best_id = trials[i].id;
      best = &trials[i];
    }
  }
  if (best == nullptr) {
    return false;
  }
  *delta = best->stop_count - best->start_count;
  return true;
}

const TrialRecord* Measurement::find_id(int id) const {
  if (id <= 0) {
    return nullptr;
  }
  const TrialRecord* slot = &trials[id % 16];
  if (!slot->used || slot->id != id) {
    return nullptr;
  }
  return slot;
}
