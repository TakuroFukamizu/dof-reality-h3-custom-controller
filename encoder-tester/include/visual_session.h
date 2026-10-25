#pragma once

#include "measure_math.h"

#include <stdint.h>
#include <string.h>

// 目視1回転セッション。駆動状態機械とは独立で、ALIGNING は駆動許可を意味しない。

enum class VisualState : uint8_t {
  Idle,
  ReferenceCaptured,
  Aligning,
  FinishConfirmed,
  Result,
  Invalid,
  Cancelled
};

inline const char* visual_state_name(VisualState state) {
  switch (state) {
    case VisualState::Idle:
      return "VISUAL_IDLE";
    case VisualState::ReferenceCaptured:
      return "REFERENCE_CAPTURED";
    case VisualState::Aligning:
      return "ALIGNING";
    case VisualState::FinishConfirmed:
      return "FINISH_CONFIRMED";
    case VisualState::Result:
      return "RESULT";
    case VisualState::Invalid:
      return "INVALID";
    case VisualState::Cancelled:
      return "CANCELLED";
  }
  return "UNKNOWN";
}

struct VisualConfig {
  char shaft[16];
  char reference[48];
  int direction;
  int revolutions;
  int max_drive_ms;
  int max_travel_counts;
  bool set;
};

struct VisualRecord {
  bool used;
  char method[24];
  char shaft[16];
  char reference[48];
  int direction;
  int visual_revolutions;
  uint64_t start_t_us;
  uint64_t end_t_us;
  int64_t start_count_x4;
  int64_t end_count_x4;
  int64_t signed_delta_x4;
  double counts_per_measured_shaft_rev_x4;
  int trial_lo;
  int trial_hi;
  bool reversal_detected;
  bool valid;
  char invalid_reason[32];
};

struct VisualSession {
  VisualConfig cfg{};
  VisualState state = VisualState::Idle;
  int64_t start_count = 0;
  uint64_t start_us = 0;
  int64_t end_count = 0;
  uint64_t end_us = 0;
  int trial_at_start = 0;
  int drive_ms = 0;
  int64_t travel = 0;
  int64_t last_motion_count = 0;
  int last_move_sign = 0;
  bool reversal = false;
  char invalid_reason[32]{};
  VisualRecord last{};
  uint32_t record_seq = 0;

  void clear_run() {
    state = VisualState::Idle;
    start_count = 0;
    start_us = 0;
    end_count = 0;
    end_us = 0;
    drive_ms = 0;
    travel = 0;
    last_motion_count = 0;
    last_move_sign = 0;
    reversal = false;
    invalid_reason[0] = '\0';
  }

  void write_invalid(const char* reason, int64_t count, uint64_t t_us, bool have_count) {
    if (state == VisualState::Idle || state == VisualState::Result || state == VisualState::Cancelled) {
      return;
    }
    const bool had_ref = state == VisualState::ReferenceCaptured || state == VisualState::Aligning ||
                         state == VisualState::FinishConfirmed;
    const int64_t start = start_count;
    const uint64_t start_t = start_us;
    const int trial = trial_at_start;
    const bool rev = reversal;
    state = VisualState::Invalid;
    strncpy(invalid_reason, reason == nullptr ? "" : reason, sizeof(invalid_reason) - 1);
    invalid_reason[sizeof(invalid_reason) - 1] = '\0';
    if (!had_ref) {
      return;
    }
    last = VisualRecord{};
    last.used = true;
    strncpy(last.method, "visual_stop_align", sizeof(last.method) - 1);
    strncpy(last.shaft, cfg.shaft, sizeof(last.shaft) - 1);
    strncpy(last.reference, cfg.reference, sizeof(last.reference) - 1);
    last.direction = cfg.direction;
    last.visual_revolutions = cfg.revolutions;
    last.start_t_us = start_t;
    last.start_count_x4 = start;
    last.trial_lo = trial;
    last.trial_hi = trial;
    last.reversal_detected = rev;
    last.valid = false;
    strncpy(last.invalid_reason, invalid_reason, sizeof(last.invalid_reason) - 1);
    if (have_count) {
      last.end_count_x4 = count;
      last.end_t_us = t_us;
      last.signed_delta_x4 = count - start;
    }
    record_seq += 1;
  }

  bool configure(const char* shaft, const char* reference, int direction, int revolutions,
                 int max_drive_ms, int max_travel_counts, const char** error) {
    if (shaft == nullptr || shaft[0] == '\0' || reference == nullptr || reference[0] == '\0') {
      *error = "visual_config";
      return false;
    }
    const bool shaft_ok = strcmp(shaft, "encoder") == 0 || strcmp(shaft, "motor") == 0 ||
                          strcmp(shaft, "gearbox") == 0 || strcmp(shaft, "output") == 0 ||
                          strcmp(shaft, "other") == 0;
    if (!shaft_ok || (direction != 1 && direction != -1) || revolutions < 1 || max_drive_ms < 1 ||
        max_travel_counts < 1) {
      *error = "visual_config";
      return false;
    }
    if (state == VisualState::ReferenceCaptured || state == VisualState::Aligning ||
        state == VisualState::FinishConfirmed) {
      write_invalid("config_change", last_motion_count, 0, true);
    }
    clear_run();
    cfg = VisualConfig{};
    strncpy(cfg.shaft, shaft, sizeof(cfg.shaft) - 1);
    strncpy(cfg.reference, reference, sizeof(cfg.reference) - 1);
    cfg.direction = direction;
    cfg.revolutions = revolutions;
    cfg.max_drive_ms = max_drive_ms;
    cfg.max_travel_counts = max_travel_counts;
    cfg.set = true;
    *error = "";
    return true;
  }

  bool capture_reference(bool disarmed, bool count_stable, int64_t count, uint64_t t_us, int trial_id,
                         const char** error) {
    if (!cfg.set || !disarmed || !count_stable ||
        (state != VisualState::Idle && state != VisualState::Cancelled && state != VisualState::Result &&
         state != VisualState::Invalid && state != VisualState::FinishConfirmed)) {
      *error = "visual_reference";
      return false;
    }
    start_count = count;
    start_us = t_us;
    trial_at_start = trial_id;
    drive_ms = 0;
    travel = 0;
    last_motion_count = count;
    last_move_sign = 0;
    reversal = false;
    invalid_reason[0] = '\0';
    state = VisualState::ReferenceCaptured;
    *error = "";
    return true;
  }

  void note_motion(int direction, int run_ms, int64_t count, uint64_t t_us) {
    if (state != VisualState::ReferenceCaptured && state != VisualState::Aligning) {
      return;
    }
    state = VisualState::Aligning;
    if (run_ms > 0) {
      drive_ms += run_ms;
    }
    const int64_t step = count - last_motion_count;
    travel += step < 0 ? -step : step;
    last_motion_count = count;
    int move_sign = 0;
    if (direction == 1 || direction == -1) {
      move_sign = direction;
    } else if (step != 0) {
      move_sign = step > 0 ? 1 : -1;
    }
    if (move_sign != 0) {
      if (last_move_sign != 0 && move_sign != last_move_sign) {
        reversal = true;
      }
      last_move_sign = move_sign;
    }
    if (drive_ms > cfg.max_drive_ms || travel > cfg.max_travel_counts) {
      write_invalid("session_limit", count, t_us, true);
    }
  }

  void invalidate(const char* reason) { write_invalid(reason, last_motion_count, 0, false); }

  void invalidate_at(const char* reason, int64_t count, uint64_t t_us) {
    write_invalid(reason, count, t_us, true);
  }

  bool finish(bool disarmed, bool count_stable, int64_t count, uint64_t t_us, int trial_id,
              const char** error) {
    if (!disarmed || !count_stable ||
        (state != VisualState::ReferenceCaptured && state != VisualState::Aligning)) {
      *error = "visual_finish";
      return false;
    }
    const int64_t delta = count - start_count;
    if (delta == 0 || cfg.revolutions < 1) {
      *error = "no_movement";
      return false;
    }
    end_count = count;
    end_us = t_us;
    state = VisualState::Result;
    last = VisualRecord{};
    last.used = true;
    strncpy(last.method, "visual_stop_align", sizeof(last.method) - 1);
    strncpy(last.shaft, cfg.shaft, sizeof(last.shaft) - 1);
    strncpy(last.reference, cfg.reference, sizeof(last.reference) - 1);
    last.direction = cfg.direction;
    last.visual_revolutions = cfg.revolutions;
    last.start_t_us = start_us;
    last.end_t_us = end_us;
    last.start_count_x4 = start_count;
    last.end_count_x4 = end_count;
    last.signed_delta_x4 = delta;
    last.counts_per_measured_shaft_rev_x4 =
        static_cast<double>(delta < 0 ? -delta : delta) / static_cast<double>(cfg.revolutions);
    last.trial_lo = trial_at_start;
    last.trial_hi = trial_id;
    last.reversal_detected = reversal;
    last.valid = true;
    record_seq += 1;
    *error = "";
    return true;
  }

  void cancel() {
    if (state == VisualState::Idle) {
      return;
    }
    if (state == VisualState::ReferenceCaptured || state == VisualState::Aligning ||
        state == VisualState::FinishConfirmed) {
      write_invalid("cancelled", last_motion_count, 0, true);
    }
    clear_run();
    state = VisualState::Cancelled;
  }
};

struct FlyingMark {
  bool active = false;
  bool ended = false;
  int64_t start_count = 0;
  uint64_t start_us = 0;
  int64_t mark_end_count = 0;
  uint64_t mark_end_us = 0;
  int64_t stopped_count = 0;
  bool has_result = false;
  VisualRecord last{};

  bool start(bool running, int64_t count, uint64_t t_us, const char** error) {
    if (!running) {
      *error = "not_running";
      return false;
    }
    active = true;
    ended = false;
    has_result = false;
    start_count = count;
    start_us = t_us;
    *error = "";
    return true;
  }

  bool end_mark(bool running, int64_t count, uint64_t t_us, const char** error) {
    if (!running || !active) {
      *error = "mark_end";
      return false;
    }
    mark_end_count = count;
    mark_end_us = t_us;
    ended = true;
    *error = "";
    return true;
  }

  void note_stopped(int64_t count, const char* shaft) {
    if (!active || !ended) {
      return;
    }
    stopped_count = count;
    const int64_t delta = mark_end_count - start_count;
    last = VisualRecord{};
    last.used = true;
    strncpy(last.method, "visual_flying_mark", sizeof(last.method) - 1);
    if (shaft != nullptr) {
      strncpy(last.shaft, shaft, sizeof(last.shaft) - 1);
    }
    last.visual_revolutions = 1;
    last.start_t_us = start_us;
    last.end_t_us = mark_end_us;
    last.start_count_x4 = start_count;
    last.end_count_x4 = mark_end_count;
    last.signed_delta_x4 = delta;
    if (delta != 0) {
      last.counts_per_measured_shaft_rev_x4 = static_cast<double>(delta < 0 ? -delta : delta);
      last.valid = true;
    } else {
      last.valid = false;
      strncpy(last.invalid_reason, "no_movement", sizeof(last.invalid_reason) - 1);
    }
    has_result = true;
    active = false;
  }
};
