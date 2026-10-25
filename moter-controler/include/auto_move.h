#pragma once

// 自動移動の状態機械。GPIO・時計・NVS を持たない。制御タスクが tick ごとに入力を渡し、
// duty_signed（+ が P 端向き、− が N 端向き）を読んで PWM に出す。
// 座標: P 端が原点 0、N 端へ向かって増える。position = (raw − origin_raw) × working_sign。

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "motor_profile.h"

enum class AutoKind : uint8_t { Idle, HomeP, HomeN, Neutral, Cal };
enum class AutoPhase : uint8_t { Idle, Moving, Dwell, Done, Cancelled, Failed };
enum class CalStep : uint8_t { None, ToP, ToN, ToNeutral };

struct AutoConfig {
  int home_duty = HOME_DUTY;
  int auto_duty_max = AUTO_DUTY_MAX;
  int neutral_duty_min = NEUTRAL_DUTY_MIN;
  int kp_percent_per_100 = NEUTRAL_KP_PERCENT_PER_100;
  int32_t neutral_band = NEUTRAL_BAND;
  uint32_t stall_ms = STALL_MS;
  int32_t min_count_rate = MIN_COUNT_RATE;
  uint32_t start_grace_ms = START_GRACE_MS;
  uint32_t move_timeout_ms = MOVE_TIMEOUT_MS;
  uint32_t dwell_ms = DWELL_MS;
  int32_t max_travel = MAX_TRAVEL;
  int32_t slow_zone = SLOW_ZONE;
  int32_t arrival_min_travel = ARRIVAL_MIN_TRAVEL;
};

struct AutoInputs {
  uint32_t now_ms = 0;
  int64_t raw_count = 0;           // ISR 累積。符号を掛ける前
  bool relay_live = false;         // 主電源リレー ON かつ settle 済み
  bool brake_released = false;     // brake_on（GPIO1 HIGH ＝ 解除）
  bool manual_speed_nonzero = false;
  bool stop = false;               // この tick に STOP が来た
  bool estop = false;              // この tick に非常停止が来た
};

inline const char* auto_kind_name(AutoKind k) {
  switch (k) {
    case AutoKind::Idle: return "idle";
    case AutoKind::HomeP: return "home";
    case AutoKind::HomeN: return "upper";
    case AutoKind::Neutral: return "neutral";
    case AutoKind::Cal: return "cal";
  }
  return "?";
}

inline const char* auto_phase_name(AutoPhase p) {
  switch (p) {
    case AutoPhase::Idle: return "idle";
    case AutoPhase::Moving: return "moving";
    case AutoPhase::Dwell: return "dwell";
    case AutoPhase::Done: return "done";
    case AutoPhase::Cancelled: return "cancelled";
    case AutoPhase::Failed: return "failed";
  }
  return "?";
}

struct AutoMachine {
  AutoConfig cfg{};
  MotorProfile profile{};
  bool origin_valid = false;
  int64_t origin_raw = 0;
  int8_t working_sign = 1;
  AutoKind kind = AutoKind::Idle;
  AutoPhase phase = AutoPhase::Idle;
  CalStep cal_step = CalStep::None;
  int duty_signed = 0;
  int32_t last_measured_stroke = 0;
  bool profile_dirty = false;
  char reason[24] = "";

  // 内部状態（現在の行程）
  AutoKind leg = AutoKind::Idle;
  int64_t start_raw = 0;
  uint32_t start_ms = 0;
  uint32_t last_change_ms = 0;
  int64_t last_raw = 0;
  uint32_t dwell_since_ms = 0;
  bool near_end_at_start = false;
  int64_t rate_anchor_raw = 0;
  uint32_t rate_anchor_ms = 0;
  bool slow_event = false;  // この tick で「遅すぎる」窓が閉じた

  int32_t position(int64_t raw) const {
    return static_cast<int32_t>((raw - origin_raw) * working_sign);
  }

  bool active() const { return phase == AutoPhase::Moving || phase == AutoPhase::Dwell; }

  void set_reason(const char* text) {
    strncpy(reason, text, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
  }

  void invalidate_origin() { origin_valid = false; }

  void set_profile(const MotorProfile& p) {
    profile = p;
    working_sign = p.sign == -1 ? -1 : 1;
    origin_valid = false;
    kind = AutoKind::Idle;
    phase = AutoPhase::Idle;
    cal_step = CalStep::None;
    duty_signed = 0;
    reason[0] = '\0';
  }

  bool can_start(AutoKind k, const AutoInputs& in, const char** err) const {
    const char* e = "";
    bool ok = true;
    if (k == AutoKind::Idle) {
      e = "kind";
      ok = false;
    } else if (active()) {
      e = "busy";
      ok = false;
    } else if (!in.relay_live) {
      e = "not_live";
      ok = false;
    } else if (!in.brake_released) {
      e = "brake";
      ok = false;
    } else if (in.manual_speed_nonzero) {
      e = "manual";
      ok = false;
    } else if (k == AutoKind::HomeN && !origin_valid) {
      e = "no_origin";
      ok = false;
    } else if (k == AutoKind::Neutral && !origin_valid) {
      e = "no_origin";
      ok = false;
    } else if (k == AutoKind::Neutral && !profile.calibrated) {
      e = "not_calibrated";
      ok = false;
    }
    if (err != nullptr) {
      *err = e;
    }
    return ok;
  }

  bool request(AutoKind k, const AutoInputs& in, const char** err) {
    if (!can_start(k, in, err)) {
      return false;
    }
    kind = k;
    if (k == AutoKind::Cal) {
      cal_step = CalStep::ToP;
      leg = AutoKind::HomeP;
    } else {
      cal_step = CalStep::None;
      leg = k;
    }
    phase = AutoPhase::Moving;
    reason[0] = '\0';
    begin_leg(in);
    return true;
  }

  void tick(const AutoInputs& in) {
    if (!active()) {
      duty_signed = 0;
      return;
    }
    if (in.estop) {
      invalidate_origin();
      finish(AutoPhase::Cancelled, "estop");
      return;
    }
    if (in.stop) {
      finish(AutoPhase::Cancelled, "stop");
      return;
    }
    if (!in.relay_live) {
      finish(AutoPhase::Cancelled, "relay");
      return;
    }
    if (!in.brake_released) {
      finish(AutoPhase::Cancelled, "brake");
      return;
    }
    if (in.manual_speed_nonzero) {
      finish(AutoPhase::Cancelled, "manual");
      return;
    }
    if (in.raw_count != last_raw) {
      last_raw = in.raw_count;
      last_change_ms = in.now_ms;
    }
    const int64_t travel = llabs(in.raw_count - start_raw);
    if (travel > cfg.max_travel) {
      finish(AutoPhase::Failed, "travel");
      return;
    }
    if (in.now_ms - start_ms > cfg.move_timeout_ms) {
      finish(AutoPhase::Failed, "timeout");
      return;
    }
    update_rate(in);
    if (leg == AutoKind::Neutral) {
      tick_neutral(in);
    } else {
      tick_end(in, travel);
    }
  }

 private:
  void finish(AutoPhase p, const char* why) {
    phase = p;
    duty_signed = 0;
    set_reason(why);
  }

  void begin_leg(const AutoInputs& in) {
    start_raw = in.raw_count;
    start_ms = in.now_ms;
    last_raw = in.raw_count;
    last_change_ms = in.now_ms;
    rate_anchor_raw = in.raw_count;
    rate_anchor_ms = in.now_ms;
    slow_event = false;
    near_end_at_start = false;
    if (leg != AutoKind::Neutral) {
      int32_t dist = 0;
      if (end_distance(in.raw_count, &dist)) {
        near_end_at_start = dist <= cfg.slow_zone;
      }
    }
    phase = AutoPhase::Moving;
    compute_duty(in);
  }

  // 端までの距離。原点が無い、または N 端でストローク未知なら false。
  bool end_distance(int64_t raw, int32_t* dist) const {
    if (!origin_valid) {
      return false;
    }
    const int32_t pos = position(raw);
    if (leg == AutoKind::HomeP) {
      *dist = pos;
      return true;
    }
    if (leg == AutoKind::HomeN && profile.calibrated && profile.stroke > 0) {
      *dist = profile.stroke - pos;
      return true;
    }
    return false;
  }

  void compute_duty(const AutoInputs& in) {
    if (leg == AutoKind::Neutral) {
      const int32_t err = profile.neutral - position(in.raw_count);
      const int32_t mag = err < 0 ? -err : err;
      int duty = static_cast<int>((static_cast<int64_t>(mag) * cfg.kp_percent_per_100) / 100);
      if (duty < cfg.neutral_duty_min) duty = cfg.neutral_duty_min;
      if (duty > cfg.auto_duty_max) duty = cfg.auto_duty_max;
      duty_signed = err > 0 ? -duty : duty;
      return;
    }
    int duty = cfg.home_duty;
    int32_t dist = 0;
    if (end_distance(in.raw_count, &dist) && dist > cfg.slow_zone) {
      duty = cfg.auto_duty_max;
    }
    duty_signed = leg == AutoKind::HomeP ? duty : -duty;
  }

  bool stalled(const AutoInputs& in) const {
    return in.now_ms - start_ms >= cfg.start_grace_ms && in.now_ms - last_change_ms >= cfg.stall_ms;
  }

  // stall_ms ごとに速度の窓を閉じ、猶予後に min_count_rate 未満なら slow_event を立てる。
  void update_rate(const AutoInputs& in) {
    slow_event = false;
    const uint32_t span = in.now_ms - rate_anchor_ms;
    if (span < cfg.stall_ms) {
      return;
    }
    const int64_t delta = llabs(in.raw_count - rate_anchor_raw);
    const int64_t rate = (delta * 1000) / static_cast<int64_t>(span);
    rate_anchor_raw = in.raw_count;
    rate_anchor_ms = in.now_ms;
    if (in.now_ms - start_ms >= cfg.start_grace_ms && rate < cfg.min_count_rate) {
      slow_event = true;
    }
  }

  void tick_end(const AutoInputs& in, int64_t travel) {
    compute_duty(in);
    const bool stall = stalled(in);
    if (!stall && !slow_event) {
      return;
    }
    if (travel < cfg.arrival_min_travel && !near_end_at_start) {
      finish(AutoPhase::Failed, stall ? "stall" : "slow");
      return;
    }
    if (leg == AutoKind::HomeP) {
      const int64_t delta = in.raw_count - start_raw;
      if (delta >= cfg.arrival_min_travel) {
        working_sign = -1;
      } else if (delta <= -cfg.arrival_min_travel) {
        working_sign = 1;
      }
      origin_raw = in.raw_count;
      origin_valid = true;
    } else {
      last_measured_stroke = position(in.raw_count);
      if (kind == AutoKind::Cal) {
        profile.stroke = last_measured_stroke;
        profile.neutral = default_neutral(last_measured_stroke);
        profile.sign = working_sign;
        profile.calibrated = true;
        profile_dirty = true;
      }
    }
    leg_done(in);
  }

  void tick_neutral(const AutoInputs& in) {
    const int32_t err = profile.neutral - position(in.raw_count);
    const int32_t mag = err < 0 ? -err : err;
    if (mag <= cfg.neutral_band) {
      duty_signed = 0;
      if (phase != AutoPhase::Dwell) {
        phase = AutoPhase::Dwell;
        dwell_since_ms = in.now_ms;
      } else if (in.now_ms - dwell_since_ms >= cfg.dwell_ms) {
        leg_done(in);
      }
      return;
    }
    if (phase == AutoPhase::Dwell) {
      phase = AutoPhase::Moving;
      last_change_ms = in.now_ms;
    }
    compute_duty(in);
    if (stalled(in)) {
      finish(AutoPhase::Failed, "stall");
    } else if (slow_event) {
      finish(AutoPhase::Failed, "slow");
    }
  }

  void leg_done(const AutoInputs& in) {
    if (kind == AutoKind::Cal) {
      if (cal_step == CalStep::ToP) {
        cal_step = CalStep::ToN;
        leg = AutoKind::HomeN;
        begin_leg(in);
        return;
      }
      if (cal_step == CalStep::ToN) {
        cal_step = CalStep::ToNeutral;
        leg = AutoKind::Neutral;
        begin_leg(in);
        return;
      }
    }
    finish(AutoPhase::Done, "ok");
  }
};
