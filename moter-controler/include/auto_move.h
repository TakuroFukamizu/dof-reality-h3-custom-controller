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
  int32_t neutral_band_exit = NEUTRAL_BAND_EXIT;
  uint32_t stall_ms = STALL_MS;
  int32_t min_count_rate = MIN_COUNT_RATE;
  uint32_t start_grace_ms = START_GRACE_MS;
  uint32_t move_timeout_ms = MOVE_TIMEOUT_MS;
  uint32_t dwell_ms = DWELL_MS;
  int32_t max_travel = MAX_TRAVEL;
  int32_t slow_zone = SLOW_ZONE;
  int32_t arrival_min_travel = ARRIVAL_MIN_TRAVEL;
  int manual_min_duty = MANUAL_GUARD_MIN_DUTY;
  int32_t retreat_counts = RETREAT_COUNTS;
  int32_t retreat_end_min_travel = RETREAT_END_MIN_TRAVEL;
};

struct AutoInputs {
  uint32_t now_ms = 0;
  int64_t raw_count = 0;           // ISR 累積。符号を掛ける前
  bool relay_live = false;         // 主電源リレー ON かつ settle 済み
  bool brake_released = false;     // brake_on（PIN_MOTOR_BRAKE = G13 HIGH ＝ 解除）
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
  bool retreating = false;   // 端当てで開始直後に止まり、反対へ退避中
  bool retreat_done = false; // この行程で退避を済ませた（2 回目は失敗にする）
  int64_t retreat_start_raw = 0;
  int neutral_boost = 0;     // 中立への Duty 下限の上乗せ。遅すぎ／無パルスのたびに上げ、home_duty で頭打ち

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
    retreating = false;
    retreat_done = false;
    neutral_boost = 0;
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
    // 校正中は保存ストロークを信じない（再校正で保存値が実際より大きいと 15 % で端に当たる）。
    if (leg == AutoKind::HomeN && kind != AutoKind::Cal && profile.calibrated && profile.stroke > 0) {
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
      int floor_duty = cfg.neutral_duty_min + neutral_boost;
      if (floor_duty > cfg.home_duty) floor_duty = cfg.home_duty;
      if (duty < floor_duty) duty = floor_duty;
      if (duty > cfg.auto_duty_max) duty = cfg.auto_duty_max;
      duty_signed = err > 0 ? -duty : duty;
      return;
    }
    if (retreating) {
      duty_signed = leg == AutoKind::HomeP ? -cfg.home_duty : cfg.home_duty;
      return;
    }
    int duty = cfg.home_duty;
    int32_t dist = 0;
    if (end_distance(in.raw_count, &dist) && dist > cfg.slow_zone) {
      duty = cfg.auto_duty_max;
    }
    duty_signed = leg == AutoKind::HomeP ? duty : -duty;
  }

  // 端当てで開始直後に止まったとき、端に居るのか動けないのかを区別するために反対へ退避する。
  void begin_retreat(const AutoInputs& in) {
    retreating = true;
    retreat_start_raw = in.raw_count;
    start_ms = in.now_ms;  // 退避にも猶予を与える
    last_change_ms = in.now_ms;
    rate_anchor_raw = in.raw_count;
    rate_anchor_ms = in.now_ms;
    slow_event = false;
    set_reason("retreat");
    compute_duty(in);
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
    if (retreating) {
      if (llabs(in.raw_count - retreat_start_raw) >= cfg.retreat_counts) {
        // 退避できた＝動ける。改めて端へ向かう。行程の起点を取り直す（移動量・時間・速度窓）。
        begin_leg(in);
        retreat_done = true;
        reason[0] = '\0';
        return;
      }
      compute_duty(in);
      if (stalled(in)) {
        // 退避が retreat_end_min_travel 以上動いてから止まった = 動ける軸が反対の端に当たった（端の近くから始めると
        // RETREAT_COUNTS に届く前に反対の端に着く。Issue #34）。そこを起点に改めて端へ向かう（退避はこの行程で済み扱い。
        // そこからも移動量不足で止まれば従来どおり失敗）。遅すぎ（slow）はこの扱いにしない（片相断線などを到達とみなさない）。
        if (llabs(in.raw_count - retreat_start_raw) >= cfg.retreat_end_min_travel) {
          begin_leg(in);
          retreat_done = true;
          reason[0] = '\0';
          return;
        }
        finish(AutoPhase::Failed, "stall");
      } else if (slow_event) {
        finish(AutoPhase::Failed, "slow");
      }
      return;
    }
    compute_duty(in);
    const bool stall = stalled(in);
    if (!stall && !slow_event) {
      return;
    }
    if (travel < cfg.arrival_min_travel && !near_end_at_start) {
      if (!retreat_done) {
        begin_retreat(in);
        return;
      }
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
    // Dwell 中は出口帯で判定する（ヒステリシス）。帯の縁で 1〜3 カウント揺れても Dwell を数え続ける。
    const int32_t band = phase == AutoPhase::Dwell ? cfg.neutral_band_exit : cfg.neutral_band;
    if (mag <= band) {
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
      // 惰性で帯を出た。Dwell 中の静止を速度窓に含めると偽の slow になるので、窓も取り直す。
      phase = AutoPhase::Moving;
      last_change_ms = in.now_ms;
      rate_anchor_raw = in.raw_count;
      rate_anchor_ms = in.now_ms;
      slow_event = false;
      neutral_boost = 0;  // 戻りは短距離なので下限から
    }
    compute_duty(in);
    const bool stall = stalled(in);
    if (!stall && !slow_event) {
      return;
    }
    // 下限 Duty では機構が動かない／遅いことがある（重力負荷の持ち上げ側）。home_duty 未満なら失敗にせず下限を上げる。
    const int mag_duty = duty_signed < 0 ? -duty_signed : duty_signed;
    if (mag_duty < cfg.home_duty) {
      const int max_boost = cfg.home_duty - cfg.neutral_duty_min;
      neutral_boost = stall ? max_boost : neutral_boost + 1;  // 無パルスなら一気に、遅すぎなら 1 % ずつ
      if (neutral_boost > max_boost) neutral_boost = max_boost;
      start_ms = in.now_ms;  // 新しい Duty にも猶予を与える
      last_change_ms = in.now_ms;
      rate_anchor_raw = in.raw_count;
      rate_anchor_ms = in.now_ms;
      slow_event = false;
      compute_duty(in);
      return;
    }
    finish(AutoPhase::Failed, stall ? "stall" : "slow");
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
