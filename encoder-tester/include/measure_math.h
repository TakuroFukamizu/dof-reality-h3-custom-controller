#pragma once

#include <cmath>
#include <stdint.h>
#include <string.h>

// 外部角度・回転数から、観測した軸の 4逓倍カウントを換算する。
// PWM や未校正カウントからは計算しない。

struct CalResult {
  bool ok;
  double counts_per_measured_rev_x4;
  double counts_per_degree;
  const char* error;
};

inline bool cal_number_ok(double value) { return std::isfinite(value) && value > 0.0; }

inline CalResult cal_reject(const char* error) {
  CalResult result{};
  result.ok = false;
  result.error = error;
  return result;
}

inline CalResult cal_from_angle(int64_t delta_x4, double angle_deg) {
  if (delta_x4 == 0) {
    return cal_reject("zero_count");
  }
  if (!cal_number_ok(angle_deg)) {
    return cal_reject("bad_angle");
  }
  CalResult result{};
  result.ok = true;
  result.counts_per_degree = static_cast<double>(delta_x4 < 0 ? -delta_x4 : delta_x4) / angle_deg;
  result.counts_per_measured_rev_x4 = result.counts_per_degree * 360.0;
  result.error = "";
  return result;
}

inline CalResult cal_from_revolutions(int64_t delta_x4, double revolutions) {
  if (delta_x4 == 0) {
    return cal_reject("zero_count");
  }
  if (!cal_number_ok(revolutions)) {
    return cal_reject("bad_revolutions");
  }
  CalResult result{};
  result.ok = true;
  result.counts_per_measured_rev_x4 =
      static_cast<double>(delta_x4 < 0 ? -delta_x4 : delta_x4) / revolutions;
  result.counts_per_degree = result.counts_per_measured_rev_x4 / 360.0;
  result.error = "";
  return result;
}

// エンコーダ軸そのものを測ったときだけ PPR にする。トークンは "encoder" のみ。
inline bool ppr_from_encoder_shaft(const char* shaft, double counts_per_rev_x4, double* ppr_out) {
  if (shaft == nullptr || ppr_out == nullptr || !cal_number_ok(counts_per_rev_x4)) {
    return false;
  }
  if (strcmp(shaft, "encoder") != 0) {
    return false;
  }
  *ppr_out = counts_per_rev_x4 / 4.0;
  return true;
}

inline int64_t displayed_count(int64_t signed_count, int64_t zero_offset) {
  return signed_count - zero_offset;
}

// PCNT の int32 累積を、取りこぼさない間隔で読んで int64 に伸ばす。
struct CountExtender {
  int32_t last = 0;
  int64_t total = 0;
  bool started = false;

  void reset(int32_t raw) {
    last = raw;
    total = raw;
    started = true;
  }

  int64_t sample(int32_t raw) {
    if (!started) {
      reset(raw);
      return total;
    }
    // uint32 差分にしてから int32 に戻す。符号付きオーバーフローに依存しない。
    const int32_t delta =
        static_cast<int32_t>(static_cast<uint32_t>(raw) - static_cast<uint32_t>(last));
    total += delta;
    last = raw;
    return total;
  }
};

// 100 ms 窓の平均 count/s。窓が埋まるまでは未確定（0 かつ window_ready=false）。
struct VelocityWindow {
  int64_t anchor_count = 0;
  uint64_t anchor_us = 0;
  bool seeded = false;
  bool window_ready = false;
  int count_per_s = 0;

  void reset() { *this = VelocityWindow{}; }

  void push(int64_t count, uint64_t t_us) {
    if (!seeded) {
      seeded = true;
      anchor_count = count;
      anchor_us = t_us;
      window_ready = false;
      count_per_s = 0;
      return;
    }
    const uint64_t dt = t_us - anchor_us;
    if (dt < 100000ULL) {
      return;
    }
    const int64_t dc = count - anchor_count;
    int64_t cps = (dc * 1000000LL) / static_cast<int64_t>(dt);
    if (cps > 2000000000LL) {
      cps = 2000000000LL;
    }
    if (cps < -2000000000LL) {
      cps = -2000000000LL;
    }
    count_per_s = static_cast<int>(cps);
    anchor_count = count;
    anchor_us = t_us;
    window_ready = true;
  }
};

// 停止確認。閾値が未設定、または速度窓が未確定の間は安定にしない。
struct StopStability {
  bool timing = false;
  uint64_t since_ms = 0;
  bool stable = false;

  void reset() { *this = StopStability{}; }

  bool update(bool window_ready, int count_per_s, bool threshold_configured, int threshold,
              int confirm_ms, uint64_t now_ms) {
    if (!threshold_configured || !window_ready || confirm_ms < 0) {
      reset();
      return false;
    }
    const int abs_cps = count_per_s < 0 ? -count_per_s : count_per_s;
    if (abs_cps <= threshold) {
      if (!timing) {
        timing = true;
        since_ms = now_ms;
        stable = confirm_ms == 0;
      } else if (now_ms - since_ms >= static_cast<uint64_t>(confirm_ms)) {
        stable = true;
      }
    } else {
      timing = false;
      stable = false;
    }
    return stable;
  }
};
