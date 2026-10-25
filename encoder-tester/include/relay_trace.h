#pragma once

#include <stdint.h>

// 制御周期ごとの記録。シリアルには出さない。出力は試験が終わってから別タスクが行う。
// a_level / b_level は、その時刻に digitalRead した 1 点である。
// サンプル間のパルスも、ノイズの波形も、ここには残らない。

enum class TraceMark : uint8_t { Tick = 0, Post = 1 };

struct TraceSample {
  uint64_t t_us = 0;
  int64_t count_total = 0;
  int64_t delta = 0;
  int32_t pcnt_raw = 0;
  int16_t pwm_raw = 0;
  uint8_t state = 0;
  uint8_t power_cmd = 0;
  uint8_t brake_cmd = 0;
  uint8_t power_gpio = 0;
  uint8_t brake_gpio = 0;
  uint8_t a_level = 0;
  uint8_t b_level = 0;
  uint8_t mark = 0;
};

// 起動後 1 回だけ。DISARMED を離れた周期から、DISARMED か FAULT のあと 2 秒まで。
// リレー指令が変わった周期は、GPIO を書く前の tick と、書いた直後の post を残す。
template <int Cap>
struct TraceSession {
  static constexpr int kCap = Cap;
  static constexpr uint64_t kTailUs = 2000000ULL;

  TraceSample samples[Cap]{};
  int count = 0;
  bool recording = false;
  bool finished = false;
  bool overflow = false;
  bool tail_marked = false;
  uint64_t tail_us = 0;
  bool have_count = false;
  int64_t last_count = 0;

  bool push(const TraceSample& in) {
    if (!recording || finished) {
      return false;
    }
    if (count >= kCap) {
      overflow = true;
      finished = true;
      recording = false;
      return false;
    }
    TraceSample row = in;
    if (!have_count) {
      row.delta = 0;
      have_count = true;
    } else {
      row.delta = row.count_total - last_count;
    }
    last_count = row.count_total;
    samples[count] = row;
    count += 1;
    return true;
  }

  // GPIO 書き込み前に呼ぶ。true なら、書いた直後にもう一度 PCNT を読んで on_post する。
  bool on_tick(uint8_t state, bool armed_state, bool tail_state, bool cmd_edge, uint64_t t_us,
               const TraceSample& pre) {
    if (finished) {
      return false;
    }
    if (!recording) {
      if (!armed_state) {
        return false;
      }
      recording = true;
    }
    if (tail_state && !tail_marked) {
      tail_marked = true;
      tail_us = t_us;
    }
    TraceSample row = pre;
    row.t_us = t_us;
    row.state = state;
    row.mark = static_cast<uint8_t>(TraceMark::Tick);
    const bool stored = push(row);
    const bool time_up = tail_marked && t_us >= tail_us + kTailUs;
    if (time_up && recording) {
      finished = true;
      recording = false;
    }
    return stored && cmd_edge && recording;
  }

  void on_post(uint64_t t_us, uint8_t state, const TraceSample& post) {
    if (!recording || finished) {
      return;
    }
    TraceSample row = post;
    row.t_us = t_us;
    row.state = state;
    row.mark = static_cast<uint8_t>(TraceMark::Post);
    push(row);
    if (tail_marked && t_us >= tail_us + kTailUs && recording) {
      finished = true;
      recording = false;
    }
  }
};

bool relay_trace_on_tick(uint8_t state, bool armed_state, bool tail_state, bool cmd_edge, uint64_t t_us,
                         const TraceSample& pre);
void relay_trace_on_post(uint64_t t_us, uint8_t state, const TraceSample& post);
bool relay_trace_sealed();
void relay_trace_flush(int max_lines);
