#include "relay_trace.h"

#include "drive_logic.h"

#include <Arduino.h>

#include <stdio.h>
#include <atomic>

namespace {

TraceSession<2048> session;
std::atomic<bool> sealed{false};
int printed = 0;
bool note_sent = false;
bool header_sent = false;
bool end_sent = false;

const char* mark_name(uint8_t mark) {
  return mark == static_cast<uint8_t>(TraceMark::Post) ? "post" : "tick";
}

void seal_if_finished() {
  if (session.finished) {
    sealed.store(true, std::memory_order_release);
  }
}

}  // namespace

bool relay_trace_on_tick(uint8_t state, bool armed_state, bool tail_state, bool cmd_edge, uint64_t t_us,
                         const TraceSample& pre) {
  if (sealed.load(std::memory_order_relaxed)) {
    return false;
  }
  const bool want_post = session.on_tick(state, armed_state, tail_state, cmd_edge, t_us, pre);
  seal_if_finished();
  return want_post && !sealed.load(std::memory_order_relaxed);
}

void relay_trace_on_post(uint64_t t_us, uint8_t state, const TraceSample& post) {
  if (sealed.load(std::memory_order_relaxed)) {
    return;
  }
  session.on_post(t_us, state, post);
  seal_if_finished();
}

bool relay_trace_sealed() { return sealed.load(std::memory_order_acquire); }

void relay_trace_flush(int max_lines) {
  if (!sealed.load(std::memory_order_acquire) || end_sent || max_lines <= 0) {
    return;
  }
  if (!note_sent) {
    Serial.println(
        "TRACE note a_level and b_level are single digitalRead samples, not a pulse or noise waveform");
    note_sent = true;
    max_lines -= 1;
  }
  if (max_lines > 0 && !header_sent) {
    Serial.println(
        "TRACE t_us,state,power_on_cmd,brake_release_cmd,power_gpio,brake_gpio,pwm_raw,pcnt_raw,"
        "count_total,delta,a_level,b_level,mark");
    header_sent = true;
    max_lines -= 1;
  }
  while (max_lines > 0 && printed < session.count) {
    const TraceSample& row = session.samples[printed];
    char line[180];
    snprintf(line, sizeof(line), "TRACE,%llu,%s,%u,%u,%u,%u,%d,%ld,%lld,%lld,%u,%u,%s",
             static_cast<unsigned long long>(row.t_us),
             drive_state_name(static_cast<DriveState>(row.state)), row.power_cmd, row.brake_cmd,
             row.power_gpio, row.brake_gpio, static_cast<int>(row.pwm_raw),
             static_cast<long>(row.pcnt_raw), static_cast<long long>(row.count_total),
             static_cast<long long>(row.delta), row.a_level, row.b_level, mark_name(row.mark));
    Serial.println(line);
    printed += 1;
    max_lines -= 1;
  }
  if (max_lines > 0 && printed >= session.count) {
    char line[80];
    snprintf(line, sizeof(line), "TRACE end n=%d overflow=%d", session.count, session.overflow ? 1 : 0);
    Serial.println(line);
    end_sent = true;
  }
}
