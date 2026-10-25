#include "telemetry.h"

#include <Arduino.h>

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include <string.h>

namespace {

constexpr int kLine = 240;
constexpr int kSamples = 128;
constexpr int kEvents = 64;

struct Line {
  char text[kLine];
};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Line samples[kSamples];
Line events[kEvents];
int sample_head = 0;
int sample_count = 0;
int event_head = 0;
int event_count = 0;
uint32_t drops = 0;
bool stream = false;
bool header_pending = false;

const char* kHeader =
    "seq,t_us,trial_id,axis,state,power_on_cmd,power_gpio_level,brake_release_cmd,brake_gpio_level,"
    "requested_command,requested_unit,applied_duty_pct,pwm_raw,pwm_full_scale,pwm_hz,dir_level,"
    "count_total_x4,count_trial_x4,velocity_count_s,a_level,b_level,elapsed_ms,stop_reason,log_drop_count";

void copy_line(char* dest, const char* src) {
  if (src == nullptr) {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, src, kLine - 1);
  dest[kLine - 1] = '\0';
}

}  // namespace

void telemetry_set_stream(bool on) {
  portENTER_CRITICAL(&mux);
  if (on && !stream) {
    header_pending = true;
  }
  stream = on;
  portEXIT_CRITICAL(&mux);
}

bool telemetry_streaming() {
  portENTER_CRITICAL(&mux);
  const bool on = stream;
  portEXIT_CRITICAL(&mux);
  return on;
}

uint32_t telemetry_drops() {
  portENTER_CRITICAL(&mux);
  const uint32_t value = drops;
  portEXIT_CRITICAL(&mux);
  return value;
}

void telemetry_push_sample(const char* line) {
  portENTER_CRITICAL(&mux);
  if (!stream) {
    portEXIT_CRITICAL(&mux);
    return;
  }
  if (sample_count >= kSamples) {
    drops += 1;
    portEXIT_CRITICAL(&mux);
    return;
  }
  copy_line(samples[sample_head].text, line);
  sample_head = (sample_head + 1) % kSamples;
  sample_count += 1;
  portEXIT_CRITICAL(&mux);
}

void telemetry_push_event(const char* line) {
  portENTER_CRITICAL(&mux);
  if (event_count >= kEvents) {
    if (stream) {
      drops += 1;
    }
    portEXIT_CRITICAL(&mux);
    return;
  }
  copy_line(events[event_head].text, line);
  event_head = (event_head + 1) % kEvents;
  event_count += 1;
  portEXIT_CRITICAL(&mux);
}

void telemetry_flush(int max_lines) {
  bool print_header = false;
  portENTER_CRITICAL(&mux);
  if (header_pending) {
    header_pending = false;
    print_header = true;
  }
  portEXIT_CRITICAL(&mux);
  if (print_header && max_lines > 0) {
    Serial.println(kHeader);
    max_lines -= 1;
  }
  while (max_lines > 0) {
    char line[kLine];
    bool have = false;
    bool is_event = false;
    portENTER_CRITICAL(&mux);
    if (event_count > 0) {
      const int index = (event_head - event_count + kEvents) % kEvents;
      copy_line(line, events[index].text);
      event_count -= 1;
      have = true;
      is_event = true;
    } else if (sample_count > 0) {
      const int index = (sample_head - sample_count + kSamples) % kSamples;
      copy_line(line, samples[index].text);
      sample_count -= 1;
      have = true;
    }
    portEXIT_CRITICAL(&mux);
    if (!have) {
      break;
    }
    (void)is_event;
    Serial.println(line);
    max_lines -= 1;
  }
}
