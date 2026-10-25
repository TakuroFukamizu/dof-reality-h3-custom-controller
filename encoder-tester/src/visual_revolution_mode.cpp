#include "visual_revolution_mode.h"

#include <stdio.h>
#include <string.h>

namespace {

void push_ring(VisualRecord* ring, int* count, const VisualRecord& record) {
  ring[*count % 12] = record;
  *count += 1;
}

}  // namespace

void VisualModule::collect() {
  if (session.record_seq != pushed_seq && session.last.used) {
    push_ring(align_log, &align_n, session.last);
    pushed_seq = session.record_seq;
  }
}

void VisualModule::push_flying_result() {
  if (!flying.has_result || !flying.last.used) {
    return;
  }
  push_ring(flying_log, &flying_n, flying.last);
  flying.has_result = false;
}

void VisualModule::stats_align(int direction, VisualStats* out) const {
  if (out == nullptr) {
    return;
  }
  *out = VisualStats{};
  double sum = 0.0;
  const int stored = align_n < 12 ? align_n : 12;
  for (int i = 0; i < stored; ++i) {
    const VisualRecord& record = align_log[i];
    if (!record.used || !record.valid) {
      continue;
    }
    if (direction != 0 && record.direction != direction) {
      continue;
    }
    const double value = record.counts_per_measured_shaft_rev_x4;
    if (out->valid_n == 0) {
      out->min_v = value;
      out->max_v = value;
    } else {
      if (value < out->min_v) out->min_v = value;
      if (value > out->max_v) out->max_v = value;
    }
    sum += value;
    out->valid_n += 1;
  }
  if (out->valid_n > 0) {
    out->mean = sum / static_cast<double>(out->valid_n);
    out->spread = out->max_v - out->min_v;
  }
}

void VisualModule::format_runs(int direction, char* buf, int size) const {
  if (buf == nullptr || size <= 1) {
    return;
  }
  buf[0] = '\0';
  int used = 0;
  int shown = 0;
  const int stored = align_n < 12 ? align_n : 12;
  for (int age = 0; age < stored && shown < 4; ++age) {
    const int index = (align_n - 1 - age) % 12;
    const VisualRecord& record = align_log[index];
    if (!record.used || !record.valid) {
      continue;
    }
    if (direction != 0 && record.direction != direction) {
      continue;
    }
    char piece[24];
    snprintf(piece, sizeof(piece), "%s%lld", shown == 0 ? "" : " ",
             static_cast<long long>(record.counts_per_measured_shaft_rev_x4 >= 0
                                        ? record.counts_per_measured_shaft_rev_x4 + 0.5
                                        : record.counts_per_measured_shaft_rev_x4 - 0.5));
    const int len = static_cast<int>(strlen(piece));
    if (used + len + 1 >= size) {
      break;
    }
    memcpy(buf + used, piece, static_cast<size_t>(len));
    used += len;
    buf[used] = '\0';
    shown += 1;
  }
}
