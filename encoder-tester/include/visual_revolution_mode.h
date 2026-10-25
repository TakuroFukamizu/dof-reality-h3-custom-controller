#pragma once

#include "visual_session.h"

struct VisualStats {
  int valid_n;
  double mean;
  double min_v;
  double max_v;
  double spread;
};

struct VisualModule {
  VisualSession session;
  FlyingMark flying;
  VisualRecord align_log[12]{};
  VisualRecord flying_log[12]{};
  int align_n = 0;
  int flying_n = 0;
  uint32_t pushed_seq = 0;

  void collect();
  void push_flying_result();
  void stats_align(int direction, VisualStats* out) const;
  void format_runs(int direction, char* buf, int size) const;
};
