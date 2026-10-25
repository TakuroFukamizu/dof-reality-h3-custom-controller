#pragma once

#include <cmath>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// ESP32 の nano printf は %f を出せないことがある。小数はここで整数化する。

inline void fmt_fixed(char* out, int size, double value) {
  if (out == nullptr || size < 2) {
    return;
  }
  if (!std::isfinite(value)) {
    strncpy(out, "nan", static_cast<size_t>(size - 1));
    out[size - 1] = '\0';
    return;
  }
  const bool neg = value < 0;
  double mag = neg ? -value : value;
  if (mag > 1.0e12) {
    strncpy(out, "ovf", static_cast<size_t>(size - 1));
    out[size - 1] = '\0';
    return;
  }
  unsigned long long whole = static_cast<unsigned long long>(mag);
  int frac = static_cast<int>((mag - static_cast<double>(whole)) * 1000.0 + 0.5);
  if (frac >= 1000) {
    frac -= 1000;
    whole += 1ULL;
  }
  snprintf(out, static_cast<size_t>(size), "%s%llu.%03d", neg ? "-" : "", whole, frac);
}
