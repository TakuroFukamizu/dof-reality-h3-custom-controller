#pragma once

#include <stdint.h>

void telemetry_set_stream(bool on);
bool telemetry_streaming();
uint32_t telemetry_drops();
void telemetry_push_sample(const char* line);
void telemetry_push_event(const char* line);
void telemetry_flush(int max_lines);
