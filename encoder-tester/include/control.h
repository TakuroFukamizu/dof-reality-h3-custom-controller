#pragma once

#include "drive_logic.h"
#include "settings.h"
#include "visual_session.h"

#include <stdint.h>

enum class ReplyKind : uint8_t {
  None,
  Arm,
  Stop,
  Estop,
  FaultReset,
  Run,
  Zero,
  MarkStart,
  MarkEnd,
  VisualConfig,
  VisualRef,
  VisualFinish,
  VisualCancel,
  VisualResult,
  Cal,
  Set,
  Authority
};

struct Snapshot {
  DriveState state;
  StopReason stop_reason;
  bool power_on_cmd;
  bool brake_release_cmd;
  int power_gpio;
  int brake_gpio;
  int applied_duty;
  int requested_signed;
  int direction;
  int pwm_raw;
  int pwm_hz;
  int dir_level;
  int64_t count_signed;
  int64_t count_display;
  int64_t count_trial;
  int count_per_s;
  int a_level;
  int b_level;
  int trial_id;
  uint32_t remaining_ms;
  uint32_t ready_remaining_ms;
  int travel_abs;
  int max_travel;
  bool count_stable;
  bool limits_ok;
  bool electrical_ok;
  bool pcnt_ok;
  bool usb_authority;
  bool stream_on;
  uint32_t log_drops;
  char fault[48];
  char reject[48];
  char missing[360];
  char lock_reason[24];
  VisualState visual_state;
  int64_t visual_delta;
  bool visual_have_delta;
  bool visual_valid;
  bool visual_reversal;
  double visual_cpr;
  char visual_shaft[16];
  char visual_reason[32];
  char visual_runs[80];
  int visual_valid_n;
  double visual_mean;
  double visual_min;
  double visual_max;
  double visual_spread;
  char ppr_text[48];
  char cal_text[120];
  int jog_duty;
  int sign;
  int glitch_ns;
};

void control_begin(bool pcnt_ok, bool electrical_ok);
void control_set_inputs(uint64_t now_ms, bool estop_level, bool jog_pos, bool jog_neg);
void control_request_arm(bool from_usb);
void control_request_stop(bool want_reply);
void control_request_estop(bool want_reply);
void control_request_fault_reset(bool want_reply);
void control_request_run(int duty, int direction, int duration_ms);
void control_request_zero(bool want_reply);
void control_request_mark_start();
void control_request_mark_end();
void control_request_visual_config(const char* shaft, const char* reference, int direction, int revolutions,
                                   int max_drive_ms, int max_travel_counts);
void control_request_visual(ReplyKind kind, bool want_reply);
void control_request_cal(bool angle, double number);
void control_request_set(const char* key, const char* value);
void control_request_authority(bool usb);
void control_heartbeat(uint64_t now_ms);
bool control_copy(Snapshot* out);
void control_copy_settings(OperatorSettings* out, bool* usb_authority, int* sign, int* glitch_ns);
bool control_take_reply(char* dest, int size);
bool control_alive(uint32_t now_ms);
void outputs_safe();
