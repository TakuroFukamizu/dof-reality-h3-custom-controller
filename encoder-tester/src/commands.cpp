#include <M5Unified.h>

#include <stdio.h>
#include <string.h>

#include "command_parse.h"
#include "commands.h"
#include "config.h"
#include "control.h"
#include "format_util.h"
#include "settings.h"
#include "telemetry.h"

namespace {

char line_buf[192];
size_t line_len = 0;
bool overflow = false;
bool waiting = false;
uint32_t wait_since = 0;

void say(const char* text) { Serial.println(text); }

void arm_wait() {
  waiting = true;
  wait_since = millis();
}

void print_help() {
  say("HELP encoder-tester. Examples are syntax, not setpoints.");
  say("STATUS");
  say("ARM");
  say("DISARM");
  say("POWER ON");
  say("POWER OFF");
  say("BRAKE RELEASE");
  say("BRAKE APPLY");
  say("RUN duty_pct=VALUE direction=positive|negative duration_ms=VALUE");
  say("STOP");
  say("ESTOP");
  say("FAULT RESET");
  say("ZERO DISPLAY");
  say("MARK START");
  say("MARK END");
  say("VISUAL CONFIG shaft=output reference=VALUE direction=positive revolutions=VALUE max_drive_ms=VALUE max_travel_counts=VALUE");
  say("VISUAL REFERENCE");
  say("VISUAL FINISH");
  say("VISUAL CANCEL");
  say("VISUAL RESULT");
  say("CAL ANGLE VALUE");
  say("CAL REVS VALUE");
  say("STREAM ON");
  say("STREAM OFF");
  say("HEARTBEAT");
  say("AUTHORITY LOCAL");
  say("AUTHORITY USB");
  say("CONFIG");
  say("SET KEY VALUE");
  say("SET only while DISARMED. Ranges are input checks, not recommended values.");
  say("MAX_DUTY_PERCENT 1..100 percent of 1023");
  say("MAX_RUN_MS 1..3600000 ms");
  say("MAX_TRAVEL_COUNTS 1..1000000000 count");
  say("POWER_SETTLE_MS 0..600000 ms");
  say("BRAKE_RELEASE_WAIT_MS 0..600000 ms");
  say("BRAKE_APPLY_WAIT_MS 0..600000 ms");
  say("STOP_SPEED_THRESHOLD 0..100000000 count/s");
  say("STOP_CONFIRM_MS 0..600000 ms");
  say("STOP_TIMEOUT_MS 1..600000 ms and greater than STOP_CONFIRM_MS");
  say("READY_HOLD_MS 1..3600000 ms");
  say("START_GRACE_MS 0..600000 ms");
  say("MAX_COUNT_PER_S 1..2000000000 count/s");
  say("UI_TIMEOUT_MS 1..60000 ms");
  say("LEASE_MS 1..60000 ms");
  say("JOG_STALE_MS 1..5000 ms");
  say("JOG_DUTY 1..100 percent. Does not unlock ARM.");
  say("ENCODER_SIGN 1 or -1");
  say("GLITCH_NS 0..1000000 ns. 0 disables the filter.");
  say("Duty 100 percent is PWM 1023. 50 percent is 511. 1 percent is 10.");
  say("CURRENT MONITOR NOT CONNECTED");
  say("ESTOP here is a latched software output-off. It is not a physical e-stop.");
}

void print_status() {
  Snapshot snap{};
  control_copy(&snap);
  char line[480];
  snprintf(line, sizeof(line),
           "STAT state=%s stop=%s power_on_cmd=%d power_gpio=%d brake_release_cmd=%d brake_gpio=%d "
           "requested=%d applied_duty_pct=%d pwm_raw=%d pwm_full=%d pwm_hz=%d dir_level=%d",
           drive_state_name(snap.state), stop_reason_name(snap.stop_reason), snap.power_on_cmd ? 1 : 0,
           snap.power_gpio, snap.brake_release_cmd ? 1 : 0, snap.brake_gpio, snap.requested_signed,
           snap.applied_duty, snap.pwm_raw, PWM_FULL_SCALE, snap.pwm_hz, snap.dir_level);
  say(line);
  snprintf(line, sizeof(line),
           "STAT count_total=%lld count_display=%lld count_trial=%lld count_per_s=%d a=%d b=%d stable=%d "
           "trial=%d rem_ms=%lu ready_ms=%lu travel=%d max_travel=%d",
           static_cast<long long>(snap.count_signed), static_cast<long long>(snap.count_display),
           static_cast<long long>(snap.count_trial), snap.count_per_s, snap.a_level, snap.b_level,
           snap.count_stable ? 1 : 0, snap.trial_id, static_cast<unsigned long>(snap.remaining_ms),
           static_cast<unsigned long>(snap.ready_remaining_ms), snap.travel_abs, snap.max_travel);
  say(line);
  snprintf(line, sizeof(line),
           "STAT lock=%s fault=%s reject=%s visual=%s visual_valid=%d authority=%s stream=%d drops=%lu",
           snap.lock_reason, snap.fault, snap.reject, visual_state_name(snap.visual_state),
           snap.visual_valid ? 1 : 0, snap.usb_authority ? "usb" : "local", snap.stream_on ? 1 : 0,
           static_cast<unsigned long>(snap.log_drops));
  say(line);
  snprintf(line, sizeof(line), "STAT missing=%s", snap.missing);
  say(line);
  snprintf(line, sizeof(line), "STAT ppr=%s", snap.ppr_text);
  say(line);
  snprintf(line, sizeof(line), "STAT cal=%s", snap.cal_text);
  say(line);
  if (snap.visual_valid) {
    char cpr[24];
    fmt_fixed(cpr, sizeof(cpr), snap.visual_cpr);
    snprintf(line, sizeof(line), "STAT visual_cpr=%s shaft=%s", cpr, snap.visual_shaft);
    say(line);
  }
}

void print_one(const char* key, int value, const char* unit) {
  char line[80];
  snprintf(line, sizeof(line), "CFG %s=%d %s", key, value, unit);
  say(line);
}

void print_config() {
  OperatorSettings settings{};
  bool usb = false;
  int sign = ENCODER_SIGN_DEFAULT;
  int glitch_ns = 0;
  control_copy_settings(&settings, &usb, &sign, &glitch_ns);
  Snapshot snap{};
  control_copy(&snap);
  print_one("MAX_DUTY_PERCENT", settings.max_duty_percent, "percent_of_1023");
  print_one("MAX_RUN_MS", settings.max_run_ms, "ms");
  print_one("MAX_TRAVEL_COUNTS", settings.max_travel_counts, "count");
  print_one("POWER_SETTLE_MS", settings.power_settle_ms, "ms");
  print_one("BRAKE_RELEASE_WAIT_MS", settings.brake_release_wait_ms, "ms");
  print_one("BRAKE_APPLY_WAIT_MS", settings.brake_apply_wait_ms, "ms");
  print_one("STOP_SPEED_THRESHOLD", settings.stop_speed_threshold, "count_per_s");
  print_one("STOP_CONFIRM_MS", settings.stop_confirm_ms, "ms");
  print_one("STOP_TIMEOUT_MS", settings.stop_timeout_ms, "ms");
  print_one("READY_HOLD_MS", settings.ready_hold_ms, "ms");
  print_one("START_GRACE_MS", settings.start_grace_ms, "ms");
  print_one("MAX_COUNT_PER_S", settings.max_count_per_s, "count_per_s");
  print_one("UI_TIMEOUT_MS", settings.ui_timeout_ms, "ms");
  print_one("LEASE_MS", settings.lease_ms, "ms");
  print_one("JOG_STALE_MS", settings.jog_stale_ms, "ms");
  print_one("JOG_DUTY", settings.jog_duty, "percent_of_1023");
  print_one("ENCODER_SIGN", sign, "sign");
  print_one("GLITCH_NS", glitch_ns, "ns");
  char line[420];
  snprintf(line, sizeof(line), "CFG AUTHORITY=%s LOCK=%s", usb ? "usb" : "local", snap.lock_reason);
  say(line);
  snprintf(line, sizeof(line), "CFG MISSING=%s", snap.missing);
  say(line);
  snprintf(line, sizeof(line), "CFG PINS power=%d brake=%d pwm=%d dir=%d enc_a=%d enc_b=%d", POWER_GPIO,
           BRAKE_GPIO, PWM_GPIO, DIR_GPIO, ENCODER_A_GPIO, ENCODER_B_GPIO);
  say(line);
  say("CFG WARN not_physical_estop current_monitor_absent g2_may_feed_all_axes one_cores3_only");
  say("CFG -1 means UNCONFIGURED");
}

void print_brake_release() {
  Snapshot snap{};
  control_copy(&snap);
  if (snap.brake_release_cmd) {
    say("OK already");
    return;
  }
  say("ERR brake_sequence");
}

void queue_command(CmdKind kind, const ParsedCommand& command) {
  switch (kind) {
    case CmdKind::Arm:
    case CmdKind::PowerOn:
      control_request_arm(true);
      break;
    case CmdKind::Disarm:
    case CmdKind::PowerOff:
    case CmdKind::BrakeApply:
      control_request_stop(true);
      break;
    case CmdKind::Run:
      control_request_run(command.duty_pct, command.direction, command.duration_ms);
      break;
    case CmdKind::Stop:
      control_request_stop(true);
      break;
    case CmdKind::Estop:
      control_request_estop(true);
      break;
    case CmdKind::FaultReset:
      control_request_fault_reset(true);
      break;
    case CmdKind::ZeroDisplay:
      control_request_zero(true);
      break;
    case CmdKind::MarkStart:
      control_request_mark_start();
      break;
    case CmdKind::MarkEnd:
      control_request_mark_end();
      break;
    case CmdKind::VisualConfig:
      control_request_visual_config(command.shaft, command.reference, command.direction, command.revolutions,
                                    command.max_drive_ms, command.max_travel_counts);
      break;
    case CmdKind::VisualReference:
      control_request_visual(ReplyKind::VisualRef, true);
      break;
    case CmdKind::VisualFinish:
      control_request_visual(ReplyKind::VisualFinish, true);
      break;
    case CmdKind::VisualCancel:
      control_request_visual(ReplyKind::VisualCancel, true);
      break;
    case CmdKind::VisualResult:
      control_request_visual(ReplyKind::VisualResult, true);
      break;
    case CmdKind::CalAngle:
      control_request_cal(true, command.number);
      break;
    case CmdKind::CalRevs:
      control_request_cal(false, command.number);
      break;
    case CmdKind::Set:
      control_request_set(command.key, command.value);
      break;
    case CmdKind::AuthorityLocal:
      control_request_authority(false);
      break;
    case CmdKind::AuthorityUsb:
      control_request_authority(true);
      break;
    default:
      say("ERR unknown");
      return;
  }
  arm_wait();
}

void handle(const char* line) {
  const ParsedCommand command = parse_command(line);
  if (!command.ok) {
    char text[64];
    snprintf(text, sizeof(text), "ERR %s", command.error != nullptr ? command.error : "unknown");
    say(text);
    return;
  }
  if (waiting) {
    if (command.kind == CmdKind::Heartbeat) {
      control_heartbeat(millis());
      say("OK heartbeat");
      return;
    }
    if (command.kind == CmdKind::Stop || command.kind == CmdKind::Disarm || command.kind == CmdKind::PowerOff) {
      control_request_stop(false);
      say("OK stop");
      return;
    }
    if (command.kind == CmdKind::Estop) {
      control_request_estop(false);
      say("OK estop");
      return;
    }
    say("ERR busy");
    return;
  }
  switch (command.kind) {
    case CmdKind::None:
      break;
    case CmdKind::Help:
      print_help();
      break;
    case CmdKind::Status:
      print_status();
      break;
    case CmdKind::Config:
      print_config();
      break;
    case CmdKind::StreamOn:
      telemetry_set_stream(true);
      say("OK stream=on");
      break;
    case CmdKind::StreamOff:
      telemetry_set_stream(false);
      say("OK stream=off");
      break;
    case CmdKind::Heartbeat:
      control_heartbeat(millis());
      say("OK heartbeat");
      break;
    case CmdKind::BrakeRelease:
      print_brake_release();
      break;
    default:
      queue_command(command.kind, command);
      break;
  }
}

}  // namespace

void commands_poll() {
  if (waiting) {
    char reply[320];
    if (control_take_reply(reply, sizeof(reply))) {
      say(reply);
      waiting = false;
    } else if (static_cast<int32_t>(millis() - wait_since) > 500) {
      control_take_reply(reply, sizeof(reply));
      say("ERR control_timeout");
      waiting = false;
    }
  }

  while (Serial.available() > 0) {
    const int raw = Serial.read();
    if (raw < 0) {
      break;
    }
    const char c = static_cast<char>(raw);
    if (c == '\n') {
      line_buf[line_len] = '\0';
      if (overflow) {
        say("ERR line");
      } else if (line_len > 0) {
        handle(line_buf);
      }
      line_len = 0;
      overflow = false;
      continue;
    }
    if (c == '\r') {
      continue;
    }
    if (line_len + 1 < sizeof(line_buf)) {
      line_buf[line_len++] = c;
    } else {
      overflow = true;
    }
  }
}
