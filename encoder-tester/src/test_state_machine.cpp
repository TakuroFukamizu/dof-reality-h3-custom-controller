#include "control.h"

#include "brake_control.h"
#include "config.h"
#include "encoder_counter.h"
#include "format_util.h"
#include "measurement.h"
#include "motor_driver.h"
#include "power_control.h"
#include "relay_trace.h"
#include "telemetry.h"
#include "visual_revolution_mode.h"

#include <atomic>
#include <cstdarg>
#include <stdio.h>
#include <string.h>

#include <driver/gpio.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <freertos/task.h>

namespace {

struct Requests {
  ReplyKind reply;
  bool arm;
  bool arm_usb;
  bool stop;
  bool estop_pulse;
  bool estop_level;
  bool fault_reset;
  bool run;
  bool run_usb;
  int run_duty;
  int run_dir;
  int run_ms;
  bool zero;
  bool mark_start;
  bool mark_end;
  bool visual_config;
  bool visual_ref;
  bool visual_finish;
  bool visual_cancel;
  bool visual_result;
  bool cal_on;
  bool cal_angle;
  double cal_number;
  bool set_on;
  char set_key[32];
  char set_value[48];
  bool set_authority;
  bool authority_usb;
  char shaft[16];
  char reference[48];
  int visual_dir;
  int visual_revs;
  int visual_drive_ms;
  int visual_travel;
  uint64_t jog_pos_ms;
  uint64_t jog_neg_ms;
  uint64_t ui_beat_ms;
};

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
Requests g_req{};
OperatorSettings g_settings = settings_unconfigured();
uint64_t g_lease_until = 0;
Snapshot g_snap{};
char g_reply[320]{};
bool g_reply_ready = false;
std::atomic<uint32_t> g_beats{0};

void copy_small(char* dest, int size, const char* text) {
  if (dest == nullptr || size <= 0) {
    return;
  }
  if (text == nullptr) {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, text, static_cast<size_t>(size - 1));
  dest[size - 1] = '\0';
}

void eventf(const char* fmt, ...) {
  char line[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  telemetry_push_event(line);
}

void reply_keep(char* reply, const char* text) {
  if (reply[0] != '\0' || text == nullptr) {
    return;
  }
  copy_small(reply, 320, text);
}

void clear_oneshots(Requests* req) {
  req->reply = ReplyKind::None;
  req->arm = false;
  req->stop = false;
  req->estop_pulse = false;
  req->fault_reset = false;
  req->run = false;
  req->zero = false;
  req->mark_start = false;
  req->mark_end = false;
  req->visual_config = false;
  req->visual_ref = false;
  req->visual_finish = false;
  req->visual_cancel = false;
  req->visual_result = false;
  req->cal_on = false;
  req->set_on = false;
  req->set_authority = false;
}

bool authority_ok(bool from_usb, bool usb, char* reply) {
  if (from_usb != usb) {
    reply_keep(reply, "ERR authority");
    return false;
  }
  return true;
}

const VisualRecord* newer(const VisualRecord* best, const VisualRecord& record) {
  if (!record.used || !record.valid) {
    return best;
  }
  if (best == nullptr || record.end_t_us >= best->end_t_us) {
    return &record;
  }
  return best;
}

void task_main(void*) {
  DriveMachine machine;
  machine.reset();
  Measurement meas;
  meas.sign = ENCODER_SIGN_DEFAULT;
  VisualModule visual;
  OperatorSettings settings = settings_unconfigured();
  bool usb = false;
  bool pcnt_ok = g_snap.pcnt_ok;
  bool electrical = g_snap.electrical_ok;
  int glitch_ns = 0;
  int latched_request = 0;
  uint32_t seq = 0;
  uint64_t last_csv_us = 0;
  char cal_text[120];
  copy_small(cal_text, sizeof(cal_text), "PPR UNKNOWN angle UNKNOWN");
  char ppr_text[48];
  copy_small(ppr_text, sizeof(ppr_text), "UNKNOWN");

  const uint64_t boot_us = static_cast<uint64_t>(esp_timer_get_time());
  portENTER_CRITICAL(&mux);
  g_req.ui_beat_ms = boot_us / 1000ULL;
  portEXIT_CRITICAL(&mux);

  if (!pcnt_ok || !electrical) {
    machine.enter_fault(StopReason::Estop, pcnt_ok ? "electrical" : "pcnt", boot_us / 1000ULL);
  }

  for (;;) {
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const uint64_t now_ms = now_us / 1000ULL;

    Requests req{};
    uint64_t lease_until = 0;
    portENTER_CRITICAL(&mux);
    req = g_req;
    clear_oneshots(&g_req);
    lease_until = g_lease_until;
    portEXIT_CRITICAL(&mux);

    char reply[320];
    reply[0] = '\0';

    if (req.set_on) {
      if (machine.st.state != DriveState::Disarmed) {
        reply_keep(reply, "ERR not_disarmed");
      } else {
        const SettingsResult result = settings_apply(&settings, req.set_key, req.set_value);
        if (!result.ok) {
          char line[64];
          snprintf(line, sizeof(line), "ERR %s", result.error);
          reply_keep(reply, line);
        } else if (result.wrote_sign) {
          meas.set_sign(result.sign);
          visual.session.invalidate_at("sign_change", 0, now_us);
          eventf("EVT,%llu,sign,%d", static_cast<unsigned long long>(now_us), result.sign);
          reply_keep(reply, "OK sign");
        } else if (result.wrote_glitch) {
          const bool restarted = encoder_set_glitch(static_cast<uint32_t>(result.glitch_ns));
          int32_t raw_now = 0;
          const bool got = encoder_get(&raw_now);
          if (!got) {
            pcnt_ok = false;
          }
          meas.reset_counter(got ? raw_now : 0);
          const int64_t signed_now = static_cast<int64_t>(meas.sign) * static_cast<int64_t>(got ? raw_now : 0);
          visual.session.invalidate_at("counter_reinit", signed_now, now_us);
          eventf("EVT,%llu,counter_reinit,glitch_ns,%d,ok,%d", static_cast<unsigned long long>(now_us),
                 result.glitch_ns, restarted && got ? 1 : 0);
          if (!pcnt_ok) {
            machine.enter_fault(StopReason::Estop, "pcnt", now_ms);
            reply_keep(reply, "ERR pcnt");
          } else if (!restarted) {
            reply_keep(reply, "ERR glitch");
          } else {
            glitch_ns = result.glitch_ns;
            reply_keep(reply, "OK glitch");
          }
        } else {
          reply_keep(reply, "OK set");
        }
      }
    }

    if (req.set_authority) {
      if (machine.st.state != DriveState::Disarmed) {
        reply_keep(reply, "ERR not_disarmed");
      } else if (req.authority_usb && settings.lease_ms < 1) {
        reply_keep(reply, "ERR lease_unset");
      } else {
        usb = req.authority_usb;
        reply_keep(reply, usb ? "OK authority=usb" : "OK authority=local");
      }
    }

    if (req.visual_config) {
      if (machine.st.state != DriveState::Disarmed) {
        reply_keep(reply, "ERR not_disarmed");
      } else {
        const char* error = "";
        if (!visual.session.configure(req.shaft, req.reference, req.visual_dir, req.visual_revs,
                                      req.visual_drive_ms, req.visual_travel, &error)) {
          reply_keep(reply, "ERR visual_config");
        } else {
          reply_keep(reply, "OK visual_config");
        }
      }
    }

    int32_t raw = 0;
    if (!encoder_get(&raw)) {
      raw = 0;
      if (pcnt_ok) {
        pcnt_ok = false;
        machine.enter_fault(StopReason::Estop, "pcnt", now_ms);
      }
    }
    const int64_t signed_count = meas.sample(raw);
    const int a_at_sample = encoder_a_level();
    const int b_at_sample = encoder_b_level();
    meas.velocity.push(signed_count, now_us);
    const bool threshold_set = settings.stop_speed_threshold >= 0 && settings.stop_confirm_ms >= 0;
    const bool count_stable =
        meas.stability.update(meas.velocity.window_ready, meas.velocity.count_per_s, threshold_set,
                              settings.stop_speed_threshold, settings.stop_confirm_ms, now_ms);

    if (req.mark_start) {
      const char* error = "";
      if (!visual.flying.start(machine.st.state == DriveState::Running, signed_count, now_us, &error)) {
        reply_keep(reply, "ERR not_running");
      } else {
        eventf("EVT,%llu,mark_start,%lld", static_cast<unsigned long long>(now_us),
               static_cast<long long>(signed_count));
        reply_keep(reply, "OK mark_start");
      }
    }
    if (req.mark_end) {
      const char* error = "";
      if (!visual.flying.end_mark(machine.st.state == DriveState::Running, signed_count, now_us, &error)) {
        reply_keep(reply, "ERR mark_end");
      } else {
        req.stop = true;
        eventf("EVT,%llu,mark_end,%lld", static_cast<unsigned long long>(now_us),
               static_cast<long long>(signed_count));
        reply_keep(reply, "OK mark_end");
      }
    }

    const bool ui_fresh = settings.ui_timeout_ms < 1 ||
                          (now_ms >= req.ui_beat_ms &&
                           now_ms - req.ui_beat_ms <= static_cast<uint64_t>(settings.ui_timeout_ms));
    const bool lease_fresh = !usb || (settings.lease_ms >= 1 && now_ms <= lease_until);

    DriveCommand command{};
    command.estop = req.estop_level || req.estop_pulse;
    command.fault_reset = req.fault_reset;
    command.stop = req.stop;
    if (req.arm) {
      if (authority_ok(req.arm_usb, usb, reply) && machine.st.state == DriveState::Disarmed) {
        command.arm = true;
      } else if (machine.st.state != DriveState::Disarmed) {
        reply_keep(reply, "ERR not_disarmed");
      }
    }
    if (req.run) {
      if (authority_ok(true, usb, reply)) {
        command.run = true;
        command.run_duty = req.run_duty;
        command.run_dir = req.run_dir;
        command.run_duration_ms = req.run_ms;
      }
    }
    if (!usb) {
      command.jog_pos = jog_is_fresh(now_ms, req.jog_pos_ms, settings.jog_stale_ms);
      command.jog_neg = jog_is_fresh(now_ms, req.jog_neg_ms, settings.jog_stale_ms);
    }
    command.jog_duty = settings.jog_duty;

    DriveSensors sensors{};
    sensors.now_ms = now_ms;
    sensors.count = signed_count;
    sensors.count_per_s = meas.velocity.count_per_s;
    sensors.ui_fresh = ui_fresh;
    sensors.usb_authority = usb;
    sensors.lease_fresh = lease_fresh;

    DriveConfig config{};
    settings_to_drive(settings, electrical && pcnt_ok, &config);
    const DriveStatus before = machine.st;
    machine.tick(config, command, sensors);

    const int power_gpio_before = power_gpio_level();
    const int brake_gpio_before = brake_gpio_level();
    const bool relay_edge = before.power_on_cmd != machine.st.power_on_cmd ||
                            before.brake_release_cmd != machine.st.brake_release_cmd ||
                            before.applied_duty_percent != machine.st.applied_duty_percent;
    power_apply(machine.st.power_on_cmd);
    brake_apply(machine.st.brake_release_cmd);
    motor_write(machine.st.applied_duty_percent, machine.st.direction);

    TraceSample trace_pre{};
    trace_pre.pcnt_raw = raw;
    trace_pre.count_total = signed_count;
    trace_pre.pwm_raw = static_cast<int16_t>(pwm_raw_from_percent(machine.st.applied_duty_percent));
    trace_pre.power_cmd = machine.st.power_on_cmd ? 1 : 0;
    trace_pre.brake_cmd = machine.st.brake_release_cmd ? 1 : 0;
    trace_pre.power_gpio = static_cast<uint8_t>(power_gpio_before ? 1 : 0);
    trace_pre.brake_gpio = static_cast<uint8_t>(brake_gpio_before ? 1 : 0);
    trace_pre.a_level = static_cast<uint8_t>(a_at_sample ? 1 : 0);
    trace_pre.b_level = static_cast<uint8_t>(b_at_sample ? 1 : 0);
    const bool trace_post = relay_trace_on_tick(
        static_cast<uint8_t>(machine.st.state), machine.st.state != DriveState::Disarmed,
        machine.st.state == DriveState::Disarmed || machine.st.state == DriveState::Fault, relay_edge,
        now_us, trace_pre);
    if (trace_post) {
      int32_t raw_post = raw;
      int64_t count_post = signed_count;
      if (encoder_get(&raw_post)) {
        count_post = meas.sample(raw_post);
      }
      TraceSample trace_after = trace_pre;
      trace_after.pcnt_raw = raw_post;
      trace_after.count_total = count_post;
      trace_after.power_gpio = static_cast<uint8_t>(power_gpio_level() ? 1 : 0);
      trace_after.brake_gpio = static_cast<uint8_t>(brake_gpio_level() ? 1 : 0);
      trace_after.a_level = static_cast<uint8_t>(encoder_a_level() ? 1 : 0);
      trace_after.b_level = static_cast<uint8_t>(encoder_b_level() ? 1 : 0);
      relay_trace_on_post(now_us, static_cast<uint8_t>(machine.st.state), trace_after);
    }

    if (before.state != DriveState::Running && machine.st.state == DriveState::Running) {
      const uint64_t duration_ms =
          machine.st.run_end_ms > now_ms ? machine.st.run_end_ms - now_ms : 0;
      meas.begin_run(machine.st.trial_id, machine.st.applied_duty_percent, machine.st.direction,
                     static_cast<int>(duration_ms), signed_count, now_us);
      latched_request = machine.st.direction * machine.st.applied_duty_percent;
      eventf("EVT,%llu,run,%d,%d,%lld", static_cast<unsigned long long>(now_us), machine.st.trial_id,
             latched_request, static_cast<long long>(signed_count));
    }
    if (machine.st.state == DriveState::Running) {
      meas.note_running(meas.velocity.count_per_s);
    }
    if (before.state == DriveState::Running && machine.st.state != DriveState::Running) {
      meas.end_pwm(now_us, signed_count, machine.st.stop_reason);
      int run_ms = 0;
      if (now_ms >= before.state_since_ms) {
        const uint64_t elapsed = now_ms - before.state_since_ms;
        run_ms = elapsed > 3600000ULL ? 3600000 : static_cast<int>(elapsed);
      }
      visual.session.note_motion(before.direction, run_ms, signed_count, now_us);
      eventf("EVT,%llu,pwm_off,%s,%lld", static_cast<unsigned long long>(now_us),
             stop_reason_name(machine.st.stop_reason), static_cast<long long>(signed_count));
    }
    if (machine.st.state == DriveState::Fault && before.state != DriveState::Fault) {
      meas.fault_run(machine.st.stop_reason);
      visual.session.invalidate_at("fault", signed_count, now_us);
      eventf("EVT,%llu,fault,%s,%lld", static_cast<unsigned long long>(now_us), machine.fault,
             static_cast<long long>(signed_count));
    }
    if (before.state != DriveState::BrakeApply && machine.st.state == DriveState::BrakeApply) {
      meas.complete_stop(now_us, signed_count);
      visual.flying.note_stopped(signed_count, visual.session.cfg.shaft);
      visual.push_flying_result();
      const TrialRecord* trial = meas.find_id(machine.st.trial_id);
      if (trial != nullptr) {
        eventf("EVT,%llu,trial_done,%d,%lld,%lld,%lld", static_cast<unsigned long long>(now_us), trial->id,
               static_cast<long long>(trial->pwm_off_count - trial->start_count),
               static_cast<long long>(trial->coast_counts),
               static_cast<long long>(trial->stop_count - trial->start_count));
      }
    }
    if (machine.st.state == DriveState::Disarmed) {
      latched_request = 0;
    }
    if (before.power_on_cmd != machine.st.power_on_cmd) {
      eventf("EVT,%llu,power_on_cmd,%d", static_cast<unsigned long long>(now_us),
             machine.st.power_on_cmd ? 1 : 0);
    }
    if (before.brake_release_cmd != machine.st.brake_release_cmd) {
      eventf("EVT,%llu,brake_release_cmd,%d", static_cast<unsigned long long>(now_us),
             machine.st.brake_release_cmd ? 1 : 0);
    }

    const bool disarmed = machine.st.state == DriveState::Disarmed;
    if (req.visual_ref) {
      const char* error = "";
      if (!visual.session.capture_reference(disarmed, count_stable, signed_count, now_us, machine.st.trial_id,
                                           &error)) {
        reply_keep(reply, "ERR visual_reference");
      } else {
        eventf("EVT,%llu,visual_reference,%lld", static_cast<unsigned long long>(now_us),
               static_cast<long long>(signed_count));
        reply_keep(reply, "OK visual_reference");
      }
    }
    if (req.visual_finish) {
      const char* error = "";
      if (!visual.session.finish(disarmed, count_stable, signed_count, now_us, machine.st.trial_id, &error)) {
        char line[48];
        snprintf(line, sizeof(line), "ERR %s", error[0] != '\0' ? error : "visual_finish");
        reply_keep(reply, line);
      } else {
        eventf("EVT,%llu,visual_finish,%lld", static_cast<unsigned long long>(now_us),
               static_cast<long long>(visual.session.last.signed_delta_x4));
        reply_keep(reply, "OK visual_finish");
      }
    }
    if (req.visual_cancel) {
      visual.session.cancel();
      reply_keep(reply, "OK visual_cancel");
    }
    if (req.zero) {
      meas.zero_display(signed_count);
      eventf("EVT,%llu,zero_display,%lld", static_cast<unsigned long long>(now_us),
             static_cast<long long>(signed_count));
      reply_keep(reply, "OK zero_display");
    }
    if (req.cal_on) {
      const VisualRecord* best = nullptr;
      const int align_stored = visual.align_n < 12 ? visual.align_n : 12;
      for (int i = 0; i < align_stored; ++i) {
        best = newer(best, visual.align_log[i]);
      }
      const int flying_stored = visual.flying_n < 12 ? visual.flying_n : 12;
      for (int i = 0; i < flying_stored; ++i) {
        best = newer(best, visual.flying_log[i]);
      }
      if (visual.session.last.used && visual.session.last.valid) {
        best = newer(best, visual.session.last);
      }
      int64_t delta = 0;
      bool have = false;
      char shaft[16];
      shaft[0] = '\0';
      if (best != nullptr) {
        delta = best->signed_delta_x4;
        have = true;
        copy_small(shaft, sizeof(shaft), best->shaft);
      } else if (meas.latest_completed_delta(&delta)) {
        have = true;
      }
      CalResult cal{};
      if (!have) {
        reply_keep(reply, "ERR no_delta");
      } else if (req.cal_angle) {
        cal = cal_from_angle(delta, req.cal_number);
      } else {
        cal = cal_from_revolutions(delta, req.cal_number);
      }
      if (have && !cal.ok) {
        char line[48];
        snprintf(line, sizeof(line), "ERR %s", cal.error);
        reply_keep(reply, line);
      } else if (have && cal.ok) {
        char cpr[24];
        char per_deg[24];
        fmt_fixed(cpr, sizeof(cpr), cal.counts_per_measured_rev_x4);
        fmt_fixed(per_deg, sizeof(per_deg), cal.counts_per_degree);
        double ppr = 0.0;
        const bool ppr_ok = ppr_from_encoder_shaft(shaft, cal.counts_per_measured_rev_x4, &ppr);
        if (ppr_ok) {
          char ppr_num[24];
          fmt_fixed(ppr_num, sizeof(ppr_num), ppr);
          snprintf(ppr_text, sizeof(ppr_text), "%s observed_not_nominal", ppr_num);
        } else {
          copy_small(ppr_text, sizeof(ppr_text), "UNKNOWN");
        }
        snprintf(cal_text, sizeof(cal_text), "obs %s count/rev x4 shaft=%s ppr=%s not_nominal", cpr,
                 shaft[0] != '\0' ? shaft : "trial", ppr_ok ? ppr_text : "UNKNOWN");
        char line[300];
        snprintf(line, sizeof(line), "OK cal cpr=%s per_deg=%s ppr=%s", cpr, per_deg,
                 ppr_ok ? "encoder_shaft" : "UNKNOWN");
        reply_keep(reply, line);
        eventf("EVT,%llu,cal,%s", static_cast<unsigned long long>(now_us), cal_text);
      }
    }

    visual.collect();

    if (reply[0] == '\0') {
      if (req.reply == ReplyKind::Arm || req.reply == ReplyKind::Run || req.reply == ReplyKind::Stop ||
          req.reply == ReplyKind::Estop || req.reply == ReplyKind::FaultReset) {
        if (machine.reject[0] != '\0') {
          snprintf(reply, sizeof(reply), "ERR %s", machine.reject);
        } else {
          snprintf(reply, sizeof(reply), "OK state=%s", drive_state_name(machine.st.state));
        }
      } else if (req.reply == ReplyKind::VisualResult) {
        VisualStats stats{};
        visual.stats_align(visual.session.cfg.direction, &stats);
        char mean[24];
        fmt_fixed(mean, sizeof(mean), stats.mean);
        snprintf(reply, sizeof(reply), "OK visual state=%s valid_n=%d mean=%s",
                 visual_state_name(visual.session.state), stats.valid_n, stats.valid_n > 0 ? mean : "na");
        const int align_stored = visual.align_n < 12 ? visual.align_n : 12;
        for (int i = 0; i < align_stored; ++i) {
          const VisualRecord& record = visual.align_log[i];
          if (!record.used) {
            continue;
          }
          eventf("EVT,%llu,visual_record,%s,%s,%d,%lld,%d,%s", static_cast<unsigned long long>(now_us),
                 record.method, record.shaft, record.direction, static_cast<long long>(record.signed_delta_x4),
                 record.valid ? 1 : 0, record.valid ? "ok" : record.invalid_reason);
        }
      }
    }

    const bool show_trial = machine.st.state == DriveState::Running || machine.st.state == DriveState::Stopping ||
                            machine.st.state == DriveState::BrakeApply || machine.st.state == DriveState::PowerOff;
    int64_t trial_delta = 0;
    int travel_abs = 0;
    if (show_trial && machine.st.trial_id > 0) {
      trial_delta = signed_count - machine.st.run_origin_count;
      const int64_t abs_delta = trial_delta < 0 ? -trial_delta : trial_delta;
      travel_abs = abs_delta > 2000000000LL ? 2000000000 : static_cast<int>(abs_delta);
    }
    int elapsed_ms = 0;
    if (show_trial && meas.run_start_us > 0 && now_us >= meas.run_start_us) {
      const uint64_t elapsed = (now_us - meas.run_start_us) / 1000ULL;
      elapsed_ms = elapsed > 2000000000ULL ? 2000000000 : static_cast<int>(elapsed);
    }
    uint32_t remaining_ms = 0;
    if (machine.st.state == DriveState::Running && machine.st.run_end_ms > now_ms) {
      remaining_ms = static_cast<uint32_t>(machine.st.run_end_ms - now_ms);
    }
    uint32_t ready_remaining_ms = 0;
    if (machine.st.state == DriveState::Ready &&
        now_ms - machine.st.state_since_ms < static_cast<uint64_t>(settings.ready_hold_ms)) {
      ready_remaining_ms = static_cast<uint32_t>(static_cast<uint64_t>(settings.ready_hold_ms) -
                                                (now_ms - machine.st.state_since_ms));
    }

    if (telemetry_streaming() && (last_csv_us == 0 || now_us - last_csv_us >= static_cast<uint64_t>(CSV_PERIOD_US))) {
      last_csv_us = now_us;
      seq += 1;
      const uint32_t drops = telemetry_drops();
      char line[240];
      const char* unit = (machine.st.state != DriveState::Disarmed && latched_request != 0) ? "duty_percent" : "none";
      snprintf(line, sizeof(line),
               "%lu,%llu,%d,1,%s,%d,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d,%lld,%lld,%d,%d,%d,%d,%s,%lu",
               static_cast<unsigned long>(seq), static_cast<unsigned long long>(now_us), machine.st.trial_id,
               drive_state_name(machine.st.state), machine.st.power_on_cmd ? 1 : 0, power_gpio_level(),
               machine.st.brake_release_cmd ? 1 : 0, brake_gpio_level(), latched_request, unit,
               machine.st.applied_duty_percent, pwm_raw_from_percent(machine.st.applied_duty_percent),
               PWM_FULL_SCALE, motor_pwm_hz(), motor_dir_level(), static_cast<long long>(signed_count),
               static_cast<long long>(trial_delta), meas.velocity.count_per_s, encoder_a_level(),
               encoder_b_level(), elapsed_ms, stop_reason_name(machine.st.stop_reason),
               static_cast<unsigned long>(drops));
      telemetry_push_sample(line);
    }

    VisualStats stats{};
    visual.stats_align(visual.session.cfg.direction, &stats);
    Snapshot snap{};
    snap.state = machine.st.state;
    snap.stop_reason = machine.st.stop_reason;
    snap.power_on_cmd = machine.st.power_on_cmd;
    snap.brake_release_cmd = machine.st.brake_release_cmd;
    snap.power_gpio = power_gpio_level();
    snap.brake_gpio = brake_gpio_level();
    snap.applied_duty = machine.st.applied_duty_percent;
    snap.requested_signed = latched_request;
    snap.direction = machine.st.direction;
    snap.pwm_raw = pwm_raw_from_percent(machine.st.applied_duty_percent);
    snap.pwm_hz = motor_pwm_hz();
    snap.dir_level = motor_dir_level();
    snap.count_signed = signed_count;
    snap.count_display = displayed_count(signed_count, meas.display_offset);
    snap.count_trial = trial_delta;
    snap.count_per_s = meas.velocity.count_per_s;
    snap.a_level = encoder_a_level();
    snap.b_level = encoder_b_level();
    snap.trial_id = machine.st.trial_id;
    snap.remaining_ms = remaining_ms;
    snap.ready_remaining_ms = ready_remaining_ms;
    snap.travel_abs = travel_abs;
    snap.max_travel = settings.max_travel_counts;
    snap.count_stable = count_stable;
    snap.limits_ok = config.limits_ok;
    snap.electrical_ok = electrical;
    snap.pcnt_ok = pcnt_ok;
    snap.usb_authority = usb;
    snap.stream_on = telemetry_streaming();
    snap.log_drops = telemetry_drops();
    copy_small(snap.fault, sizeof(snap.fault), machine.fault);
    copy_small(snap.reject, sizeof(snap.reject), machine.reject);
    settings_missing(settings, snap.missing, sizeof(snap.missing));
    if (!pcnt_ok) {
      copy_small(snap.lock_reason, sizeof(snap.lock_reason), "pcnt_init");
    } else if (!electrical) {
      copy_small(snap.lock_reason, sizeof(snap.lock_reason), "electrical");
    } else if (!config.limits_ok) {
      copy_small(snap.lock_reason, sizeof(snap.lock_reason), "limits_unset");
    }
    snap.visual_state = visual.session.state;
    snap.visual_have_delta = visual.session.state == VisualState::ReferenceCaptured ||
                             visual.session.state == VisualState::Aligning ||
                             visual.session.state == VisualState::Result ||
                             visual.session.state == VisualState::Invalid;
    if (visual.session.state == VisualState::Result) {
      snap.visual_delta = visual.session.last.signed_delta_x4;
    } else if (snap.visual_have_delta) {
      snap.visual_delta = signed_count - visual.session.start_count;
    }
    snap.visual_valid = visual.session.last.valid && visual.session.state == VisualState::Result;
    snap.visual_reversal = visual.session.reversal || visual.session.last.reversal_detected;
    snap.visual_cpr = snap.visual_valid ? visual.session.last.counts_per_measured_shaft_rev_x4 : 0.0;
    copy_small(snap.visual_shaft, sizeof(snap.visual_shaft), visual.session.cfg.shaft);
    copy_small(snap.visual_reason, sizeof(snap.visual_reason),
               visual.session.state == VisualState::Invalid ? visual.session.invalid_reason
                                                           : visual.session.last.invalid_reason);
    visual.format_runs(visual.session.cfg.direction, snap.visual_runs, sizeof(snap.visual_runs));
    snap.visual_valid_n = stats.valid_n;
    snap.visual_mean = stats.mean;
    snap.visual_min = stats.min_v;
    snap.visual_max = stats.max_v;
    snap.visual_spread = stats.spread;
    copy_small(snap.ppr_text, sizeof(snap.ppr_text), ppr_text);
    copy_small(snap.cal_text, sizeof(snap.cal_text), cal_text);
    snap.jog_duty = settings.jog_duty;
    snap.sign = meas.sign;
    snap.glitch_ns = glitch_ns;

    portENTER_CRITICAL(&mux);
    g_settings = settings;
    g_snap = snap;
    if (reply[0] != '\0') {
      copy_small(g_reply, sizeof(g_reply), reply);
      g_reply_ready = true;
    }
    portEXIT_CRITICAL(&mux);

    g_beats.fetch_add(1, std::memory_order_relaxed);
    vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

}  // namespace

void outputs_safe() {
  motor_write(0, 1);
  brake_force_off();
  power_force_off();
}

void control_begin(bool pcnt_ok, bool electrical_ok) {
  g_settings = settings_unconfigured();
  g_snap.pcnt_ok = pcnt_ok;
  g_snap.electrical_ok = electrical_ok;
  g_snap.state = DriveState::Disarmed;
  copy_small(g_snap.ppr_text, sizeof(g_snap.ppr_text), "UNKNOWN");
  copy_small(g_snap.cal_text, sizeof(g_snap.cal_text), "PPR UNKNOWN angle UNKNOWN");
  settings_missing(g_settings, g_snap.missing, sizeof(g_snap.missing));
  copy_small(g_snap.lock_reason, sizeof(g_snap.lock_reason),
             !pcnt_ok ? "pcnt_init" : (!electrical_ok ? "electrical" : "limits_unset"));
  xTaskCreatePinnedToCore(task_main, "drive", 16384, nullptr, 5, nullptr, 1);
}

void control_set_inputs(uint64_t now_ms, bool estop_level, bool jog_pos, bool jog_neg) {
  portENTER_CRITICAL(&mux);
  g_req.ui_beat_ms = now_ms;
  g_req.estop_level = estop_level;
  g_req.jog_pos_ms = jog_pos ? now_ms : 0;
  g_req.jog_neg_ms = jog_neg ? now_ms : 0;
  portEXIT_CRITICAL(&mux);
}

void control_request_arm(bool from_usb) {
  portENTER_CRITICAL(&mux);
  g_req.arm = true;
  g_req.arm_usb = from_usb;
  if (from_usb && g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Arm;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_stop(bool want_reply) {
  portENTER_CRITICAL(&mux);
  g_req.stop = true;
  if (want_reply && g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Stop;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_estop(bool want_reply) {
  portENTER_CRITICAL(&mux);
  g_req.estop_pulse = true;
  if (want_reply && g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Estop;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_fault_reset(bool want_reply) {
  portENTER_CRITICAL(&mux);
  g_req.fault_reset = true;
  if (want_reply && g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::FaultReset;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_run(int duty, int direction, int duration_ms) {
  portENTER_CRITICAL(&mux);
  g_req.run = true;
  g_req.run_usb = true;
  g_req.run_duty = duty;
  g_req.run_dir = direction;
  g_req.run_ms = duration_ms;
  if (g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Run;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_zero(bool want_reply) {
  portENTER_CRITICAL(&mux);
  g_req.zero = true;
  if (want_reply && g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Zero;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_mark_start() {
  portENTER_CRITICAL(&mux);
  g_req.mark_start = true;
  if (g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::MarkStart;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_mark_end() {
  portENTER_CRITICAL(&mux);
  g_req.mark_end = true;
  if (g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::MarkEnd;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_visual_config(const char* shaft, const char* reference, int direction, int revolutions,
                                   int max_drive_ms, int max_travel_counts) {
  portENTER_CRITICAL(&mux);
  copy_small(g_req.shaft, sizeof(g_req.shaft), shaft);
  copy_small(g_req.reference, sizeof(g_req.reference), reference);
  g_req.visual_dir = direction;
  g_req.visual_revs = revolutions;
  g_req.visual_drive_ms = max_drive_ms;
  g_req.visual_travel = max_travel_counts;
  g_req.visual_config = true;
  if (g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::VisualConfig;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_visual(ReplyKind kind, bool want_reply) {
  portENTER_CRITICAL(&mux);
  if (kind == ReplyKind::VisualRef) g_req.visual_ref = true;
  if (kind == ReplyKind::VisualFinish) g_req.visual_finish = true;
  if (kind == ReplyKind::VisualCancel) g_req.visual_cancel = true;
  if (kind == ReplyKind::VisualResult) g_req.visual_result = true;
  if (want_reply && g_req.reply == ReplyKind::None) {
    g_req.reply = kind;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_cal(bool angle, double number) {
  portENTER_CRITICAL(&mux);
  g_req.cal_on = true;
  g_req.cal_angle = angle;
  g_req.cal_number = number;
  if (g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Cal;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_set(const char* key, const char* value) {
  portENTER_CRITICAL(&mux);
  copy_small(g_req.set_key, sizeof(g_req.set_key), key);
  copy_small(g_req.set_value, sizeof(g_req.set_value), value);
  g_req.set_on = true;
  if (g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Set;
  }
  portEXIT_CRITICAL(&mux);
}

void control_request_authority(bool usb) {
  portENTER_CRITICAL(&mux);
  g_req.set_authority = true;
  g_req.authority_usb = usb;
  if (g_req.reply == ReplyKind::None) {
    g_req.reply = ReplyKind::Authority;
  }
  portEXIT_CRITICAL(&mux);
}

void control_heartbeat(uint64_t now_ms) {
  portENTER_CRITICAL(&mux);
  if (g_settings.lease_ms >= 1) {
    g_lease_until = now_ms + static_cast<uint64_t>(g_settings.lease_ms);
  }
  portEXIT_CRITICAL(&mux);
}

bool control_copy(Snapshot* out) {
  if (out == nullptr) {
    return false;
  }
  portENTER_CRITICAL(&mux);
  *out = g_snap;
  portEXIT_CRITICAL(&mux);
  return true;
}

void control_copy_settings(OperatorSettings* out, bool* usb_authority, int* sign, int* glitch_ns) {
  portENTER_CRITICAL(&mux);
  if (out != nullptr) {
    *out = g_settings;
  }
  if (usb_authority != nullptr) {
    *usb_authority = g_snap.usb_authority;
  }
  if (sign != nullptr) {
    *sign = g_snap.sign;
  }
  if (glitch_ns != nullptr) {
    *glitch_ns = g_snap.glitch_ns;
  }
  portEXIT_CRITICAL(&mux);
}

bool control_take_reply(char* dest, int size) {
  if (dest == nullptr || size <= 1) {
    return false;
  }
  bool ready = false;
  portENTER_CRITICAL(&mux);
  if (g_reply_ready) {
    copy_small(dest, size, g_reply);
    g_reply_ready = false;
    ready = true;
  }
  portEXIT_CRITICAL(&mux);
  return ready;
}

bool control_alive(uint32_t now_ms) {
  static uint32_t seen = 0;
  static uint32_t changed = 0;
  const uint32_t beats = g_beats.load(std::memory_order_relaxed);
  if (beats != seen) {
    seen = beats;
    changed = now_ms;
    return true;
  }
  if (changed == 0) {
    changed = now_ms;
    return true;
  }
  return now_ms - changed < 250;
}
