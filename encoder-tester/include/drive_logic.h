#pragma once

#include <stdint.h>
#include <string.h>

// 駆動状態機械。GPIO も時計も持たない。制御タスクが時刻とカウントを渡す。

enum class DriveState : uint8_t {
  Disarmed,
  PowerWait,
  BrakeWait,
  Ready,
  Running,
  Stopping,
  BrakeApply,
  PowerOff,
  Fault
};

enum class StopReason : uint8_t {
  None,
  Operator,
  JogRelease,
  Duration,
  Travel,
  Lease,
  UiTimeout,
  EncoderStall,
  Overspeed,
  Estop,
  ReadyTimeout,
  StopTimeout,
  BothJog,
  ReverseRequest
};

struct DriveConfig {
  bool electrical_ok;
  bool limits_ok;
  int max_duty_percent;
  int max_run_ms;
  int max_travel_counts;
  int power_settle_ms;
  int brake_release_wait_ms;
  int brake_apply_wait_ms;
  int stop_speed_threshold;
  int stop_confirm_ms;
  int stop_timeout_ms;
  int ready_hold_ms;
  int start_grace_ms;
  int max_count_per_s;
};

struct DriveCommand {
  bool arm;
  bool stop;
  bool estop;
  bool fault_reset;
  bool jog_pos;
  bool jog_neg;
  int jog_duty;
  bool run;
  int run_duty;
  int run_dir;
  int run_duration_ms;
};

struct DriveSensors {
  uint64_t now_ms;
  int64_t count;
  int count_per_s;
  bool ui_fresh;
  bool usb_authority;
  bool lease_fresh;
};

struct DriveStatus {
  DriveState state;
  StopReason stop_reason;
  bool power_on_cmd;
  bool brake_release_cmd;
  int applied_duty_percent;
  int direction;
  int64_t run_origin_count;
  int64_t count_at_pwm_off;
  int64_t coast_counts;
  bool coast_valid;
  uint64_t run_end_ms;
  uint64_t state_since_ms;
  uint64_t slow_since_ms;
  bool slow_timing;
  bool timed_run;
  int trial_id;
};

inline const char* drive_state_name(DriveState state) {
  switch (state) {
    case DriveState::Disarmed:
      return "DISARMED";
    case DriveState::PowerWait:
      return "POWER_WAIT";
    case DriveState::BrakeWait:
      return "BRAKE_WAIT";
    case DriveState::Ready:
      return "READY";
    case DriveState::Running:
      return "RUNNING";
    case DriveState::Stopping:
      return "STOPPING";
    case DriveState::BrakeApply:
      return "BRAKE_APPLY";
    case DriveState::PowerOff:
      return "POWER_OFF";
    case DriveState::Fault:
      return "FAULT";
  }
  return "UNKNOWN";
}

inline const char* stop_reason_name(StopReason reason) {
  switch (reason) {
    case StopReason::None:
      return "";
    case StopReason::Operator:
      return "operator";
    case StopReason::JogRelease:
      return "jog_release";
    case StopReason::Duration:
      return "duration";
    case StopReason::Travel:
      return "travel";
    case StopReason::Lease:
      return "lease";
    case StopReason::UiTimeout:
      return "ui_timeout";
    case StopReason::EncoderStall:
      return "encoder_stall";
    case StopReason::Overspeed:
      return "overspeed";
    case StopReason::Estop:
      return "estop";
    case StopReason::ReadyTimeout:
      return "ready_timeout";
    case StopReason::StopTimeout:
      return "stop_timeout";
    case StopReason::BothJog:
      return "both_jog";
    case StopReason::ReverseRequest:
      return "reverse_request";
  }
  return "unknown";
}

// 押下時刻が 0、または期限を過ぎていればジョグは離されたものとして扱う。
inline bool jog_is_fresh(uint64_t now_ms, uint64_t stamp_ms, int stale_ms) {
  if (stamp_ms == 0 || stale_ms < 1 || now_ms < stamp_ms) {
    return false;
  }
  return now_ms - stamp_ms <= static_cast<uint64_t>(stale_ms);
}

inline void copy_text(char* dest, int size, const char* text) {
  if (size <= 0) {
    return;
  }
  if (text == nullptr) {
    dest[0] = '\0';
    return;
  }
  strncpy(dest, text, static_cast<size_t>(size - 1));
  dest[size - 1] = '\0';
}

struct DriveMachine {
  DriveStatus st{};
  char fault[48]{};
  char reject[48]{};

  void reset() {
    st = DriveStatus{};
    st.state = DriveState::Disarmed;
    st.direction = 1;
    fault[0] = '\0';
    reject[0] = '\0';
  }

  void outputs_off() {
    st.power_on_cmd = false;
    st.brake_release_cmd = false;
    st.applied_duty_percent = 0;
  }

  void enter_fault(StopReason reason, const char* text, uint64_t now_ms) {
    outputs_off();
    st.state = DriveState::Fault;
    st.stop_reason = reason;
    st.state_since_ms = now_ms;
    copy_text(fault, sizeof(fault), text);
  }

  void enter_power_off(uint64_t now_ms) {
    outputs_off();
    st.state = DriveState::PowerOff;
    st.state_since_ms = now_ms;
  }

  void enter_brake_apply(uint64_t now_ms, int64_t count) {
    st.coast_counts = count - st.count_at_pwm_off;
    st.coast_valid = true;
    st.applied_duty_percent = 0;
    st.brake_release_cmd = false;
    st.state = DriveState::BrakeApply;
    st.state_since_ms = now_ms;
    st.slow_timing = false;
  }

  void enter_stop(StopReason reason, uint64_t now_ms, int64_t count) {
    if (st.state == DriveState::Stopping || st.state == DriveState::BrakeApply ||
        st.state == DriveState::PowerOff || st.state == DriveState::Fault ||
        st.state == DriveState::Disarmed) {
      return;
    }
    st.stop_reason = reason;
    st.applied_duty_percent = 0;
    if (st.state == DriveState::PowerWait || !st.brake_release_cmd) {
      enter_power_off(now_ms);
      return;
    }
    st.count_at_pwm_off = count;
    st.coast_valid = false;
    st.state = DriveState::Stopping;
    st.state_since_ms = now_ms;
    st.slow_timing = false;
  }

  bool duty_ok(const DriveConfig& cfg, int duty) const {
    return duty >= 1 && duty <= cfg.max_duty_percent;
  }

  void enter_run(const DriveConfig& cfg, const DriveSensors& sen, int duty, int dir, int duration_ms,
                 bool timed) {
    st.state = DriveState::Running;
    st.direction = dir;
    st.applied_duty_percent = duty;
    st.timed_run = timed;
    st.run_origin_count = sen.count;
    st.run_end_ms = sen.now_ms + static_cast<uint64_t>(duration_ms);
    st.state_since_ms = sen.now_ms;
    st.stop_reason = StopReason::None;
    st.trial_id += 1;
    st.coast_valid = false;
    (void)cfg;
  }

  void tick(const DriveConfig& cfg, const DriveCommand& cmd, const DriveSensors& sen) {
    reject[0] = '\0';

    if (cmd.estop && st.state != DriveState::Fault) {
      enter_fault(StopReason::Estop, "estop", sen.now_ms);
    }

    if (st.state == DriveState::Fault) {
      outputs_off();
      if (cmd.fault_reset && !cmd.estop) {
        const int trial = st.trial_id;
        reset();
        st.trial_id = trial;
      }
      return;
    }

    if (!cfg.electrical_ok || !cfg.limits_ok) {
      if (st.state != DriveState::Disarmed) {
        enter_fault(StopReason::Estop, "config", sen.now_ms);
      } else if (cmd.arm || cmd.run || cmd.jog_pos || cmd.jog_neg) {
        copy_text(reject, sizeof(reject), "config");
      }
      outputs_off();
      return;
    }

    const bool want_stop = cmd.stop;
    const int64_t moved = sen.count - st.run_origin_count;
    const int64_t moved_abs = moved < 0 ? -moved : moved;

    switch (st.state) {
      case DriveState::Disarmed:
        outputs_off();
        if (cmd.arm) {
          st.state = DriveState::PowerWait;
          st.power_on_cmd = true;
          st.brake_release_cmd = false;
          st.applied_duty_percent = 0;
          st.state_since_ms = sen.now_ms;
          st.stop_reason = StopReason::None;
        } else if (cmd.run || cmd.jog_pos || cmd.jog_neg) {
          copy_text(reject, sizeof(reject), "not_ready");
        }
        break;

      case DriveState::PowerWait:
        st.power_on_cmd = true;
        st.brake_release_cmd = false;
        st.applied_duty_percent = 0;
        if (want_stop) {
          enter_power_off(sen.now_ms);
        } else if (sen.usb_authority && !sen.lease_fresh) {
          enter_stop(StopReason::Lease, sen.now_ms, sen.count);
        } else if (!sen.ui_fresh) {
          enter_stop(StopReason::UiTimeout, sen.now_ms, sen.count);
        } else if (sen.now_ms - st.state_since_ms >= static_cast<uint64_t>(cfg.power_settle_ms)) {
          st.state = DriveState::BrakeWait;
          st.brake_release_cmd = true;
          st.state_since_ms = sen.now_ms;
        }
        break;

      case DriveState::BrakeWait:
        st.power_on_cmd = true;
        st.brake_release_cmd = true;
        st.applied_duty_percent = 0;
        if (want_stop) {
          enter_stop(StopReason::Operator, sen.now_ms, sen.count);
        } else if (sen.usb_authority && !sen.lease_fresh) {
          enter_stop(StopReason::Lease, sen.now_ms, sen.count);
        } else if (!sen.ui_fresh) {
          enter_stop(StopReason::UiTimeout, sen.now_ms, sen.count);
        } else if (sen.now_ms - st.state_since_ms >=
                   static_cast<uint64_t>(cfg.brake_release_wait_ms)) {
          st.state = DriveState::Ready;
          st.state_since_ms = sen.now_ms;
        }
        break;

      case DriveState::Ready:
        st.power_on_cmd = true;
        st.brake_release_cmd = true;
        st.applied_duty_percent = 0;
        if (want_stop) {
          enter_stop(StopReason::Operator, sen.now_ms, sen.count);
        } else if (sen.now_ms - st.state_since_ms >= static_cast<uint64_t>(cfg.ready_hold_ms)) {
          enter_stop(StopReason::ReadyTimeout, sen.now_ms, sen.count);
        } else if (sen.usb_authority && !sen.lease_fresh) {
          enter_stop(StopReason::Lease, sen.now_ms, sen.count);
        } else if (!sen.ui_fresh) {
          enter_stop(StopReason::UiTimeout, sen.now_ms, sen.count);
        } else if (cmd.jog_pos && cmd.jog_neg) {
          copy_text(reject, sizeof(reject), "both_jog");
        } else if (cmd.jog_pos || cmd.jog_neg) {
          if (!duty_ok(cfg, cmd.jog_duty)) {
            copy_text(reject, sizeof(reject), "duty");
          } else {
            enter_run(cfg, sen, cmd.jog_duty, cmd.jog_pos ? 1 : -1, cfg.max_run_ms, false);
          }
        } else if (cmd.run) {
          if (!duty_ok(cfg, cmd.run_duty) || (cmd.run_dir != 1 && cmd.run_dir != -1) ||
              cmd.run_duration_ms < 1 || cmd.run_duration_ms > cfg.max_run_ms) {
            copy_text(reject, sizeof(reject), "run_arg");
          } else {
            enter_run(cfg, sen, cmd.run_duty, cmd.run_dir, cmd.run_duration_ms, true);
          }
        }
        break;

      case DriveState::Running:
        st.power_on_cmd = true;
        st.brake_release_cmd = true;
        if (want_stop) {
          enter_stop(StopReason::Operator, sen.now_ms, sen.count);
        } else if (cmd.jog_pos && cmd.jog_neg) {
          enter_stop(StopReason::BothJog, sen.now_ms, sen.count);
        } else if ((st.direction > 0 && cmd.jog_neg) || (st.direction < 0 && cmd.jog_pos)) {
          enter_stop(StopReason::ReverseRequest, sen.now_ms, sen.count);
        } else if (sen.usb_authority && !sen.lease_fresh) {
          enter_stop(StopReason::Lease, sen.now_ms, sen.count);
        } else if (!sen.ui_fresh) {
          enter_stop(StopReason::UiTimeout, sen.now_ms, sen.count);
        } else if (sen.count_per_s > cfg.max_count_per_s || sen.count_per_s < -cfg.max_count_per_s) {
          enter_fault(StopReason::Overspeed, "overspeed", sen.now_ms);
        } else if (sen.now_ms >= st.run_end_ms) {
          enter_stop(StopReason::Duration, sen.now_ms, sen.count);
        } else if (moved_abs >= cfg.max_travel_counts) {
          enter_stop(StopReason::Travel, sen.now_ms, sen.count);
        } else if (!st.timed_run &&
                   ((st.direction > 0 && !cmd.jog_pos) || (st.direction < 0 && !cmd.jog_neg))) {
          enter_stop(StopReason::JogRelease, sen.now_ms, sen.count);
        } else if (sen.now_ms - st.state_since_ms >= static_cast<uint64_t>(cfg.start_grace_ms) &&
                   sen.count == st.run_origin_count) {
          enter_stop(StopReason::EncoderStall, sen.now_ms, sen.count);
        }
        break;

      case DriveState::Stopping:
        st.applied_duty_percent = 0;
        st.power_on_cmd = true;
        st.brake_release_cmd = true;
        if (sen.count_per_s <= cfg.stop_speed_threshold &&
            sen.count_per_s >= -cfg.stop_speed_threshold) {
          if (!st.slow_timing) {
            st.slow_timing = true;
            st.slow_since_ms = sen.now_ms;
          } else if (sen.now_ms - st.slow_since_ms >= static_cast<uint64_t>(cfg.stop_confirm_ms)) {
            enter_brake_apply(sen.now_ms, sen.count);
          }
        } else {
          st.slow_timing = false;
        }
        if (st.state == DriveState::Stopping &&
            sen.now_ms - st.state_since_ms >= static_cast<uint64_t>(cfg.stop_timeout_ms)) {
          enter_fault(StopReason::StopTimeout, "stop_timeout", sen.now_ms);
        }
        break;

      case DriveState::BrakeApply:
        st.applied_duty_percent = 0;
        st.brake_release_cmd = false;
        st.power_on_cmd = true;
        if (sen.now_ms - st.state_since_ms >= static_cast<uint64_t>(cfg.brake_apply_wait_ms)) {
          enter_power_off(sen.now_ms);
        }
        break;

      case DriveState::PowerOff:
        outputs_off();
        st.state = DriveState::Disarmed;
        st.state_since_ms = sen.now_ms;
        break;

      case DriveState::Fault:
        break;
    }
  }
};
