#!/usr/bin/env python3
"""One finite USB RUN for encoder-tester.

Command text is the firmware HELP/parser, not a guess:
  STATUS
  CONFIG
  SET KEY VALUE
  AUTHORITY USB
  HEARTBEAT
  STREAM ON
  ARM
  RUN duty_pct=VALUE direction=positive|negative duration_ms=VALUE
  STOP

HEARTBEAT refreshes lease_until to now+LEASE_MS (control_heartbeat).
This procedure's LEASE_MS is 2000, so the heartbeat period is 200 ms,
the same interval as capture_drive_encoder.py.

positive / negative are command tokens. positive drives DIR Low.
This script does not decide which way the mechanism moves.

Opening the port can reset the chip and erase RAM settings. If the
agreed keys are missing, they are written back. A different stored
value is not overwritten.

No second ARM and no second RUN. Ctrl+C sends one STOP and does not
start another run.
"""

import argparse
import os
import queue
import re
import signal
import sys
import threading
import time
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(ROOT, "logs")
HEARTBEAT_S = 0.2
BOOT_WAIT_S = 12.0
REPLY_WAIT_S = 3.0
READY_WAIT_S = 3.0
MONITOR_WAIT_S = 8.0
IDLE_MONITOR_S = 30.0

AGREED = (
    ("MAX_DUTY_PERCENT", "5"),
    ("MAX_RUN_MS", "100"),
    ("MAX_TRAVEL_COUNTS", "200"),
    ("POWER_SETTLE_MS", "1000"),
    ("BRAKE_RELEASE_WAIT_MS", "300"),
    ("BRAKE_APPLY_WAIT_MS", "300"),
    ("STOP_SPEED_THRESHOLD", "0"),
    ("STOP_CONFIRM_MS", "200"),
    ("STOP_TIMEOUT_MS", "1500"),
    ("READY_HOLD_MS", "3000"),
    ("START_GRACE_MS", "200"),
    ("MAX_COUNT_PER_S", "10000"),
    ("UI_TIMEOUT_MS", "1000"),
    ("LEASE_MS", "2000"),
    ("JOG_STALE_MS", "150"),
    ("JOG_DUTY", "5"),
)

CSV_FIELDS = (
    "seq",
    "t_us",
    "trial_id",
    "axis",
    "state",
    "power_on_cmd",
    "power_gpio_level",
    "brake_release_cmd",
    "brake_gpio_level",
    "requested_command",
    "requested_unit",
    "applied_duty_pct",
    "pwm_raw",
    "pwm_full_scale",
    "pwm_hz",
    "dir_level",
    "count_total_x4",
    "count_trial_x4",
    "velocity_count_s",
    "a_level",
    "b_level",
    "elapsed_ms",
    "stop_reason",
    "log_drop_count",
)

TERMINAL_BEFORE_RUN = {"DISARMED", "STOPPING", "BRAKE_APPLY", "POWER_OFF", "FAULT"}


def run_line(direction):
    return "RUN duty_pct=5 direction=%s duration_ms=100" % direction


def classify_config(cfg):
    """Return ok, wipe, or bad. bad does not authorize a SET."""
    missing = []
    mismatched = []
    for key, want in AGREED:
        got = cfg.get(key)
        if got is None or got.startswith("-"):
            missing.append(key)
        elif got != want:
            mismatched.append("%s=%s" % (key, got))
    if cfg.get("ENCODER_SIGN") not in (None, "1"):
        mismatched.append("ENCODER_SIGN=%s" % cfg.get("ENCODER_SIGN"))
    if cfg.get("GLITCH_NS") not in (None, "0"):
        mismatched.append("GLITCH_NS=%s" % cfg.get("GLITCH_NS"))
    if mismatched:
        return "bad", "config differs: " + ",".join(mismatched)
    if missing or cfg.get("ENCODER_SIGN") is None or cfg.get("GLITCH_NS") is None:
        return "wipe", "unconfigured: " + ",".join(missing)
    return "ok", ""


def parse_cfg_line(line):
    match = re.match(r"^CFG ([A-Z0-9_]+)=(-?\d+)\b", line)
    if not match:
        return None
    return match.group(1), match.group(2)


def parse_stat_line(line):
    if not line.startswith("STAT "):
        return {}
    found = {}
    for key, value in re.findall(r"([A-Za-z_]+)=([^\s]*)", line):
        found[key] = value
    return found


def parse_csv_line(line):
    if line.startswith("seq,") or not line or not line[0].isdigit():
        return None
    parts = line.split(",")
    if len(parts) != len(CSV_FIELDS):
        return None
    row = dict(zip(CSV_FIELDS, parts))
    return row


def parse_evt(line):
    if not line.startswith("EVT,"):
        return None
    return line.split(",")


class Protocol(object):
    def __init__(self, direction, now, idle_arm=False):
        if idle_arm:
            direction = "idle"
        elif direction not in ("positive", "negative"):
            raise ValueError("direction must be positive or negative")
        self.direction = direction
        self.idle_arm = idle_arm
        self.run_command = "" if idle_arm else run_line(direction)
        self.phase = "await_boot"
        self.fail = ""
        self.boot_deadline = now + BOOT_WAIT_S
        self.reply_deadline = None
        self.ready_deadline = None
        self.monitor_deadline = None
        self.cfg = {}
        self.stat = {}
        self.status_open = False
        self.gpio_errors = 0
        self.arm_sent = 0
        self.run_sent = 0
        self.stop_sent = 0
        self.run_blocked = False
        self.wants_heartbeat = False
        self.set_index = 0
        self.restoring = False
        self.arm_reply = ""
        self.run_reply = ""
        self.count_before = None
        self.csv_rows = []
        self.events = []
        self.states = []
        self.duty_seen = []
        self.pwm_seen = []
        self.dir_seen = []
        self.final_requested = False
        self.run_quiet_until = 0.0
        self.next_status_at = 0.0
        self.left_disarmed = False
        self.returned_disarmed = False
        self.trace_lines = []
        self.trace_end = ""
        self.disarm_seen_at = None

    def seconds_until_deadline(self, now):
        deadline = self._deadline()
        if deadline is None:
            wait = 0.5
        else:
            wait = max(0.01, deadline - now)
        if self.phase == "monitor" and not self.status_open:
            wake = self.next_status_at
            if now < self.run_quiet_until:
                wake = max(wake, self.run_quiet_until)
            wait = min(wait, max(0.01, wake - now))
        return wait

    def _deadline(self):
        if self.phase == "await_boot":
            return self.boot_deadline
        if self.phase == "await_ready":
            return self.ready_deadline
        if self.phase == "monitor":
            return self.monitor_deadline
        return self.reply_deadline

    def _expired(self, now):
        deadline = self._deadline()
        return deadline is not None and now >= deadline

    def on_timeout(self, now):
        if self.phase == "await_boot":
            return self._send_status(now)
        if self.phase == "await_ready":
            self.run_blocked = True
            self.fail = self.fail or "ready_not_seen"
            return self._enter_monitor(now)
        if self.phase == "monitor":
            if self.idle_arm and self._idle_finished(now):
                return []
            if self.monitor_deadline is not None and now >= self.monitor_deadline:
                self.phase = "failed"
                if self.idle_arm and self.disarm_seen_at is None:
                    self.fail = self.fail or "disarmed_not_seen"
                elif self.idle_arm and not self.trace_end:
                    self.fail = self.fail or "trace_missing"
                else:
                    self.fail = self.fail or "disarmed_not_seen"
                return []
            if not self.status_open and now >= self.run_quiet_until and now >= self.next_status_at:
                return self._request_status(now)
            return []
        if self.arm_sent:
            self.run_blocked = True
        waiting = self.phase
        self.phase = "failed"
        self.fail = self.fail or ("reply_timeout:" + waiting)
        return []

    def on_interrupt(self, now):
        self.run_blocked = True
        if not self.arm_sent:
            self.phase = "failed"
            self.fail = self.fail or "interrupted"
            return []
        if self.stop_sent:
            return []
        if self.returned_disarmed:
            self.phase = "done"
            return []
        if not self.left_disarmed and self.fail.startswith("ERR") and self.stat.get("state", "DISARMED") == "DISARMED":
            return []
        self.stop_sent = 1
        self.fail = self.fail or "interrupted"
        return self._enter_monitor(now, ["STOP"])

    def on_line(self, line, now):
        if "is not set as GPIO" in line:
            self.gpio_errors += 1
            if not self.arm_sent:
                self.phase = "failed"
                self.fail = "gpio_error"
                self._observe(line)
                return []
        if self.phase not in ("done", "failed") and self._expired(now):
            self._observe(line)
            if self.phase == "monitor":
                commands = self._on_monitor(line, now)
                if self.phase in ("done", "await_final"):
                    return commands
            if self.phase == "await_ready":
                self.run_blocked = True
            return self.on_timeout(now)
        self._observe(line)
        if self.phase == "await_boot":
            if line == "HELP" or line == "BOOT state=DISARMED":
                return self._send_status(now)
            return []
        if self.phase in ("await_status", "await_authority_status", "await_final"):
            return self._on_status_block(line, now)
        if self.phase == "await_config" or self.phase == "await_config_confirm":
            return self._on_config(line, now)
        if self.phase == "await_set":
            return self._on_set(line, now)
        if self.phase == "await_authority":
            return self._on_authority(line, now)
        if self.phase == "await_stream":
            return self._on_stream(line, now)
        if self.phase == "await_arm":
            return self._on_arm(line, now)
        if self.phase == "await_ready":
            return self._on_ready(line, now)
        if self.phase == "await_run":
            return self._on_run_reply(line, now)
        if self.phase == "monitor":
            return self._on_monitor(line, now)
        return []

    def _note_state(self, state):
        if not state or not self.arm_sent:
            return
        if state == "DISARMED":
            if self.left_disarmed:
                self.returned_disarmed = True
            return
        self.left_disarmed = True

    def _take_drive(self, state, duty, pwm, dir_level):
        if state != "RUNNING":
            return
        if duty not in (None, ""):
            self.duty_seen.append(str(duty))
        if pwm not in (None, ""):
            self.pwm_seen.append(str(pwm))
        if dir_level not in (None, ""):
            self.dir_seen.append(str(dir_level))

    def _observe(self, line):
        stat = parse_stat_line(line)
        if stat:
            prior = self.stat.get("state")
            self.stat.update(stat)
            view = stat.get("state", prior)
            self._note_state(stat.get("state"))
            if stat.get("state"):
                self.states.append(stat["state"])
            self._take_drive(view, stat.get("applied_duty_pct"), stat.get("pwm_raw"), stat.get("dir_level"))
        row = parse_csv_line(line)
        if row is not None:
            self.csv_rows.append(row)
            self._note_state(row.get("state"))
            if row.get("state"):
                self.states.append(row["state"])
            self._take_drive(row.get("state"), row.get("applied_duty_pct"), row.get("pwm_raw"), row.get("dir_level"))
        evt = parse_evt(line)
        if evt is not None:
            self.events.append(evt)
        if line.startswith("TRACE"):
            self.trace_lines.append(line)
            if line.startswith("TRACE end"):
                self.trace_end = line

    def _send_status(self, now):
        self.phase = "await_status" if self.phase == "await_boot" else self.phase
        if self.phase == "await_boot":
            self.phase = "await_status"
        self.status_open = True
        self.reply_deadline = now + REPLY_WAIT_S
        return ["STATUS"]

    def _send_run(self, now):
        if self.idle_arm or self.run_sent or self.run_blocked:
            return []
        self.run_sent = 1
        self.run_quiet_until = now + 0.35
        self.phase = "await_run"
        self.reply_deadline = now + REPLY_WAIT_S
        self.status_open = False
        return [self.run_command]

    def _request_status(self, now, gap=0.4):
        self.status_open = True
        self.reply_deadline = now + REPLY_WAIT_S
        self.next_status_at = now + gap
        return ["STATUS"]

    def _enter_monitor(self, now, extra=None):
        # CSV at 50 Hz is the PWM record. Do not poll STATUS through the 100 ms run.
        self.phase = "monitor"
        self.monitor_deadline = now + (IDLE_MONITOR_S if self.idle_arm else MONITOR_WAIT_S)
        self.status_open = False
        self.next_status_at = now + (1.0 if self.idle_arm else 0.4)
        if self.idle_arm:
            self.run_blocked = True
        return list(extra or [])

    def _on_status_block(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line == "ERR busy" and self.phase == "await_final":
            return self._request_status(now)
        if line.startswith("ERR "):
            self.phase = "failed"
            self.fail = line
            return []
        if not line.startswith("STAT cal="):
            return []
        self.status_open = False
        state = self.stat.get("state", "")
        if self.phase == "await_final":
            self.phase = "done"
            return []
        if self.phase == "await_status":
            if state != "DISARMED":
                self.phase = "failed"
                self.fail = "not_disarmed:" + state
                return []
            self.cfg = {}
            self.phase = "await_config"
            self.reply_deadline = now + REPLY_WAIT_S
            return ["CONFIG"]
        if self.phase == "await_authority_status":
            if state != "DISARMED":
                self.phase = "failed"
                self.fail = "not_disarmed:" + state
                return []
            if self.stat.get("authority") != "usb":
                self.phase = "failed"
                self.fail = "authority_not_usb"
                return []
            if self.stat.get("lock"):
                self.phase = "failed"
                self.fail = "locked:" + self.stat.get("lock", "")
                return []
            if self.gpio_errors:
                self.phase = "failed"
                self.fail = "gpio_error"
                return []
            self.phase = "await_stream"
            self.reply_deadline = now + REPLY_WAIT_S
            return ["STREAM ON"]
        return []

    def _on_config(self, line, now):
        item = parse_cfg_line(line)
        if item is not None:
            self.cfg[item[0]] = item[1]
        if line != "CFG -1 means UNCONFIGURED":
            return []
        kind, reason = classify_config(self.cfg)
        if self.phase == "await_config_confirm":
            if kind != "ok":
                self.phase = "failed"
                self.fail = "reread_failed:" + reason
                return []
            return self._send_authority(now)
        if kind == "bad":
            self.phase = "failed"
            self.fail = reason
            return []
        if kind == "wipe":
            self.restoring = True
            self.set_index = 0
            return self._send_one_set(now)
        return self._send_authority(now)

    def _send_one_set(self, now):
        key, value = AGREED[self.set_index]
        self.phase = "await_set"
        self.reply_deadline = now + REPLY_WAIT_S
        return ["SET %s %s" % (key, value)]

    def _on_set(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line == "OK set":
            self.set_index += 1
            if self.set_index < len(AGREED):
                return self._send_one_set(now)
            self.cfg = {}
            self.phase = "await_config_confirm"
            self.reply_deadline = now + REPLY_WAIT_S
            return ["CONFIG"]
        if line.startswith("ERR ") or line.startswith("OK "):
            self.phase = "failed"
            self.fail = "set_rejected:" + line
            return []
        return []

    def _send_authority(self, now):
        self.wants_heartbeat = True
        self.phase = "await_authority"
        self.reply_deadline = now + REPLY_WAIT_S
        return ["AUTHORITY USB"]

    def _on_authority(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line == "OK authority=usb":
            self.stat = {}
            self.status_open = True
            self.phase = "await_authority_status"
            self.reply_deadline = now + REPLY_WAIT_S
            return ["STATUS"]
        if line.startswith("ERR ") or line.startswith("OK "):
            self.phase = "failed"
            self.fail = line
            return []
        return []

    def _on_stream(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line == "OK stream=on":
            return self._send_arm(now)
        if line.startswith("ERR ") or line.startswith("OK "):
            self.phase = "failed"
            self.fail = line
            return []
        return []

    def _send_arm(self, now):
        if self.arm_sent or self.gpio_errors:
            self.phase = "failed"
            self.fail = self.fail or ("gpio_error" if self.gpio_errors else "arm_already_sent")
            return []
        self.arm_sent = 1
        self.phase = "await_arm"
        self.reply_deadline = now + REPLY_WAIT_S
        return ["ARM"]

    def _on_arm(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line.startswith("OK state="):
            self.arm_reply = line
            state = line.split("=", 1)[1].strip()
            self._note_state(state)
            if self.idle_arm:
                self.run_blocked = True
                return self._enter_monitor(now)
            if state in ("READY", "POWER_WAIT", "BRAKE_WAIT"):
                if state == "READY":
                    self.stat["state"] = "READY"
                self.phase = "await_ready"
                self.ready_deadline = now + READY_WAIT_S
                self.status_open = True
                return ["STATUS"]
            self.run_blocked = True
            self.fail = "arm_state:" + state
            return self._enter_monitor(now)
        if line.startswith("ERR "):
            self.arm_reply = line
            self.run_blocked = True
            self.phase = "failed"
            self.fail = line
            return []
        return []

    def _terminal_line(self, line):
        stat = parse_stat_line(line)
        if stat.get("state") in TERMINAL_BEFORE_RUN:
            return stat.get("state")
        row = parse_csv_line(line)
        if row is not None and row.get("state") in TERMINAL_BEFORE_RUN:
            return row.get("state")
        return ""

    def _on_ready(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line == "ERR busy":
            return ["STATUS"]
        if line.startswith("ERR "):
            self.run_blocked = True
            self.fail = line
            return self._enter_monitor(now)
        row = parse_csv_line(line)
        if row is not None and row.get("state") == "READY":
            if self.count_before is None:
                self.count_before = row.get("count_total_x4")
            return self._send_run(now)
        if line.startswith("STAT count_total=") and self.stat.get("state") == "READY":
            if self.count_before is None:
                self.count_before = self.stat.get("count_total")
            return self._send_run(now)
        terminal = self._terminal_line(line)
        if terminal:
            self.run_blocked = True
            self.fail = self.fail or ("left_ready_path:" + terminal)
            return self._enter_monitor(now)
        if line.startswith("STAT cal=") and not self.run_sent:
            return ["STATUS"]
        return []

    def _on_run_reply(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line.startswith("OK state=") or line.startswith("ERR "):
            self.run_reply = line
            if line.startswith("ERR "):
                self.fail = self.fail or line
                self.run_quiet_until = 0.0
            return self._enter_monitor(now)
        return []

    def _pwm_left_zero(self):
        for row in self.csv_rows:
            if row.get("pwm_raw") not in (None, "", "0"):
                return True
        for line in self.trace_lines:
            if not line.startswith("TRACE,"):
                continue
            parts = line.split(",")
            if len(parts) > 7 and parts[7] != "0":
                return True
        return False

    def _idle_finished(self, now):
        if self.disarm_seen_at is None or now < self.disarm_seen_at + 2.0 or not self.trace_end:
            return False
        if self._pwm_left_zero():
            self.fail = self.fail or "pwm_not_zero"
        self.phase = "failed" if self.fail else "done"
        return True

    def _on_idle_monitor(self, line, now):
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line == "ERR busy":
            self.status_open = False
            self.next_status_at = now + 0.2
            return []
        if self.returned_disarmed and self.disarm_seen_at is None:
            self.disarm_seen_at = now
        if self.stat.get("state") == "FAULT" and self.left_disarmed and self.disarm_seen_at is None:
            self.disarm_seen_at = now
            self.fail = self.fail or "fault"
        if self._idle_finished(now):
            return []
        if line.startswith("STAT cal="):
            self.status_open = False
        if self.status_open:
            return []
        if now >= self.next_status_at:
            return self._request_status(now, 1.0)
        return []

    def _on_monitor(self, line, now):
        if self.idle_arm:
            return self._on_idle_monitor(line, now)
        if line in ("OK heartbeat", "OK stop"):
            return []
        if line == "ERR busy":
            self.status_open = False
            self.next_status_at = now + 0.05
            return []
        row = parse_csv_line(line)
        csv_state = row.get("state") if row else ""
        if line.startswith("STAT cal="):
            self.status_open = False
            if self.stat.get("state") in ("DISARMED", "FAULT"):
                self.phase = "done"
                return []
        if csv_state in ("DISARMED", "FAULT") and not self.final_requested:
            self.final_requested = True
            self.phase = "await_final"
            return self._request_status(now)
        if self.status_open or self.final_requested:
            return []
        if now < self.run_quiet_until:
            return []
        if now >= self.next_status_at:
            return self._request_status(now)
        return []


def _num(text):
    if text is None or text == "":
        return None
    try:
        return int(text)
    except ValueError:
        return None


def format_report(proto):
    running = [row for row in proto.csv_rows if row.get("state") == "RUNNING"]
    duties = sorted({item for item in proto.duty_seen if item != ""})
    pwms = [item for item in (_num(value) for value in proto.pwm_seen) if item is not None]
    dirs = sorted({item for item in proto.dir_seen if item != ""})
    elapsed = [_num(row.get("elapsed_ms")) for row in running]
    elapsed = [item for item in elapsed if item is not None]
    velocities = []
    for row in proto.csv_rows:
        value = _num(row.get("velocity_count_s"))
        if value is not None:
            velocities.append(value)
    run_evt = next((evt for evt in proto.events if len(evt) >= 6 and evt[2] == "run"), None)
    pwm_evt = next((evt for evt in proto.events if len(evt) >= 5 and evt[2] == "pwm_off"), None)
    trial_evt = next((evt for evt in proto.events if len(evt) >= 7 and evt[2] == "trial_done"), None)
    drive_ms = ""
    if run_evt and pwm_evt:
        start_us = _num(run_evt[1])
        end_us = _num(pwm_evt[1])
        if start_us is not None and end_us is not None:
            drive_ms = "%.3f" % ((end_us - start_us) / 1000.0)
    accepted = proto.run_reply.startswith("OK state=")
    if proto.run_sent and not proto.run_reply:
        run_text = "応答なし"
    elif not proto.run_sent:
        run_text = "未送信"
    elif accepted:
        run_text = "受理 " + proto.run_reply
    else:
        run_text = "拒否 " + (proto.run_reply or proto.fail)
    stop_reason = ""
    if pwm_evt:
        stop_reason = pwm_evt[3]
    if not stop_reason:
        stop_reason = proto.stat.get("stop", "")
    lines = [
        "方向トークン: %s" % proto.direction,
        "方向の意味: USB トークン。positive は DIR Low、negative は DIR High。機構のどちらへ動くかは未測定。",
        "RUN コマンド: %s" % proto.run_command,
        "RUN: %s" % run_text,
        "ARM 応答: %s" % (proto.arm_reply or "なし"),
        "DIR level (RUNNING 中): %s" % (",".join(dirs) if dirs else "受信なし"),
        "適用 Duty (RUNNING 中): %s" % (",".join(duties) if duties else "受信なし"),
        "PWM 生値 (RUNNING 中): %s" % (",".join(str(item) for item in sorted(set(pwms))) if pwms else "受信なし"),
        "PWM が出た: %s" % ("yes" if any(item != 0 for item in pwms) else "no" if pwms else "受信なし"),
        "駆動時間 ms (EVT run→pwm_off): %s" % (drive_ms or "受信なし"),
        "駆動時間 ms (CSV elapsed 最大): %s" % (str(max(elapsed)) if elapsed else "受信なし"),
        "RUN 前 count_total: %s" % (proto.count_before if proto.count_before is not None else "受信なし"),
        "最終 count_total: %s" % (proto.stat.get("count_total") if "count_total" in proto.stat else "受信なし"),
        "EVT run 開始カウント: %s" % (run_evt[5] if run_evt else "受信なし"),
        "EVT pwm_off カウント: %s" % (pwm_evt[4] if pwm_evt else "受信なし"),
        "trial 駆動差分 (pwm_off-start): %s" % (trial_evt[4] if trial_evt else "受信なし"),
        "PWM 停止後カウント変化 (coast): %s" % (trial_evt[5] if trial_evt else "受信なし"),
        "trial 停止差分 (stop-start): %s" % (trial_evt[6] if trial_evt else "受信なし"),
        "count/s 最大: %s" % (str(max(velocities)) if velocities else "受信なし"),
        "count/s 最小: %s" % (str(min(velocities)) if velocities else "受信なし"),
        "count/s サンプル数: %d" % len(velocities),
        "count/s 注記: 受信した CSV velocity_count_s（100 ms 窓）。内部ピークは USB に出ない。",
        "停止理由: %s" % (stop_reason or "受信なし"),
        "最終状態: %s" % (proto.stat.get("state") or "受信なし"),
        "最終 stop: %s" % (proto.stat.get("stop") if "stop" in proto.stat else "受信なし"),
        "失敗: %s" % (proto.fail or "なし"),
        "GPIO エラー行: %d" % proto.gpio_errors,
        "ARM 送信回数: %d" % proto.arm_sent,
        "RUN 送信回数: %d" % proto.run_sent,
        "STOP 送信回数: %d" % proto.stop_sent,
    ]
    if proto.idle_arm:
        pwms = sorted({row.get("pwm_raw", "") for row in proto.csv_rows})
        lines.extend([
            "idle ARM: RUN は送らない。PWM は全期間 0 が条件。",
            "全 CSV の pwm_raw: %s" % (",".join(pwms) if pwms else "受信なし"),
            "TRACE 行数: %d" % len(proto.trace_lines),
            "TRACE 終端: %s" % (proto.trace_end or "受信なし"),
            "GPIO レベル: digitalRead の 1 点。サンプル間のパルスやノイズ波形ではない。",
        ])
    return "\n".join(lines) + "\n"


EDGE_FIELDS = (
    "state",
    "power_gpio_level",
    "brake_release_cmd",
    "brake_gpio_level",
    "applied_duty_pct",
    "pwm_raw",
    "dir_level",
    "count_total_x4",
    "count_trial_x4",
    "velocity_count_s",
    "a_level",
    "b_level",
    "stop_reason",
)

EDGE_HEADERS = (
    "seq",
    "t_us",
    "state",
    "power_gpio_level",
    "brake_release_cmd",
    "brake_gpio_level",
    "applied_duty_pct",
    "pwm_raw",
    "dir_level",
    "count_total_x4",
    "count_trial_x4",
    "velocity_count_s",
    "a_level",
    "b_level",
    "elapsed_ms",
    "stop_reason",
)


def tester_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def runs_dir():
    return os.path.join(tester_root(), "docs", "runs")


def edge_rows(rows):
    kept = []
    previous = None
    for row in rows:
        if previous is None or any(row.get(key) != previous.get(key) for key in EDGE_FIELDS):
            kept.append(row)
            previous = row
    if rows and (not kept or kept[-1] is not rows[-1]):
        kept.append(rows[-1])
    return kept


def markdown_table(rows, columns):
    header = "| " + " | ".join(columns) + " |"
    rule = "| " + " | ".join("---" for _column in columns) + " |"
    body = []
    for row in rows:
        cells = [str(row.get(column, "")).replace("|", "/") for column in columns]
        body.append("| " + " | ".join(cells) + " |")
    return "\n".join([header, rule] + body)


def count_step_lines(rows):
    lines = []
    previous = None
    for row in rows:
        if previous is None:
            previous = row
            continue
        if row.get("count_total_x4") != previous.get("count_total_x4"):
            lines.append(
                "- seq %s（t_us %s、%s、count %s、pwm_raw %s）の次の seq %s（t_us %s、%s、pwm_raw %s、power_gpio %s、brake_release_cmd %s、brake_gpio %s）で count_total_x4 が %s になった。"
                % (
                    previous.get("seq"),
                    previous.get("t_us"),
                    previous.get("state"),
                    previous.get("count_total_x4"),
                    previous.get("pwm_raw"),
                    row.get("seq"),
                    row.get("t_us"),
                    row.get("state"),
                    row.get("pwm_raw"),
                    row.get("power_gpio_level"),
                    row.get("brake_release_cmd"),
                    row.get("brake_gpio_level"),
                    row.get("count_total_x4"),
                )
            )
        previous = row
    if not lines:
        lines.append("- CSV の count_total_x4 は最初から最後まで同じ。")
    return lines


def boundary_lines(rows):
    running = [row for row in rows if row.get("state") == "RUNNING"]
    pwms = [item for item in (_num(row.get("pwm_raw")) for row in running) if item is not None]
    if any(item != 0 for item in pwms):
        drive = "駆動の受信: RUNNING の CSV に 0 でない pwm_raw がある。"
    elif running:
        drive = "駆動の受信: RUNNING の CSV で pwm_raw は 0。"
    else:
        drive = "駆動の受信: RUNNING の CSV 行は無い。"
    counts = [item for item in (_num(row.get("count_total_x4")) for row in running) if item is not None]
    if not running:
        count_text = "カウント: RUNNING の行が無い。駆動中の count_total_x4 はここに無い。"
    elif len(set(counts)) <= 1:
        shown = counts[0] if counts else "受信なし"
        count_text = "カウント: RUNNING の count_total_x4 は %s のまま。これは受信値であり、軸が動いた確認ではない。" % shown
    else:
        count_text = (
            "カウント: RUNNING の count_total_x4 は %s から %s に変わっている。これは受信値であり、軸が動いた確認ではない。"
            % (counts[0], counts[-1])
        )
    return [
        drive,
        count_text,
        "分解能: UNKNOWN。角度も回転数も、この記録には無い。",
        "機構の向き: 未測定。positive は DIR Low、negative は DIR High。",
    ]


def restore_line(meta):
    if not meta.get("restoring"):
        return "- この実行では SET を送っていない。"
    sent = meta.get("set_index")
    if sent == len(AGREED):
        return "- 開始時の合意キーは未設定で、16 個を SET し、その後の CONFIG を読んだ。"
    return "- SET を %s 個送ったところで、読み返しの完了前に終わった。" % sent


def format_run_markdown(direction, report_text, rows, events, cfg, meta):
    lines = [
        "# RUN %s %s" % (direction, meta.get("stamp", "")),
        "",
        "受信したログから作った記録。表は下の CSV と EVT から取り直せる。",
        "",
        "## 境界",
        "",
    ]
    for item in boundary_lines(rows):
        lines.append("- %s" % item)
    lines.extend([
        "",
        "## 条件",
        "",
        "- ポート: %s" % (meta.get("port") or "記録なし"),
        "- 方向トークン: %s" % direction,
        "- 生ログ: `%s`" % meta.get("log_rel", ""),
        "- CSV: `%s`" % meta.get("csv_rel", ""),
        "- テキスト要約: `%s`" % meta.get("txt_rel", ""),
        "- 生ログと CSV は gitignore されている。この Markdown に写した行が、リポジトリに残る検証用の写し。",
        "- ARM 前の USB 開き直し: %s 回。" % meta.get("reopens", 0),
        restore_line(meta),
        "- ARM 送信: %s。RUN 送信: %s。STOP 送信: %s。"
        % (meta.get("arm_sent", ""), meta.get("run_sent", ""), meta.get("stop_sent", "")),
        "",
        "## 要約",
        "",
        "```text",
        report_text.rstrip("\n"),
        "```",
        "",
        "## カウントが変わった箇所",
        "",
    ])
    lines.extend(count_step_lines(rows))
    lines.extend([
        "",
        "EVT の coast は trial_done を記録した時点の差分である。その後の CSV で count が動いた分は coast に入らない。",
        "",
        "## 値が変わった CSV 行",
        "",
    ])
    if rows:
        lines.append(markdown_table(edge_rows(rows), EDGE_HEADERS))
    else:
        lines.append("CSV 行は無い。")
    trace_lines = meta.get("trace_lines") or []
    trace_rows = []
    for line in trace_lines:
        if not line.startswith("TRACE,"):
            continue
        parts = line.split(",")
        if len(parts) < 14:
            continue
        if parts[10] != "0" or parts[13] == "post":
            trace_rows.append(line)
    if trace_lines:
        lines.extend(["", "## 制御周期トレース", ""])
        lines.append("a_level と b_level は、その時刻の digitalRead 1 点である。サンプルのあいだのパルスやノイズ波形ではない。")
        lines.append("")
        if trace_rows:
            lines.append("```text")
            lines.extend(trace_rows)
            lines.append("```")
        else:
            lines.append("delta が 0 でない行も、post 行も無い。")
        if meta.get("trace_end"):
            lines.append("")
            lines.append(meta["trace_end"])
    lines.extend(["", "## 最後に読んだ CONFIG の数値", ""])
    if cfg:
        for key, value in sorted(cfg.items()):
            lines.append("- %s=%s" % (key, value))
    else:
        lines.append("- なし")
    lines.extend(["", "## EVT", "", "```text"])
    if events:
        for evt in events:
            lines.append(",".join(evt))
    else:
        lines.append("(なし)")
    lines.extend(["```", "", "## CSV 全文", "", "```text", ",".join(CSV_FIELDS)])
    for row in rows:
        lines.append(",".join(row.get(field, "") for field in CSV_FIELDS))
    lines.extend(["```", ""])
    extra = meta.get("extra") or ""
    if extra:
        lines.extend([extra.rstrip("\n"), ""])
    return "\n".join(lines)


def report_field(report_text, prefix):
    for line in report_text.splitlines():
        if line.startswith(prefix):
            return line[len(prefix):].strip()
    return ""


def upsert_run_index(md_name, direction, run_cell, state_cell):
    path = os.path.join(runs_dir(), "README.md")
    os.makedirs(runs_dir(), exist_ok=True)
    row = "| [%s](%s) | %s | %s | %s | UNKNOWN |" % (md_name, md_name, direction, run_cell, state_cell)
    if not os.path.exists(path):
        text = (
            "# 実施記録\n"
            "\n"
            "`tools/run_once.py` が 1 回終わるたびに、ここに 1 ファイルを足す。"
            "生の log と csv は `logs/` にあり、gitignore されている。"
            "Markdown 側に、要約、カウントが変わった行、EVT、CSV 全文を写す。\n"
            "\n"
            "分解能の列は、外部の角度か回転数を別に入れるまで UNKNOWN のままにする。\n"
            "\n"
            "| ファイル | 方向 | RUN | 最終状態 | 分解能 |\n"
            "| --- | --- | --- | --- | --- |\n"
            "%s\n" % row
        )
    else:
        with open(path, encoding="utf-8") as handle:
            lines = handle.read().splitlines()
        needle = "](%s)" % md_name
        replaced = False
        new_lines = []
        for line in lines:
            if line.startswith("|") and needle in line:
                new_lines.append(row)
                replaced = True
            else:
                new_lines.append(line)
        if not replaced:
            new_lines.append(row)
        text = "\n".join(new_lines)
        if not text.endswith("\n"):
            text += "\n"
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(text)


def publish_run_markdown(direction, report_text, rows, events, cfg, meta):
    os.makedirs(runs_dir(), exist_ok=True)
    md_name = meta.get("md_name") or ("run_once_%s_%s.md" % (direction, meta.get("stamp", "unknown")))
    path = os.path.join(runs_dir(), md_name)
    body = format_run_markdown(direction, report_text, rows, events, cfg, meta)
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(body)
    upsert_run_index(
        md_name,
        direction,
        report_field(report_text, "RUN: ") or "記録なし",
        report_field(report_text, "最終状態: ") or "記録なし",
    )
    return path


def load_csv_rows(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            row = parse_csv_line(line.strip())
            if row is not None:
                rows.append(row)
    return rows


def load_log_pieces(path):
    events = []
    blocks = []
    current = None
    reopens = 0
    sets = 0
    with open(path, encoding="utf-8") as handle:
        for raw in handle:
            line = raw.strip()
            evt = parse_evt(line)
            if evt is not None:
                events.append(evt)
            if line.startswith("REOPEN"):
                reopens += 1
            if line.startswith(">>> SET "):
                sets += 1
            if line.startswith(">>> CONFIG"):
                current = {}
                blocks.append(current)
                continue
            if current is None:
                continue
            if line.startswith(">>> "):
                current = None
                continue
            item = parse_cfg_line(line)
            if item is not None and current is not None:
                current[item[0]] = item[1]
    cfg = blocks[-1] if blocks else {}
    return events, cfg, reopens, sets


def write_markdown_from(source, port):
    if source.endswith(".log"):
        base = source[:-4]
    else:
        base = source
    log_path = base + ".log"
    csv_path = base + ".csv"
    txt_path = base + ".txt"
    for path in (log_path, csv_path, txt_path):
        if not os.path.isfile(path):
            print("missing %s" % path, file=sys.stderr)
            return 2
    name = os.path.basename(base)
    prefix = "run_once_"
    if not name.startswith(prefix):
        print("name must start with run_once_", file=sys.stderr)
        return 2
    direction, stamp = name[len(prefix):].split("_", 1)
    if direction not in ("positive", "negative"):
        print("direction must be positive or negative", file=sys.stderr)
        return 2
    with open(txt_path, encoding="utf-8") as handle:
        report_text = handle.read()
    events, cfg, reopens, sets = load_log_pieces(log_path)
    extra = ""
    if name == "run_once_positive_20261022_230347":
        extra = (
            "## 同日の前段\n"
            "\n"
            "2026-10-22 の 22:49、22:52、22:55 の 3 回は、ARM の前に受信が無く終わった。"
            "RUN は送っていない。"
            "生ログは `logs/run_once_positive_20261022_224933`、"
            "`logs/run_once_positive_20261022_225251`、"
            "`logs/run_once_positive_20261022_225527`。"
            "この 23:03 の記録が、その夜に RUN を送った回である。"
        )
    path = publish_run_markdown(
        direction,
        report_text,
        load_csv_rows(csv_path),
        events,
        cfg,
        {
            "stamp": stamp,
            "port": port,
            "log_rel": "logs/" + os.path.basename(log_path),
            "csv_rel": "logs/" + os.path.basename(csv_path),
            "txt_rel": "logs/" + os.path.basename(txt_path),
            "reopens": reopens,
            "restoring": sets > 0,
            "set_index": sets,
            "arm_sent": report_field(report_text, "ARM 送信回数: "),
            "run_sent": report_field(report_text, "RUN 送信回数: "),
            "stop_sent": report_field(report_text, "STOP 送信回数: "),
            "extra": extra,
        },
    )
    print(path)
    return 0


def self_test():
    def play(direction, steps):
        proto = Protocol(direction, 0.0)
        sent = []
        for kind, at, payload in steps:
            if kind == "line":
                sent.extend(proto.on_line(payload, at))
            else:
                sent.extend(proto.on_timeout(at))
        return proto, sent

    def status_block(state, authority="local", lock="", count="0", duty="0", pwm="0", dir_level="0", stop=""):
        return [
            "STAT state=%s stop=%s power_on_cmd=0 power_gpio=0 brake_release_cmd=0 brake_gpio=0 requested=0 applied_duty_pct=%s pwm_raw=%s pwm_full=1023 pwm_hz=20000 dir_level=%s"
            % (state, stop, duty, pwm, dir_level),
            "STAT count_total=%s count_display=%s count_trial=0 count_per_s=0 a=0 b=0 stable=1 trial=0 rem_ms=0 ready_ms=0 travel=0 max_travel=200"
            % (count, count),
            "STAT lock=%s fault= reject= visual=VISUAL_IDLE visual_valid=0 authority=%s stream=0 drops=0"
            % (lock, authority),
            "STAT missing=",
            "STAT ppr=UNKNOWN",
            "STAT cal=PPR UNKNOWN angle UNKNOWN",
        ]

    def config_lines():
        lines = ["CFG %s=%s unit" % (key, value) for key, value in AGREED]
        lines.append("CFG ENCODER_SIGN=1 sign")
        lines.append("CFG GLITCH_NS=0 ns")
        lines.append("CFG AUTHORITY=local LOCK=")
        lines.append("CFG MISSING=")
        lines.append("CFG -1 means UNCONFIGURED")
        return lines

    assert classify_config({key: value for key, value in AGREED} | {"ENCODER_SIGN": "1", "GLITCH_NS": "0"})[0] == "ok"
    wiped = {key: "-1" for key, _value in AGREED}
    wiped.update({"ENCODER_SIGN": "1", "GLITCH_NS": "0"})
    assert classify_config(wiped)[0] == "wipe"
    changed = {key: value for key, value in AGREED}
    changed["MAX_DUTY_PERCENT"] = "9"
    changed.update({"ENCODER_SIGN": "1", "GLITCH_NS": "0"})
    assert classify_config(changed)[0] == "bad"

    steps = [("line", 0.1, "HELP")]
    for line in status_block("DISARMED"):
        steps.append(("line", 0.2, line))
    for line in config_lines():
        steps.append(("line", 0.3, line))
    steps.append(("line", 0.4, "OK heartbeat"))
    steps.append(("line", 0.4, "OK authority=usb"))
    for line in status_block("DISARMED", authority="usb"):
        steps.append(("line", 0.5, line))
    steps.append(("line", 0.6, "OK heartbeat"))
    steps.append(("line", 0.6, "OK stream=on"))
    steps.append(("line", 0.7, "OK heartbeat"))
    steps.append(("line", 0.7, "OK state=POWER_WAIT"))
    steps.append(("line", 1.0, "4,1000,0,1,POWER_WAIT,1,1,0,0,0,none,0,0,1023,20000,0,3,0,0,0,0,0,,0"))
    for line in status_block("READY", authority="usb", count="3"):
        steps.append(("line", 1.4, line))
    steps.append(("line", 1.5, "OK heartbeat"))
    steps.append(("line", 1.5, "OK state=RUNNING"))
    steps.append(("line", 1.52, "9,2000,1,1,READY,1,1,1,1,0,none,0,0,1023,20000,0,3,0,0,0,0,0,,0"))
    steps.append(("line", 1.55, "EVT,5000000,run,1,5,3"))
    steps.append(("line", 1.6, "8,2100,1,1,RUNNING,1,1,1,1,5,duty_percent,5,51,1023,20000,0,11,8,0,1,0,80,,0"))
    steps.append(("line", 1.7, "EVT,5100000,pwm_off,duration,11"))
    steps.append(("line", 1.8, "EVT,2400,trial_done,1,8,2,10"))
    for line in status_block("DISARMED", authority="usb", count="13", stop="duration"):
        steps.append(("line", 2.2, line))
    for line in status_block("DISARMED", authority="usb", count="13", stop="duration"):
        steps.append(("line", 2.4, line))
    proto, sent = play("positive", steps)
    assert sent.count("ARM") == 1, sent
    assert sent.count(run_line("positive")) == 1, sent
    assert proto.run_sent == 1 and proto.arm_sent == 1
    assert proto.phase == "done", proto.phase
    assert proto.count_before == "3"
    text = format_report(proto)
    assert "受理 OK state=RUNNING" in text
    assert "DIR level (RUNNING 中): 0" in text
    assert "適用 Duty (RUNNING 中): 5" in text
    assert "PWM 生値 (RUNNING 中): 51" in text
    assert "PWM が出た: yes" in text
    assert "駆動時間 ms (EVT run→pwm_off): 100.000" in text
    assert "trial 駆動差分 (pwm_off-start): 8" in text
    assert "PWM 停止後カウント変化 (coast): 2" in text
    assert "停止理由: duration" in text
    assert "最終状態: DISARMED" in text
    assert "RUN 前 count_total: 3" in text
    assert "最終 count_total: 13" in text
    assert "count/s 最大: 0" in text
    assert "失敗: なし" in text
    assert "RUN 送信回数: 1" in text
    assert proto.fail == ""
    markdown = format_run_markdown(
        "positive",
        text,
        proto.csv_rows,
        proto.events,
        proto.cfg,
        {
            "stamp": "selftest",
            "port": "",
            "log_rel": "logs/selftest.log",
            "csv_rel": "logs/selftest.csv",
            "txt_rel": "logs/selftest.txt",
            "reopens": 0,
            "restoring": False,
            "set_index": 0,
            "arm_sent": proto.arm_sent,
            "run_sent": proto.run_sent,
            "stop_sent": proto.stop_sent,
        },
    )
    assert "分解能: UNKNOWN" in markdown
    assert "0 でない pwm_raw がある" in markdown
    assert "RUNNING の count_total_x4 は 11 のまま" in markdown
    assert "count_total_x4 が 11 になった" in markdown
    assert "EVT,5000000,run,1,5,3" in markdown
    assert "seq,t_us,trial_id,axis,state" in markdown

    early = list(steps)
    # Replace the READY block with DISARMED before any RUN would be sent.
    ready_at = next(i for i, item in enumerate(early) if item[0] == "line" and item[2].startswith("STAT state=READY"))
    early[ready_at] = ("line", 1.4, status_block("DISARMED", authority="usb")[0])
    proto, sent = play("negative", early)
    assert run_line("negative") not in sent
    assert sent.count("ARM") == 1
    assert proto.run_sent == 0

    rejected = list(steps)
    run_ok = next(i for i, item in enumerate(rejected) if item[2] == "OK state=RUNNING")
    rejected[run_ok] = ("line", 1.5, "ERR not_ready")
    proto, sent = play("positive", rejected)
    assert sent.count(run_line("positive")) == 1
    assert sent.count("ARM") == 1
    assert "拒否 ERR not_ready" in format_report(proto)

    proto = Protocol("positive", 0.0)
    sent = []
    sent.extend(proto.on_line("HELP", 0.1))
    for line in status_block("DISARMED"):
        sent.extend(proto.on_line(line, 0.2))
    wiped_lines = ["CFG %s=-1 unit" % key for key, _value in AGREED]
    wiped_lines.extend(["CFG ENCODER_SIGN=1 sign", "CFG GLITCH_NS=0 ns", "CFG -1 means UNCONFIGURED"])
    for line in wiped_lines:
        sent.extend(proto.on_line(line, 0.3))
    for _key, _value in AGREED:
        sent.extend(proto.on_line("OK set", 0.4))
    for line in config_lines():
        sent.extend(proto.on_line(line, 0.5))
    sets = [item for item in sent if item.startswith("SET ")]
    assert sets == ["SET %s %s" % (key, value) for key, value in AGREED], sets
    assert sent.count("AUTHORITY USB") == 1
    assert "ARM" not in sent
    assert proto.phase == "await_authority"

    proto = Protocol("positive", 0.0)
    sent = []
    sent.extend(proto.on_line("HELP", 0.1))
    for line in status_block("DISARMED"):
        sent.extend(proto.on_line(line, 0.2))
    bad_lines = ["CFG %s=%s unit" % (key, "9" if key == "MAX_DUTY_PERCENT" else value) for key, value in AGREED]
    bad_lines.extend(["CFG ENCODER_SIGN=1 sign", "CFG GLITCH_NS=0 ns", "CFG -1 means UNCONFIGURED"])
    for line in bad_lines:
        sent.extend(proto.on_line(line, 0.3))
    assert not any(item.startswith("SET ") for item in sent)
    assert "ARM" not in sent
    assert proto.phase == "failed"
    assert proto.arm_sent == 0

    proto = Protocol("positive", 0.0)
    proto.phase = "await_ready"
    proto.ready_deadline = 1.0
    proto.arm_sent = 1
    proto.stat["state"] = "POWER_WAIT"
    late = proto.on_line(status_block("READY", authority="usb", count="9")[0], 1.2)
    assert run_line("positive") not in late
    assert proto.run_sent == 0
    assert proto.run_blocked

    proto = Protocol("positive", 0.0)
    assert proto.on_line("HELP", 0.1) == ["STATUS"]
    assert proto.on_timeout(4.0) == []
    assert proto.phase == "failed"
    assert proto.fail == "reply_timeout:await_status"

    idle = Protocol("positive", 0.0, idle_arm=True)
    sent = []
    sent.extend(idle.on_line("HELP", 0.1))
    for line in status_block("DISARMED"):
        sent.extend(idle.on_line(line, 0.2))
    for line in config_lines():
        sent.extend(idle.on_line(line, 0.3))
    sent.extend(idle.on_line("OK authority=usb", 0.4))
    for line in status_block("DISARMED", authority="usb"):
        sent.extend(idle.on_line(line, 0.5))
    sent.extend(idle.on_line("OK stream=on", 0.6))
    sent.extend(idle.on_line("OK state=POWER_WAIT", 0.7))
    sent.extend(idle.on_line("4,1000,0,1,READY,1,1,1,1,0,none,0,0,1023,20000,0,2,0,0,0,0,0,,0", 2.0))
    sent.extend(idle.on_line("9,5000,0,1,DISARMED,0,0,0,0,0,none,0,0,1023,20000,0,-4,0,0,0,0,0,ready_timeout,0", 6.0))
    for line in status_block("DISARMED", authority="usb", count="-4", stop="ready_timeout"):
        sent.extend(idle.on_line(line, 6.1))
    sent.extend(idle.on_line("TRACE,1000,POWER_WAIT,1,0,0,0,0,10,2,0,0,0,post", 6.2))
    sent.extend(idle.on_line("TRACE,5000,BRAKE_APPLY,1,0,1,0,0,10,-258,-260,0,0,tick", 6.3))
    assert idle.on_line("TRACE end n=3 overflow=0", 7.0) == []
    assert idle.phase != "done"
    assert idle.on_line("TRACE note still-open", 8.1) == [] or idle.phase == "done"
    assert idle.phase == "done", idle.phase
    assert sent.count("ARM") == 1
    assert run_line("positive") not in sent
    assert "RUN" not in sent
    assert idle.run_sent == 0
    text = format_report(idle)
    assert "RUN: 未送信" in text
    assert "全 CSV の pwm_raw: 0" in text
    assert "パルスやノイズ波形ではない" in text

    print("self-test ok")
    return 0


def unique_file(path):
    flags = os.O_CREAT | os.O_EXCL | os.O_WRONLY
    fd = os.open(path, flags, 0o644)
    return os.fdopen(fd, "w", encoding="utf-8", newline="\n")


def open_port(port, baud):
    import serial

    link = serial.Serial()
    link.port = port
    link.baudrate = baud
    link.timeout = 0.2
    link.write_timeout = 1
    # The firmware CDC marks the host connected only when DTR and RTS are both set.
    # Either line low discards the chip's writes, so a port can look open and stay silent.
    link.dtr = True
    link.rts = True
    link.open()
    return link


def run_serial(port, baud, direction, idle_arm=False):
    os.makedirs(LOG_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    kind = "arm_idle" if idle_arm else "run_once_%s" % direction
    base = os.path.join(LOG_DIR, "%s_%s" % (kind, stamp))
    log_path = base + ".log"
    csv_path = base + ".csv"
    report_path = base + ".txt"
    log_file = unique_file(log_path)
    csv_file = unique_file(csv_path)
    try:
        link = {"ser": open_port(port, baud)}
    except Exception as exc:
        log_file.close()
        csv_file.close()
        os.unlink(log_path)
        os.unlink(csv_path)
        print("could not open %s: %s" % (port, exc), file=sys.stderr)
        return 2

    write_lock = threading.Lock()
    lines = queue.Queue()
    stop_reader = threading.Event()
    stop_heartbeat = threading.Event()
    interrupted = threading.Event()
    reader_dead = threading.Event()
    reopens = {"count": 0}

    def log_text(text):
        log_file.write(text + "\n")
        log_file.flush()

    def send(command):
        data = (command + "\n").encode("ascii")
        try:
            with write_lock:
                link["ser"].write(data)
                link["ser"].flush()
        except Exception as exc:
            if not reader_dead.is_set():
                log_text("WRITE %s" % exc)
                print("WRITE %s" % exc, flush=True)
                reader_dead.set()
                lines.put((time.monotonic(), None))
            return
        log_text(">>> " + command)
        if command != "HEARTBEAT":
            print(">>> " + command, flush=True)

    def reader():
        while not stop_reader.is_set():
            try:
                raw = link["ser"].readline()
            except Exception as exc:
                reader_dead.set()
                lines.put((time.monotonic(), None))
                print("READER %s" % exc, flush=True)
                return
            if not raw:
                continue
            text = raw.decode("utf-8", "replace").replace("\x00", "").strip()
            if text == "":
                continue
            log_text(text)
            if text.startswith("seq,") or text[0].isdigit():
                csv_file.write(text + "\n")
                csv_file.flush()
            elif text != "OK heartbeat":
                print(text, flush=True)
            lines.put((time.monotonic(), text))

    def heartbeat():
        while not stop_heartbeat.is_set():
            try:
                send("HEARTBEAT")
            except Exception:
                return
            if stop_heartbeat.wait(HEARTBEAT_S):
                return

    def handle_sigint(signum, frame):
        del signum, frame
        interrupted.set()

    def reopen_before_arm():
        nonlocal proto, reader_thread, heartbeat_thread
        if proto.arm_sent:
            proto.run_blocked = True
            proto.phase = "failed"
            proto.fail = proto.fail or "serial_dropped_after_arm"
            return False
        if reopens["count"] >= 2:
            proto.phase = "failed"
            proto.fail = proto.fail or "serial_dropped"
            return False
        reopens["count"] += 1
        print("USB dropped before ARM, reopening %d" % reopens["count"], flush=True)
        log_text("REOPEN %d" % reopens["count"])
        stop_heartbeat.set()
        stop_reader.set()
        with write_lock:
            try:
                link["ser"].close()
            except Exception:
                pass
        reader_thread.join(timeout=2)
        if heartbeat_thread is not None:
            heartbeat_thread.join(timeout=1)
            heartbeat_thread = None
        stop_heartbeat.clear()
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not os.path.exists(port):
            time.sleep(0.1)
        time.sleep(0.8)
        try:
            with write_lock:
                link["ser"] = open_port(port, baud)
        except Exception as exc:
            proto.phase = "failed"
            proto.fail = "reopen_failed: %s" % exc
            return False
        while True:
            try:
                lines.get_nowait()
            except queue.Empty:
                break
        reader_dead.clear()
        stop_reader.clear()
        proto = Protocol(direction, time.monotonic(), idle_arm)
        reader_thread = threading.Thread(target=reader, daemon=True)
        reader_thread.start()
        return True

    previous = signal.signal(signal.SIGINT, handle_sigint)
    reader_thread = threading.Thread(target=reader, daemon=True)
    reader_thread.start()
    heartbeat_thread = None
    proto = Protocol(direction, time.monotonic(), idle_arm)
    rx = {"seen": False}
    silence_limit = {"at": time.monotonic() + 2.5}
    try:
        print("port %s" % port, flush=True)
        print("log %s" % log_path, flush=True)
        while proto.phase not in ("done", "failed"):
            now = time.monotonic()
            if not rx["seen"] and not proto.arm_sent and now >= silence_limit["at"]:
                if reopens["count"] >= 2:
                    silence_limit["at"] = now + 3600.0
                elif not reopen_before_arm():
                    break
                else:
                    silence_limit["at"] = time.monotonic() + 2.5
                    continue
            if interrupted.is_set():
                for command in proto.on_interrupt(now):
                    send(command)
                interrupted.clear()
            timeout = proto.seconds_until_deadline(time.monotonic())
            if not rx["seen"]:
                timeout = min(timeout, max(0.05, silence_limit["at"] - time.monotonic()))
            try:
                now, line = lines.get(timeout=timeout)
            except queue.Empty:
                deadline = proto._deadline()
                if deadline is not None and time.monotonic() >= deadline:
                    commands = proto.on_timeout(time.monotonic())
                else:
                    commands = []
            else:
                if line is None:
                    if not reopen_before_arm():
                        break
                    silence_limit["at"] = time.monotonic() + 2.5
                    continue
                rx["seen"] = True
                commands = proto.on_line(line, now)
            for command in commands:
                send(command)
            if proto.wants_heartbeat and heartbeat_thread is None:
                heartbeat_thread = threading.Thread(target=heartbeat, daemon=True)
                heartbeat_thread.start()
        if proto.phase != "done" and proto.arm_sent and not proto.stop_sent:
            for command in proto.on_interrupt(time.monotonic()):
                send(command)
            end = time.monotonic() + MONITOR_WAIT_S
            while time.monotonic() < end and proto.phase != "done":
                try:
                    now, line = lines.get(timeout=0.2)
                except queue.Empty:
                    commands = proto.on_timeout(time.monotonic())
                else:
                    if line is None:
                        proto.run_blocked = True
                        proto.fail = proto.fail or "serial_dropped_after_arm"
                        break
                    commands = proto.on_line(line, now)
                for command in commands:
                    send(command)
    finally:
        stop_heartbeat.set()
        stop_reader.set()
        signal.signal(signal.SIGINT, previous)
        try:
            link["ser"].close()
        except Exception:
            pass
        reader_thread.join(timeout=1.5)
        if heartbeat_thread is not None:
            heartbeat_thread.join(timeout=1.0)
        report = format_report(proto)
        try:
            report_file = unique_file(report_path)
        except FileExistsError:
            report_file = open(report_path, "a", encoding="utf-8")
        report_file.write(report)
        report_file.close()
        log_file.close()
        csv_file.close()
        md_path = publish_run_markdown(
            proto.direction,
            report,
            list(proto.csv_rows),
            list(proto.events),
            dict(proto.cfg),
            {
                "stamp": stamp,
                "port": port,
                "log_rel": "logs/" + os.path.basename(log_path),
                "csv_rel": "logs/" + os.path.basename(csv_path),
                "txt_rel": "logs/" + os.path.basename(report_path),
                "reopens": reopens["count"],
                "restoring": proto.restoring,
                "set_index": proto.set_index,
                "arm_sent": proto.arm_sent,
                "run_sent": proto.run_sent,
                "stop_sent": proto.stop_sent,
                "md_name": "%s_%s.md" % (kind, stamp),
                "trace_lines": list(proto.trace_lines),
                "trace_end": proto.trace_end,
            },
        )
    print(report, end="")
    print("report %s" % report_path)
    print("markdown %s" % md_path)
    if proto.phase != "done" or proto.fail:
        return 1
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Send one encoder-tester RUN after READY is received.")
    parser.add_argument("--self-test", action="store_true", help="check the sequence without opening a port")
    parser.add_argument("--markdown-from", help="write docs/runs markdown from an existing log stem or .log; does not open a port")
    parser.add_argument("--port", help="serial device, for example /dev/cu.usbmodem2101")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--direction", choices=("positive", "negative"), help="USB direction token. Required to open the port.")
    parser.add_argument("--idle-arm", action="store_true", help="ARM once with PWM held at 0. Does not send RUN.")
    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)
    if args.self_test:
        return self_test()
    if args.markdown_from:
        return write_markdown_from(args.markdown_from, args.port or "")
    if args.idle_arm:
        if args.direction:
            print("idle-arm does not take a direction and does not send RUN", file=sys.stderr)
            return 2
        if not args.port:
            print("pass --port for idle-arm", file=sys.stderr)
            return 2
        return run_serial(args.port, args.baud, "idle", True)
    if not args.port or not args.direction:
        print("pass --port and --direction positive|negative", file=sys.stderr)
        print("direction is the USB token, not a measured mechanism direction", file=sys.stderr)
        return 2
    return run_serial(args.port, args.baud, args.direction)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
